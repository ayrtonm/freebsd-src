/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2025 Ayrton Muñoz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

use crate::{VtSoundSoftc, VtSoundDriver};
use crate::virtio_defs::{
    VtSoundDirection, VtSoundPcmFormat, VtSoundPcmHdr, VtSoundPcmInfo, VtSoundPcmRate,
    VtSoundPcmReq, VtSoundPcmSetParams, VtSoundPcmStatus, VtSoundPcmXfer, VtSoundStatus,
};
use core::ffi::c_void;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use kpi::bindings::{kobj, kobj_t, pcmchan_caps};
use kpi::boxed::Box;
use kpi::intr::Callout;
use kpi::kobj::KobjLayout;
use kpi::prelude::*;
use kpi::sync::OnceInit;
use kpi::sync::mtx::Mutex;
use kpi::vec::Vec;
use kpi::{define_class, method_table, proj};
use sound::prelude::*;
use sound::{ChannelIf, Hz, PcmChannel, PcmChannelCaps, PcmDir, PcmTrig, SndDbuf};
use virtio::prelude::*;

// Info passed in to initialize a PCM channel
pub struct VtSoundChannelInfo {
    pub stream_id: u32,
    // The response from querying the virtio device about the PCM stream
    pub info: VtSoundPcmInfo,
}

// The PCM channel data
#[derive(Default)]
pub struct VtSoundChannel {
    // virtio sound device stream ID
    stream_id: u32,
    dir: PcmDir,

    caps: PcmChannelCaps,
    channels_min: u8,
    channels_max: u8,

    buf: OnceInit<SndDbuf>,
    chan: OnceInit<PcmChannel<VtSoundDriver>>,

    state: Mutex<VtSoundChannelState>,
    ptr: AtomicU32,
    running: AtomicBool,
}

impl VtSoundChannel {
    pub fn new(stream_id: usize, info: VtSoundPcmInfo) -> Self {
        let stream_id = stream_id.try_into().unwrap();
        let vtsnd_dir = VtSoundDirection::try_from(info.direction).unwrap();
        let dir = match vtsnd_dir {
            VtSoundDirection::Output => PCMDIR_PLAY,
            VtSoundDirection::Input => PCMDIR_REC,
        };
        let rates_bitmap = info.rates;
        let minrate = VtSoundPcmRate::try_from(rates_bitmap.trailing_zeros()).unwrap();
        let maxrate = VtSoundPcmRate::try_from(64 - rates_bitmap.leading_zeros() - 1).unwrap();

        let mut fmtlist = Vec::with_capacity(4, M_WAITOK);
        let formats_bitmap = info.formats;

        for (f, vtf) in VtSoundPcmFormat::supported_formats() {
            let bit = 1 << vtf as u64;
            if formats_bitmap & bit != 0 {
                fmtlist.push(f);
            }
        }
        fmtlist.push(0);

        let caps = PcmChannelCaps::new(minrate.into_hz(), maxrate.into_hz(), fmtlist).unwrap();

        let mut res = Self {
            stream_id,

            dir,
            caps,
            channels_min: info.channels_min,
            channels_max: info.channels_max,

            ptr: AtomicU32::new(0),
            running: AtomicBool::new(false),

            ..Default::default()
        };
        res.state.get_mut().params_changed = true;
        res
    }

    pub fn get_dir(&self) -> PcmDir {
        self.dir
    }
}

#[derive(Default)]
struct VtSoundChannelState {
    rate: VtSoundPcmRate,
    fmt: VtSoundPcmFormat,
    params_changed: bool,
}

// Use the control virtqueue to set the PCM stream parameters for the channel. This uses polling so
// it will block until the device responds.
fn vtsnd_set_params(
    sc: &VtSoundSoftc,
    chan: &VtSoundChannel,
    format: VtSoundPcmFormat,
    rate: VtSoundPcmRate,
) -> Result<()> {
    let stream_id = chan.stream_id;
    let hdr = VtSoundPcmHdr {
        hdr: VtSoundPcmReq::SetParams as u32,
        stream_id,
    };
    let buf = chan.buf.get();
    let mut req = Some(Box::new(
        VtSoundPcmSetParams {
            hdr,
            buffer_bytes: buf.buffer_size(),
            period_bytes: buf.block_size(),
            features: 0,
            channels: 2,
            format: format as u64 as u8,
            rate: rate as u64 as u8,
            _padding: 0,
        },
        M_WAITOK,
    ));

    let mut resp = Some(Box::new(0xdeadbeefu32, M_WAITOK));
    let mut list = sglist_alloc(3, M_NOWAIT)?;
    let req = sglist_append(&mut list, &mut req)?;
    let resp = sglist_append(&mut list, &mut resp)?;
    let controlq = &sc.controlq;

    virtqueue_enqueue(controlq, &mut Some(list), 1, 1)?;
    virtqueue_notify(controlq);
    let mut list = virtqueue_poll(controlq);

    sglist_reset(&mut list);

    //let req = req.get_buffer();
    if *resp.get_buffer() != VtSoundStatus::NoErr as u32 {
        return Err(EINVAL);
    }
    Ok(())
}

// Use the control virtqueue to request a state change for the PCM stream. This uses polling so it
// will block until the device responds.
fn vtsnd_stream_req(sc: &VtSoundSoftc, chan: &VtSoundChannel, state_req: VtSoundPcmReq) -> Result<()> {
    let stream_id = chan.stream_id;
    assert!(state_req != VtSoundPcmReq::Info);
    assert!(state_req != VtSoundPcmReq::SetParams);

    let mut req = Some(Box::new(
        VtSoundPcmHdr {
            hdr: state_req as u32,
            stream_id,
        },
        M_WAITOK,
    ));
    let mut resp = Some(Box::new(0xdeadbeefu32, M_WAITOK));
    let mut list = sglist_alloc(3, M_NOWAIT)?;
    let req = sglist_append(&mut list, &mut req)?;
    let resp = sglist_append(&mut list, &mut resp)?;
    let controlq = &sc.controlq;

    virtqueue_enqueue(controlq, &mut Some(list), 1, 1)?;
    virtqueue_notify(controlq);
    let mut list = virtqueue_poll(controlq);

    sglist_reset(&mut list);

    let _ = req.get_buffer();
    if *resp.get_buffer() != VtSoundStatus::NoErr as u32 {
        return Err(EINVAL);
    }
    Ok(())
}

fn vtsnd_txq_send(sc: &VtSoundSoftc, chan: &VtSoundChannel) -> Result<VtSoundPcmStatus> {
    // Create an sglist for this set of descriptor buffers
    let mut list = sglist_alloc(3, M_NOWAIT)?;

    // Add the PCM xfer header
    let mut hdr = Some(Box::try_new(
        VtSoundPcmXfer {
            stream_id: chan.stream_id,
        },
        M_NOWAIT,
    )?);
    let hdr_in_list = sglist_append(&mut list, &mut hdr)?;

    // Get the current pointer and append a buffer for the tx data
    let mut start_ptr = chan.ptr.load(Ordering::Relaxed);
    let buf = chan.buf.get();
    // Take the next block out of SndDbuf
    let mut next_block = Some(buf.take(start_ptr)?);
    let block_in_list = sglist_append(&mut list, &mut next_block)?;

    // Add a buffer for the device to write the status of the xfer
    let mut status = Some(Box::try_new(
        VtSoundPcmStatus {
            status: 0xdeadbeef,
            latency_bytes: 0xdeadbeef,
        },
        M_NOWAIT,
    )?);
    let status_in_list = sglist_append(&mut list, &mut status)?;

    // Enqueue the sglist and notify the device as soon as possible
    let txq = &sc.txq;
    virtqueue_enqueue(txq, &mut Some(list), 2, 1)?;
    virtqueue_notify(txq);

    // Compute the new pointer and notify the sound KPI
    let mut new_ptr = start_ptr;
    new_ptr += buf.block_size();
    new_ptr %= buf.buffer_size();
    chan.ptr.store(new_ptr, Ordering::Relaxed);
    chn_intr(chan.chan.get());

    // If the channel hasn't been stopped, reschedule the callout
    if chan.running.load(Ordering::Relaxed) {
        let mut callout = mtx_lock(&sc.callout);
        callout_schedule(&mut callout, 1)?;
        mtx_unlock(callout);
    }

    // The descriptor buffers were on the stack so we need to poll the virtqueue to get the SgList
    // back, reset it then call get_buffer on the SgBuffers returned by sglist_append
    let mut used_list = virtqueue_poll(txq);
    sglist_reset(&mut used_list);
    hdr_in_list.get_buffer();

    // Take the block that the device just used and replace
    let used_block = block_in_list.get_buffer();
    buf.replace(used_block)?;
    Ok(*status_in_list.get_buffer())
}

// Callout callback for PCM channel I/O
extern "C" fn vtsnd_chan_io(sc: &VtSoundSoftc) {
    for chan in &sc.chans {
        if !chan.running.load(Ordering::Relaxed) {
            continue;
        }
        // vtsnd attach should've skipped recording channels so this is just a sanity check
        if chan.dir != PCMDIR_PLAY {
            panic!("unsupported channel direction");
        }

        // If there's not enough space in the txq just reschedule
        if virtqueue_nfree(&sc.txq) < 3 {
            let mut callout = mtx_lock(&sc.callout);
            callout_schedule(&mut callout, 1).unwrap();
            mtx_unlock(callout);
            return;
        }

        vtsnd_txq_send(sc, chan).unwrap();
    }
}

// Implement the rust side of the channel_if methods in the method table
impl ChannelIf for VtSoundChannelClass {
    // The type of the data owned by the channel interface after init
    type Channel = VtSoundChannel;

    type Driver = VtSoundDriver;

    fn channel_init(
        sc: &VtSoundSoftc,
        ch: &VtSoundChannel,
        mut buf: SndDbuf,
        c: PcmChannel<Self::Driver>,
        dir: PcmDir,
    ) {
        let kb = 1024;
        let min_sz = 2 * kb;
        let max_sz = 64 * kb;
        let default = 32 * kb;
        let buf_sz = pcm_getbuffersize(&sc.dev, min_sz..max_sz, default);
        let raw_buf = Vec::with_capacity(buf_sz, M_WAITOK | M_ZERO);
        sndbuf_setup(&mut buf, raw_buf).unwrap();

        mtx_init(&ch.state, &sc.dev, c"vtsnd", None, None);
        ch.buf.init(buf);
        ch.chan.init(c);
    }

    fn channel_free(_: kobj_t, chan: &VtSoundChannel) -> Result<()> {
        Ok(())
    }

    fn channel_trigger(_: kobj_t, chan: &VtSoundChannel, go: PcmTrig) -> Result<()> {
        let sc: &VtSoundSoftc = pcm_get_softc(chan.chan.get());
        match go {
            PCMTRIG_START => {
                let mut state = mtx_lock(&chan.state);
                if state.params_changed {
                    vtsnd_set_params(sc, &chan, state.fmt, state.rate)?;
                    state.params_changed = false;
                }
                mtx_unlock(state);

                vtsnd_stream_req(sc, &chan, VtSoundPcmReq::Prepare)?;
                vtsnd_stream_req(sc, &chan, VtSoundPcmReq::Start)?;

                chan.running.store(true, Ordering::Relaxed);

                let mut callout = mtx_lock(&sc.callout);
                callout_reset(&mut callout, 1, vtsnd_chan_io, sc)?;
                mtx_unlock(callout);
            }
            PCMTRIG_STOP | PCMTRIG_ABORT => {
                chan.running.store(false, Ordering::Relaxed);
                vtsnd_stream_req(sc, &chan, VtSoundPcmReq::Stop)?;
                vtsnd_stream_req(sc, &chan, VtSoundPcmReq::Release)?;
            }
            PCMTRIG_EMLDMAWR | PCMTRIG_EMLDMARD => {}
            _ => device_println!(sc.dev, "ignoring invalid channel_trigger"),
        }
        Ok(())
    }

    fn channel_setformat(_: kobj_t, chan: &VtSoundChannel, format: u32) -> Result<()> {
        for fmt in chan.caps.get_fmtlist() {
            if format == *fmt {
                let vtsnd_fmt = VtSoundPcmFormat::new(format).unwrap();

                let mut state = mtx_lock(&chan.state);
                if state.fmt != vtsnd_fmt {
                    state.fmt = vtsnd_fmt;
                    state.params_changed = true;
                }
                return Ok(());
            }
        }
        Err(EINVAL)
    }

    fn channel_setspeed(_: kobj_t, chan: &VtSoundChannel, speed: Hz) -> Hz {
        let minspeed = chan.caps.get_minspeed();
        let maxspeed = chan.caps.get_maxspeed();
        let limited_speed = if speed < minspeed {
            minspeed
        } else if speed > maxspeed {
            maxspeed
        } else {
            speed
        };
        // Just pick the closest rate
        let closest_rate = VtSoundPcmRate::closest_to(limited_speed);

        let mut state = mtx_lock(&chan.state);
        if state.rate != closest_rate {
            state.rate = closest_rate;
            state.params_changed = true;
        }

        closest_rate.into_hz()
    }

    fn channel_setblocksize(_: kobj_t, chan: &VtSoundChannel, req_block_size: u32) -> u32 {
        let buf = chan.buf.get();
        // The period/block size must be at most half the entire buffer size
        let max_block_size = buf.buffer_size() / 2;
        sndbuf_resize(buf, 2, max_block_size).unwrap();
        max_block_size
    }

    fn channel_getptr(_: kobj_t, chan: &VtSoundChannel) -> u32 {
        chan.ptr.load(Ordering::Relaxed)
    }

    fn channel_getcaps(_: kobj_t, chan: &VtSoundChannel) -> *mut pcmchan_caps {
        chan.caps.as_ptr()
    }
}

// Define the kobj class for the PCM channel
define_class!(
    vtsnd_chan,
    c"vtsnd_chan",
    VtSoundChannelClass,
    vtsnd_chan_methods
);

// Define the size and layout of the kobj class
impl KobjLayout for VtSoundChannelClass {
    // Use the same type specified in the CHANNEL_DECLARE macro in C. Using the incorrect type here
    // would trigger a compiler error in impl ChannelIf for VtSoundChannelClass
    type Layout = kobj;
}

// Define the method table for the PCM channel kobj class
method_table! {
    vtsnd_chan, VtSoundChannelClass,
    vtsnd_chan_methods = {
        channel_init vtsnd_chan_init,
        channel_setformat vtsnd_chan_setformat,
        channel_setspeed vtsnd_chan_setspeed,
        channel_setblocksize vtsnd_chan_setblocksize,
        channel_getptr vtsnd_chan_getptr,
        channel_getcaps vtsnd_chan_getcaps,
        channel_trigger vtsnd_chan_trigger,
    };
    // When using kobj interfaces defined outside the kpi crate, the crates need to be listed
    // explicitly.
    with interfaces from { sound };
}
