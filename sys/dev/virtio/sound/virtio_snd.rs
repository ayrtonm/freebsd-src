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

/* Driver for VirtIO sound devices. */

#![no_std]
#![feature(macro_metavar_expr_concat)]
#![allow(unused, non_camel_case_types)]

use core::ffi::c_void;
use core::ops::Range;
use core::slice;
use core::sync::atomic::{AtomicUsize, Ordering};
use kpi::bindings::{
    AFMT_S16_LE, AFMT_S24_LE, AFMT_S32_LE, INTR_MPSAFE, SD_F_SOFTPCMVOL, SOUND_MASK_PCM,
    SOUND_MASK_RECLEV, SOUND_MASK_VOLUME, VIRTIO_F_VERSION_1, kobj, kobj_t, oss_mixer_enuminfo,
    pcm_channel, pcmchan_caps, snd_dbuf, snd_mixer, vtsnd_get_mixer_class,
};
use kpi::boxed::Box;
use kpi::bus::Irq;
use kpi::collections::{Pod, SgList};
use kpi::ffi::SyncPtr;
use kpi::intr::{Callout, ConfigHook};
use kpi::objects::KobjLayout;
use kpi::prelude::*;
use kpi::sync::arc::{Arc, ArcRef};
use kpi::sync::{Mutable, OnceInit};
use kpi::vec::Vec;
use kpi::{define_class, driver, method_table};
use sound::{
    ChannelIf, Hz, PCMDIR_PLAY, PCMDIR_REC, PcmChannelCaps, PcmDir, PcmSoftc, SndDbuf, pcm_addchan,
    pcm_getbuffersize, pcm_getflags, pcm_init, pcm_register, pcm_setflags, snd_format,
    snd_setup_intr, sndbuf_setup,
};
use virtio::{
    ConfigSpace, VqAllocInfo, VqPtr, read_device_config, virtio_alloc_virtqueues,
    virtio_finalize_features, virtio_negotiate_features, virtio_setup_intr, virtqueue_enqueue,
    virtqueue_full, virtqueue_notify, virtqueue_poll,
};

const NUM_VQUEUES: usize = 4;
const CONTROLQ: usize = 0;
const EVENTQ: usize = 1;
const TXQ: usize = 2;
const RXQ: usize = 3;

#[repr(u32)]
#[derive(Debug, PartialEq, Eq, Copy, Clone)]
enum VtSoundStatus {
    NoErr = 0x8000,
    BadMsg,
    NotSupp,
    IoErr,
}

#[repr(u32)]
#[derive(Debug, PartialEq, Eq, Copy, Clone)]
enum VtSoundPcmReq {
    Info = 0x0100,
    SetParams,
    Prepare,
    Release,
    Start,
    Stop,
}

type VtSoundHdr = u32;

#[repr(C)]
#[derive(Debug, Default)]
struct VtSoundQueryInfo {
    hdr: VtSoundHdr,
    start_id: u32,
    count: u32,
    size: u32,
}

#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
struct VtSoundPcmInfo {
    hdr: VtSoundHdr,
    features: u32,
    formats: u64,
    rates: u64,
    direction: u8,
    channels_min: u8,
    channels_max: u8,
    _padding: [u8; 5],
}

#[repr(C)]
#[derive(Debug, Default)]
struct VtSoundPcmHdr {
    hdr: VtSoundHdr,
    stream_id: u32,
}

#[repr(C)]
#[derive(Debug, Default)]
struct VtSoundPcmSetParams {
    hdr: VtSoundPcmHdr,
    buffer_bytes: u32,
    period_bytes: u32,
    features: u32,
    channels: u8,
    format: u8,
    rate: u8,
    _padding: u8,
}

#[repr(C)]
#[derive(Debug, Default)]
struct VtSoundPcmXfer {
    stream_id: u32,
}

#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
struct VtSoundPcmStatus {
    status: u32,
    latency_bytes: u32,
}

#[repr(u8)]
#[derive(Debug)]
enum VtSoundDirection {
    Output = 0,
    Input,
}

#[repr(u64)]
#[derive(Debug)]
enum VtSoundPcmRate {
    Rate5512 = 0,
    Rate8000,
    Rate11025,
    Rate16000,
    Rate22050,
    Rate32000,
    Rate44100,
    Rate48000,
    Rate64000,
    Rate88200,
    Rate96000,
    Rate176400,
    Rate192000,
    Rate384000,
}

macro_rules! match_enum {
    ($in_var:ident, $in_ty:ty,
        $($variant:expr$(,)?)*) => {
            match $in_var {
                $(x if x == $variant as $in_ty => Ok($variant),)*
                _ => Err(EINVAL),
            }
    };
}

impl VtSoundPcmRate {
    fn into_hz(self) -> Hz {
        match self {
            VtSoundPcmRate::Rate5512 => 5512,
            VtSoundPcmRate::Rate8000 => 8000,
            VtSoundPcmRate::Rate11025 => 11025,
            VtSoundPcmRate::Rate16000 => 16000,
            VtSoundPcmRate::Rate22050 => 22050,
            VtSoundPcmRate::Rate32000 => 32000,
            VtSoundPcmRate::Rate44100 => 44100,
            VtSoundPcmRate::Rate48000 => 48000,
            VtSoundPcmRate::Rate64000 => 64000,
            VtSoundPcmRate::Rate88200 => 88200,
            VtSoundPcmRate::Rate96000 => 96000,
            VtSoundPcmRate::Rate176400 => 176400,
            VtSoundPcmRate::Rate192000 => 192000,
            VtSoundPcmRate::Rate384000 => 384000,
        }
    }
}

impl TryFrom<u32> for VtSoundPcmRate {
    type Error = ErrCode;

    fn try_from(x: u32) -> Result<Self> {
        match_enum!(
            x,
            u32,
            VtSoundPcmRate::Rate5512,
            VtSoundPcmRate::Rate8000,
            VtSoundPcmRate::Rate11025,
            VtSoundPcmRate::Rate16000,
            VtSoundPcmRate::Rate22050,
            VtSoundPcmRate::Rate32000,
            VtSoundPcmRate::Rate44100,
            VtSoundPcmRate::Rate48000,
            VtSoundPcmRate::Rate64000,
            VtSoundPcmRate::Rate88200,
            VtSoundPcmRate::Rate96000,
            VtSoundPcmRate::Rate176400,
            VtSoundPcmRate::Rate192000,
            VtSoundPcmRate::Rate384000
        )
    }
}

#[repr(u64)]
#[derive(Debug)]
enum VtSoundPcmFormat {
    IMA_ADPCM = 0, /*  4 /  4 bits */
    MU_LAW,        /*  8 /  8 bits */
    A_LAW,         /*  8 /  8 bits */
    S8,            /*  8 /  8 bits */
    U8,            /*  8 /  8 bits */
    S16,           /* 16 / 16 bits */
    U16,           /* 16 / 16 bits */
    S18_3,         /* 18 / 24 bits */
    U18_3,         /* 18 / 24 bits */
    S20_3,         /* 20 / 24 bits */
    U20_3,         /* 20 / 24 bits */
    S24_3,         /* 24 / 24 bits */
    U24_3,         /* 24 / 24 bits */
    S20,           /* 20 / 32 bits */
    U20,           /* 20 / 32 bits */
    S24,           /* 24 / 32 bits */
    U24,           /* 24 / 32 bits */
    S32,           /* 32 / 32 bits */
    U32,           /* 32 / 32 bits */
    FLOAT,         /* 32 / 32 bits */
    FLOAT64,       /* 64 / 64 bits */
    /* digital formats (width / physical width) */
    DSD_U8,          /*  8 /  8 bits */
    DSD_U16,         /* 16 / 16 bits */
    DSD_U32,         /* 32 / 32 bits */
    IEC958_SUBFRAME, /* 32 / 32 bits */
}

// virtio sound config space
#[repr(C)]
#[derive(Debug, Default)]
struct VtSoundConfig {
    jacks: u32,
    streams: u32,
    chmaps: u32,
    controls: u32,
}

// SAFETY: VtSoundConfig is repr(C) which matches the virtio sound device's config space
unsafe impl ConfigSpace for VtSoundConfig {}

unsafe impl Pod for VtSoundQueryInfo {}
unsafe impl Pod for VtSoundPcmInfo {}
unsafe impl Pod for VtSoundPcmHdr {}
unsafe impl Pod for VtSoundPcmSetParams {}
unsafe impl Pod for VtSoundPcmXfer {}
unsafe impl Pod for VtSoundPcmStatus {}

pub struct VtSoundChannelInfo {
    stream_id: u32,
    resp: VtSoundPcmInfo,
    sc: Arc<VtSoundSoftc>,
}

pub type VtSoundSoftc = PcmSoftc<VtSoundSoftcFields>;

pub struct VtSoundChannel {
    stream_id: u32,
    dir: PcmDir,
    buf: SndDbuf,
    chan: SyncPtr<pcm_channel>,
    caps: PcmChannelCaps,
    ptr: Mutable<u32>,
    sc: Arc<VtSoundSoftc>,
}

pub struct VtSoundVqPtrs {
    eventq: VqPtr,
    controlq: VqPtr,
    txq: VqPtr,
    rxq: VqPtr,
}

pub struct VtSoundSoftcFields {
    dev: device_t,
    hook: ConfigHook,
    vqs: OnceInit<VtSoundVqPtrs>,
    streams: u32,
    irq: Irq,
    list: Mutable<Option<SgList>>,
    callout: Callout,
}

fn controlq_enqueue_notify_poll(
    sc: &VtSoundSoftc,
    list: SgList,
    num_readable: usize,
    num_writable: usize,
) -> Result<SgList> {
    let controlq = sc.vqs.get().controlq;
    virtqueue_enqueue(controlq, list, num_readable, num_writable)?;
    virtqueue_notify(controlq);
    Ok(virtqueue_poll(controlq))
}

fn vtsnd_query_info(sc: &VtSoundSoftc, stream_ids: Range<u32>) -> Result<Vec<VtSoundPcmInfo>> {
    let num_req_streams = stream_ids.end - stream_ids.start;
    if sc.streams < num_req_streams {
        return Err(EDOOFUS);
    }
    let mut req = VtSoundQueryInfo {
        hdr: VtSoundPcmReq::Info as u32,
        start_id: stream_ids.start,
        count: num_req_streams,
        size: size_of::<VtSoundPcmInfo>().try_into().unwrap(),
    };
    let mut resp_hdr: u32 = 0xdeadbeef;
    let resp: Vec<VtSoundPcmInfo> =
        Vec::with_capacity(num_req_streams as usize, M_DEVBUF, M_WAITOK);

    let mut list = sc.list.get_mut().take().unwrap();
    let req_in_list = sglist_append(&mut list, &mut req)?;
    let resp_hdr_in_list = sglist_append(&mut list, &mut resp_hdr)?;
    let resp_in_list = sglist_append(&mut list, resp)?;

    let mut list = controlq_enqueue_notify_poll(sc, list, 1, 2)?;

    sglist_reset(&mut list);
    *sc.list.get_mut() = Some(list);

    // Discard the request buffer
    let _ = req_in_list.get_buffer();
    // Discard the response header buffer
    let _ = resp_hdr_in_list.get_buffer();
    // Return the response buffer
    Ok(resp_in_list.get_buffer())
}

fn vtsnd_set_params(
    sc: &VtSoundSoftc,
    stream_id: u32,
    buffer_bytes: u32,
    period_bytes: u32,
    format: VtSoundPcmFormat,
    rate: VtSoundPcmRate,
) -> Result<u32> {
    let hdr = VtSoundPcmHdr {
        hdr: VtSoundPcmReq::SetParams as u32,
        stream_id,
    };
    let mut req = VtSoundPcmSetParams {
        hdr,
        buffer_bytes,
        period_bytes,
        features: 0,
        channels: 2,
        format: format as u64 as u8,
        rate: rate as u64 as u8,
        _padding: 0,
    };
    let mut resp: u32 = 0xdeadbeef;
    let mut list = sc.list.get_mut().take().unwrap();
    let req = sglist_append(&mut list, &mut req)?;
    let resp = sglist_append(&mut list, &mut resp)?;

    let mut list = controlq_enqueue_notify_poll(sc, list, 1, 1)?;

    sglist_reset(&mut list);
    *sc.list.get_mut() = Some(list);

    let _ = req.get_buffer();
    Ok(*resp.get_buffer())
}

fn vtsnd_stream_req(sc: &VtSoundSoftc, stream_id: u32, state_req: VtSoundPcmReq) -> Result<u32> {
    assert!(state_req != VtSoundPcmReq::Info);
    assert!(state_req != VtSoundPcmReq::SetParams);

    let mut req = VtSoundPcmHdr {
        hdr: state_req as u32,
        stream_id,
    };
    let mut resp: u32 = 0xdeadbeef;
    let mut list = sc.list.get_mut().take().unwrap();
    let req = sglist_append(&mut list, &mut req)?;
    let resp = sglist_append(&mut list, &mut resp)?;

    let mut list = controlq_enqueue_notify_poll(sc, list, 1, 1)?;

    sglist_reset(&mut list);
    *sc.list.get_mut() = Some(list);

    let _ = req.get_buffer();
    Ok(*resp.get_buffer())
}

extern "C" fn vtsnd_deferred_attach(sc: ArcRef<VtSoundSoftc>) {
    config_intrhook_disestablish(&sc.hook);
}

extern "C" fn vtsnd_event_vq_intr(sc: ArcRef<VtSoundSoftc>) {
    while !virtqueue_full(sc.vqs.get().eventq) {}
}

impl DeviceIf for VtSoundDriver {
    type Softc = VtSoundSoftc;

    fn device_attach(uninit_sc: UninitArc<VtSoundSoftc>, dev: device_t) -> Result<()> {
        let features = VIRTIO_F_VERSION_1 as u64;
        let negotiated_features = virtio_negotiate_features(dev, features);
        virtio_finalize_features(dev)?;

        let streams = read_device_config!(dev, VtSoundConfig, streams);

        let hook = ConfigHook::new();

        let list = sglist_alloc(3, M_WAITOK)?;
        let mut sc = uninit_sc.init(PcmSoftc::new(VtSoundSoftcFields {
            dev,
            hook,
            vqs: OnceInit::uninit(),
            streams,
            irq: Irq::default(),
            list: Mutable::new(Some(list)),
            callout: Callout::new(),
        }));

        callout_init(&mut sc.callout);
        let sc = sc.into_arc();
        let mut vq_info = [const { VqAllocInfo::new() }; NUM_VQUEUES];
        vq_info[EVENTQ].init_with_callback(dev, b"event", 0, vtsnd_event_vq_intr, sc.clone());
        vq_info[CONTROLQ].init(dev, b"control", 0);
        vq_info[TXQ].init(dev, b"tx", 0);
        vq_info[RXQ].init(dev, b"rx", 0);

        let vqs = virtio_alloc_virtqueues(dev, &mut vq_info)?;
        sc.vqs.init(VtSoundVqPtrs {
            eventq: vqs[EVENTQ],
            controlq: vqs[CONTROLQ],
            txq: vqs[TXQ],
            rxq: vqs[RXQ],
        });

        virtio_setup_intr(dev, INTR_TYPE_MISC)?;

        //snd_setup_intr(dev, &sc.irq, INTR_MPSAFE, vtsnd_intr, sc.clone())?;

        sc.hook.init(vtsnd_deferred_attach, sc.clone());
        config_intrhook_establish(&sc.hook)?;

        let resp = vtsnd_query_info(&sc, 0..streams)?;

        pcm_setflags(dev, pcm_getflags(dev) | bindings::SD_F_MPSAFE as u32);
        pcm_init(dev, &sc)?;

        for id in 0..streams {
            let info = &resp[id as usize];
            if info.direction == VtSoundDirection::Input as u8 {
                continue;
            };
            let direction = if info.direction == VtSoundDirection::Output as u8 {
                PCMDIR_PLAY
            } else {
                device_println!(dev, "skipping bad pcm stream info {info:x?}");
                continue;
            };
            let chan_info = Box::new(
                VtSoundChannelInfo {
                    stream_id: id,
                    resp: *info,
                    sc: sc.clone(),
                },
                M_DEVBUF,
                M_WAITOK,
            );
            pcm_addchan(dev, direction, &vtsnd_chan, chan_info)?;
        }
        pcm_register(dev, &sc, c"vtsnd").inspect_err(|e| {
            device_println!(dev, "pcm_register failed {e:?}");
        })?;

        let mixer_sc = Arc::into_raw(sc.clone());
        let mixer_class = unsafe { vtsnd_get_mixer_class() };
        let res = unsafe { bindings::mixer_init(dev, mixer_class, mixer_sc.cast::<c_void>()) };
        if res != 0 {
            device_println!(dev, "mixer_init failed {:?}", ErrCode::from(res));
            return Err(ErrCode::from(res));
        };

        Ok(())
    }

    fn device_detach(_sc: Arc<VtSoundSoftc>, _dev: device_t) -> Result<()> {
        todo!("")
    }
}

driver! {
    vtsnd_driver, c"pcm", VtSoundDriver,
    vtsnd_methods = {
        device_probe vtsnd_device_probe defined in C,
        device_attach vtsnd_device_attach,
        device_detach vtsnd_device_detach,
    }
}

impl KobjLayout for VtSoundChannelClass {
    type Layout = kobj;
}

fn txq_enqueue_notify_poll(ch: &VtSoundChannel) -> Result<VtSoundPcmStatus> {
    let mut hdr = VtSoundPcmXfer {
        stream_id: ch.stream_id,
    };
    let mut status = VtSoundPcmStatus {
        status: 0xdeadbeef,
        latency_bytes: 0xdeadbeef,
    };
    let mut list = ch.sc.list.get_mut().take().unwrap();
    let hdr = sglist_append(&mut list, &mut hdr)?;

    let mut buffer_guard = ch.buf.buffer.get_mut();
    let start = *ch.ptr.get_mut() as usize;
    let end = start + ch.buf.block_size() as usize;
    let buffer = &mut buffer_guard.as_mut_slice()[start..end];
    let buffer = sglist_append(&mut list, buffer)?;

    let status = sglist_append(&mut list, &mut status)?;
    let txq = ch.sc.vqs.get().txq;

    virtqueue_enqueue(txq, list, 2, 1)?;
    virtqueue_notify(txq);
    let mut list = virtqueue_poll(txq);

    sglist_reset(&mut list);
    *ch.sc.list.get_mut() = Some(list);

    let _ = hdr.get_buffer();
    let _ = buffer.get_buffer();
    Ok(*status.get_buffer())
}

extern "C" fn vtsnd_chan_io(ch: ArcRef<VtSoundChannel>) {
    if ch.dir == PCMDIR_PLAY {
        txq_enqueue_notify_poll(&ch).unwrap();
        let mut ptr = ch.ptr.get_mut();
        *ptr += ch.buf.block_size();
        *ptr %= ch.buf.buffer_size();
    } else if ch.dir == PCMDIR_REC {
        panic!("not supported");
    };
    unsafe { bindings::chn_intr(ch.chan.as_ptr()) };
    callout_schedule(&ch.sc.callout, 1);
}

impl ChannelIf for VtSoundChannelClass {
    type DevInfoPtr = VtSoundChannelInfo;
    type DevInfo = Box<VtSoundChannelInfo>;

    type Channel = VtSoundChannel;

    fn channel_init(
        devinfo: Box<VtSoundChannelInfo>,
        b: *mut snd_dbuf,
        c: *mut pcm_channel,
        dir: PcmDir,
    ) -> Result<Arc<VtSoundChannel>> {
        let dev = devinfo.sc.dev;
        let bufsz = pcm_getbuffersize(dev, 2048, 2048, 64 * 1024);
        let buf: Vec<u8> = Vec::with_capacity(bufsz, M_DEVBUF, M_WAITOK | M_ZERO);
        let buf = sndbuf_setup(b, buf)?;

        let rates_bitmap = devinfo.resp.rates;
        let minrate = VtSoundPcmRate::try_from(rates_bitmap.trailing_zeros())?;
        let maxrate = VtSoundPcmRate::try_from(64 - rates_bitmap.leading_zeros() - 1)?;

        let mut fmtlist = Vec::with_capacity(4, M_DEVBUF, M_WAITOK);
        let formats_bitmap = devinfo.resp.formats;
        let s32_bit = 1 << VtSoundPcmFormat::S32 as u64;
        if formats_bitmap & s32_bit != 0 {
            fmtlist.push(snd_format(AFMT_S32_LE, 2, 0));
        }
        let s24_bit = 1 << VtSoundPcmFormat::S24 as u64;
        if formats_bitmap & s24_bit != 0 {
            fmtlist.push(snd_format(AFMT_S24_LE, 2, 0));
        }
        let s16_bit = 1 << VtSoundPcmFormat::S16 as u64;
        if formats_bitmap & s16_bit != 0 {
            fmtlist.push(snd_format(AFMT_S16_LE, 2, 0));
        }
        fmtlist.push(0);

        let caps = PcmChannelCaps::new(minrate.into_hz(), maxrate.into_hz(), fmtlist);
        let ch = Arc::new(
            VtSoundChannel {
                stream_id: devinfo.stream_id,
                dir,
                buf,
                chan: SyncPtr::new(c),
                caps,
                ptr: Mutable::new(0),
                sc: devinfo.sc.clone(),
            },
            M_DEVBUF,
            M_WAITOK,
        );

        let buffer_bytes = ch.buf.buffer_size();
        let period_bytes = ch.buf.block_size();
        let set_params_resp = vtsnd_set_params(
            &devinfo.sc,
            devinfo.stream_id,
            buffer_bytes,
            period_bytes,
            VtSoundPcmFormat::S32,
            VtSoundPcmRate::Rate48000,
        )?;
        if set_params_resp != VtSoundStatus::NoErr as u32 {
            device_println!(
                dev,
                "virtio set_pcm_params request failed with {:x?}",
                set_params_resp
            );
            return Err(EINVAL);
        }
        let resp = vtsnd_stream_req(&devinfo.sc, devinfo.stream_id, VtSoundPcmReq::Prepare)?;
        if resp != VtSoundStatus::NoErr as u32 {
            device_println!(
                dev,
                "virtio prepare_pcm_stream request failed with {:x?}",
                resp
            );
            return Err(EINVAL);
        }

        Ok(ch)
    }

    fn channel_setformat(_: kobj_t, ch: ArcRef<Self::Channel>, format: u32) -> Result<()> {
        for fmt in ch.caps.get_fmtlist() {
            if format == *fmt {
                return Ok(());
            }
        }
        Err(EINVAL)
    }

    fn channel_setspeed(_: kobj_t, ch: ArcRef<Self::Channel>, speed: Hz) -> Hz {
        let minspeed = ch.caps.get_minspeed();
        let maxspeed = ch.caps.get_maxspeed();
        if speed < minspeed {
            minspeed
        } else if speed > maxspeed {
            maxspeed
        } else {
            speed
        }
    }

    fn channel_setblocksize(_: kobj_t, ch: ArcRef<Self::Channel>, blocksize: u32) -> u32 {
        ch.buf.block_size()
    }

    fn channel_getptr(_: kobj_t, ch: ArcRef<Self::Channel>) -> u32 {
        *ch.ptr.get_mut()
    }

    fn channel_getcaps(_: kobj_t, ch: ArcRef<Self::Channel>) -> *mut pcmchan_caps {
        ch.caps.as_ptr()
    }

    fn channel_trigger(_: kobj_t, ch: ArcRef<Self::Channel>, go: i32) -> Result<()> {
        match go {
            bindings::PCMTRIG_START => {
                let resp = vtsnd_stream_req(&ch.sc, ch.stream_id, VtSoundPcmReq::Start)?;
                device_println!(
                    ch.sc.dev,
                    "device responded with {resp:x?} to start stream request"
                );

                *ch.ptr.get_mut() = 0;
                callout_reset(&ch.sc.callout, 1, vtsnd_chan_io, ch.into_arc())?;
            }
            bindings::PCMTRIG_STOP | bindings::PCMTRIG_ABORT => {}
            _ => {}
        }
        Ok(())
    }
}

define_class!(
    vtsnd_chan,
    c"vtsnd_chan",
    VtSoundChannelClass,
    vtsnd_chan_methods
);

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
    with interfaces from { sound };
}
