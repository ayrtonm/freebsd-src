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

use core::ffi::{c_int, c_void};
use core::mem::MaybeUninit;
use core::ptr::null_mut;
use kpi::bindings::{
    VIRTIO_F_VERSION_1, kobj, kobj_class_t, kobj_t, pcm_channel, pcmchan_caps, pcmchan_matrix,
    snd_dbuf, snddev_info,
};
use kpi::boxed::Box;
use kpi::bus::dma::BusDmaMap;
use kpi::prelude::*;
use kpi::{define_class, method_table};
use sound::{ChannelIf, channel_init};

use kpi::collections::ScatterList;
use kpi::driver;
use kpi::ffi::SubClass;
use kpi::intr::ConfigHook;
use kpi::objects::KobjClassSize;
use kpi::sync::Mutable;
use kpi::vec::Vec;
use sound::{PcmChannel, PcmDir, pcm_addchan, pcm_init, pcm_register, sndbuf_setup};
use virtio::{
    ConfigSpace, VqAllocInfo, VqPtr, read_device_config, virtio_alloc_virtqueues,
    virtio_finalize_features, virtio_negotiate_features, virtio_setup_intr, virtqueue_enable_intr,
    virtqueue_enqueue, virtqueue_full, virtqueue_poll,
};

#[repr(u16)]
enum ControlMsg {
    JackInfo = 1,
    JackRemap,
    PcmInfo = 0x0100,
    PcmSetParams,
    PcmPrepare,
    PcmRelease,
    PcmStart,
    PcmStop,
    ChmapInfo = 0x0200,
    CtlInfo = 0x0300,
    CtlEnumItems,
    CtlRead,
    CtlWrite,
    CtlTlvRead,
    CtlTlvWrite,
    CtlTlvCommand,
    EvtJackConnected = 0x1000,
    EvtJackDisconnected,
    EvtPcmPeriodElapsed = 0x1100,
    EvtPcmXrun,
    EvtCtlNotify = 0x1200,
    OkMsg = 0x8000,
    BadMsg,
    NotSupp,
    IoErr,
}

const NUM_VQUEUES: usize = 4;
const CONTROLQ: usize = 0;
const EVENTQ: usize = 1;
const TXQ: usize = 2;
const RXQ: usize = 3;
const VIRITO_SND_F_CTLS: u64 = 0;

#[repr(C)]
#[derive(Debug, Default)]
struct VtSoundHdr {
    code: u32,
}

#[repr(C)]
#[derive(Debug, Default)]
struct VtSoundEvent {
    hdr: VtSoundHdr,
    data: u32,
}

#[repr(C)]
#[derive(Debug, Default)]
struct VtSoundQueryInfo {
    hdr: VtSoundHdr,
    start_id: u32,
    count: u32,
    size: u32,
}

#[repr(C)]
#[derive(Debug, Default)]
struct VtSoundPcmHdr {
    hdr: VtSoundHdr,
    stream_id: u32,
}

#[repr(C)]
#[derive(Debug, Default)]
struct VtSoundPcmInfo {
    hdr: VtSoundHdr,
    features: u32,
    formats: u64,
    rates: u64,
    direction: u8,
    channels_min: u8,
    channels_max: u8,
    padding: [u8; 5],
}

#[repr(C)]
#[derive(Debug, Default)]
struct VtSoundPcmSetParams {
    hdr: VtSoundHdr,
    buffer_bytes: u32,
    period_bytes: u32,
    features: u32,
    channels: u8,
    format: u8,
    rate: u8,
    padding: u8,
}

#[repr(C)]
#[derive(Debug, Default)]
struct VtSoundPcmXfer {
    stream_id: u32,
}

#[repr(C)]
#[derive(Debug, Default)]
struct VtSoundPcmStatus {
    status: u32,
    latency_bytes: u32,
}

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

pub type VtSoundSoftc = SubClass<snddev_info, VtSoundSoftcFields>;

pub struct VtSoundSoftcFields {
    dev: device_t,
    negotiated_features: u64,

    // config space
    jacks: u32,
    streams: u32,
    chmaps: u32,
    controls: Option<u32>,

    eventq: VqPtr,
    controlq: VqPtr,
    txq: VqPtr,
    rxq: VqPtr,
    hook: ConfigHook,

    resp: Mutable<Option<ListBuffer<Vec<VtSoundPcmInfo>>>>,
    //list: Mutable<ScatterList>,
    chan: VtSoundChannel,
}

unsafe impl Sync for VtSoundChannel {}

#[derive(Debug, Default)]
pub struct VtSoundChannel {
    dev: device_t,
    pcm_cap: pcmchan_caps,
    pcm_buf: *mut snd_dbuf,
    pcm_chan: *mut pcm_channel,
}

impl PcmChannel for VtSoundSoftcFields {
    type DevInfo = VtSoundChannel;
    fn get_devinfo(&self) -> *const Self::DevInfo {
        &raw const self.chan
    }
}

extern "C" fn vtsnd_deferred_attach(sc: &RefCounted<VtSoundSoftc>) {
    let mut list = virtqueue_poll(sc.controlq);
    sglist_reset(&mut list);
    device_println!(
        sc.dev,
        "recv'ed {:#x?}",
        sc.resp.get_mut().take().unwrap().get_buffer()
    );

    pcm_init(sc.dev, sc).unwrap();

    pcm_addchan(sc.grab_ref(), PcmDir::Play, &vtsnd_chan).unwrap();

    if let Err(e) = pcm_register(sc, c"vtsnd") {
        device_println!(sc.dev, "pcm_register failed ({e:?})");
    }
    config_intrhook_disestablish(&sc.hook);
}

extern "C" fn vtsnd_event_vq_intr(sc: &RefCounted<VtSoundSoftc>) {
    while !virtqueue_full(sc.eventq) {}
}

impl DeviceIf for VtSoundDriver {
    type Softc = VtSoundSoftc;

    fn device_probe(dev: device_t) -> Result<BusProbe> {
        let res = unsafe { bindings::vtsnd_device_probe(dev) };
        if res > 0 {
            return Err(ErrCode::from(res));
        }
        Ok(BusProbe(res))
    }

    fn device_attach(uninit_sc: UninitPtr<VtSoundSoftc>, dev: device_t) -> Result<()> {
        let features = VIRTIO_F_VERSION_1 as u64;
        let negotiated_features = virtio_negotiate_features(dev, features);
        virtio_finalize_features(dev)?;

        let hook = ConfigHook::try_new(M_DEVBUF, M_NOWAIT)?;

        let mut sc = uninit_sc.init(VtSoundSoftc::new(VtSoundSoftcFields {
            dev,
            negotiated_features,
            jacks: 0,
            streams: 0,
            chmaps: 0,
            controls: None,
            eventq: VqPtr::null(),
            controlq: VqPtr::null(),
            txq: VqPtr::null(),
            rxq: VqPtr::null(),
            hook,
            resp: Mutable::new(None),
            chan: VtSoundChannel::default(),
            //list: Mutable::new(list),
        }));
        sc.chan.dev = dev;
        sc.chan.pcm_cap.fmtlist = todo!("");
        //unsafe { (*sc.chan.pcm_cap.assume_init_mut()).fmtlist = todo!("") };//bindings::vtsnd_get_fmt() };

        let hook_ctx = sc.weak_ref();
        config_intrhook_establish(&mut sc.hook, vtsnd_deferred_attach, hook_ctx)?;

        let mut vq_info = [const { VqAllocInfo::new() }; NUM_VQUEUES];
        let event_vq_ctx = sc.weak_ref();
        vq_info[EVENTQ].init_with_callback(dev, b"event", 0, vtsnd_event_vq_intr, event_vq_ctx);
        vq_info[CONTROLQ].init(dev, b"control", 0);
        vq_info[TXQ].init(dev, b"tx", 0);
        vq_info[RXQ].init(dev, b"rx", 0);

        let vqs = virtio_alloc_virtqueues(dev, &mut vq_info)?;
        sc.eventq = vqs[EVENTQ];
        sc.controlq = vqs[CONTROLQ];
        sc.txq = vqs[TXQ];
        sc.rxq = vqs[RXQ];

        let streams = read_device_config!(dev, VtSoundConfig, streams);
        sc.streams = streams;

        let req = Box::try_new(
            VtSoundQueryInfo {
                hdr: VtSoundHdr {
                    code: u32::from(ControlMsg::PcmInfo as u16),
                },
                start_id: 0,
                count: streams,
                size: size_of::<VtSoundPcmInfo>().try_into().unwrap(),
            },
            M_DEVBUF,
            M_NOWAIT,
        )?;
        let mut resp_hdr = Box::try_new(VtSoundHdr { code: 0xdeadbeef }, M_DEVBUF, M_NOWAIT)?;
        let mut resp = Vec::try_with_capacity(req.count.try_into().unwrap(), M_DEVBUF, M_NOWAIT)?;
        let mut list = sglist_alloc(3, M_NOWAIT)?;
        let req_in_list = sglist_append(&mut list, req)?;
        let resp_hdr_in_list = sglist_append(&mut list, resp_hdr)?;
        let resp_in_list = sglist_append(&mut list, resp)?;
        virtqueue_enqueue(sc.controlq, list, 1, 2)?;
        req_in_list.leak_buffer();
        resp_hdr_in_list.leak_buffer();
        *sc.resp.get_mut() = Some(resp_in_list);

        virtio_setup_intr(dev, INTR_TYPE_MISC)?;

        Ok(())
    }

    fn device_detach(_sc: &RefCounted<VtSoundSoftc>, _dev: device_t) -> Result<()> {
        todo!("")
    }
}

driver! {
    vtsnd_driver, c"vtsnd", VtSoundDriver, vtsnd_methods = {
        // The _rust suffix avoids a name collision with the probe function in virtio_snd.c
        device_probe vtsnd_device_probe_rust,
        device_attach vtsnd_device_attach,
        device_detach vtsnd_device_detach,
    }
}

impl KobjClassSize for VtSoundChannelClass {
    const SIZE: usize = size_of::<kobj>();
}

impl ChannelIf for VtSoundChannelClass {
    type DevInfo = VtSoundChannel;
    fn channel_init(
        devinfo: &mut VtSoundChannel,
        b: *mut snd_dbuf,
        c: *mut pcm_channel,
        dir: PcmDir,
    ) {
        sndbuf_setup(b, Vec::try_with_capacity(1024, M_DEVBUF, M_WAITOK).unwrap()).unwrap();
        devinfo.pcm_buf = b;
        devinfo.pcm_chan = c;
    }
    fn channel_setformat(_: kobj_t, devinfo: &VtSoundChannel, format: u32) -> Result<()> {
        device_println!(devinfo.dev, "selecting format {format:#x?}");
        Ok(())
    }
    fn channel_setspeed(_: kobj_t, devinfo: &Self::DevInfo, speed: u32) -> u32 {
        speed
    }
    fn channel_setblocksize(_: kobj_t, devinfo: &Self::DevInfo, blocksize: u32) -> u32 {
        blocksize
    }
    fn channel_setfragments(
        _: kobj_t,
        devinfo: &Self::DevInfo,
        blocksize: u32,
        blockcount: u32,
    ) -> Result<()> {
        Ok(())
    }
    fn channel_trigger(_: kobj_t, devinfo: &Self::DevInfo, go: c_int) -> Result<()> {
        Ok(())
    }
    fn channel_getptr(_: kobj_t, devinfo: &Self::DevInfo) -> u32 {
        todo!("")
    }
    fn channel_getcaps(_: kobj_t, devinfo: &Self::DevInfo) -> *mut pcmchan_caps {
        (&raw const devinfo.pcm_cap).cast_mut()
    }
    fn channel_getmatrix(_: kobj_t, devinfo: &Self::DevInfo, format: u32) -> *mut pcmchan_matrix {
        unsafe { bindings::feeder_matrix_format_map(format) }
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
        channel_setfragments vtsnd_chan_setfragments,
        channel_trigger vtsnd_chan_trigger,
        channel_getptr vtsnd_chan_getptr,
        channel_getcaps vtsnd_chan_getcaps,
        channel_getmatrix vtsnd_chan_getmatrix,
    };
    with imports { sound };
}
