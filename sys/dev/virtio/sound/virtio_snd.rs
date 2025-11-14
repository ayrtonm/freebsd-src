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

use core::ffi::c_void;
use core::ptr::null_mut;
use kpi::bindings::VIRTIO_F_VERSION_1;
use kpi::boxed::Box;
use kpi::bus::dma::BusDmaMap;
use kpi::driver;
use kpi::intr::ConfigHook;
use kpi::vec::Vec;
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
#[derive(Debug)]
struct VtSoundHdr {
    code: u32,
}

#[repr(C)]
#[derive(Debug)]
struct VtSoundEvent {
    hdr: VtSoundHdr,
    data: u32,
}

#[repr(C)]
#[derive(Debug)]
struct VtSoundQueryInfo {
    hdr: VtSoundHdr,
    start_id: u32,
    count: u32,
    size: u32,
}

#[repr(C)]
#[derive(Debug)]
struct VtSoundPcmHdr {
    hdr: VtSoundHdr,
    stream_id: u32,
}

#[repr(C)]
#[derive(Debug)]
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
#[derive(Debug)]
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
#[derive(Debug)]
struct VtSoundPcmXfer {
    stream_id: u32,
}

#[repr(C)]
#[derive(Debug)]
struct VtSoundPcmStatus {
    status: u32,
    latency_bytes: u32,
}

#[repr(C)]
#[derive(Debug)]
struct VtSoundConfig {
    jacks: u32,
    streams: u32,
    chmaps: u32,
    controls: u32,
}

// SAFETY: VtSoundConfig is repr(C) which matches the virtio sound device's config space
unsafe impl ConfigSpace for VtSoundConfig {}

pub struct VtSoundSoftc {
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
}

extern "C" fn vtsnd_deferred_attach(sc: &RefCounted<VtSoundSoftc>) {
    let cookiep = virtqueue_poll(sc.controlq);
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

        let mut sc = uninit_sc.init(VtSoundSoftc {
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
        });

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

        let mut list = sglist_alloc(2, M_NOWAIT)?;
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
        let resp_size = size_of::<VtSoundHdr>() + ((req.count * req.size) as usize);
        let mut resp: Vec<u8> = Vec::try_with_capacity(resp_size, M_DEVBUF, M_NOWAIT)?;
        let req_in_list = sglist_append(&mut list, req)?;
        let resp_in_list = sglist_append(&mut list, resp)?;
        virtqueue_enqueue(
            sc.controlq,
            (&raw const *sc).cast::<c_void>().cast_mut(),
            &mut list,
            1,
            1,
        )?;

        virtio_setup_intr(dev, INTR_TYPE_MISC)?;

        Ok(())
        //let jacks = read_device_config!(dev, VtSoundConfig, jacks);
    }
    //fn device_attach(uninit_sc: UninitPtr<VtSoundSoftc>, dev: device_t) -> Result<()> {
    //    let features = 0;
    //    let negotiated_features = virtio_negotiate_features(dev, features);
    //    virtio_finalize_features(dev)?;

    //    let jacks = read_device_config!(dev, VtSoundConfig, jacks);
    //    let streams = read_device_config!(dev, VtSoundConfig, streams);
    //    let chmaps = read_device_config!(dev, VtSoundConfig, chmaps);
    //    // virtio 1.3 spec 5.14.5.1: The driver MUST NOT read the controls field if
    //    // VIRITO_SND_F_CTLS has not been negotiated
    //    let has_controls = negotiated_features & (1 << VIRITO_SND_F_CTLS) != 0;
    //    let controls = if has_controls {
    //        Some(read_device_config!(dev, VtSoundConfig, controls))
    //    } else {
    //        None
    //    };

    //    let sc = VtSoundSoftc {
    //        dev,
    //        negotiated_features,

    //        jacks,
    //        streams,
    //        chmaps,
    //        controls,

    //        eventq: VqPtr::null(),
    //        controlq: VqPtr::null(),
    //        txq: VqPtr::null(),
    //        rxq: VqPtr::null(),
    //        hook: ConfigHook::try_new(M_DEVBUF, M_NOWAIT)?,
    //    };
    //    let mut sc = uninit_sc.init(sc);

    //    let hook_ctx = sc.weak_ref();
    //    config_intrhook_establish(&mut sc.hook, vtsnd_deferred_attach, hook_ctx)?;

    //    let mut list = sglist_alloc(1, M_NOWAIT)?;
    //    let req = VtSoundQueryInfo {
    //        hdr: VtSoundHdr {
    //            code: u32::from(ControlMsg::PcmInfo as u16),
    //        },
    //        start_id: 0,
    //        count: streams,
    //        size: size_of::<VtSoundPcmInfo>().try_into().unwrap(),
    //    };
    //    let req = Box::try_new(req, M_DEVBUF, M_NOWAIT)?;
    //    sglist_append(&mut list, req)?;

    //    let mut vq_info = [const { VqAllocInfo::new() }; NUM_VQUEUES];

    //    vq_info[CONTROLQ].init(dev, b"control", 0);

    //    let event_vq_ctx = sc.weak_ref();
    //    vq_info[EVENTQ].init_with_callback(dev, b"event", 0, vtsnd_event_vq_intr, event_vq_ctx);

    //    vq_info[TXQ].init(dev, b"tx", 0);
    //    vq_info[RXQ].init(dev, b"rx", 0);

    //    let vqs = virtio_alloc_virtqueues(dev, &mut vq_info)?;

    //    sc.controlq = vqs[CONTROLQ];
    //    sc.eventq = vqs[EVENTQ];
    //    sc.txq = vqs[TXQ];
    //    sc.rxq = vqs[RXQ];

    //    let dma_tag = bus_dma_tag_create(bus_get_dma_tag(dev)).build()?;

    //    //virtqueue_enqueue(
    //    //    sc.controlq,
    //    //    null_mut(),
    //    //    &mut list,
    //    //    0,
    //    //    1, /* num_writable */
    //    //)?;

    //    //let mut dmat = bus_dma_tag_create(bus_get_dma_tag(sc.dev)).build()?;

    //    //let dma_map = bus_dmamap_create(dmat, None)?;

    //    struct Event {
    //        dma_map: BusDmaMap,
    //    }
    //    // TODO: populate the event vqueue with empty buffers
    //    let dma_map = bus_dmamap_create(dma_tag, None)?;
    //    let evt = Box::try_new(Event { dma_map }, M_DEVBUF, M_NOWAIT)?;

    //    //virtqueue_enqueue(sc.eventq, null_mut(), &mut list,

    //    virtio_setup_intr(dev, INTR_TYPE_MISC)?;

    //    virtqueue_enable_intr(sc.eventq)?;

    //    Ok(())
    //}

    fn device_detach(_sc: &RefCounted<VtSoundSoftc>, _dev: device_t) -> Result<()> {
        todo!("")
    }
}

driver! {
    vtsnd_driver, c"vtsnd", VtSoundDriver, vtsnd_methods,
    INTERFACES {
        // The _rust suffix avoids a name collision with the probe function in virtio_snd.c
        device_probe vtsnd_device_probe_rust,
        device_attach vtsnd_device_attach,
        device_detach vtsnd_device_detach,
    }
}
