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
use kpi::driver;
use kpi::sync::Mutable;
use virtio::{
    ConfigSpace, VqAllocInfo, VqPtr, read_device_config, virtio_alloc_virtqueues,
    virtio_finalize_features, virtio_negotiate_features, virtio_setup_intr, virtqueue_enable_intr,
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

    vq_info: Mutable<[VqAllocInfo; NUM_VQUEUES]>,
    controlq: VqPtr,
    eventq: VqPtr,
    txq: VqPtr,
    rxq: VqPtr,
}

extern "C" fn vtsnd_control_vq_intr(sc: *mut c_void) {}
extern "C" fn vtsnd_event_vq_intr(sc: *mut c_void) {}
extern "C" fn vtsnd_tx_vq_intr(sc: *mut c_void) {}
extern "C" fn vtsnd_rx_vq_intr(sc: *mut c_void) {}

impl DeviceIf for VtSoundDriver {
    type Softc = VtSoundSoftc;

    fn device_probe(dev: device_t) -> Result<BusProbe> {
        let res = unsafe { bindings::vtsnd_device_probe(dev) };
        if res > 0 {
            return Err(ErrCode::from(res));
        }
        Ok(BusProbe(res))
    }

    fn device_attach(uninit_sc: &mut Uninit<VtSoundSoftc>, dev: device_t) -> Result<()> {
        let features = 0;
        let negotiated_features = virtio_negotiate_features(dev, features);
        virtio_finalize_features(dev)?;

        let jacks = read_device_config!(dev, VtSoundConfig, jacks);
        let streams = read_device_config!(dev, VtSoundConfig, streams);
        let chmaps = read_device_config!(dev, VtSoundConfig, chmaps);
        // virtio 1.3 spec 5.14.5.1: The driver MUST NOT read the controls field if
        // VIRITO_SND_F_CTLS has not been negotiated
        let has_controls = negotiated_features & (1 << VIRITO_SND_F_CTLS) != 0;
        let controls = if has_controls {
            Some(read_device_config!(dev, VtSoundConfig, controls))
        } else {
            None
        };

        let vq_info = Default::default();

        let sc = VtSoundSoftc {
            dev,
            negotiated_features,

            jacks,
            streams,
            chmaps,
            controls,

            vq_info: Mutable::new(vq_info),
            controlq: VqPtr::null(),
            eventq: VqPtr::null(),
            txq: VqPtr::null(),
            rxq: VqPtr::null(),
        };
        let sc = uninit_sc.init(sc);

        let mut dmat = bus_dma_tag_create(bus_get_dma_tag(sc.dev)).build()?;

        let dma_map = bus_dmamap_create(dmat, None)?;

        let mut vq_info = sc.vq_info.get_mut();
        vq_info[0].vqai_name = [0; 32]; //c"foo control".as_ptr();
        vq_info[0].vqai_maxindirsz = 0;
        vq_info[0].vqai_intr = Some(vtsnd_control_vq_intr);
        vq_info[0].vqai_intr_arg = core::ptr::null_mut(); //sc.grab_ref();
        vq_info[0].vqai_vq = (&raw const sc.controlq)
            .cast_mut()
            .cast::<*mut bindings::virtqueue>();

        virtio_alloc_virtqueues(dev, &mut *vq_info)?;

        // TODO: populate the event vqueue with empty buffers

        virtio_setup_intr(dev, INTR_TYPE_MISC)?;
        virtqueue_enable_intr(sc.controlq)?;

        Ok(())
    }

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
