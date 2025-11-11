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

use core::ops::Range;
use core::pin::Pin;
use kpi::bindings::{SD_F_MPSAFE, VIRTIO_F_VERSION_1, VIRTIO_ID_SOUND};
use kpi::boxed::Box;
use kpi::collections::{LocalSgList};
use kpi::device::{BusProbe, DeviceIf, Device};
use kpi::ffi::{PinIndex, Softc, Uninit};
use kpi::intr::Callout;
use kpi::prelude::*;
use kpi::sync::mtx::Mutex;
use kpi::vec::Vec;
use kpi::{define_driver, define_pinned_struct};
use sound::PcmSoftc;
use sound::prelude::*;
use virtio::prelude::*;
use virtio::{Virtqueue, VqSet, VirtioDevice};

// The build system doesn't capture dependencies on these files yet but whatever
mod channel;
mod mixer;
mod virtio_defs;

use channel::{VtSoundChannel, VtSoundChannelInfo, vtsnd_chan};
use mixer::vtsnd_mixer;
use virtio_defs::{
    VtSoundStatus, VtSoundConfig, VtSoundDirection, VtSoundPcmInfo, VtSoundPcmReq, VtSoundQueryInfo,
};

const NUM_VQUEUES: usize = 4;
const CONTROLQ: usize = 0;
const EVENTQ: usize = 1;
const TXQ: usize = 2;
const RXQ: usize = 3;

// Use the control virtqueue to query the device about a range of PCM streams. This uses polling so
// it will block until the device responds.
fn vtsnd_query_info(dev: &Device, controlq: &mut Virtqueue, stream_ids: Range<u32>) -> Result<Vec<VtSoundPcmInfo>> {
    let num_req_streams = stream_ids.end - stream_ids.start;

    let req = VtSoundQueryInfo {
            hdr: VtSoundPcmReq::Info as u32,
            start_id: stream_ids.start,
            count: num_req_streams,
            size: size_of::<VtSoundPcmInfo>().try_into().unwrap(),
        };
    let mut resp_hdr = 0xdeadbeefu32;
    let mut resp: Vec<VtSoundPcmInfo> = Vec::with_capacity(num_req_streams as usize, M_DEVBUF, M_WAITOK);
    resp.fill_to_capacity(VtSoundPcmInfo::default());

    let mut list = LocalSgList::<4>::new();
    list.append_readable(&req)?;
    list.append_writeable(&mut resp_hdr)?;
    list.append_writeable_slice(&mut resp)?;

    virtqueue_enqueue_and_wait(controlq, &mut list)?;

    if resp_hdr != VtSoundStatus::NoErr as u32 {
        device_println!(dev, "failed to query device {resp:x?}");
        return Err(ENXIO);
    }

    Ok(resp)
}

// An alias to avoid using tons of angle brackets everywhere. The driver needs to use the PCM sound
// interfaces so it must be a subclass of snddev_info which is ensured by PcmSoftc.
pub type VtSoundSoftc = PcmSoftc<VtSoundSoftcFields>;

define_pinned_struct! {
    // The extra fields in the subclass of snddev_info
    pub struct VtSoundSoftcFields {
        callout: Mutex<Callout>,
        // The channels are pinned since the sound KPI holds pointers to them
        chans: Pin<Box<[VtSoundChannel]>>,
        eventq: Virtqueue,
        controlq: Mutex<Virtqueue>,
        txq: Mutex<Virtqueue>,
        rxq: Virtqueue,
    }
}

// Implement the rust side of the device_if methods in the method table
impl DeviceIf for VtSoundDriver {
    type Softc = VtSoundSoftc;

    fn device_probe(dev: &Device) -> Result<BusProbe> {
        let dev = VirtioDevice::new(dev)?;
        if virtio_get_device_type(dev) != VIRTIO_ID_SOUND {
            return Err(ENXIO);
        }
        device_set_desc(&dev, c"VirtIO Sound Adapter");
        Ok(BUS_PROBE_DEFAULT)
    }

    fn device_attach(uninit_sc: &Uninit<VtSoundSoftc>) -> Result<()> {
        let vdev = VirtioDevice::new(uninit_sc.device())?;
        let dev = &vdev;

        let features = VIRTIO_F_VERSION_1 as u64;
        let negotiated_features = virtio_negotiate_features(vdev, features);
        virtio_finalize_features(vdev)?;

        let mut vqs: VqSet<NUM_VQUEUES> = VqSet::new(vdev);
        vqs.init(CONTROLQ, c"controlq", 0)?;
        vqs.init(EVENTQ, c"eventq", 0)?;
        vqs.init(TXQ, c"txq", 0)?;
        vqs.init(RXQ, c"rxq", 0)?;

        virtio_alloc_virtqueues(&mut vqs)?;

        let mut controlq = vqs.get(CONTROLQ)?;
        let eventq = vqs.get(EVENTQ)?;
        let txq = vqs.get(TXQ)?;
        let rxq = vqs.get(RXQ)?;

        // FIXME: requires an initialized softc
        virtio_setup_intr(vdev, INTR_TYPE_MISC)?;

        // Read the virtio config space to see how many PCM streams the device supports
        let num_streams = virtio_read_device_config(vdev, VtSoundConfig::streams);

        // Get info on all the PCM streams as a Vec of VtSoundPcmInfo
        let stream_info = vtsnd_query_info(dev, &mut controlq, 0..num_streams)?;
        device_println!(dev, "QEMU replied with vtsnd info {stream_info:?}");

        // Now that we know the number of streams let's allocate space for the channels
        let mut chans = Vec::with_capacity(num_streams as usize, M_DEVBUF, M_WAITOK);

        for (stream_id, &info) in stream_info.iter().enumerate() {
            // Skip recording streams
            if info.direction == VtSoundDirection::Input as u8 {
                continue;
            };
            chans.push(VtSoundChannel::new(stream_id, info));
        }
        if chans.is_empty() {
            device_println!(dev, "no playback streams found");
            return Err(ENXIO);
        }
        // into_boxed_slice shrinks the capacity to match the length
        let chans = Box::into_pin(chans.into_boxed_slice(M_WAITOK));

        // Write to the softc pointer, returning a reference to the initialized softc
        let sc = uninit_sc.init(PcmSoftc::new(VtSoundSoftcFields {
            callout: Mutex::new(Callout::new()),
            chans,
            eventq,
            controlq: Mutex::new(controlq),
            txq: Mutex::new(txq),
            rxq,
        }));

        mtx_init(sc.project().callout, c"vtsnd_callout", None, None);
        mtx_init(sc.project().controlq, c"vtsnd_controlq", None, None);
        mtx_init(sc.project().txq, c"vtsnd_txq", None, None);

        let mut callout = mtx_lock(&sc.callout);
        callout_init(&mut callout)?;
        mtx_unlock(callout);

        // Mark the driver as multiprocessor-safe for the sound interface since we don't register an
        // interrupt handler with snd_setup_intr
        pcm_setflags(dev, pcm_getflags(dev) | SD_F_MPSAFE as u32);

        // Initialize the snddev_info base class of the softc
        pcm_init(dev, sc)?;

        // Add a PCM channel for each stream
        let chans = sc.chans.as_ref();
        for i in 0..chans.len() {
            let chan = chans.pin_index(i);
            pcm_addchan(dev, chan.get_dir(), &vtsnd_chan, chan)?;
        }

        // After adding all PCM channels register the device
        pcm_register(dev, c"vtsnd").inspect_err(|e| {
            device_println!(dev, "pcm_register failed {e:?}");
        })?;

        // Initialize the mixer
        mixer_init(dev, &vtsnd_mixer, sc).inspect_err(|e| {
            device_println!(dev, "mixer_init failed {e:?}");
        })?;

        Ok(())
    }

    fn device_detach(sc: &Softc<VtSoundSoftc>) -> Result<()> {
        pcm_unregister(sc.device()).inspect_err(|e| {
            device_println!(sc.device(), "pcm_unregister failed {e:?}");
        })
    }
}

// Defines the VtSoundDriver type for the driver and its method table
define_driver! {
    static vtsnd_driver: VtSoundDriver = {
        name: c"pcm",
    }
    static vtsnd_methods = {
        device_probe: vtsnd_device_probe,
        device_attach: vtsnd_device_attach,
        device_detach: vtsnd_device_detach,
    }
}
