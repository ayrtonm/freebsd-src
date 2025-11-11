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
use kpi::bindings::{SD_F_MPSAFE, VIRTIO_F_VERSION_1, device_t};
use kpi::boxed::Box;
use kpi::device::DeviceIf;
use kpi::driver;
use kpi::ffi::{DevRef, UninitDevRef};
use kpi::intr::Callout;
use kpi::prelude::*;
use kpi::sync::mtx::Mutex;
use kpi::vec::Vec;
use sound::PcmSoftc;
use sound::prelude::*;
use virtio::prelude::*;
use virtio::{Virtqueue, VqAllocInfo};

// The build system doesn't capture dependencies on these files yet but whatever
mod channel;
mod mixer;
mod virtio_defs;

use channel::{VtSoundChannelInfo, vtsnd_chan};
use mixer::vtsnd_mixer;
use virtio_defs::{
    VtSoundConfig, VtSoundDirection, VtSoundPcmInfo, VtSoundPcmReq, VtSoundQueryInfo,
};

const NUM_VQUEUES: usize = 4;
const CONTROLQ: usize = 0;
const EVENTQ: usize = 1;
const TXQ: usize = 2;
const RXQ: usize = 3;

// Use the control virtqueue to query the device about a range of PCM streams. This uses polling so
// it will block until the device responds.
fn vtsnd_query_info(sc: &VtSoundSoftc, stream_ids: Range<u32>) -> Result<Vec<VtSoundPcmInfo>> {
    let num_req_streams = stream_ids.end - stream_ids.start;

    let mut req = Some(Box::new(
        VtSoundQueryInfo {
            hdr: VtSoundPcmReq::Info as u32,
            start_id: stream_ids.start,
            count: num_req_streams,
            size: size_of::<VtSoundPcmInfo>().try_into().unwrap(),
        },
        M_DEVBUF,
        M_WAITOK,
    ));
    let mut resp_hdr = Some(Box::new(0xdeadbeefu32, M_DEVBUF, M_WAITOK));
    let mut resp: Option<Vec<VtSoundPcmInfo>> = Some(Vec::with_capacity(
        num_req_streams as usize,
        M_DEVBUF,
        M_WAITOK,
    ));

    let mut list = sglist_alloc(3, M_NOWAIT)?;
    // Need to assign the sglist_append result to a name to avoid immediately dropping the result
    // and panicking because the buffers are in the list. These variables will instead be dropped
    // at the end of this function
    let _req_in_list = sglist_append(&mut list, &mut req)?;
    let _resp_hdr_in_list = sglist_append(&mut list, &mut resp_hdr)?;
    let resp_in_list = sglist_append(&mut list, &mut resp)?;
    let controlq = &sc.controlq;

    virtqueue_enqueue(controlq, &mut Some(list), 1, 2)?;
    virtqueue_notify(controlq);
    let mut list = virtqueue_poll(controlq);

    // Reset the list so we can access the buffers in the list or free them
    sglist_reset(&mut list);

    // Return the response buffer
    Ok(resp_in_list.get_buffer())
}

// An alias to avoid using tons of angle brackets everywhere. The driver needs to use the PCM sound
// interfaces so it must be a subclass of snddev_info which is ensured by PcmSoftc.
pub type VtSoundSoftc = PcmSoftc<VtSoundSoftcFields>;

// The extra fields in the subclass of snddev_info
pub struct VtSoundSoftcFields {
    dev: device_t,
    callout: Mutex<Callout>,
    eventq: Virtqueue,
    controlq: Virtqueue,
    txq: Virtqueue,
    rxq: Virtqueue,
}

// Implement the rust side of the device_if methods in the method table
impl DeviceIf for VtSoundDriver {
    // This ensures the vtsnd_driver allocates the right amount of memory for each softc
    type Softc = VtSoundSoftc;

    fn device_attach(uninit_sc: UninitDevRef<VtSoundSoftc>, dev: device_t) -> Result<()> {
        let features = VIRTIO_F_VERSION_1 as u64;
        let negotiated_features = virtio_negotiate_features(dev, features);
        virtio_finalize_features(dev)?;

        // Initialize the virtqueues using virtio_alloc_virtqueues
        let mut vq_info = VqAllocInfo::array::<NUM_VQUEUES>();
        vq_info[CONTROLQ].init(dev, c"control", 0);
        vq_info[EVENTQ].init(dev, c"event", 0);
        vq_info[TXQ].init(dev, c"tx", 0);
        vq_info[RXQ].init(dev, c"rx", 0);

        // This returns an array of `Option<Virtqueue>`s which are all `Some` to allow taking
        // them out of the array one at a time
        let mut vqs = virtio_alloc_virtqueues(dev, vq_info)?;

        let eventq = vqs[EVENTQ].take().unwrap();
        let controlq = vqs[CONTROLQ].take().unwrap();
        let txq = vqs[TXQ].take().unwrap();
        let rxq = vqs[RXQ].take().unwrap();

        // Write to the softc pointer and turn it into an Arc
        let sc = uninit_sc.init(PcmSoftc::new(VtSoundSoftcFields {
            dev,
            callout: Mutex::new(Callout::new()),
            eventq,
            controlq,
            txq,
            rxq,
        }));

        mtx_init(&sc.callout, c"vtsnd_callout", None);

        let mut callout = mtx_lock(&sc.callout);
        callout_init(dev, &mut callout)?;
        mtx_unlock(callout);

        virtio_setup_intr(dev, INTR_TYPE_MISC)?;

        // Read the virtio config space to see how many PCM streams the device supports
        let num_streams = virtio_read_device_config(dev, VtSoundConfig::streams);

        // Get info on all the PCM streams as a Vec of VtSoundPcmInfo
        let stream_info = vtsnd_query_info(&sc, 0..num_streams)?;

        // Mark the driver as multiprocessor-safe for the sound interface since we don't register an
        // interrupt handler with snd_setup_intr
        pcm_setflags(dev, pcm_getflags(dev) | SD_F_MPSAFE as u32);

        // Initialize the snddev_info base class of the softc
        pcm_init(dev, &sc)?;

        // Add a PCM channel for each stream
        for (stream_id, &info) in stream_info.iter().enumerate() {
            // Skip recording streams
            if info.direction == VtSoundDirection::Input as u8 {
                continue;
            };
            // If direction is anything other that output, print out the bad PCM stream info and
            // skip it
            let direction = if info.direction == VtSoundDirection::Output as u8 {
                PCMDIR_PLAY
            } else {
                device_println!(dev, "skipping bad pcm stream info {info:x?}");
                continue;
            };
            let stream_id = stream_id as u32;
            // Add the PCM channel with a mutable reference to channel info that pcm_addchan pipes
            // in to channel_init
            let mut chan_info = VtSoundChannelInfo { stream_id, info };
            pcm_addchan(dev, direction, &vtsnd_chan, &mut chan_info)?;
        }

        // After adding all PCM channels register the device
        pcm_register(dev, c"vtsnd").inspect_err(|e| {
            device_println!(dev, "pcm_register failed {e:?}");
        })?;

        // Initialize the mixer
        mixer_init(dev, &vtsnd_mixer, &sc).inspect_err(|e| {
            device_println!(dev, "mixer_init failed {e:?}");
        })?;

        Ok(())
    }

    fn device_detach(sc: DevRef<VtSoundSoftc>, dev: device_t) -> Result<()> {
        pcm_unregister(sc.dev).inspect_err(|e| {
            device_println!(dev, "pcm_unregister failed {e:?}");
        })
    }
}

// Defines the VtSoundDriver type for the driver and its method table
driver! {
    vtsnd_driver, c"pcm", VtSoundDriver,
    vtsnd_methods = {
        device_probe vtsnd_device_probe defined in C,
        device_attach vtsnd_device_attach,
        device_detach vtsnd_device_detach,
    }
}
