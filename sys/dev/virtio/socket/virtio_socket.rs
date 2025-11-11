/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Ayrton Muñoz
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

/* Driver for VirtIO socket devices. */

#![no_std]

use core::pin::Pin;
use kpi::bindings::VIRTIO_F_VERSION_1;
use kpi::device::{Device, DeviceIf};
use kpi::define_driver;
use kpi::ffi::{Uninit, Loan};
use kpi::prelude::*;
use virtio::{Virtqueue, VqAllocInfo, define_config_space};
use virtio::prelude::*;

const NUM_VQUEUES: usize = 3;
const RXQ: usize = 0;
const TXQ: usize = 1;
const EVENTQ: usize = 2;

extern "C" fn vtsocket_rx_cb(sc: &VtSocketSoftc) {
}

extern "C" fn vtsocket_tx_cb(sc: &VtSocketSoftc) {
}

extern "C" fn vtsocket_event_cb(sc: &VtSocketSoftc) {
}

pub struct VtSocketSoftc {
    rxq: Virtqueue,
    txq: Virtqueue,
    eventq: Virtqueue,
    // upper 32 bits are always zero
    cid: u64,
}

impl DeviceIf for VtSocketDriver {
    type Softc = VtSocketSoftc;

    fn device_attach(uninit_sc: Uninit<VtSocketSoftc>) -> Result<()> {
        let dev = uninit_sc.device();
        let features = VIRTIO_F_VERSION_1 as u64;
        let negotiated_features = virtio_negotiate_features(dev, features);
        virtio_finalize_features(dev)?;

        // Initialize the virtqueues using virtio_alloc_virtqueues
        let mut vq_info = VqAllocInfo::array::<NUM_VQUEUES>();
        vq_info[RXQ].init_with_callback::<Self>(dev, c"rx", 0, vtsocket_rx_cb);
        vq_info[TXQ].init_with_callback::<Self>(dev, c"tx", 0, vtsocket_tx_cb);
        vq_info[EVENTQ].init_with_callback::<Self>(dev, c"event", 0, vtsocket_event_cb);

        let mut vqs = virtio_alloc_virtqueues(dev, vq_info)?;
        let rxq = vqs[RXQ].take().unwrap();
        let txq = vqs[TXQ].take().unwrap();
        let eventq = vqs[EVENTQ].take().unwrap();

        virtio_setup_intr(dev, INTR_TYPE_MISC)?;

        let cid = virtio_read_device_config(dev, VtSocketConfig::guest_cid);

        uninit_sc.init(VtSocketSoftc {
            rxq,
            txq,
            eventq,
            cid,
        });

        // TODO: populate rx and event vqs

        Ok(())
    }
    fn device_detach(sc: Loan<VtSocketSoftc>) -> Result<()> {
        Ok(())
    }
}

define_driver! {
    static vtsocket_driver: VtSocketDriver = {
        name: c"vtsocket"
    };
    static vtsocket_methods = {
        device_probe: vtsocket_device_probe defined in C,
        device_attach: vtsocket_device_attach,
        device_detach: vtsocket_device_detach,
    };
}

define_config_space! {
    struct VtSocketConfig {
        guest_cid: u64,
    }
}
