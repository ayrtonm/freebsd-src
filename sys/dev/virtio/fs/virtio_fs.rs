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

/* Driver for VirtIO file system devices. */

#![no_std]

use kpi::bindings::{VIRTIO_F_VERSION_1, device_t};
use kpi::device::DeviceIf;
use kpi::driver;
use kpi::ffi::{DevRef, UninitDevRef};
use kpi::prelude::*;
use virtio::prelude::*;
use virtio::{Virtqueue, VqAllocInfo, define_config_space};

pub struct VtFsSoftc {
    dev: device_t,
    vqs: VtFsVirtqueues,
}

struct VtFsVirtqueues {
    hiprio: Virtqueue,
    notif: Virtqueue,
    req: Virtqueue,
}

define_config_space! {
    struct VtFsConfig {
        tag: [u8; 36],
        num_req_queues: u32,
        notify_buf_size: u32,
    }
}

extern "C" fn vtfs_notif_callback(sc: DevRef<VtFsSoftc>) {}

impl DeviceIf for VtFsDriver {
    type Softc = VtFsSoftc;

    fn device_attach(uninit_sc: UninitDevRef<VtFsSoftc>, dev: device_t) -> Result<()> {
        let features = VIRTIO_F_VERSION_1 as u64;
        let negotiated_features = virtio_negotiate_features(dev, features);
        virtio_finalize_features(dev)?;

        let tag = virtio_read_device_config(dev, VtFsConfig::tag);
        let tag_str = str::from_utf8(&tag).map_err(|_| EINVAL)?;
        device_println!(
            dev,
            "attaching virtio-fs device with tag {tag_str:?} {tag:?}"
        );

        let num_req_queues: u32 = virtio_read_device_config(dev, VtFsConfig::num_req_queues);
        if num_req_queues < 1 {
            device_println!(dev, "found less than one request queue in config space");
            return Err(ENXIO);
        }

        let mut vq_info = VqAllocInfo::array::<3>();

        const HIPRIOQ: usize = 0;
        const NOTIFQ: usize = 1;
        const REQQ: usize = 2;

        vq_info[HIPRIOQ].init(dev, c"hiprio", 0);
        vq_info[NOTIFQ].init_with_callback::<Self>(dev, c"notif", 0, vtfs_notif_callback)?;
        // FIXME: when QEMU says one vq it actually means just the hiprio queue so the alloc vqs fails
        vq_info[REQQ].init(dev, c"req", 0);
        let mut vqs = virtio_alloc_virtqueues(dev, vq_info)?;
        let hiprio = vqs[HIPRIOQ].take().unwrap();
        let notif = vqs[NOTIFQ].take().unwrap();
        let req = vqs[REQQ].take().unwrap();

        let vqs = VtFsVirtqueues { hiprio, notif, req };

        let sc = uninit_sc.init(VtFsSoftc { dev, vqs });

        Ok(())
    }

    fn device_detach(sc: DevRef<VtFsSoftc>, dev: device_t) -> Result<()> {
        Ok(())
    }
}

driver! {
    vtfs_driver, c"virtio_fs", VtFsDriver,
    vtfs_methods = {
        device_probe vtfs_device_probe defined in C,
        device_attach vtfs_device_attach,
        device_detach vtfs_device_detach,
    }
}
