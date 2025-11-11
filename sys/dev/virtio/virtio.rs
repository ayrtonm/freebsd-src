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

/* Bindings for VirtIO. */

#![no_std]
use core::ffi::c_void;
use core::mem::transmute;
use core::ptr::null_mut;
use core::slice;
use kpi::bindings::{device_t, sglist, virtqueue, virtqueue_intr_t, vq_alloc_info};
use kpi::collections::ScatterList;
use kpi::ffi::{Ptr, RefCounted, SharedPtr, SyncPtr};
use kpi::intr::IntrType;
use kpi::prelude::*;
use kpi::{ErrCode, Result, bindings};

pub type VirtioFeatures = u64;

type VqFunc<T> = extern "C" fn(&RefCounted<T>);
pub type VqCallback<T> = (VqFunc<T>, Ptr<T>);

#[repr(C)]
#[derive(Debug, Default)]
pub struct VqAllocInfo {
    info: vq_alloc_info,
}

impl VqAllocInfo {
    pub const fn new() -> Self {
        Self {
            info: vq_alloc_info {
                vqai_name: [0; 32],
                vqai_maxindirsz: 0,
                vqai_intr: None,
                vqai_intr_arg: null_mut(),
                vqai_vq: null_mut(),
            },
        }
    }

    pub fn init(&mut self, dev: device_t, vq_name: &[u8], max_indirect_size: usize) {
        let devname = device_get_nameunit(dev);
        let prefix_len = devname.len();
        let prefix_dst = unsafe {
            slice::from_raw_parts_mut(self.info.vqai_name.as_mut_ptr().cast::<u8>(), prefix_len)
        };
        prefix_dst.copy_from_slice(devname.as_bytes());
        let name_dst = unsafe {
            slice::from_raw_parts_mut(
                self.info
                    .vqai_name
                    .as_mut_ptr()
                    .cast::<u8>()
                    .byte_add(prefix_len),
                vq_name.len(),
            )
        };
        name_dst.copy_from_slice(vq_name);
        self.info.vqai_maxindirsz = max_indirect_size.try_into().unwrap();
    }

    pub fn init_with_callback<T, P: SharedPtr<T>>(
        &mut self,
        dev: device_t,
        vq_name: &[u8],
        max_indirect_size: usize,
        func: VqFunc<T>,
        arg: P,
    ) {
        self.init(dev, vq_name, max_indirect_size);
        self.info.vqai_intr =
            unsafe { transmute::<Option<VqFunc<T>>, virtqueue_intr_t>(Some(func)) };
        let (arg_ptr, _metadata_ptr) = SharedPtr::into_raw_parts(arg);
        self.info.vqai_intr_arg = arg_ptr.cast::<c_void>();
    }
}

unsafe impl Sync for VqAllocInfo {}

pub type VqPtr = SyncPtr<virtqueue>;

/// Implement on a type to use read_device_config!
///
/// # Safety
///
/// The implementor must ensure that it is a repr(C) struct and matches the config space exposed by
/// the virtio device.
pub unsafe trait ConfigSpace: Sized {
    fn read_device_config<T: Default>(dev: device_t, offset: usize, field: &mut T) {
        assert!(offset <= size_of::<Self>() - size_of::<T>());
        unsafe {
            bindings::virtio_read_device_config(
                dev,
                offset.try_into().unwrap(),
                (field as *mut T).cast::<c_void>(),
                size_of::<T>().try_into().unwrap(),
            )
        };
    }
}
/// Read the device config space
#[macro_export]
macro_rules! read_device_config {
    ($dev:expr, $config_ty:ty, $field:ident) => {{
        let mut res = Default::default();
        <$config_ty as virtio::ConfigSpace>::read_device_config(
            $dev,
            core::mem::offset_of!($config_ty, $field),
            &mut res,
        );
        res
    }};
}

pub fn virtio_setup_intr(dev: device_t, flags: IntrType) -> Result<()> {
    let res = unsafe { bindings::virtio_setup_intr(dev, flags.0) };
    if res != 0 {
        return Err(ErrCode::from(res));
    }
    Ok(())
}
pub fn virtqueue_enable_intr(vq: VqPtr) -> Result<()> {
    let res = unsafe { bindings::virtqueue_enable_intr(vq.as_ptr()) };
    if res != 0 {
        return Err(ErrCode::from(res));
    }
    Ok(())
}

pub fn virtio_alloc_virtqueues<const N: usize>(
    dev: device_t,
    vq_info: &mut [VqAllocInfo; N],
) -> Result<[VqPtr; N]> {
    let mut vq_out_ptrs = [null_mut(); N];
    for n in 0..N {
        vq_info[n].info.vqai_vq = &raw mut vq_out_ptrs[n];
    }
    let res = unsafe {
        bindings::virtio_alloc_virtqueues(
            dev,
            vq_info.len().try_into().unwrap(),
            vq_info.as_mut_ptr().cast::<bindings::vq_alloc_info>(),
        )
    };
    if res != 0 {
        return Err(ErrCode::from(res));
    }
    Ok(vq_out_ptrs.map(|v| SyncPtr::new(v)))
}

pub fn virtio_negotiate_features(dev: device_t, features: VirtioFeatures) -> VirtioFeatures {
    unsafe { bindings::virtio_negotiate_features(dev, features) }
}

pub fn virtio_finalize_features(dev: device_t) -> Result<()> {
    let res = unsafe { bindings::virtio_finalize_features(dev) };
    if res != 0 {
        return Err(ErrCode::from(res));
    }
    Ok(())
}

pub fn virtqueue_full(vq: VqPtr) -> bool {
    unsafe { bindings::virtqueue_full(vq.as_ptr()) }
}

pub fn virtqueue_enqueue(
    vq: VqPtr,
    sg: ScatterList,
    num_readable: usize,
    num_writable: usize,
) -> Result<()> {
    let cookie = sg.as_ptr().cast::<c_void>();
    let res = unsafe {
        bindings::virtqueue_enqueue(
            vq.as_ptr(),
            cookie,
            sg.as_ptr(),
            num_readable.try_into().unwrap(),
            num_writable.try_into().unwrap(),
        )
    };
    if res != 0 {
        return Err(ErrCode::from(res));
    }
    Ok(())
}

pub fn virtqueue_poll(vq: VqPtr) -> ScatterList {
    let mut len = 0;
    let cookie = unsafe { bindings::virtqueue_poll(vq.as_ptr(), &raw mut len) };
    let ptr = cookie.cast::<sglist>();
    unsafe { ScatterList::from_raw(ptr) }
}
