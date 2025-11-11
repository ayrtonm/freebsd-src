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
use core::ops::{Deref, DerefMut};
use kpi::bindings::device_t;
use kpi::ffi::SyncPtr;
use kpi::intr::IntrType;
use kpi::{ErrCode, Result, bindings};

pub type VirtioFeatures = u64;

#[repr(C)]
#[derive(Debug, Default)]
pub struct VqAllocInfo(bindings::vq_alloc_info);

impl Deref for VqAllocInfo {
    type Target = bindings::vq_alloc_info;
    fn deref(&self) -> &<Self as Deref>::Target {
        &self.0
    }
}

impl DerefMut for VqAllocInfo {
    fn deref_mut(&mut self) -> &mut <Self as Deref>::Target {
        &mut self.0
    }
}

unsafe impl Sync for VqAllocInfo {}

pub type VqPtr = SyncPtr<bindings::virtqueue>;

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
) -> Result<()> {
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
    Ok(())
}

pub fn virtio_negotiate_features(dev: device_t, features: VirtioFeatures) -> VirtioFeatures {
    unsafe {
        bindings::virtio_negotiate_features(dev, features)
    }
}

pub fn virtio_finalize_features(dev: device_t) -> Result<()> {
    let res = unsafe {
        bindings::virtio_finalize_features(dev)
    };
    if res != 0 {
        return Err(ErrCode::from(res));
    }
    Ok(())
}
