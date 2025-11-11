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

use core::cmp::min;
use core::ffi::{CStr, c_int, c_void};
use core::marker::PhantomData;
use core::mem::{MaybeUninit, transmute};
use core::ptr::null_mut;
use core::{array, slice};
use kpi::bindings::{device_t, sglist, virtqueue, virtqueue_intr_t, vq_alloc_info};
use core::ops::Deref;
use kpi::collections::{Pod, SgList, LocalSgList};
use kpi::device::{Device, DeviceIf};
use kpi::ffi::{ArrayCString, Softc};
use kpi::intr::IntrType;
use kpi::prelude::*;
use kpi::{ErrCode, Result, bindings};

#[derive(Debug, Copy, Clone)]
pub struct VirtioDevice<'a>(&'a Device);

impl<'a> VirtioDevice<'a> {
    pub fn new(dev: &'a Device) -> Result<VirtioDevice<'a>> {
        let parent = device_get_parent(dev)?;
        let transport = device_get_name(parent)?;
        let name = transport.as_c_str();
        if name != c"virtio_pci" && name != c"virtio_mmio" {
            return Err(EINVAL);
        }
        Ok(Self(dev))
    }
}

impl Deref for VirtioDevice<'_> {
    type Target = Device;

    fn deref(&self) -> &Device {
        &self.0
    }
}

/// The info on a field in a struct defining a virtio device's config space.
///
/// Do not construct this using `new`. Instead invoke define_config_space! with the config space
/// struct definition. This creates associated consts with the same name as each field on the
/// struct. These associated consts can be used to read the config space as follows
/// `virtio_read_device_config(vdev, TheConfigSpaceType::the_field_name)`
pub struct ConfigSpaceField<T: Pod>(usize, PhantomData<T>);

impl<T: Pod> ConfigSpaceField<T> {
    /// # Safety
    ///
    /// This method should not be called directly.
    pub const unsafe fn new(offset: usize) -> Self {
        Self(offset, PhantomData)
    }
}

/// Defines ConfigSpaceField values for each field in the config space
///
/// This creates a #[repr(C)] struct for the config space and defines ConfigSpaceField values for
/// each field to allow calling virtio_read_device_config. This macro's invocation is just passed
/// the struct definition as follows
///
/// define_config_space! {
///     struct MyConfigSpace {
///         some_field: u64,
///         another_field: u32,
///         last_field: u32,
///     }
/// }
#[macro_export]
macro_rules! define_config_space {
    (struct $name:ident {
        $($field:ident: $field_ty:ty,)*
    }) => {
        #[repr(C)]
        #[derive(Debug)]
        pub struct $name {
            $($field: $field_ty,)*
        }
        impl $name {
            $(pub const $field: virtio::ConfigSpaceField<$field_ty> = {
                use core::mem::offset_of;
                use virtio::ConfigSpaceField;
                unsafe { ConfigSpaceField::new(offset_of!($name, $field)) }
            };)*
        }
    };
}

pub type VirtioFeatures = u64;

type VqFunc<D> = extern "C" fn(&Softc<<D as DeviceIf>::Softc>);

#[derive(Debug)]
pub struct VqSet<'a, const NUM_QUEUES: usize> {
    vdev: VirtioDevice<'a>,
    // C needs this layout so this can't be merged with the vqs array
    info: [vq_alloc_info; NUM_QUEUES],
    // output virtqueue objects
    vqs: [Option<Virtqueue>; NUM_QUEUES],
}

impl<'a, const NUM_QUEUES: usize> VqSet<'a, NUM_QUEUES> {
    pub fn new(vdev: VirtioDevice<'a>) -> Self {
        Self {
            vdev,
            info: array::from_fn(|_| {
                vq_alloc_info::default()
            }),
            vqs: array::from_fn(|_| None),
        }
    }

    pub fn init(&mut self, idx: usize, vq_name: &CStr, max_indirect: usize) -> Result<()> {
        // Bubble up error on idx OOB
        let info = self.info.get_mut(idx).ok_or(EINVAL)?;

        // Already initialized the virtqueue at this index
        if info.vqai_name[0] != 0 {
            return Err(EEXIST);
        }

        let nameunit = device_get_nameunit(&self.vdev)?;
        let name_parts = [nameunit.to_bytes(), b" ", vq_name.to_bytes()];
        let len: usize = name_parts.iter().map(|p| p.len()).sum();
        if len >= info.vqai_name.len() {
            return Err(EINVAL);
        }
        let name_parts_iter = name_parts.iter().flat_map(|p| p.iter());
        for (dst, &src) in info.vqai_name.iter_mut().zip(name_parts_iter) {
            *dst = src;
        }
        // trailing bytes in info.vqai_name are 0 from vq_alloc_info::default() in constructor
        info.vqai_maxindirsz = max_indirect.try_into().map_err(|_| EINVAL)?;

        Ok(())
    }

    pub fn get(&mut self, idx: usize) -> Result<Virtqueue> {
        self.vqs.get_mut(idx).ok_or(EINVAL)?.take().ok_or(EEXIST)
    }
}

#[derive(Debug)]
pub struct Virtqueue {
    ptr: *mut virtqueue,
    dev: device_t,
}

unsafe impl Sync for Virtqueue {}
unsafe impl Send for Virtqueue {}

impl Virtqueue {
    fn device(&self) -> &Device {
        // SAFETY: The virtqueue is owned by the device's softc so the device outlives it
        unsafe { self.dev.cast::<Device>().as_ref().unwrap() }
    }
}

pub mod prelude {
    use super::*;

    pub fn virtio_get_device_type(vdev: VirtioDevice) -> i32 {
        unsafe { bindings::virtio_get_device_type(vdev.0.as_ptr()) }
    }

    pub fn virtio_negotiate_features(vdev: VirtioDevice, features: VirtioFeatures) -> VirtioFeatures {
        unsafe { bindings::virtio_negotiate_features(vdev.0.as_ptr(), features) }
    }

    pub fn virtio_finalize_features(vdev: VirtioDevice) -> Result<()> {
        let dev_ptr = vdev.0.as_ptr();
        let res = unsafe { bindings::virtio_finalize_features(dev_ptr) };
        if res != 0 {
            return Err(ErrCode::from(res));
        }
        Ok(())
    }

    /// Read a field from the config space created by invoking the define_config_space! macro
    ///
    /// This ensures no more than 32-bits are read at a time since the driver is not allowed to
    /// assume reads larger than that are atomic.
    pub fn virtio_read_device_config<T: Pod>(vdev: VirtioDevice, field: ConfigSpaceField<T>) -> T {
        let dev_ptr = vdev.0.as_ptr();
        let mut res: MaybeUninit<T> = MaybeUninit::uninit();

        let start_ptr = res.as_mut_ptr().cast::<c_void>();
        let total_len = size_of::<T>();
        let field_offset = field.0;

        let mut read = 0;

        while read < total_len {
            let ptr = unsafe { start_ptr.byte_add(read) };
            let remaining_len = total_len - read;
            let len = min(4, remaining_len);
            let offset = field_offset + read;
            unsafe {
                bindings::virtio_read_device_config(
                    dev_ptr,
                    offset.try_into().unwrap(),
                    ptr,
                    len.try_into().unwrap(),
                )
            };
            read += len;
        }

        unsafe { res.assume_init() }
    }

    // TODO: Support initializing variable number of virtqueues
    pub fn virtio_alloc_virtqueues<const N: usize>(vq_set: &mut VqSet<N>) -> Result<()> {
        let mut vq_out_ptrs: [*mut virtqueue; N] = [null_mut(); N];
        for (info, out_ptr) in vq_set.info.iter_mut().zip(vq_out_ptrs.iter_mut()) {
            info.vqai_vq = out_ptr;
        }

        let dev = vq_set.vdev.0.as_ptr();
        let res = unsafe {
            bindings::virtio_alloc_virtqueues(
            dev,
            N.try_into().unwrap(),
            vq_set.info.as_mut_ptr(),
            )
        };
        if res != 0 {
            return Err(ErrCode::from(res));
        }
        for (slot, ptr) in vq_set.vqs.iter_mut().zip(vq_out_ptrs) {
            *slot = Some(Virtqueue { ptr, dev });
        }
        Ok(())
    }

    pub fn virtio_setup_intr(vdev: VirtioDevice, flags: IntrType) -> Result<()> {
        let res = unsafe { bindings::virtio_setup_intr(vdev.0.as_ptr(), flags.0 as u32) };
        if res != 0 {
            return Err(ErrCode::from(res));
        }
        Ok(())
    }

    pub fn virtqueue_notify(vq: &mut Virtqueue) {
        unsafe { bindings::virtqueue_notify(vq.ptr) }
    }

    pub fn virtqueue_full(vq: &Virtqueue) -> bool {
        unsafe { bindings::virtqueue_full(vq.ptr) }
    }

    pub fn virtqueue_empty(vq: &Virtqueue) -> bool {
        unsafe { bindings::virtqueue_empty(vq.ptr) }
    }

    pub fn virtqueue_nused(vq: &Virtqueue) -> u16 {
        unsafe { bindings::virtqueue_nused(vq.ptr) as u16 }
    }

    pub fn virtqueue_nfree(vq: &Virtqueue) -> u16 {
        unsafe { bindings::virtqueue_nfree(vq.ptr) as u16 }
    }

    /// Enqueues an SgList as a chain of descriptors
    ///
    /// The user must make sure there are enough avail descriptors in the queue
    pub fn virtqueue_enqueue(
        vq: &mut Virtqueue,
        sg_opt: &mut Option<SgList>,
        num_readable: u16,
        num_writable: u16,
    ) -> Result<()> {
        let sg = match sg_opt.take() {
            Some(sg) => sg,
            None => {
                device_println!(vq.device(), "Must pass `&mut Some(sglist)` to virtqueue_enqueue");
                return Err(EDOOFUS);
            }
        };
        let sglist_ptr = SgList::into_raw(sg);
        let cookie = sglist_ptr.cast::<c_void>();
        let res = unsafe {
            bindings::virtqueue_enqueue(
                vq.ptr,
                cookie,
                sglist_ptr,
                num_readable as c_int,
                num_writable as c_int,
            )
        };
        if res != 0 {
            let rc = ErrCode::from(res);
            device_println!(
                vq.device(),
                "Returning sglist back to caller since virtqueue_enqueue failed with {rc:?}"
            );
            unsafe {
                sg_opt.replace(SgList::from_raw(sglist_ptr));
            }
            return Err(rc);
        }
        Ok(())
    }

    /// Poll the queue until the next used element is available
    ///
    /// The enqueued sglist was added as a descriptor chain so we only need to call poll once, but the
    /// user must ensure that the enqueued list arg matches the descriptor cookie
    pub fn virtqueue_poll(vq: &mut Virtqueue) -> SgList {
        let mut len = 0;
        let cookie = unsafe { bindings::virtqueue_poll(vq.ptr, &raw mut len) };
        unsafe { SgList::from_raw(cookie.cast::<sglist>()) }
    }

    pub fn virtqueue_enqueue_and_wait<const N: usize>(vq: &mut Virtqueue, sg: &mut LocalSgList<N>) -> Result<u32> {
        if !virtqueue_empty(vq) {
            return Err(EBUSY);
        }
        let sglist_ptr = sg.as_ptr();
        let cookie = sglist_ptr.cast::<c_void>();
        let res = unsafe {
            bindings::virtqueue_enqueue(
                vq.ptr,
                cookie,
                sglist_ptr,
                i32::from(sg.nseg_readable()),
                i32::from(sg.nseg_writeable())
            )
        };
        if res != 0 {
            return Err(ErrCode::from(res));
        }
        virtqueue_notify(vq);
        let mut len = 0;
        let same_cookie = unsafe { bindings::virtqueue_poll(vq.ptr, &mut len) };
        // The C code should prevent this from happening
        if cookie != same_cookie {
            return Err(EDOOFUS);
        }
        Ok(len)
    }

    pub fn virtqueue_disable_intr(vq: &Virtqueue) {
        unsafe { bindings::virtqueue_disable_intr(vq.ptr) }
    }

    pub fn virtqueue_enable_intr(vq: &Virtqueue) -> c_int {
        // This result isn't an ErrCode
        let res = unsafe { bindings::virtqueue_enable_intr(vq.ptr) };
        res
    }
}
