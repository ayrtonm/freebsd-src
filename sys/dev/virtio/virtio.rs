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
use kpi::collections::{Pod, SgList};
use kpi::device::DeviceIf;
use kpi::driver::Driver;
use kpi::ffi::{CString, DevRef};
use kpi::intr::IntrType;
use kpi::prelude::*;
use kpi::{ErrCode, Result, bindings};

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

type VqFunc<D> = extern "C" fn(DevRef<<D as DeviceIf>::Softc>);

#[derive(Debug)]
pub struct VqAllocInfo {
    name: Option<CString>,
    max_indirect_size: usize,
    callback: Option<(virtqueue_intr_t, *mut c_void)>,
}

impl VqAllocInfo {
    pub const fn new() -> Self {
        Self {
            name: None,
            max_indirect_size: 0,
            callback: None,
        }
    }

    pub const fn array<const N: usize>() -> [Self; N] {
        [const { Self::new() }; N]
    }

    pub fn init(&mut self, dev: device_t, vq_name: &CStr, max_indirect_size: usize) {
        let mut name = device_get_nameunit(dev).unwrap();
        name.push_cstr(vq_name);
        self.name = Some(name);
        self.max_indirect_size = max_indirect_size;
    }

    pub fn init_with_callback<D: DeviceIf>(
        &mut self,
        dev: device_t,
        vq_name: &CStr,
        max_indirect_size: usize,
        func: VqFunc<D>,
    ) -> Result<()> {
        if <D as Driver>::DRIVER != device_get_driver(dev) {
            return Err(EINVAL);
        }
        self.init(dev, vq_name, max_indirect_size);
        let func = unsafe { transmute::<Option<VqFunc<D>>, virtqueue_intr_t>(Some(func)) };
        let arg = unsafe { bindings::device_get_softc(dev) };
        self.callback = Some((func, arg));
        Ok(())
    }
}

pub struct Virtqueue {
    ptr: *mut virtqueue,
    // TODO: Expose a function from C to get this from ptr since it's already a field there
    dev: device_t,
}

unsafe impl Sync for Virtqueue {}
unsafe impl Send for Virtqueue {}

pub mod prelude {
    use super::*;

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

    /// Read a field from the config space created by invoking the define_config_space! macro
    ///
    /// This ensures no more than 32-bits are read at a time since the driver is not allowed to
    /// assume reads larger than that are atomic.
    pub fn virtio_read_device_config<T: Pod>(dev: device_t, field: ConfigSpaceField<T>) -> T {
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
                    dev,
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
    pub fn virtio_alloc_virtqueues<const N: usize>(
        dev: device_t,
        mut vq_info: [VqAllocInfo; N],
    ) -> Result<[Option<Virtqueue>; N]> {
        let mut c_vq_info: [vq_alloc_info; N] = array::from_fn(|_| vq_alloc_info::default());
        let mut vq_out_ptrs: [Virtqueue; N] = array::from_fn(|_| Virtqueue {
            ptr: null_mut(),
            dev,
        });

        for (n, info) in c_vq_info.iter_mut().enumerate() {
            let name_ref = vq_info[n]
                .name
                .as_ref()
                .expect("Failed to init VqAllocInfo entry")
                .as_c_str()
                .to_bytes();
            let out_name_slice = unsafe {
                slice::from_raw_parts_mut(info.vqai_name.as_mut_ptr().cast(), info.vqai_name.len())
            };
            out_name_slice[0..name_ref.len()].copy_from_slice(name_ref);

            info.vqai_maxindirsz = vq_info[n].max_indirect_size.try_into().unwrap();
            if let Some((func, arg)) = vq_info[n].callback.take() {
                info.vqai_intr = func;
                info.vqai_intr_arg = arg;
            }
            info.vqai_vq = &raw mut vq_out_ptrs[n].ptr;
        }
        let res = unsafe {
            bindings::virtio_alloc_virtqueues(
                dev,
                c_vq_info.len().try_into().unwrap(),
                c_vq_info.as_mut_ptr(),
            )
        };
        if res != 0 {
            return Err(ErrCode::from(res));
        }
        Ok(vq_out_ptrs.map(|vq| Some(vq)))
    }

    pub fn virtio_setup_intr(dev: device_t, flags: IntrType) -> Result<()> {
        let res = unsafe { bindings::virtio_setup_intr(dev, flags.0) };
        if res != 0 {
            return Err(ErrCode::from(res));
        }
        Ok(())
    }

    pub fn virtqueue_notify(vq: &Virtqueue) {
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
        vq: &Virtqueue,
        sg_opt: &mut Option<SgList>,
        num_readable: u16,
        num_writable: u16,
    ) -> Result<()> {
        let sg = match sg_opt.take() {
            Some(sg) => sg,
            None => {
                device_println!(vq.dev, "Must pass `&mut Some(sglist)` to virtqueue_enqueue");
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
                vq.dev,
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
    pub fn virtqueue_poll(vq: &Virtqueue) -> SgList {
        let mut len = 0;
        let cookie = unsafe { bindings::virtqueue_poll(vq.ptr, &raw mut len) };
        unsafe { SgList::from_raw(cookie.cast::<sglist>()) }
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
