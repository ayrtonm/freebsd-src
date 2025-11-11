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

#![no_std]

use core::ffi::{c_int, c_void};
use kpi::bindings::{device_t, hid_intr_t, hid_rdesc_info, hid_size_t};
use kpi::device::DeviceIf;
use kpi::ffi::{Ref, SubClass, SubClassOf};
use kpi::prelude::*;
use kpi::{ErrCode, base, define_interface};

#[allow(unused_variables)]
pub trait HidIf: DeviceIf {
    fn hid_intr_setup(
        sc: Ref<Self::Softc>,
        child: device_t,
        intr: hid_intr_t,
        context: *mut c_void,
        rdesc: *mut hid_rdesc_info,
    ) {
        unimplemented!()
    }
    fn hid_intr_unsetup(sc: Ref<Self::Softc>, child: device_t) {
        unimplemented!()
    }
    fn hid_intr_start(sc: Ref<Self::Softc>, child: device_t) -> Result<()> {
        unimplemented!()
    }
    fn hid_intr_stop(sc: Ref<Self::Softc>, child: device_t) -> Result<()> {
        unimplemented!()
    }
    fn hid_intr_poll(sc: Ref<Self::Softc>, child: device_t) {
        unimplemented!()
    }
    fn hid_get_rdesc(sc: Ref<Self::Softc>, child: device_t, buf: &mut [u8]) -> Result<()> {
        unimplemented!()
    }
    fn hid_get_report(
        sc: Ref<Self::Softc>,
        child: device_t,
        data: *mut c_void,
        max_len: hid_size_t,
        actlen: *mut hid_size_t,
        ty: u8,
        id: u8,
    ) -> Result<()> {
        unimplemented!()
    }
    fn hid_set_report(
        sc: Ref<Self::Softc>,
        child: device_t,
        buf: *const c_void,
        len: hid_size_t,
        ty: u8,
        id: u8,
    ) -> Result<()> {
        unimplemented!()
    }
}
define_interface! {
    in HidIf
    fn hid_intr_setup(dev: device_t, child: device_t, intr: hid_intr_t, context: *mut c_void, rdesc: *mut hid_rdesc_info),
        with desc hid_intr_setup_desc
        and typedef hid_intr_setup_t;
    fn hid_intr_unsetup(dev: device_t, child: device_t),
        with desc hid_intr_unsetup_desc
        and typedef hid_intr_unsetup_t;
    fn hid_intr_start(dev: device_t, child: device_t) -> c_int,
        with desc hid_intr_start_desc
        and typedef hid_intr_start_t;
    fn hid_intr_stop(dev: device_t, child: device_t) -> c_int,
        with desc hid_intr_stop_desc
        and typedef hid_intr_stop_t;
    fn hid_intr_poll(dev: device_t, child: device_t),
        with desc hid_intr_poll_desc
        and typedef hid_intr_poll_t;
    fn hid_get_report(dev: device_t, child: device_t, data: *mut c_void, max_len: hid_size_t, actlen: *mut hid_size_t, ty: u8, id: u8) -> c_int,
        with desc hid_get_report_desc
        and typedef hid_get_report_t;
    fn hid_set_report(dev: device_t, child: device_t, buf: *const c_void, len: hid_size_t, ty: u8, id: u8) -> c_int,
        with desc hid_set_report_desc
        and typedef hid_set_report_t;
}

#[macro_export]
macro_rules! hid_get_rdesc {
    (get_desc) => {
        kpi::bindings::hid_get_rdesc_desc
    };
    ($driver_ty:ident $driver_sym:ident $impl_fn_name:ident) => {
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $impl_fn_name(
            dev: device_t,
            child: device_t,
            buf: *mut c_void,
            len: hid_size_t,
        ) -> c_int {
            // TODO: Typecheck this macro invocation against nvme_delayed_attach_t
            //use core::any::{Any, TypeId};
            //let typedef_val = nvme_delayed_attach_t::default();
            //let typedef_id = typedef_val.type_id();
            //let this_fn_id = Some($impl_fn_name).type_id();
            //assert!(typedef_id == this_fn_id);

            let sc = dev.as_rust_type();
            let mut buf =
                unsafe { core::slice::from_raw_parts_mut(buf.cast::<u8>(), len as usize) };
            let res = <$driver_ty as $crate::HidIf>::hid_get_rdesc(sc, child, buf);
            match res {
                Ok(()) => 0,
                Err(e) => e.as_c_type(),
            }
        }
    };
}
