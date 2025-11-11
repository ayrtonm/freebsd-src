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

#![no_std]

use core::ffi::{c_void, c_int};
use kpi::bindings::{device_t, nvme_controller, nvme_qpair, nvme_tracker};
use kpi::device::DeviceIf;
use kpi::ffi::{Ref, SubClass, SubClassOf};
use kpi::driver::Driver;
use kpi::prelude::*;
use kpi::{ErrCode, base, define_interface};

pub type NvmeSoftc<T> = SubClass<nvme_controller, T>;

pub mod prelude {
    use super::*;

    pub fn nvme_setup_intr<T>(dev: device_t, sc: &NvmeSoftc<T>) -> Result<()> {
        let res = unsafe {
            bindings::bus_setup_intr(
                dev,
                base!(sc->res),
                INTR_TYPE_MISC.0 | INTR_MPSAFE.0,
                None,
                Some(bindings::nvme_ctrlr_shared_handler),
                base!(&sc).cast::<c_void>(),
                base!(&sc->tag),
            )
        };
        if res != 0 {
            return Err(ErrCode::from(res));
        }
        Ok(())
    }

    pub fn nvme_qpair_construct<T>(
        dev: device_t,
        qpair: *mut nvme_qpair,
        num_entries: u32,
        num_trackers: u32,
        sc: Ref<NvmeSoftc<T>>,
    ) -> Result<()> {
        let res = unsafe {
            bindings::nvme_qpair_construct(dev, qpair, num_entries, num_trackers, base!(&sc))
        };
        if res != 0 {
            return Err(ErrCode::from(res));
        }
        Ok(())
    }
}

#[allow(unused_variables)]
pub trait NvmeIf: DeviceIf
where
    <Self as DeviceIf>::Softc: SubClassOf<nvme_controller>,
{
    fn nvme_delayed_attach(sc: Ref<Self::Softc>) {
        unimplemented!()
    }
    fn nvme_enable(sc: Ref<Self::Softc>) {
        unimplemented!()
    }
    fn nvme_sq_enter(sc: Ref<Self::Softc>, qpair: *mut nvme_qpair, tr: &nvme_tracker) -> u32 {
        unimplemented!()
    }
    fn nvme_sq_leave(sc: Ref<Self::Softc>, qpair: &nvme_qpair, tr: &nvme_tracker) {
        unimplemented!()
    }
    fn nvme_cq_done(sc: Ref<Self::Softc>, qpair: &nvme_qpair, tr: &nvme_tracker) {
        unimplemented!()
    }
    fn nvme_qpair_construct(
        sc: Ref<Self::Softc>,
        qpair: *mut nvme_qpair,
        num_entries: u32,
        num_trackers: u32,
        ctrlr: *mut nvme_controller,
    ) -> Result<()> {
        unimplemented!()
    }
    fn nvme_attach(dev: device_t) -> Result<()> {
        assert_eq!(device_get_driver(dev), <Self as Driver>::DRIVER);
        let res = unsafe { bindings::nvme_attach(dev) };
        if res != 0 {
            return Err(ErrCode::from(res));
        }
        Ok(())
    }

}

define_interface! {
    in NvmeIf
    fn nvme_enable(dev: device_t),
        with desc nvme_enable_desc
        and typedef nvme_enable_t;
    fn nvme_sq_enter(dev: device_t, qpair: *mut nvme_qpair, tr: *mut nvme_tracker),
        with desc nvme_sq_enter_desc
        and typedef nvme_sq_enter_t;
    fn nvme_sq_leave(dev: device_t, qpair: *mut nvme_qpair, tr: *mut nvme_tracker),
        with desc nvme_sq_leave_desc
        and typedef nvme_sq_leave_t;
    fn nvme_cq_done(dev: device_t, qpair: *mut nvme_qpair, tr: *mut nvme_tracker),
        with desc nvme_cq_done_desc
        and typedef nvme_cq_done_t;
    fn nvme_qpair_construct(dev: device_t, qpair: *mut nvme_qpair, num_entries: u32, num_trackers: u32, ctrlr: *mut nvme_controller) -> int,
        with desc nvme_qpair_construct_desc
        and typedef nvme_qpair_construct_t;
}

#[macro_export]
macro_rules! nvme_delayed_attach {
    (get_desc) => { kpi::bindings::nvme_delayed_attach_desc };
    ($driver_ty:ident $driver_sym:ident $impl_fn_name:ident) => {
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $impl_fn_name(dev: device_t, ctrlr: *mut nvme_controller) -> c_int {
            // TODO: Typecheck this macro invocation against nvme_delayed_attach_t
            //use core::any::{Any, TypeId};
            //let typedef_val = nvme_delayed_attach_t::default();
            //let typedef_id = typedef_val.type_id();
            //let this_fn_id = Some($impl_fn_name).type_id();
            //assert!(typedef_id == this_fn_id);

            // SAFETY: Cast base class to softc specified with DeviceIf impl. NvmeIf ensures that
            // this is a subclass of nvme_controller.
            let sc = unsafe {
                let sc_ptr = ctrlr.cast::<<$driver_ty as DeviceIf>::Softc>();
                kpi::ffi::Ref::from_raw(sc_ptr)
            };
            <$driver_ty as $crate::NvmeIf>::nvme_delayed_attach(sc);

            // The implementation does return an error because this function's caller doesn't do
            // anything with it anyway.
            0
        }
    };
}
