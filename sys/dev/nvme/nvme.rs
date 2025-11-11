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

use kpi::prelude::*;
use kpi::define_dev_interface;
use kpi::device::DeviceIf;
use kpi::sync::arc::ArcRef;
use kpi::bindings::{device_t, nvme_controller, nvme_qpair, nvme_tracker};

#[allow(unused_variables)]
pub trait NvmeIf: DeviceIf {
    fn nvme_delayed_attach(
        sc: ArcRef<Self::Softc>,
        dev: device_t,
        ctrlr: *mut nvme_controller,
    ) -> Result<()> {
        unimplemented!()
    }
    fn nvme_enable(sc: ArcRef<Self::Softc>, dev: device_t) {
        unimplemented!()
    }
    fn nvme_sq_enter(
        sc: ArcRef<Self::Softc>,
        dev: device_t,
        qpair: *mut nvme_qpair,
        tr: &nvme_tracker,
    ) -> u32 {
        unimplemented!()
    }
    fn nvme_sq_leave(
        sc: ArcRef<Self::Softc>,
        dev: device_t,
        qpair: &nvme_qpair,
        tr: &nvme_tracker,
    ) {
        unimplemented!()
    }
    fn nvme_cq_done(
        sc: ArcRef<Self::Softc>,
        dev: device_t,
        qpair: &nvme_qpair,
        tr: &nvme_tracker,
    ) {
        unimplemented!()
    }
    fn nvme_qpair_construct(
        sc: ArcRef<Self::Softc>,
        dev: device_t,
        qpair: &mut nvme_qpair,
        num_entries: u32,
        num_trackers: u32,
        ctrlr: *mut nvme_controller,
    ) -> Result<()> {
        unimplemented!()
    }
}

define_dev_interface! {
    in NvmeIf
    fn nvme_delayed_attach(dev: device_t, ctrlr: *mut nvme_controller) -> int,
        with desc nvme_delayed_attach_desc
        and typedef nvme_delayed_attach_t;
    fn nvme_enable(dev: device_t),
        with desc nvme_enable_desc
        and typedef nvme_enable_t;
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

#[doc(hidden)]
#[macro_export]
macro_rules! nvme_sq_enter {
    (get_desc) => {
        nvme_sq_enter_desc
    };
    (get_typedef) => {
        nvme_sq_enter_t
    };
    ($driver_ty:ident $impl_fn_name:ident) => {
        kpi::define_c_function! {
            $driver_ty impls $impl_fn_name in NvmeIf as
            fn nvme_sq_enter(dev: device_t, qpair: *mut nvme_qpair, tr: *mut nvme_tracker) -> u32;
            with init glue {
                use kpi::bindings;
                use kpi::kobj::KobjLayout;
                use kpi::sync::arc::ArcRef;

                let void_ptr = unsafe { bindings::device_get_softc(dev) };
                let sc_ptr = void_ptr.cast::<<$driver_ty as KobjLayout>::Layout>();
                let sc = unsafe { ArcRef::from_raw(sc_ptr) };
            }
            with prefix args { sc }
            infallible
        }
    };
}
