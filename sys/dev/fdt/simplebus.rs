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

use core::ffi::c_void;
use kpi::ErrCode;
use kpi::bindings::{device_t, simplebus_softc};
use kpi::device::{DeviceIf, Device};
use kpi::driver::Driver;
use kpi::prelude::*;
use kpi::ofw::Node;
use core::ffi::c_int;

pub mod prelude {
    use super::*;

    pub fn simplebus_attach_impl(dev: &Device, flags: c_int, node: Option<Node>) -> Result<()> {
        if device_get_driver(dev).is_null() {
            return Err(EINVAL);
        }
        let node_value = match node {
            Some(n) => n.as_phandle(),
            None => 0,
        };
        let res = unsafe { bindings::simplebus_attach_impl(dev.as_ptr(), flags, node_value) };
        if res != 0 {
            return Err(ErrCode::from(res));
        }
        Ok(())
    }

    pub fn simplebus_attach(dev: &Device) -> Result<()> {
        if device_get_driver(dev).is_null() {
            return Err(EINVAL);
        }
        let res = unsafe { bindings::simplebus_attach(dev.as_ptr()) };
        if res != 0 {
            return Err(ErrCode::from(res));
        }
        Ok(())
    }
}
