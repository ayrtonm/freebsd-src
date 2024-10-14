/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2024 Ayrton Muñoz
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
#![feature(concat_idents)]

extern crate alloc;

use alloc::boxed::Box;
use core::ffi::c_int;
use kpi::bus::{Resource, ResourceSpec};
use kpi::device::{Device, DeviceIf, ProbeRes};
use kpi::driver;
use kpi::ofw::XRef;
use rtkit::RTKit;

const CPU_CTRL: u64 = 0x44;
const CPU_CTRL_RUN: u32 = 1 << 4;

// this does not need to be explicitly null-terminated
const SPEC: [ResourceSpec; 2] = [
    ResourceSpec::new(SYS_RES_MEMORY, 0), /* asc */
    ResourceSpec::new(SYS_RES_MEMORY, 1), /* sram */
];

struct Softc {
    mem: [Resource; 2],
    rtkit: Box<RTKit>,
}

impl DeviceIf for Driver {
    fn device_probe(&self, dev: Device) -> Result<ProbeRes> {
        if !dev.ofw_bus_status_okay() {
            return Err(ENXIO);
        }

        if !dev.ofw_bus_is_compatible(c"apple,rtk-helper-asc4") {
            return Err(ENXIO);
        }

        dev.set_desc(c"Apple RTKit helper");

        Ok(BUS_PROBE_SPECIFIC)
    }

    fn device_attach(&self, mut dev: Device) -> Result<()> {
        let mem = dev.bus_alloc_resources(SPEC)?;

        let node = dev.ofw_bus_get_node();
        let xref = node.xref_from_node();

        dev.register_xref(xref);

        let rtkit = RTKit::new(dev, false);

        self.init_softc(dev, Softc { mem, rtkit })?;

        Ok(())
    }

    fn device_detach(&self, _dev: Device) -> Result<()> {
        panic!("not yet")
    }
}

fn apple_rtkit_boot2(helper: XRef) -> Result<()> {
    let dev = helper.device_from_xref()?;
    let mut sc = apple_rtkit_driver.claim_softc(dev)?;
    let ctrl = sc.mem[0].read_4(CPU_CTRL);
    sc.mem[0].write_4(CPU_CTRL, ctrl | CPU_CTRL_RUN);

    sc.rtkit.boot()?;
    sc.rtkit
        .set_ap_pwrstate(bindings::RTKIT_MGMT_PWR_STATE_ON as u16);
    Ok(())
}

#[no_mangle]
unsafe extern "C" fn apple_rtkit_boot(helper: XRef) -> c_int {
    apple_rtkit_boot2(helper).unwrap();
    0
}

driver!(apple_rtkit_driver, c"apple_rtkit", apple_rtkit_methods, Softc,
    device_probe apple_rtkit_probe,
    device_attach apple_rtkit_attach,
    device_detach apple_rtkit_detach
);
