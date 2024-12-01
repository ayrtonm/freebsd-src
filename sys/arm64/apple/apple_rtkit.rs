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

use core::ffi::c_int;
use kpi::bus::{Register, ResourceSpec};
use kpi::device::{Device, DeviceIf, ProbeRes};
use kpi::driver;
use kpi::ofw::XRef;
use rtkit::{PwrState, RTKit};
use apple_mbox::Boot;

const CPU_CTRL: u64 = 0x44;
const CPU_CTRL_RUN: u32 = 1 << 4;

// this does not need to be explicitly null-terminated
const SPEC: [ResourceSpec; 2] = [
    ResourceSpec::new(SYS_RES_MEMORY, 0), /* asc */
    ResourceSpec::new(SYS_RES_MEMORY, 1), /* sram */
];

#[derive(Debug)]
pub struct AppleRTKitSoftc<S> {
    mem: [Register; 2],
    rtkit: RTKit<S>,
}

impl DeviceIf for Driver {
    type Softc<S> = UniqueCell<AppleRTKitSoftc<S>, Boot, S>;
    fn device_probe(dev: &Device) -> Result<ProbeRes> {
        if !dev.ofw_bus_status_okay() {
            return Err(ENXIO);
        }

        if !dev.ofw_bus_is_compatible(c"apple,rtk-helper-asc4") {
            return Err(ENXIO);
        }

        dev.set_desc(c"Apple RTKit helper");

        Ok(BUS_PROBE_SPECIFIC)
    }

    fn device_attach(dev: &mut Device) -> Result<AttachRes> {
        let resources = dev.bus_alloc_resources(SPEC)?;
        let mem = resources.map(|r| r.whole_register());
        let rtkit = RTKit::new(dev.copy_ptr())?;

        let node = dev.ofw_bus_get_node();
        let xref = node.xref_from_node();

        dev.register_xref(xref);

        let sc = UniqueCell::new(AppleRTKitSoftc { mem, rtkit });
        let res = Driver::init_softc(dev, sc);

        Ok(res)
    }

    fn device_detach(dev: &mut Device) -> Result<()> {
        unreachable!("device cannot be detached")
    }
}

impl Driver {
    fn boot(&self, dev: &mut Device<Boot>) -> Result<()> {
        let sc = Driver::get_softc_mut(dev).get_mut();
        let ctrl = sc.mem[0].read_4(CPU_CTRL);
        sc.mem[0].write_4(CPU_CTRL, ctrl | CPU_CTRL_RUN);

        sc.rtkit.boot()?;

        let sc = Driver::get_softc_mut(dev).get();

        sc.rtkit.set_iop_pwr_state(PwrState::On)?;
        sc.rtkit.set_ap_pwr_state(PwrState::On)?;
        Ok(())
    }
}

#[no_mangle]
unsafe extern "C" fn apple_rtkit_boot(helper: XRef) -> c_int {
    let mut dev = unsafe { helper.device_from_xref().unwrap().assume_state() };
    match apple_rtkit_driver.boot(&mut dev) {
        Ok(_) => 0,
        Err(e) => e.as_c_type(),
    }
}

driver!(apple_rtkit_driver, c"apple_rtkit", apple_rtkit_methods, AppleRTKitSoftc,
    device_probe apple_rtkit_probe,
    device_attach apple_rtkit_attach,
    device_detach apple_rtkit_detach
);
