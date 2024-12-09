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
use kpi::device::{Device, ProbeRes};
use kpi::driver;
use kpi::ofw::XRef;
use rtkit::{ManagesRTKit, PwrState, RTKit};

const CPU_CTRL: u64 = 0x44;
const CPU_CTRL_RUN: u32 = 1 << 4;

// this does not need to be explicitly null-terminated
const SPEC: [ResourceSpec; 2] = [
    ResourceSpec::new(SYS_RES_MEMORY, 0), /* asc */
    ResourceSpec::new(SYS_RES_MEMORY, 1), /* sram */
];

#[derive(Debug)]
pub struct AppleRTKitSoftc {
    mem: UniqueCell<[Register; 2]>,
    rtkit: RTKit,
}

impl ManagesRTKit for AppleRTKitDriver {
    fn rtkit_from_sc(sc: &Self::Softc) -> &RTKit {
        &sc.rtkit
    }
}

impl DeviceIf for AppleRTKitDriver {
    type Softc = AppleRTKitSoftc;

    fn device_probe(&self, dev: &Device) -> Result<ProbeRes> {
        if !ofw_bus_status_okay(dev) {
            return Err(ENXIO);
        }

        if !ofw_bus_is_compatible(dev, c"apple,rtk-helper-asc4") {
            return Err(ENXIO);
        }

        device_set_desc(dev, c"Apple RTKit helper");

        Ok(BUS_PROBE_SPECIFIC)
    }

    fn device_attach(&self, dev: &mut Device) -> Result<AttachRes> {
        let resources = bus_alloc_resources(dev, SPEC)?;
        let mem = resources.map(|r| r.whole_register());
        let rtkit = RTKit::new(dev.clone())?;

        let node = ofw_bus_get_node(dev);
        let xref = OF_xref_from_node(node);

        OF_device_register_xref(dev, xref);

        let sc = AppleRTKitSoftc {
            mem: UniqueCell::new(mem),
            rtkit,
        };
        let res = self.init_softc(dev, sc);

        Ok(res)
    }

    fn device_detach(&self, dev: &mut Device) -> Result<()> {
        unreachable!("device cannot be detached")
    }
}

impl AppleRTKitDriver {
    fn boot(&self, dev: &mut Device) -> Result<()> {
        let sc: &AppleRTKitSoftc = todo!("");//self.get_softc_with_state(dev);
        let mem = sc.mem.get_mut();
        let ctrl = mem[0].read_4(CPU_CTRL);
        mem[0].write_4(CPU_CTRL, ctrl | CPU_CTRL_RUN);

        self.rtkit_boot(dev)?;

        sc.rtkit.set_iop_pwr_state(PwrState::On)?;
        sc.rtkit.set_ap_pwr_state(PwrState::On)?;
        Ok(())
    }
}

#[no_mangle]
unsafe extern "C" fn apple_rtkit_boot(helper: XRef) -> c_int {
    let mut dev = OF_device_from_xref(helper).unwrap();
    match apple_rtkit_driver.boot(&mut dev) {
        Ok(_) => 0,
        Err(e) => e.as_c_type(),
    }
}

driver!(apple_rtkit_driver, c"apple_rtkit", AppleRTKitDriver, apple_rtkit_methods,
    device_probe apple_rtkit_probe,
    device_attach apple_rtkit_attach,
    device_detach apple_rtkit_detach
);
