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

use kpi::bindings::device_t;
use kpi::bus::{Register, ResourceSpec};
use kpi::device::{BusProbe, DeviceIf};
use kpi::ffi::{DevRef, UninitDevRef};
use kpi::driver;
use kpi::ofw::XRef;
use kpi::prelude::*;
use kpi::sync::Mutable;
use rtkit::{RTKit, HasRTKit};

const CPU_CTRL: u64 = 0x44;
const CPU_CTRL_RUN: u32 = 1 << 4;

// this does not need to be explicitly null-terminated
const SPEC: [ResourceSpec; 2] = [
    ResourceSpec::new(SYS_RES_MEMORY, 0), /* asc */
    ResourceSpec::new(SYS_RES_MEMORY, 1), /* sram */
];

#[derive(Debug)]
pub struct AppleRTKitSoftc {
    asc: Mutable<Register>,
    sram: Mutable<Register>,
    rtkit: RTKit,
}

impl HasRTKit for AppleRTKitSoftc {
    fn get_rtkit(&self) -> &RTKit {
        &self.rtkit
    }
}

impl DeviceIf for AppleRTKitDriver {
    type Softc = AppleRTKitSoftc;

    fn device_probe(dev: device_t) -> Result<BusProbe> {
        if !ofw_bus_status_okay(dev) {
            return Err(ENXIO);
        }

        if !ofw_bus_is_compatible(dev, c"apple,rtk-helper-asc4") {
            return Err(ENXIO);
        }

        device_set_desc(dev, c"Apple RTKit helper");

        Ok(BUS_PROBE_SPECIFIC)
    }

    fn device_attach(uninit_sc: UninitDevRef<AppleRTKitSoftc>, dev: device_t) -> Result<()> {
        let [asc_res, sram_res] = bus_alloc_resources(dev, SPEC).inspect_err(|e| {
            device_println!(dev, "could not allocate device resources {e}");
        })?;
        let asc_reg = Register::new(asc_res).inspect_err(|e| {
            device_println!(dev, "ASC does not have type SYS_RES_MEMORY {e}");
        })?;
        let sram_reg = Register::new(sram_res).inspect_err(|e| {
            device_println!(dev, "SRAM does not have type SYS_RES_MEMORY {e}");
        })?;
        let asc = Mutable::new(asc_reg);
        let sram = Mutable::new(sram_reg);

        let node = ofw_bus_get_node(dev);
        let xref = OF_xref_from_node(node);

        OF_device_register_xref(xref, dev);

        let rtkit =
            RTKit::new(dev).inspect_err(|e| device_println!(dev, "failed to create RTKit {e}"))?;

        let sc = AppleRTKitSoftc {
            asc,
            sram,
            rtkit,
        };

        let sc = uninit_sc.init(sc).into_ref();

        sc.rtkit.set_rx(sc)?;


        Ok(())
    }

    fn device_detach(_sc: DevRef<AppleRTKitSoftc>, _dev: device_t) -> Result<()> {
        unreachable!("apple RTKit helper cannot be detached")
    }
}

impl AppleRTKitDriver {
    fn apple_rtkit_boot(client: device_t, helper: XRef) -> Result<()> {
        let dev = OF_device_from_xref(helper).inspect_err(|e| {
            device_println!(client, "could not get device from xref {e}");
        })?;

        let sc = device_get_softc::<Self>(dev);

        let mut asc = sc.asc.get_mut();
        let ctrl = bus_read_4!(asc, CPU_CTRL);
        bus_write_4!(asc, CPU_CTRL, ctrl | CPU_CTRL_RUN);

        //let sc_rtkit = sc.clone();
        //rtkit_start(project!(sc_rtkit->rtkit)).inspect_err(|e| {
        //    device_println!(dev, "failed to boot RTKit {e}");
        //})?;

        //sc.rtkit.boot().unwrap();
        //sc.rtkit.set_iop(PwrState::On).inspect_err(|e| {
        //    device_println!(dev, "failed to set IOP power state ON");
        //})?;
        //sc.rtkit.set_ap(PwrState::On).inspect_err(|e| {
        //    device_println!(dev, "failed to set AP power state ON");
        //})?;

        Ok(())
    }
}

driver!(apple_rtkit_driver, c"apple_rtkit", AppleRTKitDriver,
    apple_rtkit_methods = {
        device_probe apple_rtkit_probe,
        device_attach apple_rtkit_attach,
        device_detach apple_rtkit_detach,
    }
    //EXPORTS {
    //    apple_rtkit_boot(client: device_t, helper: phandle_t) -> int;
    //}
);
