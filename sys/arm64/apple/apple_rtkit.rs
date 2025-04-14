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

use core::ops::DerefMut;
use kpi::bus::{Register, ResourceSpec};
use kpi::device::{Device, BusProbe};
use kpi::driver;
use kpi::ofw::XRef;
use kpi::cell::Checked;
use kpi::sync::{Arc, Mutex};
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
    asc: Checked<Register>,
    //sram: Register,
    rtkit: Arc<RTKit>,
}

impl ManagesRTKit for AppleRTKitDriver {
    fn get_rtkit(sc: &Self::Softc) -> &Arc<RTKit> {
        &sc.rtkit
    }
}

impl DeviceIf for AppleRTKitDriver {
    type Softc = AppleRTKitSoftc;

    fn device_probe(dev: Device) -> Result<BusProbe> {
        if !ofw_bus_status_okay(dev) {
            return Err(ENXIO);
        }

        if !ofw_bus_is_compatible(dev, c"apple,rtk-helper-asc4") {
            return Err(ENXIO);
        }

        device_set_desc(dev, c"Apple RTKit helper");

        Ok(BUS_PROBE_SPECIFIC)
    }

    fn device_attach(mut dev: Device) -> Result<()> {
        let resources = bus_alloc_resources(dev, SPEC)?;
        let mut regs = resources.map(|r| r.take_register().ok());
        let asc = Checked::new(regs[0].take().unwrap());
        //let sram = regs[1].take().unwrap();

        let node = ofw_bus_get_node(dev);
        let xref = OF_xref_from_node(node);

        OF_device_register_xref(dev, xref);

        let rtkit = Arc::new(RTKit::new(dev)?, M_NOWAIT);

        let sc = AppleRTKitSoftc {
            asc,
            //sram,
            rtkit,
        };

        device_init_softc!(dev, sc);

        Ok(())
    }

    fn device_detach(_dev: Device) -> Result<()> {
        unreachable!("device cannot be detached")
    }
}

impl AppleRTKitDriver {
    fn apple_rtkit_boot(helper: XRef) -> Result<()> {
        let dev = OF_device_from_xref(helper)?;

        let sc = device_get_softc!(dev);

        let mut asc_guard = sc.asc.get_mut();
        let mut asc = asc_guard.deref_mut();
        let ctrl = bus_read_4(asc, CPU_CTRL);
        bus_write_4(asc, CPU_CTRL, ctrl | CPU_CTRL_RUN);

        Self::rtkit_boot(dev)?;

        sc.rtkit.set_iop(PwrState::On)?;
        sc.rtkit.set_ap(PwrState::On)?;

        Ok(())
    }
}

driver!(apple_rtkit_driver, c"apple_rtkit", AppleRTKitDriver, apple_rtkit_methods,
    INTERFACES {
        device_probe apple_rtkit_probe,
        device_attach apple_rtkit_attach,
        device_detach apple_rtkit_detach,
    },
    EXPORTS {
        int apple_rtkit_boot(phandle_t helper);
    }
);
