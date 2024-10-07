#![no_std]
#![feature(concat_idents)]

extern crate alloc;

use alloc::boxed::Box;
use core::ffi::c_int;
use core::ptr::{addr_of_mut, null_mut};
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

        let rtkit = RTKit::new(&dev, false);

        let sc = self.claim_softc(&mut dev)?.init_from(Softc { mem, rtkit });

        self.release_softc(&mut dev, sc);

        Ok(())
    }

    fn device_detach(&self, dev: Device) -> Result<()> {
        panic!("not yet")
    }
}

fn apple_rtkit_boot2(helper: XRef) -> Result<()> {
    let mut dev = helper.device_from_xref()?;
    let mut sc = unsafe { apple_rtkit_driver.driver.claim_softc(&mut dev)?.is_init() };
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
    device_if device_probe apple_rtkit_probe,
    device_if device_attach apple_rtkit_attach,
    device_if device_detach apple_rtkit_detach
);
