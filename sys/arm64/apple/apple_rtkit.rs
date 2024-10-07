#![no_std]
#![feature(concat_idents)]

use core::ffi::c_int;
use core::ptr::{addr_of_mut, null_mut};
use kpi::bus::{Resource, ResourceSpec};
use kpi::device::{Device, DeviceIf, ProbeRes, UniqDevice};
use kpi::driver;
use kpi::ofw::XRef;

const CPU_CTRL: u64 = 0x44;
const CPU_CTRL_RUN: u32 = 1 << 4;

// this does not need to be explicitly null-terminated
const SPEC: [ResourceSpec; 2] = [
    ResourceSpec::new(SYS_RES_MEMORY, 0), /* asc */
    ResourceSpec::new(SYS_RES_MEMORY, 1), /* sram */
];

struct Softc {
    mem: [Resource; 2],
    rtkit_state: *mut bindings::rtkit_state,
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

    fn device_attach(&self, mut dev: UniqDevice) -> Result<()> {
        let mem = dev.bus_alloc_resources(SPEC)?;

        let node = dev.ofw_bus_get_node();
        let xref = node.xref_from_node();

        dev.register_xref(xref);

        let mut sc = self.claim_softc(&mut dev)?.init_from(Softc {
            mem,
            rtkit_state: null_mut(),
        });

        let rc = unsafe { bindings::rtkit_init(dev.dup(), addr_of_mut!(sc.rtkit_state), false) };
        if rc != 0 {
            return Err(ENXIO);
        }

        self.release_softc(&mut dev, sc);

        Ok(())
    }

    fn device_detach(&self, dev: Device) -> Result<()> {
        panic!("not yet")
    }
}

fn apple_rtkit_boot2(helper: XRef) -> Result<()> {
    let dev = helper.device_from_xref()?;
    let mut dev = unsafe { dev.is_unique() };
    let sc = unsafe { apple_rtkit_driver.driver.claim_softc(&mut dev)?.is_init() };
    let ctrl = sc.mem[0].read_4(CPU_CTRL);
    sc.mem[0].write_4(CPU_CTRL, ctrl | CPU_CTRL_RUN);

    let rc = unsafe { bindings::rtkit_boot(sc.rtkit_state) };
    if rc != 0 {
        return Err(ENXIO);
    }
    unsafe {
        bindings::rtkit_set_ap_pwrstate(sc.rtkit_state, bindings::RTKIT_MGMT_PWR_STATE_ON as u16);
    }
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
