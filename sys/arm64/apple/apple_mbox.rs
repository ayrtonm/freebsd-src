#![no_std]
#![feature(concat_idents)]

use core::ffi::{c_int, c_void};
use core::ptr::{addr_of_mut, null_mut};
use kpi::bus::Resource;
use kpi::device::{Device, DeviceIf, ProbeRes};
use kpi::ofw::XRef;
use kpi::{dprintln, driver, AsRustType, Ref};

type CbTy = unsafe extern "C" fn(*mut c_void, bindings::apple_mbox_msg) -> c_int;

struct Softc {
    dev: Device,
    mem: Resource,
    irq: Option<Resource>,

    intrhand: *mut c_void,
    callback: Option<CbTy>,
    arg: *mut c_void,
}

impl DeviceIf for Driver {
    fn device_probe(&self, dev: Device) -> Result<ProbeRes> {
        if !dev.ofw_bus_status_okay() {
            return Err(ENXIO);
        }

        if !dev.ofw_bus_is_compatible(c"apple,asc-mailbox-v4") {
            return Err(ENXIO);
        }

        dev.set_desc(c"Apple Mailbox");

        Ok(BUS_PROBE_SPECIFIC)
    }

    fn device_attach(&self, mut dev: Device) -> Result<()> {
        let mem = dev.bus_alloc_resource(SYS_RES_MEMORY, 0)?;
        let node = dev.ofw_bus_get_node();
        let rid = node.ofw_bus_find_string_index(c"interrupt-names", c"recv-not-empty")?;
        let irq = Some(dev.bus_alloc_resource(SYS_RES_IRQ, rid)?);
        let xref = node.xref_from_node();
        dev.register_xref(xref);

        let sc = Softc {
            dev,
            mem,
            irq,
            intrhand: null_mut(),
            callback: None,
            arg: null_mut(),
        };
        self.softc_init(dev, sc)?;

        Ok(())
    }

    fn device_detach(&self, dev: Device) -> Result<()> {
        panic!("not yet")
    }
}

#[no_mangle]
pub extern "C" fn apple_mbox_write(
    mbox: *mut bindings::_device,
    msg: *const bindings::apple_mbox_msg,
) -> c_int {
    let mbox = mbox.as_rust_type();
    apple_mbox_driver.driver.write(mbox, msg)
}

const MBOX_A2I_CTRL: u64 = 0x110;
const MBOX_A2I_CTRL_FULL: u32 = 1 << 16;
const MBOX_I2A_CTRL: u64 = 0x114;
const MBOX_I2A_CTRL_EMPTY: u32 = 1 << 17;
const MBOX_A2I_SEND0: u64 = 0x800;
const MBOX_A2I_SEND1: u64 = 0x808;
const MBOX_I2A_RECV0: u64 = 0x830;
const MBOX_I2A_RECV1: u64 = 0x838;

extern "C" fn apple_mbox_intr(mut sc: Ref<Softc>) {
    while (sc.mem.read_4(MBOX_I2A_CTRL) & MBOX_I2A_CTRL_EMPTY) == 0 {
        let msg = bindings::apple_mbox_msg {
            data0: sc.mem.read_8(MBOX_I2A_RECV0),
            data1: sc.mem.read_8(MBOX_I2A_RECV1) as u32,
        };
        match sc.callback {
            Some(callback) => unsafe {
                callback(sc.arg, msg);
            },
            None => {
                dprintln!(sc.dev, "Received RTKit msg w/o callback installed\n");
            }
        }
    }
}

impl Driver {
    pub fn get(client: Device) -> Result<Device> {
        let client_node = client.ofw_bus_get_node();
        let mbox_ref = client_node.get_xref_prop(c"mboxes")?;
        mbox_ref.device_from_xref()
    }

    pub fn set_rx(&self, mut mbox: Device, callback: CbTy, arg: *mut c_void) -> Result<()> {
        let mut sc = self.softc_claim(mbox)?;
        sc.callback = Some(callback);
        sc.arg = arg;
        let irq = sc.irq.take().unwrap();
        let intrhand = addr_of_mut!(sc.intrhand);
        self.softc_release(mbox, sc);

        let sc = self.softc_share(mbox)?;

        let flags = bindings::INTR_MPSAFE | bindings::INTR_TYPE_MISC;
        mbox.bus_setup_intr(irq, flags, None, Some(apple_mbox_intr), sc, intrhand)
            .unwrap();

        Ok(())
    }

    fn write(&self, mut mbox: Device, msg: *const bindings::apple_mbox_msg) -> c_int {
        let mut sc = self.softc_share(mbox).unwrap();
        let ctrl = sc.mem.read_4(MBOX_A2I_CTRL);
        if (ctrl & MBOX_A2I_CTRL_FULL) != 0 {
            return bindings::EBUSY;
        }
        unsafe {
            sc.mem.write_8(MBOX_A2I_SEND0, (*msg).data0);
            sc.mem.write_8(MBOX_A2I_SEND1, (*msg).data1 as u64);
        }
        0
    }
}

driver!(apple_mbox_driver, c"mbox", apple_mbox_methods, Softc,
    device_probe apple_mbox_probe,
    device_attach apple_mbox_attach,
    device_detach apple_mbox_detach
);
