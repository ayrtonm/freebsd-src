#![no_std]
#![feature(concat_idents)]

use core::ffi::{c_int, c_void};
use core::ptr::{addr_of_mut, null_mut};
use kpi::bus::Resource;
use kpi::device::{Device, DeviceIf, ProbeRes, RawDevice, UniqDevice};
use kpi::ofw::XRef;
use kpi::{dprintln, driver, Ref, UniqRef};

type CbTy = extern "C" fn(*mut c_void, bindings::apple_mbox_msg) -> c_int;

struct Softc {
    dev: RawDevice,
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

    fn device_attach(&self, mut dev: UniqDevice) -> Result<()> {
        let mem = dev.bus_alloc_resource(SYS_RES_MEMORY, 0)?;
        let node = dev.ofw_bus_get_node();
        let rid = node.ofw_bus_find_string_index(c"interrupt-names", c"recv-not-empty")?;
        let irq = Some(dev.bus_alloc_resource(SYS_RES_IRQ, rid)?);
        let xref = node.xref_from_node();
        dev.register_xref(xref);

        let sc = Softc {
            dev: dev.dup(),
            mem,
            irq,
            intrhand: null_mut(),
            callback: None,
            arg: null_mut(),
        };
        let sc = self.claim_softc(&mut dev)?.init_from(sc);
        self.release_softc(&mut dev, sc);

        Ok(())
    }

    fn device_detach(&self, dev: Device) -> Result<()> {
        panic!("not yet")
    }
}

pub fn apple_mbox_get2(client: Device) -> Result<Device> {
    let client_node = client.ofw_bus_get_node();
    let mbox_ref = client_node.get_xref_prop(c"mboxes")?;
    mbox_ref.device_from_xref()
}

use kpi::AsRustType;

#[no_mangle]
pub extern "C" fn apple_mbox_get(client: *mut bindings::_device) -> *mut bindings::_device {
    let client = client.as_rust_type();

    apple_mbox_get2(client).unwrap().as_ptr()
    //match apple_mbox_get2(client) {
    //    Ok(res) => res.as_ptr(),
    //    Err(e) => core::ptr::null_mut(),
    //}
}

#[no_mangle]
pub extern "C" fn apple_mbox_set_rx(mbox: bindings::device_t, cb: Option<CbTy>, arg: *mut c_void) {
    let mbox = mbox.as_rust_type();
    apple_mbox_driver.driver.set_rx(mbox, cb, arg).unwrap();
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
            Some(callback) => {
                callback(sc.arg, msg);
            }
            None => {
                dprintln!(sc.dev, "Received RTKit msg w/o callback installed\n");
            }
        }
    }
}

impl Driver {
    fn set_rx(&self, mbox: Device, callback: Option<CbTy>, arg: *mut c_void) -> Result<()> {
        let mut mbox = unsafe { mbox.is_unique() };
        let mut sc = unsafe { self.claim_softc(&mut mbox)?.is_init() };
        sc.callback = callback;
        sc.arg = arg;
        let irq = sc.irq.take().unwrap();
        let intrhand = addr_of_mut!(sc.intrhand);
        self.release_softc(&mut mbox, sc);

        let sc = unsafe { self.share_softc(&mut mbox)?.is_init() };

        // TODO: FreeBSD tends to avoid reusing identifiers so tweak bindgen flags to remove the prefixes
        let flags = bindings::intr_type_INTR_MPSAFE | bindings::intr_type_INTR_TYPE_MISC;
        mbox.bus_setup_intr(irq, flags, None, Some(apple_mbox_intr), sc, intrhand)
            .unwrap();

        Ok(())
    }

    fn write(&self, mbox: Device, msg: *const bindings::apple_mbox_msg) -> c_int {
        let mut mbox = unsafe { mbox.is_unique() };
        let mut sc = unsafe { self.share_softc(&mut mbox).unwrap().is_init() };
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
    device_if device_probe apple_mbox_probe,
    device_if device_attach apple_mbox_attach,
    device_if device_detach apple_mbox_detach
);
