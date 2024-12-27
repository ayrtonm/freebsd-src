/*	$OpenBSD: aplmbox.c,v 1.2 2022/01/04 20:55:48 kettenis Exp $	*/
/*
 * Copyright (c) 2021 Mark Kettenis <kettenis@openbsd.org>
 * Copyright (c) 2022 Kyle Evans <kevans@FreeBSD.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#![no_std]

use core::ffi::c_void;
use core::mem::transmute;
use core::ops::DerefMut;
use core::ptr::null_mut;
use kpi::bindings::{INTR_MPSAFE, INTR_TYPE_MISC};
use kpi::bus::{Register, Resource};
use kpi::cell::{FFICell, Mutable};
use kpi::device::{BusProbe, Device, SoftcInit};
use kpi::driver;

const MBOX_A2I_CTRL: u64 = 0x110;
const MBOX_A2I_CTRL_FULL: u32 = 1 << 16;

const MBOX_I2A_CTRL: u64 = 0x114;
const MBOX_I2A_CTRL_EMPTY: u32 = 1 << 17;

const MBOX_A2I_SEND0: u64 = 0x800;
const MBOX_A2I_SEND1: u64 = 0x808;

const MBOX_I2A_RECV0: u64 = 0x830;
const MBOX_I2A_RECV1: u64 = 0x838;

type A2ICtrl = Register<MBOX_A2I_CTRL, 4>;
type A2ISend = Register<MBOX_A2I_SEND0, 0x10>;
type I2ACtrl = Register<MBOX_I2A_CTRL, 4>;
type I2ARecv = Register<MBOX_I2A_RECV0, 0x10>;

// This needs repr(C) while it remains shared with C code
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct AppleMboxMsg {
    pub data0: u64,
    pub data1: u32,
}

// This callback type's callsites are in rust so it doesn't need to be extern "C". If it were in C
// it would need to go through the KPI crate which would enforce the extern "C" to avoid a compiler
// error.
pub type AppleMboxRx<T> = fn(&T, AppleMboxMsg) -> Result<()>;
pub type RawAppleMboxRx = fn(*mut c_void, AppleMboxMsg) -> Result<()>;

#[derive(Debug)]
pub struct AppleMboxSoftc {
    dev: Device,
    irq: Resource,
    intr_softc: Mutable<IntrSoftc>,
    write_msg_softc: Mutable<WriteMsgSoftc>,
    intrhand: FFICell<*mut c_void>,
}

#[derive(Debug)]
struct IntrSoftc {
    i2a_ctrl: I2ACtrl,
    i2a_recv: I2ARecv,
    callback: RawAppleMboxRx,
    arg: *mut c_void,
}

#[derive(Debug)]
struct WriteMsgSoftc {
    a2i_ctrl: A2ICtrl,
    a2i_send: A2ISend,
}

impl DeviceIf for AppleMboxDriver {
    type Softc = AppleMboxSoftc;

    fn device_probe(&self, dev: Device) -> Result<BusProbe> {
        if !ofw_bus_status_okay(dev) {
            return Err(ENXIO);
        }

        if !ofw_bus_is_compatible(dev, c"apple,asc-mailbox-v4") {
            return Err(ENXIO);
        }

        device_set_desc(dev, c"Apple Mailbox");

        Ok(BUS_PROBE_SPECIFIC)
    }

    fn device_attach(&self, dev: Device) -> Result<SoftcInit> {
        let node = ofw_bus_get_node(dev);

        let rid = ofw_bus_find_string_index(node, c"interrupt-names", c"recv-not-empty")?;
        let irq = bus_alloc_resource(dev, SYS_RES_IRQ, rid)?;
        let mut mem = bus_alloc_resource(dev, SYS_RES_MEMORY, 0)?;
        // generic parameters on take_register are inferred from the types of the result which are
        // determined by the definitions in AppleMboxSoftc
        let a2i_ctrl = mem.take_register()?;
        let a2i_send = mem.take_register()?;
        let i2a_ctrl = mem.take_register()?;
        let i2a_recv = mem.take_register()?;

        let intr_softc = Mutable::new(IntrSoftc {
            i2a_ctrl,
            i2a_recv,
            callback: |_, _|  {
                panic!("Received RTKit message without a callback installed")
            },
            arg: null_mut(),
        });

        let write_msg_softc = Mutable::new(WriteMsgSoftc { a2i_ctrl, a2i_send });

        let xref = OF_xref_from_node(node);
        OF_device_register_xref(dev, xref);

        let res = self.init_softc(
            dev,
            AppleMboxSoftc {
                dev,
                irq,
                intr_softc,
                write_msg_softc,
                intrhand: FFICell::zeroed(),
            },
        );

        Ok(res)
    }

    fn device_detach(&self, _dev: Device) -> Result<()> {
        unreachable!("device cannot be detached")
    }
}

impl AppleMboxDriver {
    // Get a mailbox device_t from the client's mboxes devicetree property. The mailbox must've
    // previously registered its devicetree node xref which happens when this driver attaches.
    pub fn get_mbox(&self, client: Device) -> Result<Device> {
        let client_node = ofw_bus_get_node(client);
        let mbox_xref = OF_getencprop_as_xref(client_node, c"mboxes")?;
        OF_device_from_xref(mbox_xref)
    }

    pub fn set_rx<D: DeviceIf>(
        &self,
        mbox: Device,
        client: Device,
        driver: &D,
        func: AppleMboxRx<D::Softc>,
    ) -> Result<()> {
        let sc = self.get_softc(mbox);

        let mut intr_sc = sc.intr_softc.get_mut();
        let func = unsafe { transmute(func) };
        intr_sc.callback = func;
        intr_sc.arg = driver.get_softc_as_mut(client) as *mut c_void;

        let flags = INTR_MPSAFE | INTR_TYPE_MISC;
        let intrhand = sc.intrhand.as_ptr();

        self.bus_setup_intr(mbox, &sc.irq, flags, None, Some(apple_mbox_intr), intrhand)?;

        Ok(())
    }

    pub fn write_msg(&self, mbox: Device, msg: AppleMboxMsg) -> Result<()> {
        let mut write_msg_sc = self.get_softc(mbox).write_msg_softc.get_mut();
        let ctrl = &mut write_msg_sc.a2i_ctrl;
        if (ctrl.read_4(0) & MBOX_A2I_CTRL_FULL) != 0 {
            return Err(EBUSY);
        }
        let send = &mut write_msg_sc.a2i_send;
        send.write_8(0, msg.data0);
        send.write_8(8, u64::from(msg.data1));
        Ok(())
    }
}

// extern "C" doesn't preclude making this a method of AppleMboxDriver but making it standalone
// makes the arguments to bus_setup_intr clearer
extern "C" fn apple_mbox_intr(sc: &AppleMboxSoftc) {
    let mut intr_sc = sc.intr_softc.get_mut();
    let mut intr_sc = intr_sc.deref_mut();

    let mut ctrl = &mut intr_sc.i2a_ctrl;
    let mut recv = &mut intr_sc.i2a_recv;
    while (ctrl.read_4(0) & MBOX_I2A_CTRL_EMPTY) == 0 {
        let msg = AppleMboxMsg {
            data0: recv.read_8(0),
            data1: recv.read_8(8) as u32,
        };
        let arg = intr_sc.arg;
        (intr_sc.callback)(arg, msg).unwrap();
    }
}

driver!(apple_mbox_driver, c"mbox", AppleMboxDriver, apple_mbox_methods,
    device_probe apple_mbox_probe,
    device_attach apple_mbox_attach,
    device_detach apple_mbox_detach,
);
