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
#![feature(concat_idents)]

use core::mem::transmute;
use core::ffi::{c_int, c_void};
use core::ops::DerefMut;
use core::ptr::null_mut;
use kpi::bindings::{INTR_MPSAFE, INTR_TYPE_MISC};
use kpi::bus::{Register, Resource};
use kpi::device::{Device, DeviceIf, ProbeRes};
use kpi::sync::SpinLock;
use kpi::{dprintln, driver, get_field};

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
pub type AppleMboxRx<T> = fn(Ptr<T>, AppleMboxMsg) -> Result<()>;
type TypeErasedAppleMboxRx = fn(*mut c_void, AppleMboxMsg) -> Result<()>;

struct Softc {
    intr_softc: Claimable<IntrSoftc>,
    a2i_ctrl: A2ICtrl,
    a2i_send: A2ISend,
}

struct IntrSoftc {
    dev: Device,
    irq: Option<Resource>,
    i2a_ctrl: I2ACtrl,
    i2a_recv: I2ARecv,
    intrhand: *mut c_void,
    // group callback function and its context since they're either both `None` or both `Some`
    callback: Option<(TypeErasedAppleMboxRx, *mut c_void)>,
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
        let node = dev.ofw_bus_get_node();
        let xref = node.xref_from_node();
        dev.register_xref(xref);

        let rid = node.ofw_bus_find_string_index(c"interrupt-names", c"recv-not-empty")?;

        let irq = Some(dev.bus_alloc_resource(SYS_RES_IRQ, rid)?);

        let mut mem = dev.bus_alloc_resource(SYS_RES_MEMORY, 0)?;
        // generic parameters on take_register are inferred from the types of the result which are
        // determined by the definitions in Softc and IntrSoftc
        let a2i_ctrl = mem.take_register()?;
        let a2i_send = mem.take_register()?;
        let i2a_ctrl = mem.take_register()?;
        let i2a_recv = mem.take_register()?;

        let sc = Softc {
            intr_softc: Claimable::new(IntrSoftc {
                dev,
                irq,
                i2a_ctrl,
                i2a_recv,
                intrhand: null_mut(),
                callback: None,
            }),
            a2i_ctrl,
            a2i_send,
        };
        self.init_softc(dev, sc)?;

        Ok(())
    }

    fn device_detach(&self, _dev: Device) -> Result<()> {
        panic!("not yet")
    }
}

#[no_mangle]
pub extern "C" fn apple_mbox_write(
    mbox: *mut bindings::_device,
    msg: &AppleMboxMsg,
) -> c_int {
    let mbox = mbox.as_rust_type();
    // TODO: I can't impl AsRustType for apple_mbox_msg from here since it's defined in bindings.rs.
    // Maybe I should consider per-crate bindgen invocations
    match apple_mbox_driver.write(mbox, msg) {
        Ok(_) => 0,
        Err(e) => e.as_c_type(),
    }
}

impl Driver {
    // Get a mailbox device_t from the client's mboxes devicetree property. The mailbox must've
    // previously registered its devicetree node xref which happens when this driver attaches.
    pub fn get(&self, client: Device) -> Result<Device> {
        let client_node = client.ofw_bus_get_node();
        let mbox_ref = client_node.get_xref_prop(c"mboxes")?;
        mbox_ref.device_from_xref()
    }

    pub fn set_rx<T>(&self, mut mbox: Device, func: AppleMboxRx<T>, arg: Ptr<T>) -> Result<()> {
        let mut sc = self.claim_softc(mbox)?;
        let mut intr_softc = sc.intr_softc.claim()?;
        self.release_softc(mbox, sc)?;

        let func = unsafe { transmute(func) };
        intr_softc.callback = Some((func, arg.as_type_erased_ptr()));

        let irq = intr_softc.irq.take().ok_or(EDOOFUS)?;
        let flags = INTR_MPSAFE | INTR_TYPE_MISC;
        let intrhand = &raw mut intr_softc.intrhand;
        mbox.bus_setup_intr(irq, flags, None, Some(Self::intr), intr_softc, intrhand)?;
        Ok(())
    }

    extern "C" fn intr(mut sc: RefMut<IntrSoftc>) {
        // TODO: rust doesn't understand split borrows through a smart pointer deref.
        // figure out a more ergonomic approach to this
        let sc: &mut IntrSoftc = sc.deref_mut();
        let mut ctrl = &mut sc.i2a_ctrl;
        let mut recv = &mut sc.i2a_recv;
        while (ctrl.read_4(0) & MBOX_I2A_CTRL_EMPTY) == 0 {
            let msg = AppleMboxMsg {
                data0: recv.read_8(0),
                data1: recv.read_8(8) as u32,
            };
            match sc.callback {
                Some((func, arg)) => {
                    func(arg, msg).unwrap();
                }
                None => {
                    dprintln!(sc.dev, "Received RTKit msg w/o callback installed\n");
                }
            }
        }
    }


    pub fn write(&self, mbox: Device, msg: &AppleMboxMsg) -> Result<()> {
        let mut sc = self.claim_softc(mbox)?;
        if (sc.a2i_ctrl.read_4(0) & MBOX_A2I_CTRL_FULL) != 0 {
            return Err(EBUSY);
        }
        sc.a2i_send.write_8(0, msg.data0);
        sc.a2i_send.write_8(8, msg.data1 as u64);
        self.release_softc(mbox, sc)?;
        Ok(())
    }
}

driver!(apple_mbox_driver, c"mbox", apple_mbox_methods, Softc,
    device_probe apple_mbox_probe,
    device_attach apple_mbox_attach,
    device_detach apple_mbox_detach
);
