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
#![feature(macro_metavar_expr_concat)]
#![allow(dead_code)]

use core::ffi::c_void;
use core::mem::transmute;
use kpi::bindings::{INTR_MPSAFE, INTR_TYPE_MISC, device_t};
use kpi::bus::{Irq, Register};
use kpi::cell::{Mutable, Ref};
use kpi::device::BusProbe;
use kpi::driver;

const MBOX_A2I_CTRL: u64 = 0x110;
const MBOX_A2I_CTRL_FULL: u32 = 1 << 16;

const MBOX_I2A_CTRL: u64 = 0x114;
const MBOX_I2A_CTRL_EMPTY: u32 = 1 << 17;

const MBOX_A2I_SEND0: u64 = 0x800;
const MBOX_A2I_SEND1: u64 = 0x808;

const MBOX_I2A_RECV0: u64 = 0x830;
const MBOX_I2A_RECV1: u64 = 0x838;

// This needs repr(C) while it remains shared with C code
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct AppleMboxMsg {
    pub data0: u64,
    pub data1: u32,
}

//#[derive(Debug)]
//struct AppleMboxClosure {
//    callback: RawAppleMboxRx,
//    arg: OwnedPtr,
//}
//
//impl AppleMboxClosure {
//    pub fn new<T>(callback: AppleMboxRx<T>, arg: OwnedPtr<T>) -> Self {
//        let callback = unsafe { transmute(callback) };
//        let arg = unsafe { transmute(arg) };
//        Self { callback, arg }
//    }
//
//    pub fn invoke(&mut self, msg: AppleMboxMsg) -> Result<()> {
//        (self.callback)(self.arg.clone(), msg)
//    }
//}

// This callback type's callsites are in rust so it doesn't need to be extern "C". If it were in C
// it would need to go through the KPI crate which would enforce the extern "C" to avoid a compiler
// error.
//pub type AppleMboxRx<T> = extern "C" fn(OwnedPtr<T>, AppleMboxMsg) -> Result<()>;
//type RawAppleMboxRx = extern "C" fn(OwnedPtr, AppleMboxMsg) -> Result<()>;

pub type AppleMboxRx<T> = fn(Ref<T>, AppleMboxMsg) -> Result<()>;
type RawAppleMboxRx = fn(AppleMboxMsg) -> Result<()>;

#[derive(Debug)]
pub struct AppleMboxSoftc {
    dev: device_t,
    irq: Irq,
    intr: Mutable<Intr>,
    write_msg: Mutable<WriteMsg>,
}

#[derive(Debug)]
struct Intr {
    i2a_ctrl: Register,
    i2a_recv: Register,
    //closure: Option<AppleMboxClosure>,
}

#[derive(Debug)]
struct WriteMsg {
    a2i_ctrl: Register,
    a2i_send: Register,
}

impl DeviceIf for AppleMboxDriver {
    type Softc = AppleMboxSoftc;

    fn device_probe(dev: device_t) -> Result<BusProbe> {
        if !ofw_bus_status_okay(dev) {
            return Err(ENXIO);
        }

        if !ofw_bus_is_compatible(dev, c"apple,asc-mailbox-v4") {
            return Err(ENXIO);
        }

        device_set_desc(dev, c"Apple Mailbox");

        Ok(BUS_PROBE_SPECIFIC)
    }

    fn device_attach(dev: device_t) -> Result<()> {
        let node = ofw_bus_get_node(dev);

        let rid = ofw_bus_find_string_index(node, c"interrupt-names", c"recv-not-empty").map_err(
            |e| {
                device_println!(
                    dev,
                    "could not find 'recv-not-empty' property in 'interrupt-names' {e}"
                );
                return ENXIO;
            },
        )?;
        let irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, rid, RF_ACTIVE).map_err(|e| {
            device_println!(dev, "could not allocate irq resource {e}");
            return ENXIO;
        })?;
        let irq = irq_res.as_irq().map_err(|e| {
            device_println!(dev, "tried to create non-irq resource {e}");
            return ENXIO;
        })?;
        let mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, 0, RF_ACTIVE).map_err(|e| {
            device_println!(dev, "could not allocate memory resource {e}");
            return ENXIO;
        })?;
        let mut regs = mem.split_registers::<4>().map_err(|e| {
            device_println!(dev, "tried to split non-memory resource");
            return ENXIO;
        })?;

        let a2i_ctrl = regs.take_register(MBOX_A2I_CTRL, 4).map_err(|e| {
            device_print!(dev, "failed to split {MBOX_A2I_CTRL:x?} from register");
            return ENXIO;
        })?;
        let a2i_send = regs.take_register(MBOX_A2I_SEND0, 0x10).map_err(|e| {
            device_print!(dev, "failed to split {MBOX_A2I_SEND0:x?} from register");
            return ENXIO;
        })?;
        let i2a_ctrl = regs.take_register(MBOX_I2A_CTRL, 4).map_err(|e| {
            device_print!(dev, "failed to split {MBOX_I2A_CTRL:x?} from register");
            return ENXIO;
        })?;
        let i2a_recv = regs.take_register(MBOX_I2A_RECV0, 0x10).map_err(|e| {
            device_print!(dev, "failed to split {MBOX_I2A_RECV0:x?} from register");
            return ENXIO;
        })?;

        let intr = Mutable::new(Intr {
            i2a_ctrl,
            i2a_recv,
            //closure: None,
        });

        let write_msg = Mutable::new(WriteMsg { a2i_ctrl, a2i_send });

        let xref = OF_xref_from_node(node);
        OF_device_register_xref(xref, dev);

        device_init_softc!(
            dev,
            AppleMboxSoftc {
                dev,
                irq,
                intr,
                write_msg,
            }
        );
        Ok(())
    }

    fn device_detach(_dev: device_t) -> Result<()> {
        unreachable!("device cannot be detached")
    }
}

impl AppleMboxDriver {
    // Get a mailbox device_t from the client's mboxes devicetree property. The mailbox must've
    // previously registered its devicetree node xref which happens when this driver attaches.
    pub fn get_mbox(&self, client: device_t) -> Result<device_t> {
        let client_node = ofw_bus_get_node(client);
        let mbox_xref = OF_getencprop_as_xref(client_node, c"mboxes")?;
        OF_device_from_xref(mbox_xref)
    }

    pub fn set_rx<T>(
        &self,
        mbox: device_t,
        client: device_t,
        //func: AppleMboxRx<T>,
        arg: Ref<T>,
    ) -> Result<()> {
        let sc = device_get_softc!(mbox);

        let flags = INTR_MPSAFE | INTR_TYPE_MISC;

        // Make sure the callback argument will live as long as the client's softc
        //if arg.get_owner() != client {
        //    return Err(EDOOFUS);
        //}

        //let mut intr = sc.intr.get_mut();
        //let func = unsafe { transmute(func) };
        //let func = monomorphize!(func);
        //let func = kpi::strip_type!(func);
        //intr.callback = Some(func);
        //intr.arg = arg.erase_lifetime().erase_generic();
        //intr.closure = Some(AppleMboxClosure::new(func, arg));
        //intr.arg = arg.get_var_ptr().cast::<c_void>();
        // Store a pointer to the argument's owner so we can rematerialize an OwnedRef out of a
        // thin pointer
        //drop(intr);

        bus_setup_intr(
            mbox,
            project!(sc->irq),
            flags,
            None,
            Some(apple_mbox_intr),
            sc,
        )?;
        //bus_setup_intr(
        //    mbox,
        //    &sc.irq,
        //    flags,
        //    None,
        //    Some(apple_mbox_intr),
        //    sc,
        //    project!(sc->intrhand),
        //)?;

        Ok(())
    }

    pub fn write_msg(&self, mbox: device_t, msg: AppleMboxMsg) -> Result<()> {
        let sc = device_get_softc!(mbox);

        let mut write_msg = sc.write_msg.get_mut();

        let mut ctrl = &mut write_msg.a2i_ctrl;
        if (bus_read_4!(ctrl, MBOX_A2I_CTRL) & MBOX_A2I_CTRL_FULL) != 0 {
            return Err(EBUSY);
        }

        let mut send = &mut write_msg.a2i_send;
        bus_write_8!(send, MBOX_A2I_SEND0, msg.data0);
        bus_write_8!(send, MBOX_A2I_SEND1, u64::from(msg.data1));
        Ok(())
    }
}

// extern "C" doesn't preclude making this a method of AppleMboxDriver but making it standalone
// makes the arguments to bus_setup_intr clearer
fn apple_mbox_intr(sc: Ref<AppleMboxSoftc>) {
    let mut intr = sc.intr.get_mut();

    while (bus_read_4!(&mut intr.i2a_ctrl, MBOX_I2A_CTRL) & MBOX_I2A_CTRL_EMPTY) == 0 {
        let msg = AppleMboxMsg {
            data0: bus_read_8!(&mut intr.i2a_recv, MBOX_I2A_RECV0),
            data1: bus_read_8!(&mut intr.i2a_recv, MBOX_I2A_RECV1) as u32,
        };
        //intr.closure.as_mut().unwrap().invoke(msg).unwrap();
    }
}

driver!(apple_mbox_driver, c"mbox", AppleMboxDriver, apple_mbox_methods,
    INTERFACES {
        device_probe apple_mbox_probe,
        device_attach apple_mbox_attach,
        device_detach apple_mbox_detach,
    }
);
