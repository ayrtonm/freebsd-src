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
use kpi::device::BusProbe;
use kpi::driver;
use kpi::ffi::{FatPtr, OwnedPtr, Ptr, RefCountData, RefCounted};
use kpi::sync::Mutable;

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

pub type AppleMboxRx<T> = fn(&'static T, AppleMboxMsg) -> Result<()>;
type RawAppleMboxRx = fn(*mut c_void, *const c_void, AppleMboxMsg) -> Result<()>;

#[derive(Debug)]
pub struct AppleMboxSoftc {
    dev: device_t,
    irq: Irq,
    intr: Mutable<Intr>,
    write_msg: Mutable<WriteMsg>,
}

#[derive(Debug)]
struct AppleMboxCallback {
    user_func: *mut c_void,
    invoke: RawAppleMboxRx,
    arg: *const c_void,
}

#[derive(Debug)]
struct Intr {
    i2a_ctrl: Register,
    i2a_recv: Register,
    callback: Option<AppleMboxCallback>,
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

    fn device_attach(uninit_sc: &mut Uninit<AppleMboxSoftc>, dev: device_t) -> Result<()> {
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
            callback: None,
        });

        let write_msg = Mutable::new(WriteMsg { a2i_ctrl, a2i_send });

        let xref = OF_xref_from_node(node);
        OF_device_register_xref(xref, dev);

        uninit_sc.init(AppleMboxSoftc {
            dev,
            irq,
            intr,
            write_msg,
        });
        Ok(())
    }

    fn device_detach(_sc: &RefCounted<AppleMboxSoftc>, _dev: device_t) -> Result<()> {
        unreachable!("device cannot be detached")
    }
}

impl AppleMboxDriver {
    // Get a mailbox device_t from the client's mboxes devicetree property. The mailbox must've
    // previously registered its devicetree node xref which happens when this driver attaches.
    pub fn get_mbox(client: device_t) -> Result<device_t> {
        let client_node = ofw_bus_get_node(client);
        let mbox_xref = OF_getencprop_as_xref(client_node, c"mboxes")?;
        OF_device_from_xref(mbox_xref)
    }

    pub fn set_rx<T>(
        mbox: device_t,
        client: device_t,
        func: AppleMboxRx<T>,
        arg: FatPtr<T>,
    ) -> Result<()> {
        fn invoke_user_func<T: 'static>(
            user_func: *mut c_void,
            arg: *const c_void,
            msg: AppleMboxMsg,
        ) -> Result<()> {
            let arg = unsafe { arg.cast::<T>().as_ref().unwrap() };
            let user_func = unsafe { transmute::<*mut c_void, AppleMboxRx<T>>(user_func) };
            user_func(arg, msg)
        }

        let sc = device_get_softc!(mbox);

        let user_func = unsafe { transmute::<AppleMboxRx<T>, *mut c_void>(func) };
        let arg_ptr = OwnedPtr::leak(arg) as *const T;

        sc.intr.get_mut().callback = Some(AppleMboxCallback {
            user_func,
            invoke: invoke_user_func::<T>,
            arg: arg_ptr.cast::<c_void>(),
        });

        let flags = INTR_MPSAFE | INTR_TYPE_MISC;
        bus_setup_intr(
            mbox,
            &sc.irq,
            flags,
            None,
            Some(AppleMboxDriver::handle_intr),
            sc.clone(),
        )
    }

    pub fn write_msg(mbox: device_t, msg: AppleMboxMsg) -> Result<()> {
        let sc = device_get_softc!(mbox);

        let mut write_msg = sc.write_msg.get_mut();

        let mut ctrl = &mut write_msg.a2i_ctrl;
        if (bus_read_4!(ctrl, MBOX_A2I_CTRL) & MBOX_A2I_CTRL_FULL) != 0 {
            device_println!(sc.dev, "mailbox full");
            return Err(EBUSY);
        }

        let mut send = &mut write_msg.a2i_send;
        bus_write_8!(send, MBOX_A2I_SEND0, msg.data0);
        bus_write_8!(send, MBOX_A2I_SEND1, u64::from(msg.data1));
        Ok(())
    }

    extern "C" fn handle_intr(sc: &RefCounted<AppleMboxSoftc>) {
        let mut intr = sc.intr.get_mut();

        while (bus_read_4!(&mut intr.i2a_ctrl, MBOX_I2A_CTRL) & MBOX_I2A_CTRL_EMPTY) == 0 {
            let msg = AppleMboxMsg {
                data0: bus_read_8!(&mut intr.i2a_recv, MBOX_I2A_RECV0),
                data1: bus_read_8!(&mut intr.i2a_recv, MBOX_I2A_RECV1) as u32,
            };
            let callback = intr.callback.as_mut().unwrap();
            (callback.invoke)(callback.user_func, callback.arg, msg).unwrap();
        }
    }
}

driver!(apple_mbox_driver, c"mbox", AppleMboxDriver, apple_mbox_methods,
    INTERFACES {
        device_probe apple_mbox_probe,
        device_attach apple_mbox_attach,
        device_detach apple_mbox_detach,
    }
);
