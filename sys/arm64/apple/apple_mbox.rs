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
use kpi::bindings::device_t;
use kpi::bus::{Irq, Register};
use kpi::device::{BusProbe, DeviceIf, Device};
use core::pin::Pin;
use kpi::ffi::{Ptr, Uninit, Loan, Lease};
use kpi::ofw::XRef;
use kpi::prelude::*;
use kpi::sync::Checked;
use kpi::{define_driver, proj};

const MBOX_A2I_CTRL: u64 = 0x110;
const MBOX_A2I_CTRL_FULL: u32 = 1 << 16;

const MBOX_I2A_CTRL: u64 = 0x114;
const MBOX_I2A_CTRL_EMPTY: u32 = 1 << 17;

const MBOX_A2I_SEND0: u64 = 0x800;
const MBOX_A2I_SEND1: u64 = 0x808;

const MBOX_I2A_RECV0: u64 = 0x830;
const MBOX_I2A_RECV1: u64 = 0x838;

// Only for some mild type-safety
#[derive(Debug, Copy, Clone)]
pub struct MboxDevice(Device<'static>);

// This needs repr(C) while it appears in an extern "C" function signature
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct AppleMboxMsg {
    pub data0: u64,
    pub data1: u32,
}

// These two type aliases must be kept ABI-compatible to justify the function transmute below.
// TODO: Find some way to avoid storing the *mut c_void for each client softc in the mbox. This
// would allow removing this cursed transmute.
pub type AppleMboxRx<T> = extern "C" fn(Loan<T>, AppleMboxMsg);
type RawAppleMboxRx = extern "C" fn(*mut c_void, AppleMboxMsg);

#[derive(Debug)]
pub struct AppleMboxSoftc {
    irq: Irq,
    intr_ctx: Checked<IntrCtx>,
    write_msg: Checked<WriteMsg>,
}

#[derive(Debug)]
struct IntrCtx {
    i2a_ctrl: Register,
    i2a_recv: Register,
    callback: Option<(RawAppleMboxRx, Ptr<c_void>)>,
}

#[derive(Debug)]
struct WriteMsg {
    a2i_ctrl: Register,
    a2i_send: Register,
}

impl DeviceIf for AppleMboxDriver {
    type Softc = AppleMboxSoftc;

    fn device_probe(dev: Device) -> Result<BusProbe> {
        if !ofw_bus_status_okay(dev) {
            return Err(ENXIO);
        }

        if !ofw_bus_is_compatible(dev, c"apple,asc-mailbox-v4") {
            return Err(ENXIO);
        }

        device_set_desc(dev, c"Apple Mailbox");

        Ok(BUS_PROBE_SPECIFIC)
    }

    fn device_attach(uninit_sc: Uninit<AppleMboxSoftc>) -> Result<()> {
        let dev = uninit_sc.device();
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
        let irq = irq_res.into_irq().map_err(|e| {
            device_println!(dev, "tried to create non-irq resource {e}");
            return ENXIO;
        })?;
        let mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, 0, RF_ACTIVE).map_err(|e| {
            device_println!(dev, "could not allocate memory resource {e}");
            return ENXIO;
        })?;
        let mut regs = mem.split_registers::<4>().map_err(|e| {
            device_println!(dev, "tried to split non-memory resource {e}");
            return ENXIO;
        })?;

        let a2i_ctrl = regs.take_register(MBOX_A2I_CTRL, 4).map_err(|e| {
            device_println!(dev, "failed to split {MBOX_A2I_CTRL:x?} from register {e}");
            return ENXIO;
        })?;
        let a2i_send = regs.take_register(MBOX_A2I_SEND0, 0x10).map_err(|e| {
            device_println!(dev, "failed to split {MBOX_A2I_SEND0:x?} from register {e}");
            return ENXIO;
        })?;
        let i2a_ctrl = regs.take_register(MBOX_I2A_CTRL, 4).map_err(|e| {
            device_println!(dev, "failed to split {MBOX_I2A_CTRL:x?} from register {e}");
            return ENXIO;
        })?;
        let i2a_recv = regs.take_register(MBOX_I2A_RECV0, 0x10).map_err(|e| {
            device_println!(dev, "failed to split {MBOX_I2A_RECV0:x?} from register {e}");
            return ENXIO;
        })?;

        let intr_ctx = Checked::new(IntrCtx {
            i2a_ctrl,
            i2a_recv,
            callback: None,
        });

        let write_msg = Checked::new(WriteMsg { a2i_ctrl, a2i_send });

        // Register the device with the xref for its devicetree node. Clients devices will have an
        // mbox property in their devicetree nodes which is a xref to their corresponding mailbox.
        let xref = OF_xref_from_node(node);
        OF_device_register_xref(xref, dev);

        uninit_sc.init(AppleMboxSoftc {
            irq,
            intr_ctx,
            write_msg,
        });
        Ok(())
    }
}

// Get a mailbox device_t from the client's mboxes devicetree property. The mailbox must've
// previously registered its devicetree node xref which happens when this driver attaches.
pub fn apple_mbox_get_dev(client: Device) -> Result<MboxDevice> {
    let client_node = ofw_bus_get_node(client);

    // SAFETY: This devicetree property is a u32 which is intended to be interpreted as an XRef
    let mbox_xref = unsafe { OF_getencprop_unchecked::<XRef>(client_node, c"mboxes")? };

    let mbox_dev_ptr = OF_device_from_xref(mbox_xref)?;

    // SAFETY: This mailbox doesn't implement device_detach so it can be used indefinitely
    let mbox_dev = unsafe { Device::new_unchecked(mbox_dev_ptr) };

    // Sanity check the device is actually managed by this mailbox driver
    if !device_matches_driver::<AppleMboxDriver>(mbox_dev) {
        return Err(ENXIO);
    }
    assert!(device_is_undetachable(mbox_dev));

    Ok(MboxDevice(mbox_dev))
}

pub fn apple_mbox_set_rx<T>(mbox: MboxDevice, func: AppleMboxRx<T>, arg: Lease<T>) -> Result<()> {
    let sc = device_get_softc::<AppleMboxDriver>(mbox.0);

    let func = unsafe { transmute::<Option<AppleMboxRx<T>>, RawAppleMboxRx>(Some(func)) };
    let (arg_ptr, _count_ptr) = Lease::into_raw(arg);

    sc.intr_ctx.get_mut().callback = Some((func, Ptr::new(arg_ptr.cast::<c_void>())));

    let flags = INTR_MPSAFE.0 | INTR_TYPE_MISC.0;
    bus_setup_intr(
        mbox.0,
        proj!(&sc.irq),
        flags,
        None,
        Some(apple_mbox_handle_intr),
        sc.lease(),
    )
}

extern "C" fn apple_mbox_handle_intr(sc: Loan<AppleMboxSoftc>) {
    let mut intr = sc.intr_ctx.get_mut();

    while (bus_read_4!(&mut intr.i2a_ctrl, MBOX_I2A_CTRL) & MBOX_I2A_CTRL_EMPTY) == 0 {
        let msg = AppleMboxMsg {
            data0: bus_read_8!(&mut intr.i2a_recv, MBOX_I2A_RECV0),
            data1: bus_read_8!(&mut intr.i2a_recv, MBOX_I2A_RECV1) as u32,
        };
        let callback = intr.callback.as_mut().unwrap();
        (callback.0)(callback.1.as_ptr(), msg);
    }
}

impl AppleMboxDriver {
    pub fn write_msg(mbox: MboxDevice, msg: AppleMboxMsg) -> Result<()> {
        let sc = device_get_softc::<Self>(mbox.0);

        let mut write_msg = sc.write_msg.get_mut();

        let mut ctrl = &mut write_msg.a2i_ctrl;
        if (bus_read_4!(ctrl, MBOX_A2I_CTRL) & MBOX_A2I_CTRL_FULL) != 0 {
            device_println!(sc.device(), "mailbox full");
            return Err(EBUSY);
        }

        let mut send = &mut write_msg.a2i_send;
        bus_write_8!(send, MBOX_A2I_SEND0, msg.data0);
        bus_write_8!(send, MBOX_A2I_SEND1, u64::from(msg.data1));
        Ok(())
    }
}

define_driver!(
    static apple_mbox_driver: AppleMboxDriver = {
        name: c"mbox",
    }
    static apple_mbox_methods = {
        device_probe: apple_mbox_probe,
        device_attach: apple_mbox_attach,
    }
);
