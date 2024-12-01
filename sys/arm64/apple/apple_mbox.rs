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
use core::ffi::c_void;
use core::ops::DerefMut;
use core::ptr::null_mut;
use core::sync::atomic::{AtomicPtr, Ordering};
use kpi::bindings::{INTR_MPSAFE, INTR_TYPE_MISC};
use kpi::bus::{Register, Resource};
use kpi::device::{Device, ProbeRes};
use kpi::sync::SpinLock;
use kpi::{dprintln, driver};

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

macro_rules! device_state {
    ($state:ident) => {
        #[derive(Debug)]
        pub struct $state(());
        unsafe impl UniqueOwner for $state {}
        impl SoftcInit for $state {}
    }
}

device_state!(Boot);
device_state!(Intr);

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
pub type AppleMboxRx<T> = fn(T, AppleMboxMsg) -> Result<()>;
pub type TypeErasedAppleMboxRx = fn(*mut c_void, AppleMboxMsg) -> Result<()>;

#[derive(Debug)]
pub struct AppleMboxSoftc<S = ()> {
    dev: Device,
    irq: UniqueCell<Option<Resource>, Boot, S>,
    intr_softc: UniqueCell<IntrSoftc, Intr, S>,
    write_msg_softc: SpinLock<WriteMsgSoftc>,
    callback: AtomicPtr<TypeErasedAppleMboxRx>,
    arg: AtomicPtr<c_void>,
    intrhand: FFICell<*mut c_void>,
}

#[derive(Debug)]
struct IntrSoftc {
    i2a_ctrl: I2ACtrl,
    i2a_recv: I2ARecv,
}

#[derive(Debug)]
struct WriteMsgSoftc {
    a2i_ctrl: A2ICtrl,
    a2i_send: A2ISend,
}

impl ManagesSoftc for Driver {
    type Softc<S> = AppleMboxSoftc<S>;
}

impl DeviceIf for Driver {
    fn device_probe(dev: &Device) -> Result<ProbeRes> {
        if !dev.ofw_bus_status_okay() {
            return Err(ENXIO);
        }

        if !dev.ofw_bus_is_compatible(c"apple,asc-mailbox-v4") {
            return Err(ENXIO);
        }

        dev.set_desc(c"Apple Mailbox");

        Ok(BUS_PROBE_SPECIFIC)
    }

    fn device_attach(dev: &mut Device) -> Result<AttachRes> {
        let node = dev.ofw_bus_get_node();

        let rid = node.ofw_bus_find_string_index(c"interrupt-names", c"recv-not-empty")?;
        let irq = UniqueCell::new(Some(dev.bus_alloc_resource(SYS_RES_IRQ, rid)?));
        let mut mem = dev.bus_alloc_resource(SYS_RES_MEMORY, 0)?;
        // generic parameters on take_register are inferred from the types of the result which are
        // determined by the definitions in AppleMboxSoftc
        let a2i_ctrl = mem.take_register()?;
        let a2i_send = mem.take_register()?;
        let i2a_ctrl = mem.take_register()?;
        let i2a_recv = mem.take_register()?;

        let xref = node.xref_from_node();
        dev.register_xref(xref);

        let res = Driver::init_softc(dev, AppleMboxSoftc {
            dev: dev.copy_ptr(),
            irq,
            intr_softc: UniqueCell::new(IntrSoftc {
                i2a_ctrl,
                i2a_recv,
            }),
            write_msg_softc: SpinLock::new(WriteMsgSoftc {
                a2i_ctrl,
                a2i_send,
            }, c"", None, NOWAIT),
            callback: AtomicPtr::new(core::ptr::null_mut()),
            arg: AtomicPtr::new(core::ptr::null_mut()),
            intrhand: FFICell::zeroed(),
        });

        Ok(res)
    }

    fn device_detach(dev: &mut Device) -> Result<()> {
        unreachable!("device cannot be detached")
    }
}

impl Driver {
    // Get a mailbox device_t from the client's mboxes devicetree property. The mailbox must've
    // previously registered its devicetree node xref which happens when this driver attaches.
    pub fn get_mbox(client: &Device) -> Result<Device> {
        let client_node = client.ofw_bus_get_node();
        let mbox_ref = client_node.get_xref_prop(c"mboxes")?;
        mbox_ref.device_from_xref()
    }

    pub fn set_rx<T>(mbox: &mut Device<Boot>, func: AppleMboxRx<T>, arg: &T) -> Result<()> {
        let sc = Driver::get_softc_with_state(mbox);

        let func = unsafe { transmute(func) };
        sc.callback.store(func, Ordering::Relaxed);

        let arg = arg as *const T as *const c_void as *mut c_void;
        sc.arg.store(arg, Ordering::Relaxed);

        let irq = sc.irq.get_mut().take().ok_or(EDOOFUS)?;
        let flags = INTR_MPSAFE | INTR_TYPE_MISC;
        let intrhand = sc.intrhand.as_ptr();

        Driver::bus_setup_intr(mbox, irq, flags, None, Some(Self::intr), intrhand)?;
        Ok(())
    }

    extern "C" fn intr(sc: &AppleMboxSoftc<Intr>) {
        let intr_sc = sc.intr_softc.get_mut();

        let mut ctrl = &mut intr_sc.i2a_ctrl;
        let mut recv = &mut intr_sc.i2a_recv;
        while (ctrl.read_4(0) & MBOX_I2A_CTRL_EMPTY) == 0 {
            let msg = AppleMboxMsg {
                data0: recv.read_8(0),
                data1: recv.read_8(8) as u32,
            };
            let func = sc.callback.load(Ordering::Relaxed);
            let arg = sc.arg.load(Ordering::Relaxed);
            if !func.is_null() {
                unsafe {
                    (*func)(arg, msg).unwrap();
                }
            } else {
                dprintln!(sc.dev, "Received RTKit msg w/o callback installed\n");
            }
        }
    }


    pub fn write_msg(mbox: &Device<Boot>, msg: &AppleMboxMsg) -> Result<()> {
        let mut write_msg_sc = Driver::get_softc(mbox).write_msg_softc.lock();
        let mut ctrl = &mut write_msg_sc.a2i_ctrl;
        if (ctrl.read_4(0) & MBOX_A2I_CTRL_FULL) != 0 {
            return Err(EBUSY);
        }
        let mut send = &mut write_msg_sc.a2i_send;
        send.write_8(0, msg.data0);
        send.write_8(8, u64::from(msg.data1));
        Ok(())
    }
}

driver!(apple_mbox_driver, c"mbox", apple_mbox_methods,
    device_probe apple_mbox_probe,
    device_attach apple_mbox_attach,
    device_detach apple_mbox_detach
);
