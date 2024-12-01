/*	$OpenBSD: rtkit.c,v 1.6 2022/09/03 19:04:28 kettenis Exp $	*/
/*
 * Copyright (c) 2021 Mark Kettenis <kettenis@openbsd.org>
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
#![feature(allocator_api)]

extern crate alloc;

use core::ffi::{c_int, c_void};
use core::mem::MaybeUninit;
use core::marker::PhantomData;
use core::ptr::{addr_of_mut, null_mut};
use core::convert::From;
use core::sync::atomic::{AtomicU16, Ordering};
use kpi::allocator::NOWAIT;
use kpi::device::Device;
use kpi::prelude::*;
use kpi::taskq;
use kpi::taskq::Task;
use kpi::{bindings, enum_c_macros};

use apple_mbox::{AppleMboxMsg, AppleMboxRx, Boot, Intr};

#[repr(u16)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum PwrState {
    Sleep = 0x0001,
    Quiesced = 0x0010,
    On = 0x0020,
}

impl Into<u16> for PwrState {
    fn into(self) -> u16 {
        self as u16
    }
}

#[repr(C)]
#[derive(Debug)]
pub struct RTKit<S = ()> {
    client: Device,
    mbox: Device<S>,

    iop_pwr_state: Sleepable<AtomicU16>,
    ap_pwr_state: Sleepable<AtomicU16>,
    verbose: bool,
    noalloc: bool,
    ep_map: u64,
    callbacks: [Option<(AppleMboxRx, *mut c_void)>; 32],
}

// RTKitTask is a subclass of `struct task`
type RTKitTask = SubClass<Task, RTKitTaskFields>;

#[derive(Debug)]
struct RTKitTaskFields {
    rtkit: *mut RTKit,
    msg: AppleMboxMsg,
}

impl RTKit {
    pub fn new(client: Device) -> Result<Self> {
        let mbox = apple_mbox::Driver::get_mbox(&client)?;
        let iop_pwr_state = Sleepable::new(AtomicU16::new(PwrState::Sleep.into()));
        let ap_pwr_state = Sleepable::new(AtomicU16::new(PwrState::Quiesced.into()));
        Ok(Self {
            client,
            mbox,
            iop_pwr_state,
            ap_pwr_state,
            verbose: false,
            noalloc: false,
            ep_map: 0,
            callbacks: [None; 32],
        })
    }
    fn rx_callback(client: Device, msg: AppleMboxMsg) -> Result<()> {
        Ok(())
    }
}

impl RTKit<Boot> {
    pub fn boot(&mut self) -> Result<()> {
        let arg = self.client.copy_ptr();
        apple_mbox::Driver::set_rx(&mut self.mbox, RTKit::rx_callback, arg)
    }

    pub fn set_iop_pwr_state(&self, pwr_state: PwrState) -> Result<()> {
        let pwr_state = pwr_state.into();
        // If already in the correct power state do nothing
        if self.iop_pwr_state.load(Ordering::Relaxed) == pwr_state {
            return Ok(());
        }

        // Try setting the power state
        let msg = EpTxMsg::Mgmt(MgmtTxMsg::IopPwrState { pwr_state });
        apple_mbox::Driver::write_msg(&self.mbox, &msg.as_apple_mbox_msg())?; 

        // If the power state hasn't changed sleep
        if self.iop_pwr_state.load(Ordering::Relaxed) != pwr_state {
            self.iop_pwr_state.tsleep_in_hz(bindings::PWAIT, c"ioppwr", 1)
        } else {
            Ok(())
        }
    }

    pub fn set_ap_pwr_state(&self, pwr_state: PwrState) -> Result<()> {
        let pwr_state = pwr_state.into();
        if self.ap_pwr_state.load(Ordering::Relaxed) == pwr_state {
            return Ok(());
        }

        let msg = EpTxMsg::Mgmt(MgmtTxMsg::ApPwrState { pwr_state });
        apple_mbox::Driver::write_msg(&self.mbox, &msg.as_apple_mbox_msg())?;

        if self.ap_pwr_state.load(Ordering::Relaxed) != pwr_state {
            self.ap_pwr_state.tsleep_in_hz(bindings::PWAIT, c"appwr", 1)
        } else {
            Ok(())
        }
    }
}

//fn rx_callback(/*rtkit: *mut RTKit<Intr>*/mut dev: Device, msg: AppleMboxMsg) -> Result<()> {
//    /*
//    let ctx = RTKitTaskFields { rtkit, msg };
//    // Put the task and its context in a `Box` (i.e. on the heap). The `struct task` is
//    // zero-initialized while the other context fields are initialized with the arguments
//    let mut task = Box::new_in(RTKitTask::new(ctx), NOWAIT);
//    // Set the task's own address as the context and enqueue on taskqueue_thread with the
//    // callback rx_task. `enqueue_with_self` consumes `task` so it can't be used again in this
//    // function and internally `enqueue_with_self` calls `Box::into_raw` to avoid freeing `task`
//    // itself. When the callback is invoked it's passed the boxed task as an argument so it
//    // either explicitly passes ownership of the Box to something else or implicitly drops it at
//    // the end of the function (i.e. frees the allocated task and context)
//    task.enqueue_with_self(rx_task, taskq::thread())?;
//    */
//    Ok(())
//}

fn mbox_send_from_task(ctx: &RTKitTask, msg: EpTxMsg) -> Result<()> {
    /*
    let mbox = get_field!(ctx.rtkit, mbox).flatten();
    mbox_send(mbox, msg)
    */
    todo!("")
}
fn mbox_send(mut mbox: Device<Boot>, msg: EpTxMsg) -> Result<()> {
    apple_mbox::Driver::write_msg(&mut mbox, &msg.as_apple_mbox_msg())
}

const MSG_TYPE_SHIFT: u64 = 52;
const OSLOG_TYPE_SHIFT: u64 = 56;

#[derive(Debug, Copy, Clone)]
pub enum MgmtRxMsg {
    Hello {
        minver: u16,
        maxver: u16,
    },
    IopPwrStateAck(u16),
    ApPwrStateAck(u16),
    EpMap {
        base: u8, // only 3 bits
        bitmap: u32,
        last: bool,
    },
    Unknown(u8),
}

impl MgmtRxMsg {
    pub fn new(data0: u64) -> Self {
        match mgmt::msg_type(data0) {
            mgmt::HELLO => {
                let minver = mgmt::hello_minver(data0);
                let maxver = mgmt::hello_maxver(data0);
                Self::Hello { minver, maxver }
            }
            mgmt::IOP_PWR_STATE_ACK => Self::IopPwrStateAck(mgmt::pwr_state(data0)),
            // next one is not a typo
            mgmt::AP_PWR_STATE => Self::ApPwrStateAck(mgmt::pwr_state(data0)),
            mgmt::EP_MAP => {
                let base = mgmt::ep_map_base(data0);
                let bitmap = mgmt::ep_map_bitmap(data0);
                let last = mgmt::ep_map_last(data0);
                Self::EpMap { base, bitmap, last }
            }
            x => Self::Unknown(x),
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub enum MgmtTxMsg {
    HelloAck { minver: u16, maxver: u16 },
    IopPwrState { pwr_state: u16 },
    ApPwrState { pwr_state: u16 },
    StartEp { ep: u32 },
    EpMap { base: u8, last: bool },
}

impl MgmtTxMsg {
    pub fn as_apple_mbox_msg(self) -> AppleMboxMsg {
        let data0;
        match self {
            Self::HelloAck { minver, maxver } => {
                data0 = mgmt::hello_ack(minver, maxver);
            }
            Self::IopPwrState { pwr_state } => {
                data0 = mgmt::iop_pwr_state(pwr_state);
            }
            Self::ApPwrState { pwr_state } => {
                data0 = mgmt::ap_pwr_state(pwr_state);
            }
            Self::EpMap { base, last } => {
                data0 = mgmt::ep_map_reply(base, last);
            }
            Self::StartEp { ep } => {
                data0 = mgmt::start_ep(ep);
            }
        }
        AppleMboxMsg {
            data0,
            data1: endpoint::MGMT,
        }
    }
}

mod mgmt {
    use super::MSG_TYPE_SHIFT;

    pub const HELLO: u8 = 1;
    pub const HELLO_ACK: u8 = 2;
    pub const START_EP: u8 = 5;
    pub const IOP_PWR_STATE: u8 = 6;
    pub const IOP_PWR_STATE_ACK: u8 = 7;
    pub const EP_MAP: u8 = 8;
    pub const AP_PWR_STATE: u8 = 11;

    const MAXVER_SHIFT: u64 = 16;
    const EP_MAP_BASE_SHIFT: u64 = 32;
    const EP_MAP_LAST_SHIFT: u64 = 51;
    const EP_MAP_MORE: u64 = 1 << 0;
    const START_EP_SHIFT: u64 = 32;
    const START_EP_START: u64 = 1 << 1;

    // All implicit truncations using `as` in the following functions are intentional. I'm
    // intentionally not including the `& 0xffff...` to avoid accidentally overtruncating the cases
    // where the number of `f`s is not immediately obvious
    pub fn msg_type(x: u64) -> u8 {
        (x >> MSG_TYPE_SHIFT) as u8
    }

    pub fn hello_minver(x: u64) -> u16 {
        x as u16
    }

    pub fn hello_maxver(x: u64) -> u16 {
        (x >> MAXVER_SHIFT) as u16
    }

    pub fn hello_ack(minver: u16, maxver: u16) -> u64 {
        (minver as u64) | (maxver as u64) << MAXVER_SHIFT | (HELLO_ACK as u64) << MSG_TYPE_SHIFT
    }

    pub fn pwr_state(x: u64) -> u16 {
        x as u16
    }

    pub fn iop_pwr_state(pwr_state: u16) -> u64 {
        (pwr_state as u64) | (IOP_PWR_STATE as u64) << MSG_TYPE_SHIFT
    }

    pub fn ap_pwr_state(pwr_state: u16) -> u64 {
        (pwr_state as u64) | (AP_PWR_STATE as u64) << MSG_TYPE_SHIFT
    }

    pub fn ep_map_base(x: u64) -> u8 {
        (x >> EP_MAP_BASE_SHIFT) as u8
    }

    pub fn ep_map_bitmap(x: u64) -> u32 {
        x as u32
    }

    pub fn ep_map_last(x: u64) -> bool {
        x & (1 << EP_MAP_LAST_SHIFT) != 0
    }

    pub fn ep_map_reply(base: u8, last: bool) -> u64 {
        let res = (base as u64) << EP_MAP_BASE_SHIFT | (EP_MAP as u64) << MSG_TYPE_SHIFT;
        if last {
            res | (1 << EP_MAP_LAST_SHIFT)
        } else {
            res | EP_MAP_MORE
        }
    }

    pub fn start_ep(ep: u32) -> u64 {
        (ep as u64) << START_EP_SHIFT | START_EP_START | (START_EP as u64) << MSG_TYPE_SHIFT
    }
}

/* versions we support */
const MINVER: u16 = 11;
const MAXVER: u16 = 12;

fn handle_mgmt(ctx: &RTKitTask, msg: MgmtRxMsg) -> Result<()> {
    /*
    match msg {
        MgmtRxMsg::Hello { minver, maxver } => {
            if minver > MAXVER {
                return Err(EINVAL);
            }
            if maxver < MINVER {
                return Err(EINVAL);
            }
            let mut ver = maxver;
            if MAXVER < ver {
                ver = MAXVER;
            }
            let ack = MgmtTxMsg::HelloAck {
                minver: ver,
                maxver: ver,
            };
            mbox_send_from_task(ctx, EpTxMsg::Mgmt(ack))
        }
        MgmtRxMsg::IopPwrStateAck(pwr_state) => {
            let mut iop_pwr_state = get_field!(ctx.rtkit, iop_pwr_state);
            unsafe {
                iop_pwr_state.write(pwr_state);
            }
            wakeup(iop_pwr_state);
            Ok(())
        }
        MgmtRxMsg::ApPwrStateAck(pwr_state) => {
            let mut ap_pwr_state = get_field!(ctx.rtkit, ap_pwr_state);
            unsafe {
                ap_pwr_state.write(pwr_state);
            }
            wakeup(ap_pwr_state);
            Ok(())
        }
        MgmtRxMsg::EpMap { base, bitmap, last } => {
            let old_bitmap = unsafe { get_field!(ctx.rtkit, ep_map).read() };
            let new_bits = (bitmap as u64) << (base * 32);
            let new_bitmap = old_bitmap | new_bits;
            unsafe { get_field!(ctx.rtkit, ep_map).write(new_bitmap) }
            let reply = MgmtTxMsg::EpMap { base, last };
            mbox_send_from_task(ctx, EpTxMsg::Mgmt(reply))?;
            if last {
                let dev = get_field!(ctx.rtkit, client).flatten();
                // range bounds are not a typo, bit 0 is not an endpoint
                for ep in 1..32 {
                    if new_bitmap & (1 << ep) == 0 {
                        continue;
                    }
                    match ep {
                        endpoint::MGMT => (),
                        endpoint::CRASHLOG
                        | endpoint::SYSLOG
                        | endpoint::DEBUG
                        | endpoint::IOREPORT
                        | endpoint::OSLOG
                        | endpoint::TRACEKIT => {
                            dprintln!(dev, "starting rtkit endpoint {ep:?}");
                            let start_ep = MgmtTxMsg::StartEp { ep };
                            mbox_send_from_task(ctx, EpTxMsg::Mgmt(start_ep))?;
                        }
                        _ => {
                            dprintln!(dev, "skipping endpoint {ep:?}");
                        }
                    }
                }
            }
            Ok(())
        }
        MgmtRxMsg::Unknown(_) => todo!(""),
    }
    */
    todo!("")
}

#[derive(Debug, Copy, Clone)]
enum EpRxMsg {
    Mgmt(MgmtRxMsg),
    Crashlog,
    Syslog,
    Debug,
    IOReport,
    OSlog,
    Tracekit,
    Custom(u32),
    Unknown(u32),
}

#[derive(Debug, Copy, Clone)]
enum EpTxMsg {
    Mgmt(MgmtTxMsg),
}

// matching const in the same module is kinda ugly since the "obvious" way to do it does something
// unexpected (but will not compile when using #![deny(unreachable_patterns)]) so let's put them in
// a module
mod endpoint {
    pub const MGMT: u32 = 0;
    pub const CRASHLOG: u32 = 1;
    pub const SYSLOG: u32 = 2;
    pub const DEBUG: u32 = 3;
    pub const IOREPORT: u32 = 4;
    pub const OSLOG: u32 = 8;
    pub const TRACEKIT: u32 = 10;
}

impl EpRxMsg {
    pub fn new(msg: AppleMboxMsg) -> Self {
        match msg.data1 {
            endpoint::MGMT => Self::Mgmt(MgmtRxMsg::new(msg.data0)),
            endpoint::CRASHLOG => Self::Crashlog,
            endpoint::SYSLOG => Self::Syslog,
            endpoint::DEBUG => Self::Debug,
            endpoint::IOREPORT => Self::IOReport,
            endpoint::OSLOG => Self::OSlog,
            endpoint::TRACEKIT => Self::Tracekit,
            x => {
                if x >= 32 && x < 64 {
                    EpRxMsg::Custom(x)
                } else {
                    EpRxMsg::Unknown(x)
                }
            }
        }
    }
}

impl EpTxMsg {
    pub fn as_apple_mbox_msg(self) -> AppleMboxMsg {
        match self {
            Self::Mgmt(msg) => msg.as_apple_mbox_msg(),
        }
    }
}

extern "C" fn rx_task(ctx: Box<RTKitTask>, pending: c_int) {
    let ep = EpRxMsg::new(ctx.msg);
    let res = match ep {
        EpRxMsg::Mgmt(msg) => handle_mgmt(&ctx, msg),
        EpRxMsg::Crashlog => todo!(""),
        EpRxMsg::Syslog => todo!(""),
        EpRxMsg::Debug => todo!(""),
        EpRxMsg::IOReport => todo!(""),
        EpRxMsg::OSlog => todo!(""),
        EpRxMsg::Tracekit => todo!(""),
        EpRxMsg::Custom(ep) => todo!(""),
        EpRxMsg::Unknown(ep) => todo!(""),
    };
    /*
    if let Err(err) = res {
        // We have one extra level of indirection (i.e. `device_t *` or equivalently
        // `struct _device **` so we need to flatten the `Ptr<Ptr<_device>>` into `Ptr<_device>`.
        let client_dev: *mut Device = get_field!(ctx.rtkit, client);
        dprintln!(
            client_dev.flatten(),
            "Error ({err:?}) while handling RTKit message from endpoint {ep:?}"
        );
    }
    */
}

#[no_mangle]
extern "C" fn rtkit_init(dev: Device, noalloc: bool) -> *mut RTKit {
    //RTKit::new(dev/*, noalloc*/).unwrap();
    todo!("")
}

#[no_mangle]
unsafe extern "C" fn rtkit_boot(rtkit: *mut RTKit) -> c_int {
    //RTKit::boot(rtkit).unwrap();
    todo!("")
}
