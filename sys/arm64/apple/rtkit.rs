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
#![allow(unused)]

use kpi::prelude::*;
use core::ffi::c_int;
use core::ops::Deref;
use core::sync::atomic::{AtomicU64, AtomicU16, Ordering};
use kpi::device::Device;
use kpi::taskq::Task;
use kpi::sleep::Sleepable;
use kpi::sync::Arc;

use apple_mbox::{apple_mbox_driver, AppleMboxMsg};

type Box<T, M> = kpi::boxed::Box<T, M>;

#[repr(u16)]
#[derive(Debug)]
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
pub struct RTKit {
    client: Device,
    mbox: Device,

    iop: Sleepable<AtomicU16>,
    ap: Sleepable<AtomicU16>,

    verbose: bool,
    noalloc: bool,
    ep_map: AtomicU64,
    // TODO: Use Checked here
    //callbacks: [Option<(AppleMboxRx<RTKit>, AtomicPtr<c_void>)>; 32],
}

#[derive(Debug)]
struct RTKitTaskCtx {
    rtkit: Arc<RTKit, M_DEVBUF>,
    msg: AppleMboxMsg,
}

pub trait ManagesRTKit: DeviceIf + IsDriver + Sized {
    fn get_rtkit(sc: &Self::Softc) -> &Arc<RTKit, M_DEVBUF>;

    fn rtkit_boot(client: Device) -> Result<()> {
        let sc = device_get_softc!(client);
        let rtkit = Self::get_rtkit(sc.deref());
        apple_mbox_driver.set_rx(rtkit.mbox, rtkit.client, /*self,*/ Self::rx_callback, sc.deref())
    }

    fn rx_callback(sc: &Self::Softc, msg: AppleMboxMsg) -> Result<()> {
        let rtkit = Self::get_rtkit(sc.deref()).clone();
        let ctx = RTKitTaskCtx {
            rtkit,
            msg,
        };
        let mut task = Task::new(ctx);
        task.init(rx_task);
        let boxed_task = Box::try_new(task, M_NOWAIT).inspect_err(|e| {
            println!("failed to allocate memory for task {e}");
        })?;
        taskqueue_enqueue(taskqueue_thread(), boxed_task)
    }
}

extern "C" fn rx_task(ctx: Box<Task<RTKitTaskCtx>, M_DEVBUF>, pending: c_int) {
    let ep = EpRxMsg::new(ctx.msg);
    let _res = match ep {
        EpRxMsg::Mgmt(msg) => handle_mgmt(&ctx, msg),
        EpRxMsg::Crashlog => todo!(""),
        EpRxMsg::Syslog => todo!(""),
        EpRxMsg::Debug => todo!(""),
        EpRxMsg::IOReport => todo!(""),
        EpRxMsg::OSlog => todo!(""),
        EpRxMsg::Tracekit => todo!(""),
        EpRxMsg::Custom(_ep) => todo!(""),
        EpRxMsg::Unknown(_ep) => todo!(""),
    };
}

impl RTKit {
    pub fn new(client: Device) -> Result<Self> {
        let mbox = apple_mbox_driver.get_mbox(client)?;
        let iop = Sleepable::new(AtomicU16::new(PwrState::Sleep.into()));
        let ap = Sleepable::new(AtomicU16::new(PwrState::Quiesced.into()));
        Ok(Self {
            client,
            mbox,

            iop,
            ap,

            verbose: false,
            noalloc: false,
            ep_map: AtomicU64::new(0),
            //callbacks: [const { None }; 32],
        })
    }

    pub fn boot(&self) -> Result<()> {
        todo!("")
    }
    pub fn set_iop(&self, pwr_state: PwrState) -> Result<()> {
        let pwr_state = pwr_state.into();
        // If already in the correct power state do nothing
        if self.iop.load(Ordering::Relaxed) == pwr_state {
            return Ok(());
        }

        // Try setting the power state
        let msg = EpTxMsg::Mgmt(MgmtTxMsg::IopPwrState { pwr_state });
        apple_mbox_driver.write_msg(self.mbox, msg.as_apple_mbox_msg())?;

        // If the power state hasn't changed then sleep
        if self.iop.load(Ordering::Relaxed) != pwr_state {
            tsleep(&self.iop, bindings::PWAIT, c"ioppwr", 1 * unsafe { bindings::hz })
        } else {
            Ok(())
        }
    }

    pub fn set_ap(&self, pwr_state: PwrState) -> Result<()> {
        let pwr_state = pwr_state.into();
        if self.ap.load(Ordering::Relaxed) == pwr_state {
            return Ok(());
        }

        let msg = EpTxMsg::Mgmt(MgmtTxMsg::ApPwrState { pwr_state });
        apple_mbox_driver.write_msg(self.mbox, msg.as_apple_mbox_msg())?;

        if self.ap.load(Ordering::Relaxed) != pwr_state {
            tsleep(&self.ap, bindings::PWAIT, c"appwr", 1 * unsafe { bindings::hz })
        } else {
            Ok(())
        }
    }
}

fn mbox_send_from_task(ctx: &Task<RTKitTaskCtx>, msg: EpTxMsg) -> Result<()> {
    apple_mbox_driver.write_msg(ctx.rtkit.mbox, msg.as_apple_mbox_msg())
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

fn handle_mgmt(ctx: &Task<RTKitTaskCtx>, msg: MgmtRxMsg) -> Result<()> {
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
            ctx.rtkit.iop.store(pwr_state, Ordering::Relaxed);
            wakeup(&ctx.rtkit.iop);
            Ok(())
        }
        MgmtRxMsg::ApPwrStateAck(pwr_state) => {
            ctx.rtkit.ap.store(pwr_state, Ordering::Relaxed);
            wakeup(&ctx.rtkit.ap);
            Ok(())
        }
        MgmtRxMsg::EpMap { base, bitmap, last } => {
            let old_bitmap = ctx.rtkit.ep_map.load(Ordering::Relaxed);
            let new_bits = (bitmap as u64) << (base * 32);
            let new_bitmap = old_bitmap | new_bits;
            ctx.rtkit.ep_map.store(new_bitmap, Ordering::Relaxed);
            let reply = MgmtTxMsg::EpMap { base, last };
            mbox_send_from_task(ctx, EpTxMsg::Mgmt(reply))?;
            if last {
                let dev = ctx.rtkit.client;
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
                            device_println!(dev, "starting rtkit endpoint {ep:?}");
                            let start_ep = MgmtTxMsg::StartEp { ep };
                            mbox_send_from_task(ctx, EpTxMsg::Mgmt(start_ep))?;
                        }
                        _ => {
                            device_println!(dev, "skipping endpoint {ep:?}");
                        }
                    }
                }
            }
            Ok(())
        }
        MgmtRxMsg::Unknown(_) => todo!(""),
    }
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

//#[no_mangle]
//extern "C" fn rtkit_init(dev: Device) -> *mut RTKit {
//    RTKit::new(dev).unwrap();
//    todo!("")
//}
//
//#[no_mangle]
//unsafe extern "C" fn rtkit_boot(rtkit: *mut RTKit) -> c_int {
//    //RTKit::boot(rtkit).unwrap();
//    todo!("")
//}
