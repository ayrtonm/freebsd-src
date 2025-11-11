/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2024 Ayrton Muñoz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#![no_std]

use apple_mbox::{AppleMboxDriver, AppleMboxMsg};
use core::sync::atomic::{AtomicU16, Ordering};
use kpi::bindings::device_t;
use kpi::boxed::Box;
use kpi::prelude::*;
use kpi::ffi::DevRef;
use kpi::taskq::Task;

//pub type Endpoint = u8;
//pub type RTKitRxFn<T> = fn(DevRef<T>, u64) -> Result<()>;

#[repr(u16)]
#[derive(Debug)]
pub enum PwrState {
    Off = 0x0000,
    Sleep = 0x0001,
    Idle = 0x0201,
    Quiesced = 0x0010,
    On = 0x0020,
    Init = 0x0220,
}

//enum MgmtRxMsg {
//    Hello { minver: u16, maxver: u16 },
//    IopPwrAck(u16),
//    ApPwrAck(u16),
//    EpMap { base: u8, bitmap: u32, last: bool },
//}
//
//enum MgmtTxMsg {
//    HelloAck(u16),
//    IopPwrReq(u16),
//    ApPwrReq(u16),
//}
//
//impl Into<AppleMboxMsg> for MgmtTxMsg {
//    fn into(self) -> AppleMboxMsg {
//        let data0 = match self {
//            Self::HelloAck(ver) => {
//                let minver = ver as u64;
//                let maxver = (ver as u64) << MAXVER_SHIFT;
//                let ty = (MGMT_HELLO_ACK as u64) << MSG_TYPE_SHIFT;
//                minver | maxver | ty
//            }
//            Self::IopPwrReq(pwr_state) => pwr_state as u64,
//            Self::ApPwrReq(pwr_state) => pwr_state as u64,
//        };
//        AppleMboxMsg {
//            data0,
//            data1: EP_MGMT,
//        }
//    }
//}
//
//// Management message types
//const MGMT_HELLO: u8 = 1;
//const MGMT_HELLO_ACK: u8 = 2;
//const MGMT_START_EP: u8 = 5;
//const MGMT_IOP_PWR: u8 = 6;
//const MGMT_IOP_PWR_ACK: u8 = 7;
//const MGMT_EP_MAP: u8 = 8;
//const MGMT_AP_PWR: u8 = 11;
//const MGMT_AP_PWR_ACK: u8 = MGMT_AP_PWR;
//
//const MAXVER_SHIFT: u64 = 16;
//const EP_MAP_BASE_SHIFT: u64 = 32;
//const EP_MAP_LAST_SHIFT: u64 = 51;
//const EP_MAP_MORE: u64 = 1 << 0;
//const START_EP_SHIFT: u64 = 32;
//const START_EP_START: u64 = 1 << 1;
//
//const MSG_TYPE_SHIFT: u64 = 52;
//
//impl MgmtRxMsg {
//    pub fn new(data0: u64) -> Result<Self> {
//        let msg_ty = (data0 >> MSG_TYPE_SHIFT) as u8;
//        match msg_ty {
//            MGMT_HELLO => {
//                let minver = data0 as u16;
//                let maxver = (data0 >> MAXVER_SHIFT) as u16;
//                Ok(Self::Hello { minver, maxver })
//            }
//            MGMT_IOP_PWR_ACK => Ok(Self::IopPwrAck(data0 as u16)),
//            MGMT_AP_PWR_ACK => Ok(Self::ApPwrAck(data0 as u16)),
//            MGMT_EP_MAP => {
//                let base = (data0 >> EP_MAP_BASE_SHIFT) as u8;
//                let bitmap = data0 as u32;
//                let last = data0 & (1 << EP_MAP_LAST_SHIFT) != 0;
//                Ok(Self::EpMap { base, bitmap, last })
//            }
//            _ => Err(EINVAL),
//        }
//    }
//}

#[derive(Debug)]
pub struct RTKit {
    client: device_t,
    mbox: device_t,
    verbose: bool,
    iop: AtomicU16,
}

pub trait HasRTKit {
    fn get_rtkit(&self) -> &RTKit;
}

//#[derive(Debug)]
//struct RTKitTaskCtx {
//    rtkit: Arc<RTKit>,
//    msg: AppleMboxMsg,
//}
//
impl RTKit {
    pub fn new(client: device_t) -> Result<Self> {
        let mbox = AppleMboxDriver::get_mbox(client)?;
        Ok(Self {
            client,
            mbox,
            verbose: false,
            iop: AtomicU16::new(PwrState::Sleep as u16),
        })
    }

    pub fn set_rx<T: HasRTKit>(&self, sc: DevRef<T>) -> Result<()> {
        AppleMboxDriver::set_rx(self.mbox, self.client, rtkit_rx_callback::<T>, sc)
    }

//    pub fn boot(&self) -> Result<()> {
//        self.set_iop(PwrState::On)
//    }
//
//    fn set_iop(&self, pwr_state: PwrState) -> Result<()> {
//        assert!(!cold());
//        let pwr_state = pwr_state as u16;
//        if self.iop.load(Ordering::Relaxed) == pwr_state {
//            return Ok(());
//        }
//        let msg = MgmtTxMsg::IopPwrReq(pwr_state);
//        self.send(msg)?;
//
//        if self.iop.load(Ordering::Relaxed) != (pwr_state & 0xFF) {
//            tsleep(&self.iop, Some(PWAIT), c"ioppwr", 5 * hz())?;
//        }
//        Ok(())
//    }
//
//    pub fn set_verbose(&mut self) {
//        self.verbose = true;
//    }
//
//    pub fn start_endpoint<T>(&self, ep: Endpoint, func: RTKitRxFn<T>, arg: Arc<T>) -> Result<()> {
//        Ok(())
//    }
//
//    pub fn send<T: Into<AppleMboxMsg>>(&self, msg: T) -> Result<()> {
//        wmb!();
//        AppleMboxDriver::write_msg(self.mbox, msg.into())
//    }
//
//    fn handle_mgmt(&self, data0: u64) -> Result<()> {
//        let msg = MgmtRxMsg::new(data0)?;
//        match msg {
//            MgmtRxMsg::Hello { minver, maxver } => {
//                /* versions we support */
//                const RTKIT_MINVER: u16 = 11;
//                const RTKIT_MAXVER: u16 = 12;
//
//                if minver > RTKIT_MAXVER {
//                    return Err(EINVAL);
//                }
//                if maxver < RTKIT_MINVER {
//                    return Err(EINVAL);
//                }
//                let mut ver = maxver;
//                if RTKIT_MAXVER < ver {
//                    ver = RTKIT_MAXVER;
//                }
//                let ack = MgmtTxMsg::HelloAck(ver);
//                self.send(ack)
//            }
//            MgmtRxMsg::IopPwrAck(_pwr_state) => {
//                todo!("")
//            }
//            MgmtRxMsg::ApPwrAck(_pwr_state) => {
//                todo!("")
//            }
//            MgmtRxMsg::EpMap { base, bitmap, last } => {
//                todo!("")
//            }
//        }
//    }
//
//    //fn handle_crashlog(&self, data0: u64)
}

extern "C" fn rtkit_rx_callback<T: HasRTKit>(sc: DevRef<T>, msg: AppleMboxMsg) {
    callback(sc, msg)
        .inspect_err(|e| {
            device_println!(sc.get_rtkit().client, "callback failed {e}");
        })
        .unwrap();

    fn callback<T>(sc: DevRef<T>, msg: AppleMboxMsg) -> Result<()> {
        rmb!();
        //let ctx = Arc::new(RTKitTaskCtx { rtkit, msg }, M_DEVBUF, M_WAITOK);
        //let mut task = Box::new(Task::new(), M_DEVBUF, M_WAITOK);
        //task.init(rtkit_rx_task, ctx);
        //taskqueue_enqueue(taskqueue_thread(), task)?;
        Ok(())
    }
}
//
//const EP_MGMT: u32 = 0;
//const EP_CRASHLOG: u32 = 1;
//const EP_SYSLOG: u32 = 2;
//const EP_DEBUG: u32 = 3;
//const EP_IOREPORT: u32 = 4;
//const EP_OSLOG: u32 = 8;
//const EP_TRACEKIT: u32 = 10;
//
//extern "C" fn rtkit_rx_task(ctx: DevRef<RTKitTaskCtx>, pending: u32) {
//    task_fn(ctx)
//        .inspect_err(|e| {
//            device_println!(ctx.rtkit.client, "task fn failed {e}");
//        })
//        .unwrap();
//
//    fn task_fn(ctx: DevRef<RTKitTaskCtx>) -> Result<()> {
//        let ep = ctx.msg.data1;
//        match ep {
//            EP_MGMT => ctx.rtkit.handle_mgmt(ctx.msg.data0),
//            EP_CRASHLOG => {
//                todo!("")
//            }
//            EP_SYSLOG => {
//                todo!("")
//            }
//            EP_DEBUG => {
//                todo!("")
//            }
//            EP_IOREPORT => {
//                todo!("")
//            }
//            EP_OSLOG => {
//                todo!("")
//            }
//            EP_TRACEKIT => {
//                todo!("")
//            }
//            other_eps => {
//                todo!("")
//            }
//        }
//    }
//}
