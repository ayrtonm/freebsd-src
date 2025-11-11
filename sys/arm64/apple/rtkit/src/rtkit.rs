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

use core::ffi::{c_int, c_void};
use core::mem::transmute;
use core::ptr::null_mut;
use core::sync::atomic::{AtomicU16, AtomicU64, Ordering};
use kpi::bindings;
use kpi::bindings::{
    bus_addr_t, bus_dma_segment_t, bus_dma_tag_t, bus_dmamap_t, bus_size_t, device_t,
};
use kpi::boxed::Box;
use kpi::ffi::{FatPtr, OwnedPtr, Ptr, RefCountData};
use kpi::prelude::*;
use kpi::sync::{Mutable, RefMut};
use kpi::taskq::Task;

use apple_mbox::{AppleMboxMsg, AppleMboxDriver};

macro_rules! dbg {
    ($rtkit:ident, $($rest:tt)*) => {
        if $rtkit.verbose {
            device_println!($rtkit.client, $($rest)*)
        }
    };
}

#[path = "buffer.rs"]
mod buffer;
#[path = "endpoints.rs"]
mod endpoints;
#[path = "pwr.rs"]
mod pwr;
#[path = "requests.rs"]
mod requests;

pub use crate::endpoints::{Endpoint, EpMap};
use buffer::{RTKitBuffer, handle_buffer_req};
pub use pwr::PwrState;
use requests::{
    BUFFER_REQUEST, BUFFER_SIZE_SHIFT, EpTxMsg, MgmtRxMsg, MgmtTxMsg, buffer_addr, buffer_size,
    mgmt_msg_type,
};

pub type RTKitRx<T> = fn(Ptr<T>, u64) -> Result<()>;
type RawRTKitRx = fn(*mut c_void, *mut RefCountData, u64) -> Result<()>;

#[derive(Debug)]
struct RTKitCallback {
    func: RawRTKitRx,
    arg: *mut c_void,
    metadata: *mut RefCountData,
}

#[repr(C)]
#[derive(Debug)]
pub struct RTKit {
    client: device_t,
    mbox: device_t,

    iop: AtomicU16,
    ap: AtomicU16,

    verbose: bool,
    noalloc: bool,
    ep_map: EpMap,
    callbacks: Mutable<[Option<RTKitCallback>; 32]>,
    crashlog: Mutable<RTKitBuffer>,
    syslog: Mutable<RTKitBuffer>,
    ioreport: Mutable<RTKitBuffer>,
    oslog: Mutable<RTKitBuffer>,
}

#[derive(Debug)]
struct RTKitTaskCtx {
    rtkit: &'static RTKit,
    msg: AppleMboxMsg,
}

impl RTKit {
    pub fn new(client: device_t) -> Result<Self> {
        let mbox = AppleMboxDriver::get_mbox(client)?;
        let iop = AtomicU16::new(PwrState::Sleep.into());
        let ap = AtomicU16::new(PwrState::Quiesced.into());
        Ok(Self {
            client,
            mbox,
            iop,
            ap,
            verbose: false,
            noalloc: false,
            ep_map: EpMap::new(),
            callbacks: Mutable::new([const { None }; 32]),
            crashlog: Mutable::new(RTKitBuffer::default()),
            syslog: Mutable::new(RTKitBuffer::default()),
            ioreport: Mutable::new(RTKitBuffer::default()),
            oslog: Mutable::new(RTKitBuffer::default()),
        })
    }

    pub fn set_verbose(&mut self) {
        self.verbose = true;
    }

    pub fn wake(&self) -> Result<()> {
        self.set_iop(PwrState::Init).inspect_err(|e| {
            device_println!(self.client, "failed to set IOP power state INIT");
        })?;
        Ok(())
    }

    pub fn boot(&self) -> Result<()> {
        dbg!(self, "setting IOP power state ON from rtkit_boot");
        let _ = self.set_iop(PwrState::On).inspect_err(|e| {
            device_println!(self.client, "failed to set IOP power state ON");
        });
        Ok(())
    }

    pub fn start_endpoint<T>(&self, ep: Endpoint, func: RTKitRx<T>, arg: Ptr<T>) -> Result<()> {
        dbg!(self, "starting endpoint {ep:x?}");
        let _ = tsleep(&self.ep_map, bindings::PWAIT, c"ep_map", 5 * hz()).inspect_err(|e| {
            device_println!(
                self.client,
                "timed out waiting for ep map {:x?}",
                self.ep_map
            );
        });
        if !self.ep_map.contains(ep) {
            device_println!(
                self.client,
                "WARNING: endpoint not set in map {:x?}",
                self.ep_map
            );
        }
        let ep_num: u32 = ep.into();
        let ep_callback = &mut self.callbacks.get_mut()[32 - (ep_num as usize)];
        //let leaked_ref = OwnedPtr::leak_ref(arg);
        //*ep_callback = Some(RTKitCallback {
        //    func: unsafe { transmute::<RTKitRx<T>, RawRTKitRx>(func) },
        //    arg: leaked_ref.0.cast::<c_void>(),
        //    metadata: leaked_ref.1,
        //});
        let msg = MgmtTxMsg::StartEp { ep };
        self.send(msg)
    }

    pub fn send<T: Into<AppleMboxMsg>>(&self, msg: T) -> Result<()> {
        wmb!();
        AppleMboxDriver::write_msg(self.mbox, msg.into())
    }

    fn handle_syslog(&self, data0: u64) -> Result<()> {
        let ty = mgmt_msg_type(data0);
        todo!("")
    }

    fn handle_mgmt(&self, data0: u64) -> Result<()> {
        let msg = MgmtRxMsg::new(data0);
        match msg {
            MgmtRxMsg::Hello { minver, maxver } => {
                dbg!(self, "recv'ed hello ack {msg:?}");
                /* versions we support */
                const RTKIT_MINVER: u16 = 11;
                const RTKIT_MAXVER: u16 = 12;

                if minver > RTKIT_MAXVER {
                    return Err(EINVAL);
                }
                if maxver < RTKIT_MINVER {
                    return Err(EINVAL);
                }
                let mut ver = maxver;
                if RTKIT_MAXVER < ver {
                    ver = RTKIT_MAXVER;
                }
                let ack = MgmtTxMsg::HelloAck {
                    minver: ver,
                    maxver: ver,
                };
                self.send(ack)
            }
            MgmtRxMsg::IopPwrStateAck(pwr_state) => {
                dbg!(
                    self,
                    "setting IOP power state in callback to {pwr_state:x?}"
                );
                self.iop.store(pwr_state, Ordering::Relaxed);
                wakeup(&self.iop);
                Ok(())
            }
            MgmtRxMsg::ApPwrStateAck(pwr_state) => {
                self.ap.store(pwr_state, Ordering::Relaxed);
                wakeup(&self.ap);
                Ok(())
            }
            MgmtRxMsg::EpMap { base, bitmap, last } => {
                dbg!(self, "recv'ed ep map req {msg:x?}");

                let new_bits = u64::from(bitmap) << u64::from(base * 32);
                let new_bitmap = self.ep_map.insert(new_bits);
                let reply = MgmtTxMsg::EpMap { base, last };
                self.send(reply)?;
                if last {
                    // range bounds are not a typo, bit 0 is not a valid endpoint here
                    for ep_num in 1..32 {
                        if new_bitmap & (1 << ep_num) == 0 {
                            continue;
                        }
                        let ep = Endpoint::new(ep_num);
                        match ep {
                            Endpoint::Mgmt => (),
                            Endpoint::CrashLog
                            | Endpoint::SysLog
                            | Endpoint::Debug
                            | Endpoint::IOReport
                            | Endpoint::OsLog
                            | Endpoint::TraceKit => {
                                dbg!(self, "starting rtkit endpoint {ep:?}");
                                let start_ep = MgmtTxMsg::StartEp { ep };
                                self.send(start_ep)?;
                            }
                            Endpoint::Other(another_ep) => {
                                dbg!(self, "skipping endpoint {another_ep:?}");
                            }
                        }
                    }
                    wakeup(&self.ep_map);
                }
                Ok(())
            }
            MgmtRxMsg::Unknown(_) => todo!(""),
        }
    }
}

fn handle_crashlog(rtkit: &'static RTKit, data0: u64) -> Result<()> {
    let ty = mgmt_msg_type(data0);
    if ty != BUFFER_REQUEST {
        device_println!(
            rtkit.client,
            "unexpected RTKit msg of type {ty} from crashlog endpoint"
        );
        return Err(EINVAL);
    }
    if rtkit.crashlog.get_mut().addr != 0 {
        panic!("RTKit crashed");
    }
    handle_buffer_req(rtkit, Endpoint::CrashLog, data0, |rtk| {
        rtk.crashlog.get_mut()
    })?;
    Ok(())
}

fn handle_ioreport(rtkit: &'static RTKit, data0: u64) -> Result<()> {
    const IOREPORT_UNKNOWN1: u8 = 8;
    const IOREPORT_UNKNOWN2: u8 = 12;
    let ty = mgmt_msg_type(data0);
    match ty {
        BUFFER_REQUEST => {
            handle_buffer_req(rtkit, Endpoint::IOReport, data0, |rtk| {
                rtk.ioreport.get_mut()
            })?;
        }
        IOREPORT_UNKNOWN1 | IOREPORT_UNKNOWN2 => {
            device_println!(
                rtkit.client,
                "Ack'ed unknown message of type {ty} from RTKit ioreport endpoint"
            );
        }
        _ => {
            panic!("unknown ioreport message of type {ty}");
        }
    }
    Ok(())
}

pub fn rtkit_start(rtkit: FatPtr<RTKit>) -> Result<()> {
    AppleMboxDriver::set_rx(rtkit.mbox, rtkit.client, rtkit_rx_callback, rtkit)
}

fn rtkit_rx_callback(rtkit: &'static RTKit, msg: AppleMboxMsg) -> Result<()> {
    rmb!();
    let ctx = RTKitTaskCtx { rtkit, msg };
    let mut task =
        Box::try_new(Task::new(rtkit_rx_task, ctx), M_DEVBUF, M_NOWAIT).inspect_err(|e| {
            device_println!(rtkit.client, "failed to allocate memory for task {e}");
        })?;
    taskqueue_enqueue(taskqueue_thread(), task).inspect_err(|e| {
        device_println!(rtkit.client, "failed to enqueue task {e}");
    })
}

fn rtkit_rx_task(ctx: &RTKitTaskCtx, pending: u32) -> Result<()> {
    let rtkit = ctx.rtkit;
    let ep = Endpoint::new(ctx.msg.data1);
    match ep {
        Endpoint::Mgmt => rtkit.handle_mgmt(ctx.msg.data0),
        Endpoint::CrashLog => handle_crashlog(rtkit, ctx.msg.data0),
        Endpoint::SysLog => rtkit.handle_syslog(ctx.msg.data0),
        Endpoint::Debug => todo!(""), //handle_crashlog(rtkit, ctx.msg.data0),
        Endpoint::IOReport => handle_ioreport(rtkit, ctx.msg.data0),
        Endpoint::OsLog => todo!(""), //handle_crashlog(rtkit, ctx.msg.data0),
        Endpoint::TraceKit => todo!(""), //handle_crashlog(rtkit, ctx.msg.data0),
        Endpoint::Other(another_ep) => todo!(""), //handle_crashlog(rtkit, ctx.msg.data0),
    }
}
