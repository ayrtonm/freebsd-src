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

use crate::bindings;
use crate::bindings::{bus_addr_t, bus_dma_tag_t, bus_dmamap_t, bus_size_t, device_t};
use core::ffi::{c_int, c_void};
use core::mem::transmute;
use core::sync::atomic::{AtomicU16, AtomicU64, Ordering};
use kpi::cell::{CRef, CRefMetadata, Mutable, Ptr};
use kpi::prelude::*;
use kpi::taskq::Task;

use apple_mbox::{AppleMboxMsg, apple_mbox_driver};

type Box<T> = kpi::boxed::Box<T, M_DEVBUF>;

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

pub type RTKitRx<T> = fn(CRef<T>, u64) -> Result<()>;
type RawRTKitRx = fn(*mut c_void, *mut CRefMetadata, u64) -> Result<()>;

#[derive(Debug, Default)]
struct RTKitBuffer {
    addr: bus_addr_t,
    size: bus_size_t,
    kva: *mut c_void,
    tag: bus_dma_tag_t,
    map: bus_dmamap_t,
}

#[derive(Debug)]
struct RTKitCallback {
    func: RawRTKitRx,
    arg: *mut c_void,
    metadata: *mut CRefMetadata,
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
    ep_map: AtomicU64,
    callbacks: Mutable<[Option<RTKitCallback>; 32]>,
    crashlog: Mutable<RTKitBuffer>,
    syslog: Mutable<RTKitBuffer>,
    ioreport: Mutable<RTKitBuffer>,
    oslog: Mutable<RTKitBuffer>,
}

#[derive(Debug)]
struct RTKitTaskCtx {
    rtkit: CRef<RTKit>,
    msg: AppleMboxMsg,
}

impl RTKit {
    pub fn new(client: device_t) -> Result<Self> {
        let mbox = apple_mbox_driver.get_mbox(client)?;
        let iop = AtomicU16::new(PwrState::Sleep.into());
        let ap = AtomicU16::new(PwrState::Quiesced.into());
        Ok(Self {
            client,
            mbox,
            iop,
            ap,
            verbose: false,
            noalloc: false,
            ep_map: AtomicU64::new(0),
            callbacks: Mutable::new([const { None }; 32]),
            crashlog: Mutable::new(RTKitBuffer::default()),
            syslog: Mutable::new(RTKitBuffer::default()),
            ioreport: Mutable::new(RTKitBuffer::default()),
            oslog: Mutable::new(RTKitBuffer::default()),
        })
    }

    pub fn set_iop(&self, pwr_state: PwrState) -> Result<()> {
        let pwr_state = pwr_state.into();
        // If already in the correct power state do nothing
        if self.iop.load(Ordering::Relaxed) == pwr_state {
            return Ok(());
        }

        // Try setting the power state
        let msg = MgmtTxMsg::IopPwrState { pwr_state };
        apple_mbox_driver.write_msg(self.mbox, msg.as_apple_mbox_msg())?;

        // If the power state hasn't changed then sleep
        if self.iop.load(Ordering::Relaxed) != pwr_state {
            tsleep(&self.iop, bindings::PWAIT, c"ioppwr", hz())
        } else {
            Ok(())
        }
    }

    pub fn set_ap(&self, pwr_state: PwrState) -> Result<()> {
        let pwr_state = pwr_state.into();
        if self.ap.load(Ordering::Relaxed) == pwr_state {
            return Ok(());
        }

        let msg = MgmtTxMsg::ApPwrState { pwr_state };
        apple_mbox_driver.write_msg(self.mbox, msg.as_apple_mbox_msg())?;

        if self.ap.load(Ordering::Relaxed) != pwr_state {
            tsleep(&self.ap, bindings::PWAIT, c"appwr", hz())
        } else {
            Ok(())
        }
    }

    pub fn start_endpoint<T>(&self, ep: u32, func: RTKitRx<T>, arg: CRef<T>) -> Result<()> {
        let endpoint = &mut self.callbacks.get_mut()[32 - (ep as usize)];
        let leaked_ref = unsafe { arg.leak_ref() };
        *endpoint = Some(RTKitCallback {
            func: unsafe { transmute::<RTKitRx<T>, RawRTKitRx>(func) },
            arg: leaked_ref.0.cast::<c_void>(),
            metadata: leaked_ref.1,
        });
        let msg = MgmtTxMsg::StartEp { ep };
        apple_mbox_driver.write_msg(self.mbox, msg.as_apple_mbox_msg())
    }

    fn send(&self, msg: MgmtTxMsg) -> Result<()> {
        apple_mbox_driver.write_msg(self.mbox, msg.as_apple_mbox_msg())
    }

    fn handle_crashlog(&self, data0: u64) -> Result<()> {
        let ty = mgmt_msg_type(data0);
        if ty != BUFFER_REQUEST {
            device_println!(
                self.client,
                "unexpected RTKit msg of type {ty} from crashlog endpoint"
            );
            return Err(EINVAL);
        }
        if self.crashlog.get_mut().addr != 0 {
            panic!("RTKit crashed");
        }
        self.handle_buffer_req(RTKIT_EP_CRASHLOG, data0, &mut self.crashlog.get_mut())?;
        Ok(())
    }

    fn handle_mgmt(&self, data0: u64) -> Result<()> {
        let msg = MgmtRxMsg::new(data0);
        match msg {
            MgmtRxMsg::Hello { minver, maxver } => {
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
                let old_bitmap = self.ep_map.load(Ordering::Relaxed);
                let new_bits = (bitmap as u64) << (base * 32);
                let new_bitmap = old_bitmap | new_bits;
                self.ep_map.store(new_bitmap, Ordering::Relaxed);
                let reply = MgmtTxMsg::EpMap { base, last };
                self.send(reply)?;
                if last {
                    let dev = self.client;
                    // range bounds are not a typo, bit 0 is not an endpoint
                    for ep in 1..32 {
                        if new_bitmap & (1 << ep) == 0 {
                            continue;
                        }
                        match ep {
                            RTKIT_EP_MGMT => (),
                            RTKIT_EP_CRASHLOG | RTKIT_EP_SYSLOG | RTKIT_EP_DEBUG
                            | RTKIT_EP_IOREPORT | RTKIT_EP_OSLOG | RTKIT_EP_TRACEKIT => {
                                device_println!(dev, "starting rtkit endpoint {ep:?}");
                                let start_ep = MgmtTxMsg::StartEp { ep };
                                self.send(start_ep)?;
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
    unsafe fn alloc_buffer(&self, req_size: bus_size_t, buffer: &mut RTKitBuffer) -> Result<()> {
        /*
        use bindings::*;
        let dev = rtkit.client;

        let rc = bus_dma_tag_create(
            bus_get_dma_tag(dev.as_ptr()),
            PAGE_SIZE as u64, /* 16K alignment */
            0,                /* bounds */
            BUS_SPACE_MAXADDR as u64, /* low addr */
            BUS_SPACE_MAXADDR as u64, /* high addr */
            None,
            null_mut(), /* filter deprecated and not supported anyway */
            req_size,   /* maxsize */
            1,          /* nsegments */
            req_size,   /* maxsegsize */
            BUS_DMA_COHERENT,
            None,
            null_mut(), /* lockfunc */
            &mut buffer.tag,
        );
        if rc != 0 {
            device_println!(dev, "bus_dma_tag_create failed {rc}");
            return Err(ErrCode::from(rc));
        }
        rc = bus_dmamem_alloc(
            buffer.tag,
            &buffer.kva,
            BUS_DMA_WAITOK | BUS_DMA_ZERO,
            &buffer.map,
        );
        if rc != 0 {
            device_println!(dev, "bus_dmamem_alloc failed {rc}");
            return Err(ErrCode::from(rc));
        }
        buffer.size = req_size;
        buffer.addr = 0;
        rc = bus_dmamap_load(
            buffer.tag,
            buffer.map,
            buffer.kva,
            req_size,
            Some(rtkit_dmamap_cb),
            buffer,
            0,
        );
        if (rc != 0) && (rc != EINPROGRESS) {
            device_println!(dev, "bus_dmamap_load failed {rc}");
            return Err(ErrCode::from(rc));
        }
        */
        Ok(())
    }
    fn handle_buffer_req(&self, ep: u32, data0: u64, buffer: &mut RTKitBuffer) -> Result<()> {
        let size = buffer_size(data0);
        device_println!(
            self.client,
            "RTKit endpoint {ep} requested {} byte buffer",
            size << bindings::PAGE_SHIFT_4K
        );
        unsafe { self.alloc_buffer(size << bindings::PAGE_SHIFT_4K, buffer)? };
        //rtkit_send(rtkit, EpTxMsg::Mgmt(
        Ok(())
    }
    //
    //    fn handle_ioreport(&self, data0: u64) -> Result<()> {
    //        let ty = mgmt_msg_type(data0);
    //        match ty {
    //            BUFFER_REQUEST => {
    //                self.handle_buffer_req(
    //                    RTKIT_EP_IOREPORT,
    //                    data0,
    //                    &mut self.ioreport.get_mut(),
    //                )?;
    //            }
    //            IOREPORT_UNKNOWN1 | IOREPORT_UNKNOWN2 => {
    //                device_println!(
    //                    self.client,
    //                    "Ack'ed unknown message of type {ty} from RTKit ioreport endpoint"
    //                );
    //            }
    //            _ => {
    //                panic!("unknown ioreport message of type {ty}");
    //            }
    //        }
    //        Ok(())
    //    }
}

pub fn rtkit_boot(rtkit: CRef<RTKit>) -> Result<()> {
    apple_mbox_driver.set_rx(rtkit.mbox, rtkit.client, rtkit_rx_callback, rtkit)
}

fn rtkit_rx_callback(rtkit: &CRef<RTKit>, msg: AppleMboxMsg) -> Result<()> {
    let ctx = RTKitTaskCtx {
        rtkit: rtkit.clone(),
        msg,
    };
    let mut task = Box::try_new(Task::new_with_ctx(ctx), M_NOWAIT).inspect_err(|e| {
        device_println!(rtkit.client, "failed to allocate memory for task {e}");
    })?;
    task.init_inline(rtkit_rx_task);
    taskqueue_enqueue(taskqueue_thread(), task).inspect_err(|e| {
        device_println!(rtkit.client, "failed to enqueue task {e}");
    })
}

const RTKIT_EP_MGMT: u32 = 0;
const RTKIT_EP_CRASHLOG: u32 = 1;
const RTKIT_EP_SYSLOG: u32 = 2;
const RTKIT_EP_DEBUG: u32 = 3;
const RTKIT_EP_IOREPORT: u32 = 4;
const RTKIT_EP_OSLOG: u32 = 8;
const RTKIT_EP_TRACEKIT: u32 = 10;

fn rtkit_rx_task(ctx: &RTKitTaskCtx, pending: u32) -> Result<()> {
    let rtkit = &ctx.rtkit;
    let endpoint = ctx.msg.data1;
    match endpoint {
        RTKIT_EP_MGMT => rtkit.handle_mgmt(ctx.msg.data0),
        RTKIT_EP_CRASHLOG => rtkit.handle_crashlog(ctx.msg.data0),
        RTKIT_EP_SYSLOG => todo!(""), //handle_syslog(&ctx, ctx.msg.data0),
        RTKIT_EP_DEBUG => todo!(""),  //handle_syslog(&ctx, ctx.msg.data0),
        RTKIT_EP_IOREPORT => todo!(""), //handle_syslog(&ctx, ctx.msg.data0),
        RTKIT_EP_OSLOG => todo!(""),  //handle_syslog(&ctx, ctx.msg.data0),
        RTKIT_EP_TRACEKIT => todo!(""), //handle_syslog(&ctx, ctx.msg.data0),
        another_endpoint => {
            todo!("")
        }
    };
    Ok(())
}

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
        match mgmt_msg_type(data0) {
            MGMT_HELLO => {
                let minver = mgmt_hello_minver(data0);
                let maxver = mgmt_hello_maxver(data0);
                Self::Hello { minver, maxver }
            }
            MGMT_IOP_PWR_STATE_ACK => Self::IopPwrStateAck(mgmt_pwr_state(data0)),
            // next one is not a typo
            MGMT_AP_PWR_STATE => Self::ApPwrStateAck(mgmt_pwr_state(data0)),
            MGMT_EP_MAP => {
                let base = mgmt_ep_map_base(data0);
                let bitmap = mgmt_ep_map_bitmap(data0);
                let last = mgmt_ep_map_last(data0);
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
        let data0 = match self {
            Self::HelloAck { minver, maxver } => mgmt_hello_ack(minver, maxver),
            Self::IopPwrState { pwr_state } => mgmt_iop_pwr_state(pwr_state),
            Self::ApPwrState { pwr_state } => mgmt_ap_pwr_state(pwr_state),
            Self::EpMap { base, last } => mgmt_ep_map_reply(base, last),
            Self::StartEp { ep } => mgmt_start_ep(ep),
        };
        AppleMboxMsg {
            data0,
            data1: RTKIT_EP_MGMT,
        }
    }
}
const BUFFER_REQUEST: u8 = 1;

fn buffer_size(x: u64) -> u64 {
    ((x >> 44) as u8) as u64
}
const IOREPORT_UNKNOWN1: u8 = 8;
const IOREPORT_UNKNOWN2: u8 = 12;

const MGMT_HELLO: u8 = 1;
const MGMT_HELLO_ACK: u8 = 2;
const MGMT_START_EP: u8 = 5;
const MGMT_IOP_PWR_STATE: u8 = 6;
const MGMT_IOP_PWR_STATE_ACK: u8 = 7;
const MGMT_EP_MAP: u8 = 8;
const MGMT_AP_PWR_STATE: u8 = 11;

const MAXVER_SHIFT: u64 = 16;
const EP_MAP_BASE_SHIFT: u64 = 32;
const EP_MAP_LAST_SHIFT: u64 = 51;
const EP_MAP_MORE: u64 = 1 << 0;
const START_EP_SHIFT: u64 = 32;
const START_EP_START: u64 = 1 << 1;

const MSG_TYPE_SHIFT: u64 = 52;
const OSLOG_TYPE_SHIFT: u64 = 56;

fn mgmt_msg_type(x: u64) -> u8 {
    (x >> MSG_TYPE_SHIFT) as u8
}

fn mgmt_hello_minver(x: u64) -> u16 {
    x as u16
}

fn mgmt_hello_maxver(x: u64) -> u16 {
    (x >> MAXVER_SHIFT) as u16
}

fn mgmt_hello_ack(minver: u16, maxver: u16) -> u64 {
    (minver as u64) | (maxver as u64) << MAXVER_SHIFT | (MGMT_HELLO_ACK as u64) << MSG_TYPE_SHIFT
}

fn mgmt_pwr_state(x: u64) -> u16 {
    x as u16
}

fn mgmt_iop_pwr_state(pwr_state: u16) -> u64 {
    (pwr_state as u64) | (MGMT_IOP_PWR_STATE as u64) << MSG_TYPE_SHIFT
}

fn mgmt_ap_pwr_state(pwr_state: u16) -> u64 {
    (pwr_state as u64) | (MGMT_AP_PWR_STATE as u64) << MSG_TYPE_SHIFT
}

fn mgmt_ep_map_base(x: u64) -> u8 {
    (x >> EP_MAP_BASE_SHIFT) as u8
}

fn mgmt_ep_map_bitmap(x: u64) -> u32 {
    x as u32
}

fn mgmt_ep_map_last(x: u64) -> bool {
    x & (1 << EP_MAP_LAST_SHIFT) != 0
}

fn mgmt_ep_map_reply(base: u8, last: bool) -> u64 {
    let res = (base as u64) << EP_MAP_BASE_SHIFT | (MGMT_EP_MAP as u64) << MSG_TYPE_SHIFT;
    if last {
        res | (1 << EP_MAP_LAST_SHIFT)
    } else {
        res | EP_MAP_MORE
    }
}

fn mgmt_start_ep(ep: u32) -> u64 {
    (ep as u64) << START_EP_SHIFT | START_EP_START | (MGMT_START_EP as u64) << MSG_TYPE_SHIFT
}

/* versions we support */
const RTKIT_MINVER: u16 = 11;
const RTKIT_MAXVER: u16 = 12;

extern "C" fn rtkit_dmamap_cb() {}
