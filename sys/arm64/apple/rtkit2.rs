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

use apple_mbox::{AppleMboxDriver, AppleMboxMsg};
use core::ffi::{c_int, c_void};
use core::mem::transmute;
use core::ptr::null_mut;
use core::sync::atomic::{AtomicU16, AtomicU64, Ordering};
use kpi::bindings::{
    bus_addr_t, bus_dma_segment_t, bus_dma_tag_t, bus_dmamap_t, bus_size_t, device_t,
};
use kpi::boxed::Box;
use kpi::bus::dma::BusDmaTag;
use kpi::misc::Sleepable;
use kpi::prelude::*;
use kpi::sync::{Mutable, RefMut};
use kpi::taskq::Task;

macro_rules! dbg {
    ($rtkit:ident, $($rest:tt)*) => {
        if $rtkit.verbose {
            device_println!($rtkit.client, $($rest)*)
        }
    };
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
    StartEp { ep: Endpoint },
    EpMap { base: u8, last: bool },
}

impl Into<AppleMboxMsg> for MgmtTxMsg {
    fn into(self) -> AppleMboxMsg {
        let data0 = match self {
            Self::HelloAck { minver, maxver } => mgmt_hello_ack(minver, maxver),
            Self::IopPwrState { pwr_state } => mgmt_iop_pwr_state(pwr_state),
            Self::ApPwrState { pwr_state } => mgmt_ap_pwr_state(pwr_state),
            Self::EpMap { base, last } => {
                let res = (u64::from(base) << EP_MAP_BASE_SHIFT)
                    | ((MGMT_EP_MAP as u64) << MSG_TYPE_SHIFT);
                if last {
                    res | (1 << EP_MAP_LAST_SHIFT)
                } else {
                    res | EP_MAP_MORE
                }
            }
            Self::StartEp { ep } => {
                let ep: u32 = ep.into();
                let mut res = u64::from(ep) << START_EP_SHIFT;
                res |= START_EP_START;
                res |= u64::from(MGMT_START_EP) << MSG_TYPE_SHIFT;
                res
            }
        };
        AppleMboxMsg {
            data0,
            data1: Endpoint::Mgmt.into(),
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub enum EpTxMsg {
    BufferReq { ep: Endpoint, data: u64 },
}

impl Into<AppleMboxMsg> for EpTxMsg {
    fn into(self) -> AppleMboxMsg {
        let (data0, ep) = match self {
            Self::BufferReq { ep, data } => {
                let shift = if ep == Endpoint::OsLog {
                    OSLOG_TYPE_SHIFT
                } else {
                    MSG_TYPE_SHIFT
                };
                let res = (u64::from(BUFFER_REQUEST) << shift) | data;
                (res, ep)
            }
        };
        AppleMboxMsg {
            data0,
            data1: ep.into(),
        }
    }
}

pub const BUFFER_REQUEST: u8 = 1;
pub const BUFFER_SIZE_SHIFT: u64 = 44;

pub fn buffer_addr(x: u64) -> u64 {
    let mask = (1 << BUFFER_SIZE_SHIFT) - 1;
    x & mask
}
pub fn buffer_size(x: u64) -> u64 {
    ((x >> BUFFER_SIZE_SHIFT) as u8) as u64
}

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

pub fn mgmt_msg_type(x: u64) -> u8 {
    (x >> MSG_TYPE_SHIFT) as u8
}

fn mgmt_hello_minver(x: u64) -> u16 {
    x as u16
}

fn mgmt_hello_maxver(x: u64) -> u16 {
    (x >> MAXVER_SHIFT) as u16
}

fn mgmt_hello_ack(minver: u16, maxver: u16) -> u64 {
    let mut res = minver as u64;
    res |= (maxver as u64) << MAXVER_SHIFT;
    res |= (MGMT_HELLO_ACK as u64) << MSG_TYPE_SHIFT;
    res
}

fn mgmt_pwr_state(x: u64) -> u16 {
    x as u16
}

fn mgmt_iop_pwr_state(pwr_state: u16) -> u64 {
    let mut res = pwr_state as u64;
    res |= (MGMT_IOP_PWR_STATE as u64) << MSG_TYPE_SHIFT;
    res
}

fn mgmt_ap_pwr_state(pwr_state: u16) -> u64 {
    let mut res = pwr_state as u64;
    res |= (MGMT_AP_PWR_STATE as u64) << MSG_TYPE_SHIFT;
    res
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

impl Into<u16> for PwrState {
    fn into(self) -> u16 {
        self as u16
    }
}

impl RTKit {
    pub fn set_iop(&self, pwr_state: PwrState) -> Result<()> {
        dbg!(self, "called set_iop with {pwr_state:x?}");
        let pwr_state = pwr_state.into();
        assert!(unsafe { bindings::cold } == 0);
        // If already in the correct power state do nothing
        if self.iop.load(Ordering::Relaxed) == pwr_state {
            dbg!(self, "IOP already in state {pwr_state:x?}");
            return Ok(());
        }

        // Try setting the power state
        let msg = MgmtTxMsg::IopPwrState { pwr_state };
        dbg!(self, "sending IOP pwr state request {msg:x?}");
        self.send(msg)?;

        // If the power state hasn't changed then sleep
        if self.iop.load(Ordering::Relaxed) != (pwr_state & 0xff) {
            dbg!(self, "sleep since IOP pwr state didn't change");
            let _ = tsleep(&self.iop, bindings::PWAIT, c"ioppwr", 5 * hz()).inspect_err(|e| {
                device_println!(
                    self.client,
                    "timed out waiting for IOP power state change {e:?}"
                );
            });
            Ok(())
        } else {
            device_println!(self.client, "IOP power state changed to {pwr_state:x?}");
            Ok(())
        }
    }

    pub fn set_ap(&self, pwr_state: PwrState) -> Result<()> {
        let pwr_state = pwr_state.into();
        if self.ap.load(Ordering::Relaxed) == pwr_state {
            return Ok(());
        }

        let msg = MgmtTxMsg::ApPwrState { pwr_state };
        self.send(msg)?;

        if self.ap.load(Ordering::Relaxed) != pwr_state {
            tsleep(&self.ap, bindings::PWAIT, c"appwr", hz())
        } else {
            Ok(())
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Endpoint {
    Mgmt,
    CrashLog,
    SysLog,
    Debug,
    IOReport,
    OsLog,
    TraceKit,
    Other(u32),
}

const RTKIT_EP_MGMT: u32 = 0;
const RTKIT_EP_CRASHLOG: u32 = 1;
const RTKIT_EP_SYSLOG: u32 = 2;
const RTKIT_EP_DEBUG: u32 = 3;
const RTKIT_EP_IOREPORT: u32 = 4;
const RTKIT_EP_OSLOG: u32 = 8;
const RTKIT_EP_TRACEKIT: u32 = 10;

impl Endpoint {
    pub fn new(ep: u32) -> Self {
        match ep {
            RTKIT_EP_MGMT => Endpoint::Mgmt,
            RTKIT_EP_CRASHLOG => Endpoint::CrashLog,
            RTKIT_EP_SYSLOG => Endpoint::SysLog,
            RTKIT_EP_DEBUG => Endpoint::Debug,
            RTKIT_EP_IOREPORT => Endpoint::IOReport,
            RTKIT_EP_OSLOG => Endpoint::OsLog,
            RTKIT_EP_TRACEKIT => Endpoint::TraceKit,
            another_ep => Endpoint::Other(another_ep),
        }
    }
}

impl Into<u32> for Endpoint {
    fn into(self) -> u32 {
        match self {
            Endpoint::Mgmt => RTKIT_EP_MGMT,
            Endpoint::CrashLog => RTKIT_EP_CRASHLOG,
            Endpoint::SysLog => RTKIT_EP_SYSLOG,
            Endpoint::Debug => RTKIT_EP_DEBUG,
            Endpoint::IOReport => RTKIT_EP_IOREPORT,
            Endpoint::OsLog => RTKIT_EP_OSLOG,
            Endpoint::TraceKit => RTKIT_EP_TRACEKIT,
            Endpoint::Other(another_ep) => another_ep,
        }
    }
}

#[derive(Debug)]
pub struct EpMap(AtomicU64);

impl Sleepable for EpMap {
    fn as_ptr(&self) -> *mut c_void {
        <AtomicU64 as Sleepable>::as_ptr(&self.0)
    }
}

impl EpMap {
    pub fn new() -> Self {
        Self(AtomicU64::new(0))
    }

    pub fn contains(&self, ep: Endpoint) -> bool {
        let ep: u32 = ep.into();
        let ep_bitset = self.0.load(Ordering::Relaxed);
        ep_bitset & u64::from(1u32 << ep) != 0
    }

    pub fn insert(&self, new_bitset: u64) -> u64 {
        let prev = self.0.fetch_or(new_bitset, Ordering::Relaxed);
        prev | new_bitset
    }
}

#[derive(Debug, Default)]
pub struct RTKitBuffer {
    pub addr: bus_addr_t,
    size: bus_size_t,
    kva: *mut c_void,
    tag: BusDmaTag,
    map: bus_dmamap_t,
    state: Option<&'static RTKit>,
}

pub fn handle_buffer_req(
    rtkit: &'static RTKit,
    ep: Endpoint,
    data0: u64,
    get_buffer: fn(&RTKit) -> RefMut<RTKitBuffer>,
) -> Result<()> {
    let size = buffer_size(data0);
    device_println!(
        rtkit.client,
        "RTKit endpoint {ep:?} requested {} byte buffer",
        size << bindings::PAGE_SHIFT_4K
    );
    rtkit_alloc(rtkit, size << bindings::PAGE_SHIFT_4K, get_buffer)?;

    let data = (size << BUFFER_SIZE_SHIFT) | buffer_addr(get_buffer(&rtkit).addr);
    rtkit.send(EpTxMsg::BufferReq { ep, data })
}

extern "C" fn rtkit_dmamap_cb(
    buffer_as_void_ptr: *mut c_void,
    segs: *mut bus_dma_segment_t,
    nsegs: i32,
    error: i32,
) {
    let buffer_ptr = buffer_as_void_ptr.cast::<RTKitBuffer>();
    let buffer = unsafe { buffer_ptr.as_mut().unwrap() };
    let dev = buffer.state.as_ref().unwrap().client;
    device_println!(
        dev,
        "dma map callback reported {nsegs} segments and error {error}"
    );
    buffer.addr = unsafe { (*segs).ds_addr };
    buffer.size = unsafe { (*segs).ds_len };
    buffer.state = None;
    // TODO: call buffer.state.map_fn
    //rtkit.
}

fn rtkit_alloc(
    rtkit: &'static RTKit,
    req_size: bus_size_t,
    get_buffer: fn(&RTKit) -> RefMut<RTKitBuffer>,
) -> Result<()> {
    let dev = rtkit.client;
    let mut buffer = get_buffer(&rtkit);
    let parent_tag = bus_get_dma_tag(dev);
    buffer.tag = bus_dma_tag_create(parent_tag)
        .alignment(PAGE_SIZE) /* 16K alignment */
        .max_size(req_size)
        .max_seg_size(req_size)
        .flags(Some(BUS_DMA_COHERENT))
        .build()
        .inspect_err(|e| {
            device_println!(dev, "bus_dma_tag_create failed {e}");
        })?;
    buffer.map = bus_dmamem_alloc(buffer.tag.0, &mut buffer.kva, BUS_DMA_WAITOK | BUS_DMA_ZERO)
        .inspect_err(|e| {
            device_println!(dev, "bus_dmamem_alloc failed {e}");
        })?;

    // these should be initialized by the callback in the next function
    buffer.size = req_size;
    buffer.addr = 0;
    buffer.state = Some(rtkit.clone());
    //let buffer_cref: FatPtr<RTKitBuffer> = rtkit.project(|rtk| get_buffer(rtk));

    //let rc = bus_dmamap_load(
    //    buffer.tag,
    //    buffer.map,
    //    buffer.kva,
    //    req_size,
    //    Some(rtkit_dmamap_cb),
    //    // TODO: don't leak the rtkit buffer
    //    //null_mut(), //buffer,
    //    //unsafe { buffer_cref.leak_ref() }.0.cast::<c_void>(),
    //    (buffer.deref() as *const RTKitBuffer)
    //        .cast::<c_void>()
    //        .cast_mut(),
    //    None,
    //);
    //use core::ops::Deref;
    //if (rc != Ok(())) && (rc != Err(EINPROGRESS)) {
    //    device_println!(dev, "bus_dmamap_load failed {rc:?}");
    //    return rc;
    //}

    Ok(())
}

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
