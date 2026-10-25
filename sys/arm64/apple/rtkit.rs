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

use apple_mbox::{
    AppleMboxDriver, AppleMboxMsg, MboxDevice, apple_mbox_get_dev, apple_mbox_read_msgs,
    apple_mbox_setup_intr,
};
use core::ffi::c_void;
use core::mem::transmute;
use core::pin::Pin;
use core::ptr;
use core::slice;
use core::ops::Deref;
use core::sync::atomic::{AtomicU16, AtomicU32, AtomicU64, Ordering};
use kpi::bindings::{bus_addr_t, bus_dma_segment_t, bus_size_t};
use kpi::bus::dma::{BusDmaMap, BusDmaMem, BusDmaTag};
use kpi::device::{Device, DeviceIf, BusyDevice};
use kpi::ffi::{ArrayCString, Ptr, Softc, SubClass, Embeds, PinProject};
use kpi::prelude::*;
use kpi::define_projectable;
use kpi::sync::mtx::SpinLock;
use kpi::sync::{Checked, OnceInit};
use kpi::taskqueue::{Task, Taskqueue};
use kpi::vec::VecDeque;
use core::ffi::CStr;

// Endpoints
const EP_MGMT: u32 = 0;
const EP_CRASHLOG: u32 = 1;
const EP_SYSLOG: u32 = 2;
const EP_DEBUG: u32 = 3;
const EP_IOREPORT: u32 = 4;
const EP_OSLOG: u32 = 8;
const EP_TRACEKIT: u32 = 10;

// Management message types
const MGMT_HELLO: u8 = 1;
const MGMT_HELLO_ACK: u8 = 2;
const MGMT_START_EP: u8 = 5;
const MGMT_IOP_PWR: u8 = 6;
const MGMT_IOP_PWR_ACK: u8 = 7;
const MGMT_EP_MAP: u8 = 8;
const MGMT_AP_PWR: u8 = 11;
const MGMT_AP_PWR_ACK: u8 = MGMT_AP_PWR;

const MAXVER_SHIFT: u64 = 16;
const EP_MAP_BASE_SHIFT: u64 = 32;
const EP_MAP_BASE_WIDTH: u64 = 0x07;
const EP_MAP_LAST_SHIFT: u64 = 51;
const EP_MAP_MORE: u64 = 1 << 0;
const START_EP_SHIFT: u64 = 32;
const START_EP_START: u64 = 1 << 1;

const MSG_TYPE_SHIFT: u64 = 52;
const OSLOG_TYPE_SHIFT: u64 = 56;

fn mgmt_msg_type(x: u64) -> u8 {
    (x >> MSG_TYPE_SHIFT) as u8
}

fn oslog_msg_type(x: u64) -> u8 {
    (x >> OSLOG_TYPE_SHIFT) as u8
}

const BUFFER_REQUEST: u8 = 1;
const BUFFER_SIZE_SHIFT: u64 = 44;

const OSLOG_BUFFER_SIZE_SHIFT: u64 = 36;

fn buffer_addr(x: u64) -> u64 {
    let mask = (1 << BUFFER_SIZE_SHIFT) - 1;
    x & mask
}

fn buffer_size(x: u64) -> u64 {
    (x >> BUFFER_SIZE_SHIFT) & u64::from(u8::MAX)
}

fn oslog_buffer_addr(x: u64) -> u64 {
    let mask = (1 << OSLOG_BUFFER_SIZE_SHIFT) - 1;
    x & mask
}

fn oslog_buffer_size(x: u64) -> u64 {
    (x >> OSLOG_BUFFER_SIZE_SHIFT) & 0xFFFFF
}

fn syslog_init_entries(x: u64) -> u64 {
    x & u64::from(u8::MAX)
}

fn syslog_init_msg_size(x: u64) -> u64 {
    (x >> 24) & u64::from(u8::MAX)
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

pub type Endpoint = u32;

struct IoReportTxMsg {
    ack: u64,
}

impl Into<AppleMboxMsg> for IoReportTxMsg {
    fn into(self) -> AppleMboxMsg {
        AppleMboxMsg {
            data0: self.ack,
            data1: EP_IOREPORT,
        }
    }
}

struct OsLogTxMsg {
    ack: u64,
}

impl Into<AppleMboxMsg> for OsLogTxMsg {
    fn into(self) -> AppleMboxMsg {
        AppleMboxMsg {
            data0: self.ack,
            data1: EP_OSLOG,
        }
    }
}

struct SysLogTxMsg {
    ack: u64,
}

impl Into<AppleMboxMsg> for SysLogTxMsg {
    fn into(self) -> AppleMboxMsg {
        AppleMboxMsg {
            data0: self.ack,
            data1: EP_SYSLOG,
        }
    }
}
#[derive(Debug)]
enum MgmtRxMsg {
    Hello { minver: u16, maxver: u16 },
    IopPwrAck(u16),
    ApPwrAck(u16),
    EpMap { base: u8, bitmap: u32, last: bool },
}

enum MgmtTxMsg {
    HelloAck(u16),
    IopPwrReq(u16),
    ApPwrReq(u16),
    StartEp { ep: Endpoint },
    EpMap { base: u8, last: bool },
}

impl Into<AppleMboxMsg> for MgmtTxMsg {
    fn into(self) -> AppleMboxMsg {
        let data0 = match self {
            Self::HelloAck(ver) => {
                let minver = u64::from(ver);
                let maxver = u64::from(ver) << MAXVER_SHIFT;
                let ty = u64::from(MGMT_HELLO_ACK) << MSG_TYPE_SHIFT;
                minver | maxver | ty
            }
            Self::IopPwrReq(pwr_state) => {
                let req_state = u64::from(pwr_state);
                let ty = u64::from(MGMT_IOP_PWR) << MSG_TYPE_SHIFT;
                req_state | ty
            }
            Self::ApPwrReq(pwr_state) => {
                let req_state = u64::from(pwr_state);
                let ty = u64::from(MGMT_AP_PWR) << MSG_TYPE_SHIFT;
                req_state | ty
            }
            Self::EpMap { base, last } => {
                let res = (u64::from(base) << EP_MAP_BASE_SHIFT)
                    | (u64::from(MGMT_EP_MAP) << MSG_TYPE_SHIFT);
                if last {
                    res | (1 << EP_MAP_LAST_SHIFT)
                } else {
                    res | EP_MAP_MORE
                }
            }
            Self::StartEp { ep } => {
                let mut res = u64::from(ep) << START_EP_SHIFT;
                res |= START_EP_START;
                res |= u64::from(MGMT_START_EP) << MSG_TYPE_SHIFT;
                res
            }
        };
        AppleMboxMsg {
            data0,
            data1: EP_MGMT,
        }
    }
}

impl MgmtRxMsg {
    pub fn new(data0: u64) -> Result<Self> {
        let msg_ty = mgmt_msg_type(data0);
        match msg_ty {
            MGMT_HELLO => {
                let minver = data0 as u16;
                let maxver = (data0 >> MAXVER_SHIFT) as u16;
                Ok(Self::Hello { minver, maxver })
            }
            MGMT_IOP_PWR_ACK => Ok(Self::IopPwrAck(data0 as u16)),
            MGMT_AP_PWR_ACK => Ok(Self::ApPwrAck(data0 as u16)),
            MGMT_EP_MAP => {
                let base = ((data0 >> EP_MAP_BASE_SHIFT) & EP_MAP_BASE_WIDTH) as u8;
                let bitmap = data0 as u32;
                let last = data0 & (1 << EP_MAP_LAST_SHIFT) != 0;
                Ok(Self::EpMap { base, bitmap, last })
            }
            _ => Err(EINVAL),
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
                let shift = if ep != EP_OSLOG {
                    MSG_TYPE_SHIFT
                } else {
                    OSLOG_TYPE_SHIFT
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

#[derive(Debug, PartialEq, Eq, Copy, Clone)]
enum RTKitBufferId {
    IoReport,
    CrashLog,
    OsLog,
    SysLog,
}

#[derive(Debug)]
struct RTKitBuffer {
    addr: bus_addr_t,
    size: bus_size_t,
    kva: BusDmaMem,
    tag: BusDmaTag,
    map: BusDmaMap,
    id: RTKitBufferId,
}

impl RTKitBuffer {
    fn new(id: RTKitBufferId) -> Self {
        Self {
            addr: Default::default(),
            size: Default::default(),
            kva: Default::default(),
            tag: Default::default(),
            map: Default::default(),
            id,
        }
    }
}

type RTKitTaskCallback = fn(Device<'static>, u64) -> Result<()>;
type RTKitMapCallback = fn(Device, bus_addr_t, bus_size_t);

define_projectable! {
    #[derive(Debug)]
    pub struct RTKit {
        client: Device<'static>,
        // A Device with the lifetime 'static. This is allowed since the mailbox cannot be detached
        mbox: MboxDevice,
        pub verbose: bool,
        //pub no_alloc: bool,
        iop: AtomicU16,
        ap: AtomicU16,
        ep_map: AtomicU64,
        queue: Taskqueue,
        task: Task,
        msgs: SpinLock<VecDeque<AppleMboxMsg>>,
        ioreport: Checked<RTKitBuffer>,
        crashlog: Checked<RTKitBuffer>,
        oslog: Checked<RTKitBuffer>,
        syslog: Checked<RTKitBuffer>,
        // Set by the syslog endpoint's init message and used to decode its log messages
        syslog_msg_size: AtomicU32,
        pub map_callback: Option<RTKitMapCallback>,
        ep_callback: OnceInit<(RTKitTaskCallback, Endpoint)>,
    }
}

impl RTKit {
    pub fn new(client: Device<'static>) -> Result<Self> {
        let mbox = apple_mbox_get_dev(client)?;
        Ok(Self {
            client,
            mbox,
            verbose: false,
            iop: AtomicU16::new(PwrState::Sleep as u16),
            ap: AtomicU16::new(PwrState::Sleep as u16),
            ep_map: AtomicU64::new(0),
            queue: Taskqueue::new(),
            task: Task::new(),
            msgs: SpinLock::new(VecDeque::with_capacity(32, M_WAITOK)),
    
            ioreport: Checked::new(RTKitBuffer::new(RTKitBufferId::IoReport)),
            crashlog: Checked::new(RTKitBuffer::new(RTKitBufferId::CrashLog)),
            oslog: Checked::new(RTKitBuffer::new(RTKitBufferId::OsLog)),
            syslog: Checked::new(RTKitBuffer::new(RTKitBufferId::SysLog)),
            syslog_msg_size: AtomicU32::new(0),

            map_callback: None,
            ep_callback: OnceInit::uninit(),
        })
    }

    pub fn init<T: Embeds<RTKit> + Sync>(self: Pin<&RTKit>, sc: Ptr<T>) -> Result<()> {
        let queue_name = ArrayCString::new(c"rtkit queue");
        taskqueue_create(queue_name, M_WAITOK, self.project().queue)?;
    
        let thread_name = ArrayCString::new(c"rtkit thread");
        taskqueue_start_threads(&self.queue, 1, PI_INTR, thread_name)?;
    
        mtx_init(self.project().msgs, c"rtk lock", None, None);
        self.task.init(rtkit_rx_task, sc.get_ptr())?;
        apple_mbox_setup_intr(self.mbox, Some(rtkit_intr), sc)?;
        Ok(())
    }

    pub fn send<M: Into<AppleMboxMsg>>(&self, msg: M) -> Result<()> {
        wmb!();
        AppleMboxDriver::write_msg(self.mbox, msg.into())
    }

    pub fn boot(&self) -> Result<()> {
        rtkit_set_iop(self, PwrState::On)
    }


    pub fn start_endpoint(&self, ep: Endpoint, callback: RTKitTaskCallback) -> Result<()> {
        if ep < 32 || ep >= 64 {
            device_println!(self.client, "invalid endpoint {ep:?}");
            return Err(EINVAL);
        }
        device_println!(self.client, "waiting on ep map response");
        //tsleep(&self.ep_map, Some(PWAIT), c"epmap", 5 * hz())?;
        device_println!(self.client, "waited on ep map response");
        let ep_map = self.ep_map.load(Ordering::Relaxed);
        if ep_map & (1 << ep) == 0 {
            device_println!(self.client, "endpoint {ep:?} not set in map {ep_map:x?}");
            return Err(EINVAL);
        }
        self.ep_callback.init((callback, ep));
        let start_ep = MgmtTxMsg::StartEp { ep };
        self.send(start_ep)?;
        if self.verbose {
            device_println!(self.client, "started endpoint {ep:?}");
        }
        Ok(())
    }
}

fn rtkit_set_iop(rtk: &RTKit, pwr_state: PwrState) -> Result<()> {
    assert!(!cold());
    let pwr_state = pwr_state as u16;
    if rtk.iop.load(Ordering::Relaxed) == (pwr_state & 0xFF) {
        device_println!(rtk.client, "RTKit already in power state {pwr_state:x?}");
        return Ok(());
    }
    let msg = MgmtTxMsg::IopPwrReq(pwr_state);
    rtk.send(msg)?;

    if rtk.iop.load(Ordering::Relaxed) != (pwr_state & 0xFF) {
        if let Err(e) = tsleep(&rtk.iop, Some(PWAIT), c"ioppwr", 5 * hz()) {
            let cur = rtk.iop.load(Ordering::Relaxed);
            device_println!(rtk.client, "ioppwr sleep failed with {e} iop is {cur:x?} instead of {pwr_state:x?}");
            if cur != (pwr_state & 0xFF) {
                return Err(e);
            }
        }
    }
    Ok(())
}

pub fn rtkit_set_ap(rtk: &RTKit, pwr_state: PwrState) -> Result<()> {
    assert!(!cold());
    let pwr_state = pwr_state as u16;
    if rtk.ap.load(Ordering::Relaxed) == (pwr_state & 0xFF) {
        device_println!(rtk.client, "RTKit already in power state {pwr_state:x?}");
        return Ok(());
    }
    let msg = MgmtTxMsg::ApPwrReq(pwr_state);
    rtk.send(msg)?;

    if rtk.ap.load(Ordering::Relaxed) != (pwr_state & 0xFF) {
        tsleep(&rtk.ap, Some(PWAIT), c"appwr", 5 * hz())?;
    }
    Ok(())
}

fn rtkit_handle_mgmt(rtk: &RTKit, data0: u64) -> Result<()> {
    let msg = MgmtRxMsg::new(data0)?;
    if rtk.verbose {
        device_println!(rtk.client, "recv'd msg from rtkit {msg:x?}");
    }
    match msg {
        MgmtRxMsg::Hello { minver, maxver } => {
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
            let ack = MgmtTxMsg::HelloAck(ver);
            rtk.send(ack)?;
        }
        MgmtRxMsg::IopPwrAck(pwr_state) => {
            rtk.iop.store(pwr_state, Ordering::Relaxed);
            wakeup(&rtk.iop);
        }
        MgmtRxMsg::ApPwrAck(pwr_state) => {
            rtk.ap.store(pwr_state, Ordering::Relaxed);
            wakeup(&rtk.ap);
        }
        MgmtRxMsg::EpMap { base, bitmap, last } => {
            let new_bits = u64::from(bitmap) << u64::from(base * 32);
            let old_bitmap = rtk.ep_map.fetch_or(new_bits, Ordering::Relaxed);
            let new_bitmap = old_bitmap | new_bits;
            let reply = MgmtTxMsg::EpMap { base, last };
            rtk.send(reply)?;
            if last {
                // range bounds are not a typo, bit 0 is not a valid endpoint here
                for ep in 1..32 {
                    if new_bitmap & (1 << ep) == 0 {
                        continue;
                    }
                    match ep {
                        EP_MGMT => (),
                        EP_CRASHLOG | EP_SYSLOG | EP_DEBUG | EP_IOREPORT | EP_OSLOG
                        | EP_TRACEKIT => {
                            let start_ep = MgmtTxMsg::StartEp { ep };
                            if rtk.verbose {
                                device_println!(rtk.client, "starting endpoint {ep:?}");
                            }
                            rtk.send(start_ep)?;
                        }
                        other_ep => {
                            if rtk.verbose {
                                device_println!(rtk.client, "skipping endpoint {other_ep:?}");
                            }
                        }
                    }
                }
                //wakeup(&rtk.ep_map);
            }
        }
    }
    Ok(())
}

fn rtkit_handle_ioreport<T: Embeds<RTKit> + 'static>(rtk: &RTKit, sc: &Softc<T>, data0: u64) -> Result<()> {
    const IOREPORT_UNKNOWN1: u8 = 8;
    const IOREPORT_UNKNOWN2: u8 = 12;
    let msg_ty = mgmt_msg_type(data0);
    match msg_ty {
        BUFFER_REQUEST => {
            rtkit_handle_buffer_req(rtk, sc, EP_IOREPORT, data0, &rtk.ioreport)?;
        }
        IOREPORT_UNKNOWN1 | IOREPORT_UNKNOWN2 => {
            let echoed_msg = IoReportTxMsg { ack: data0 };
            rtk.send(echoed_msg)?;
            if rtk.verbose {
                device_println!(
                    rtk.client,
                    "Ack'ed unknown message of type {msg_ty} from RTKit ioreport endpoint"
                );
            }
        }
        _ => {
            panic!("unknown ioreport message of type {msg_ty:?}");
        }
    }
    Ok(())
}

fn rtkit_handle_crashlog<T: Embeds<RTKit> + 'static>(rtk: &RTKit, sc: &Softc<T>, data0: u64) -> Result<()> {
    let msg_ty = mgmt_msg_type(data0);
    if msg_ty != BUFFER_REQUEST {
        device_println!(
            rtk.client,
            "unexpected RTKit msg of type {msg_ty:x?} from crashlog endpoint"
        );
        return Err(EINVAL);
    }
    if rtk.crashlog.get_mut().addr != 0 {
        device_println!(rtk.client, "got another crashlog message");
        panic!("RTKit crashed");
    }
    rtkit_handle_buffer_req(rtk, sc, EP_CRASHLOG, data0, &rtk.crashlog)?;
    Ok(())
}

fn rtkit_handle_oslog<T: Embeds<RTKit> + 'static>(rtk: &RTKit, sc: &Softc<T>, data0: u64) -> Result<()> {
    const OSLOG_UNKNOWN1: u8 = 3;
    const OSLOG_UNKNOWN2: u8 = 4;
    const OSLOG_UNKNOWN3: u8 = 5;
    let msg_ty = oslog_msg_type(data0);
    match msg_ty {
        BUFFER_REQUEST => {
            rtkit_handle_buffer_req(rtk, sc, EP_OSLOG, data0, &rtk.oslog)?;
        }
        OSLOG_UNKNOWN1 | OSLOG_UNKNOWN2 | OSLOG_UNKNOWN3 => {
            let echoed_msg = OsLogTxMsg { ack: data0 };
            rtk.send(echoed_msg)?;
            if rtk.verbose {
                device_println!(
                    rtk.client,
                    "Ack'ed unknown message of type {msg_ty} from RTKit oslog endpoint"
                );
            }
        }
        _ => {
            device_println!(
                rtk.client,
                "ignoring unknown oslog message of type {msg_ty:?}"
            );
        }
    }
    Ok(())
}

fn rtkit_handle_syslog<T: Embeds<RTKit> + 'static>(rtk: &RTKit, sc: &Softc<T>, data0: u64) -> Result<()> {
    const SYSLOG_INIT: u8 = 8;
    const SYSLOG_LOG: u8 = 5;
    let msg_ty = mgmt_msg_type(data0);
    match msg_ty {
        BUFFER_REQUEST => {
            rtkit_handle_buffer_req(rtk, sc, EP_SYSLOG, data0, &rtk.syslog)?;
        }
        SYSLOG_INIT => {
            let msg_size = syslog_init_msg_size(data0).try_into().unwrap();
            rtk.syslog_msg_size.store(msg_size, Ordering::Relaxed);
        }
        SYSLOG_LOG => {
            // explicitly ignore the error instead of bubbling it up so we ack it unconditionally
            let _res = rtkit_syslog_print(rtk, data0);
            let log_ack = SysLogTxMsg { ack: data0 };
            rtk.send(log_ack)?;
        }
        _ => {
            panic!("unknown syslog message of type {msg_ty:?}");
        }
    }
    Ok(())
}

fn rtkit_syslog_print(rtk: &RTKit, data0: u64) -> Result<()> {
    let idx = (data0 & 0xff) as usize;
    let syslog_size = rtk.syslog_msg_size.load(Ordering::Relaxed) as usize;
    if syslog_size == 0 {
        return Err(EINVAL);
    }
    let entry_size = 0x20 + syslog_size;

    let buffer = rtk.syslog.get_mut();
    if (idx + 1) * entry_size > buffer.size as usize {
        device_println!(rtk.client, "syslog entry {idx:?} out of bounds");
        return Err(EINVAL);
    }

    bus_dmamap_sync(buffer.tag, buffer.map, BUS_DMASYNC_POSTREAD);
    // SAFETY: The entry was bounds checked against the buffer size above
    let entry = unsafe {
        let kva = buffer.kva.as_ptr().cast::<u8>();
        slice::from_raw_parts(kva.add(idx * entry_size), entry_size)
    };
    let ctx = CStr::from_bytes_until_nul(&entry[0x8..0x20]);
    let msg = CStr::from_bytes_until_nul(&entry[0x20..]);
    device_println!(rtk.client, "syslog: {ctx:?}: {msg:?}");
    bus_dmamap_sync(buffer.tag, buffer.map, BUS_DMASYNC_PREREAD);
    Ok(())
}

fn rtkit_handle_buffer_req<T: Embeds<RTKit> + 'static>(
    rtk: &RTKit,
    sc: &Softc<T>,
    ep: Endpoint,
    data0: u64,
    buffer: &Checked<RTKitBuffer>,
) -> Result<()> {
    let size = if ep != EP_OSLOG {
        buffer_size(data0)
    } else {
        oslog_buffer_size(data0)
    };
    let req_size = if ep != EP_OSLOG {
        size << PAGE_SHIFT_4K
    } else {
        size
    };
    if rtk.verbose {
        device_println!(
            rtk.client,
            "RTKit endpoint {ep:?} requested {req_size} byte buffer"
        );
    }

    let req_addr = if ep != EP_OSLOG {
        buffer_addr(data0)
    } else {
        oslog_buffer_addr(data0)
    };
    if req_addr != 0 {
        return Ok(());
    }
    let addr = rtkit_alloc(rtk, sc, req_size, buffer)?;
    // TODO: See comment at the end of rtkit_alloc
    assert!(addr != 0);
    let resp = if ep != EP_OSLOG {
        (size << BUFFER_SIZE_SHIFT) | buffer_addr(addr)
    } else {
        (size << OSLOG_BUFFER_SIZE_SHIFT) | oslog_buffer_addr(addr >> PAGE_SHIFT_4K)
    };
    if rtk.verbose {
        device_println!(
            rtk.client,
            "responding to RTKit buffer request with {resp:x?}"
        );
    }
    rtk.send(EpTxMsg::BufferReq { ep, data: resp })
}

fn rtkit_alloc<T: Embeds<RTKit> + 'static>(
    rtk: &RTKit,
    sc: &Softc<T>,
    req_size: bus_size_t,
    buffer: &Checked<RTKitBuffer>,
) -> Result<bus_addr_t> {
    //if rtk.no_alloc {
    //    return Ok(0);
    //}

    if sc.device().as_ptr() != rtk.client.as_ptr() {
        return Err(EDOOFUS);
    }
    let parent_tag = bus_get_dma_tag(sc.device());

    let mut buffer_guard = buffer.get_mut();

    //buffer_guard.rtk = unsafe { Ptr::from_ref(&rtk) };
    buffer_guard.tag = bus_dma_tag_create(parent_tag)
        .alignment(PAGE_SIZE)
        .max_size(req_size)
        .max_seg_size(req_size)
        .flags(Some(BUS_DMA_COHERENT))
        .build()?;
    let (map, kva) = bus_dmamem_alloc(buffer_guard.tag, BUS_DMA_WAITOK | BUS_DMA_ZERO)?;
    buffer_guard.map = map;
    buffer_guard.kva = kva;
    // buffer.size and buffer.addr will be initialized by the dmamap callback
    buffer_guard.addr = 0;
    buffer_guard.size = req_size;
    let tag = buffer_guard.tag;
    let buffer_id = buffer_guard.id;
    // The dmamap callback will grab the buffer again so explicitly drop it. We use an explicit call
    // to drop rather than a smaller scope because of all the local variables that get copied onto
    // the stack and have lifetimes that extend past the drop.
    drop(buffer_guard);

    let callback = match buffer_id {
        RTKitBufferId::IoReport => rtkit_dmamap_cb::<_, 0>,
        RTKitBufferId::CrashLog => rtkit_dmamap_cb::<_, 1>,
        RTKitBufferId::OsLog => rtkit_dmamap_cb::<_, 2>,
        RTKitBufferId::SysLog => rtkit_dmamap_cb::<_, 3>,
    };

    let res = bus_dmamap_load(tag, map, kva, req_size, Some(callback), sc.get_ptr(), None);
    match res {
        Ok(_) | Err(EINPROGRESS) => {}
        Err(e) => {
            device_println!(rtk.client, "bus_dmamap_load failed {res:?}");
            return Err(e);
        }
    }
    // TODO: There is no guarantee this will be initialized by this point here or in the C version.
    Ok(buffer.get_mut().addr)
}

extern "C" fn rtkit_dmamap_cb<T: Embeds<RTKit>, const BUFFER: usize>(
    sc: Ptr<T>,
    segs: &bus_dma_segment_t,
    nsegs: i32,
    error: i32,
) {
    let rtk = sc.as_pin().get_embedded();
    let buffer = match BUFFER {
        0 => &rtk.ioreport,
        1 => &rtk.crashlog,
        2 => &rtk.oslog,
        3 => &rtk.syslog,
        _ => unreachable!(""),
    };

    let mut buffer = buffer.get_mut();
    buffer.addr = segs.ds_addr;
    buffer.size = segs.ds_len;
    // SAFETY: The `rtk` field was initialized from a `&Softc<RTKit>` so it's non-null, cannot have
    // mutable references and couldn't have moved since initialization (or to be extremely
    // pendantic... the RTKit instance can be moved, but the address is guaranteed to point to some
    // RTKit instance until the client device is detached which is what we really care about here).
    //let rtk = unsafe { buffer.rtk.get() };
    if rtk.verbose {
        device_println!(
            rtk.client,
            "initializing RTKit buffer address to {:x?}",
            segs.ds_addr
        );
    }
    if let Some(map_cb) = rtk.map_callback {
        map_cb(rtk.client, buffer.addr, buffer.size);
    }
}

extern "C" fn rtkit_intr<T: Embeds<RTKit>>(sc: &Softc<T>) {
    let rtk = sc.as_pin().get_embedded();
    apple_mbox_read_msgs(rtk.mbox, |msg| {
        try_rx_callback(rtk, msg)
            .inspect_err(|e| {
                device_println!(rtk.client, "callback failed {e}");
            })
            .unwrap();
    });
}

fn try_rx_callback(rtk: Pin<&RTKit>, msg: AppleMboxMsg) -> Result<()> {
    rmb!();
    mtx_lock_spin(&rtk.msgs).push_back(msg);
    if rtk.verbose {
        device_println!(rtk.client, "enqueuing rtkit task");
    }
    taskqueue_enqueue(&rtk.queue, rtk.project().task)?;
    Ok(())
}

extern "C" fn rtkit_rx_task<T: Embeds<RTKit> + 'static>(sc: &Softc<T>, pending: u32) {
    let rtk = sc.as_pin().get_embedded();
    try_rx_task(rtk.get_ref(), sc, pending)
        .inspect_err(|e| {
            device_println!(rtk.client, "task fn failed {e}");
        })
        .unwrap();
}

fn try_rx_task<T: Embeds<RTKit> + 'static>(rtk: &RTKit, sc: &Softc<T>, pending: u32) -> Result<()> {
    for _ in 0..pending {
        let msg = mtx_lock_spin(&rtk.msgs).pop_front().unwrap();
        let ep = msg.data1;
        match ep {
            EP_MGMT => rtkit_handle_mgmt(rtk, msg.data0)?,
            EP_IOREPORT => rtkit_handle_ioreport(rtk, sc, msg.data0)?,
            EP_CRASHLOG => rtkit_handle_crashlog(rtk, sc, msg.data0)?,
            EP_SYSLOG => rtkit_handle_syslog(rtk, sc, msg.data0)?,
            EP_DEBUG => {
                todo!("")
            }
            EP_OSLOG => rtkit_handle_oslog(rtk, sc, msg.data0)?,
            EP_TRACEKIT => {
                todo!("")
            }
            other_ep => {
                let (callback, started_ep) = rtk.ep_callback.get();
                assert!(other_ep == *started_ep);
                callback(rtk.client, msg.data0)?;
            }
        }
    }
    Ok(())
}
