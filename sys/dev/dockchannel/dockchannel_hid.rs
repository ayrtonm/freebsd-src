/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Ayrton Muñoz
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

use apple_rtkit::AppleRTKitDriver;
use apple_smc::AppleSmcDriver;
use core::cmp::min;
use core::ffi::{CStr, c_char, c_void};
use core::mem::MaybeUninit;
use core::slice;
use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use dockchannel::DockchannelDriver;
use hid::HidIf;
use kpi::bindings::{device_t, hid_device_info, hid_intr_t, hid_rdesc_info, hid_size_t, phandle_t};
use kpi::boxed::{Box, LinkedList};
use kpi::bus::{Filter, Irq, Register, Resource, SysRes};
use kpi::device::{BusProbe, DeviceIf};
use kpi::ffi::{ArrayCString, Ptr, Ref, UninitRef};
use kpi::intr::ConfigHook;
use kpi::ofw::{Node, XRef};
use kpi::prelude::*;
use kpi::sync::mtx::Mutex;
use kpi::sync::{Checked, OnceInit};
use kpi::taskqueue::{Task, Taskqueue};
use kpi::vec::Vec;
use kpi::{ErrCode, driver, proj};

const CONFIG_TX_THRESH: u64 = 0;
const CONFIG_RX_THRESH: u64 = 0x4;

const DATA_TX8: u64 = 0x4;
const DATA_TX16: u64 = 0x8;
const DATA_TX24: u64 = 0xc;
const DATA_TX32: u64 = 0x10;
const DATA_TX_FREE: u64 = 0x14;
const DATA_RX8: u64 = 0x1c;
const DATA_RX16: u64 = 0x20;
const DATA_RX24: u64 = 0x24;
const DATA_RX32: u64 = 0x28;
const DATA_RX_COUNT: u64 = 0x2c;

const MTP_IFACE_COMM: u8 = 0;

const MTP_CMD_IFACE_RESET: u8 = 0x40;
const MTP_CMD_SEND_FIRMWARE: u8 = 0x95;
const MTP_CMD_IFACE_ENABLE: u8 = 0xb4;
const MTP_CMD_ACK_GPIO_CMD: u8 = 0xa1;
const MTP_CMD_GET_DIMENSIONS: u8 = 0xd9;

const HID_DESC_MAX: usize = 512;

#[repr(packed)]
#[derive(Default)]
struct MtpHdr {
    hdr_len: u8,
    chan: u8,
    pkt_len: u16,
    seq: u8,
    iface: u8,
    pad: u16,
}

impl MtpHdr {
    fn as_slice(&self) -> &[u8] {
        let ptr = self as *const Self as *const u8;
        unsafe { slice::from_raw_parts(ptr, size_of::<Self>()) }
    }
    fn as_mut_slice(&mut self) -> &mut [u8] {
        let ptr = self as *mut Self as *mut u8;
        unsafe { slice::from_raw_parts_mut(ptr, size_of::<Self>()) }
    }
}

#[repr(packed)]
#[derive(Debug, Default, PartialEq, Eq, Copy, Clone)]
struct MtpSubHdr {
    flags: u8,
    unk: u8,
    len: u16,
    retcode: u32,
}

#[repr(u8)]
#[derive(Debug, PartialEq, Eq, Copy, Clone)]
enum MtpGroup {
    Input = 0,
    Output,
    Cmd,
}

#[repr(u8)]
#[derive(Debug, PartialEq, Eq, Copy, Clone)]
enum MtpReq {
    SetReport = 0,
    GetReport,
}

const MTP_CHAN_CMD: u8 = 0x11;
const MTP_CHAN_REPORT: u8 = 0x12;

const MTP_REQ_SHIFT: u8 = 0;
const MTP_GROUP_SHIFT: u8 = 6;

macro_rules! impl_from_slice {
    ($type_name:ident) => {
        impl $type_name {
            fn from_slice(s: &[u8]) -> &Self {
                assert!(s.len() >= size_of::<Self>());
                unsafe { s.as_ptr().cast::<Self>().as_ref().unwrap() }
            }

            fn as_slice(&self) -> &[u8] {
                let ptr = self as *const Self as *const u8;
                unsafe { slice::from_raw_parts(ptr, size_of::<Self>()) }
            }

            fn as_mut_slice(&mut self) -> &mut [u8] {
                let ptr = self as *mut Self as *mut u8;
                unsafe { slice::from_raw_parts_mut(ptr, size_of::<Self>()) }
            }
        }
    };
}

impl MtpSubHdr {
    fn mtp_req(&self) -> Result<MtpReq> {
        let req = self.flags & 0x3f;
        match req {
            x if x == MtpReq::SetReport as u8 => Ok(MtpReq::SetReport),
            x if x == MtpReq::GetReport as u8 => Ok(MtpReq::GetReport),
            _ => Err(EINVAL),
        }
    }
    fn mtp_group(&self) -> Result<MtpGroup> {
        let group = (self.flags >> MTP_GROUP_SHIFT) & 0x3;
        match group {
            x if x == MtpGroup::Input as u8 => Ok(MtpGroup::Input),
            x if x == MtpGroup::Output as u8 => Ok(MtpGroup::Output),
            x if x == MtpGroup::Cmd as u8 => Ok(MtpGroup::Cmd),
            _ => Err(EINVAL),
        }
    }
}

impl_from_slice!(MtpSubHdr);

#[repr(packed)]
struct MtpInitHdr {
    ty: u8,
    unk0: u8,
    unk1: u8,
    iface: u8,
    name: [c_char; 16],
}

const MTP_EVENT_GPIO_CMD: u8 = 0xa0;
const MTP_EVENT_INIT: u8 = 0xf0;
const MTP_EVENT_READY: u8 = 0xf1;

impl_from_slice!(MtpInitHdr);

#[repr(packed)]
struct MtpInitBlockHdr {
    ty: u16,
    subty: u16,
    len: u16,
}

impl_from_slice!(MtpInitBlockHdr);

const MTP_BLOCK_DESCRIPTOR: u16 = 0;
const MTP_BLOCK_GPIO_REQ: u16 = 1;
const MTP_BLOCK_END: u16 = 2;

#[repr(packed)]
struct MtpGpioReq {
    unk: u16,
    id: u16,
    name: [c_char; 32],
}

impl_from_slice!(MtpGpioReq);

#[repr(packed)]
struct MtpGpioCmd {
    ty: u8,
    iface: u8,
    gpio: u8,
    unk: u8,
    cmd: u8,
}

impl_from_slice!(MtpGpioCmd);

#[repr(packed)]
struct MtpGpioAck {
    ty: u8,
    retcode: u32,
    cmd: [u8; 512],
}

impl_from_slice!(MtpGpioAck);

const STM_REPORT_ID: u8 = 0x10;
const STM_REPORT_SERIAL: u8 = 0x11;

#[repr(packed)]
#[derive(Debug, Default, Copy, Clone)]
struct DockchannelHidStmId {
    unk0: u8,
    vendor_id: u16,
    product_id: u16,
    version: u16,
    unk1: u16,
    keyboard_ty: u8,
    serial_len: u8,
}

impl_from_slice!(DockchannelHidStmId);

#[derive(Debug, Default, Copy, Clone)]
struct DockchannelCmdRet {
    seq: u8,
    code: u32,
}

#[derive(Debug, Default)]
struct DockchannelHidIface {
    iface: OnceInit<u8>,
    desc: OnceInit<Vec<u8, M_DEVBUF>>,
    ready: AtomicBool,
    seq: AtomicU8,
    ret: Mutex<LinkedList<DockchannelCmdRet>>,
    smc: OnceInit<XRef>,
    gpio: OnceInit<u32>,
}

#[derive(Debug, Default)]
struct DockchannelHidChild {
    hidbus: OnceInit<device_t>,
    hw: Checked<hid_device_info>,
    handler: OnceInit<hid_intr_t>,
    ctx: OnceInit<Ptr<c_void>>,
}

#[derive(Debug)]
struct DockchannelHidRegister {
    config: Register,
    data: Register,
}

#[derive(Debug)]
pub struct DockchannelHidSoftc {
    dev: device_t,
    node: Node,
    mtp: XRef,
    fifo_size: u32,
    task: Task,
    hook: ConfigHook,
    regs: Mutex<DockchannelHidRegister>,
    //config: Checked<Register>,
    //data: Checked<Register>,
    rx_irq: Irq,
    tx_irq: Irq,
    rx_avail: AtomicBool,
    tx_avail: AtomicBool,
    queue: Taskqueue,
    comm: DockchannelHidIface,
    stm: DockchannelHidIface,
    kbd: DockchannelHidIface,
    mt: DockchannelHidIface,

    kbd_hid: DockchannelHidChild,
    mt_hid: DockchannelHidChild,

    stm_id: Checked<DockchannelHidStmId>,
    serial: Checked<[c_char; 64]>,
}

fn dockchannel_alloc_res(
    dev: device_t,
    node: Node,
    sys_ty: SysRes,
    name: &CStr,
) -> Result<Resource> {
    let res_names = match sys_ty {
        SYS_RES_IRQ => c"interrupt-names",
        SYS_RES_MEMORY => c"reg-names",
        _ => {
            return Err(EINVAL);
        }
    };
    let idx = ofw_bus_find_string_index(node, res_names, name)?;
    let res = bus_alloc_resource_any(dev, sys_ty, idx, RF_ACTIVE)?;
    Ok(res)
}

impl DeviceIf for DockchannelHidDriver {
    type Softc = DockchannelHidSoftc;

    fn device_probe(dev: device_t) -> Result<BusProbe> {
        if !ofw_bus_status_okay(dev) {
            return Err(ENXIO);
        }
        if !ofw_bus_is_compatible(dev, c"apple,dockchannel-hid") {
            return Err(ENXIO);
        }
        device_set_desc(dev, c"Apple DockChannel HID");
        Ok(BUS_PROBE_DEFAULT)
    }

    fn device_attach(uninit_sc: UninitRef<DockchannelHidSoftc>, dev: device_t) -> Result<()> {
        let node = ofw_bus_get_node(dev);
        let mtp = unsafe {
            OF_getencprop_unchecked::<XRef>(node, c"apple,helper-cpu").map_err(|e| {
                device_println!(dev, "couldn't find apple,helper-cpu devicetree prop {e}");
                ENXIO
            })?
        };
        let fifo_size = OF_getencprop::<u32>(node, c"apple,fifo-size").map_err(|e| {
            device_println!(dev, "couldn't find apple,fifo-size devicetree prop {e}");
            ENXIO
        })?;
        let rx_irq = dockchannel_alloc_res(dev, node, SYS_RES_IRQ, c"rx")?.into_irq()?;
        let tx_irq = dockchannel_alloc_res(dev, node, SYS_RES_IRQ, c"tx")?.into_irq()?;
        let config =
            dockchannel_alloc_res(dev, node, SYS_RES_MEMORY, c"config")?.into_register()?;
        let data = dockchannel_alloc_res(dev, node, SYS_RES_MEMORY, c"data")?.into_register()?;
        let task = task_init!(dev, dockchannel_hid_task);
        let hook = config_intrhook_init!(dev, dockchannel_hid_start_config_hook);
        let queue = Taskqueue::new();

        let mut sc = uninit_sc.init(DockchannelHidSoftc {
            dev,
            node,
            mtp,
            fifo_size,
            task,
            hook,
            regs: Mutex::new(DockchannelHidRegister { config, data }),
            rx_irq,
            tx_irq,
            rx_avail: AtomicBool::new(true),
            tx_avail: AtomicBool::new(true),
            queue,
            comm: DockchannelHidIface::default(),
            stm: DockchannelHidIface::default(),
            kbd: DockchannelHidIface::default(),
            mt: DockchannelHidIface::default(),

            kbd_hid: DockchannelHidChild::default(),
            mt_hid: DockchannelHidChild::default(),

            stm_id: Checked::new(DockchannelHidStmId::default()),
            serial: Checked::new([0; 64]),
        });

        sc.comm.iface.init(MTP_IFACE_COMM);
        sc.comm.seq = AtomicU8::new(0);
        sc.stm.seq = AtomicU8::new(0);
        let sc = sc.into_ref();
        mtx_init(proj!(&sc->comm->ret), c"", None, None);
        mtx_init(proj!(&sc->stm->ret), c"", None, None);
        mtx_init(proj!(&sc->kbd->ret), c"", None, None);
        mtx_init(proj!(&sc->mt->ret), c"", None, None);
        mtx_init(proj!(&sc->regs), c"", None, Some(MTX_RECURSE));

        taskqueue_create_fast(
            ArrayCString::new(c"dockchannel taskq"),
            M_WAITOK,
            proj!(&sc->queue),
        )?;
        taskqueue_start_threads(
            proj!(&sc->queue),
            8,
            PWAIT,
            ArrayCString::new(c"dockchannel thead"),
        )?;

        config_intrhook_establish(&sc.hook).unwrap();

        Ok(())
    }
}

extern "C" fn dockchannel_hid_start_config_hook(sc: Ref<DockchannelHidSoftc>) {
    DockchannelHidDriver::try_dockchannel_hid_start_config_hook(sc).unwrap()
}

impl DockchannelHidDriver {
    fn try_dockchannel_hid_start_config_hook(sc: Ref<DockchannelHidSoftc>) -> Result<()> {
        let dev = sc.dev;
        AppleRTKitDriver::boot_helper(dev, sc.mtp)?;
        //device_println!(dev, "booted apple_rtkit helper");

        let regs = mtx_lock(&sc.regs);
        let mut config = &regs.config;
        bus_write_4!(config, CONFIG_RX_THRESH, 8);
        bus_write_4!(config, CONFIG_TX_THRESH, 8);
        mtx_unlock(regs);
        //device_println!(dev, "set config rx/tx threshold");

        bus_setup_intr!(
            dev,
            proj!(&sc->rx_irq),
            INTR_MPSAFE.0 | INTR_TYPE_MISC.0,
            Some(dockchannel_hid_rx_filter),
            None,
        )
        .inspect_err(|e| {
            device_println!(dev, "couldn't set up rx irq {e}");
        })?;

        bus_setup_intr!(
            dev,
            proj!(&sc->tx_irq),
            INTR_MPSAFE.0 | INTR_TYPE_MISC.0,
            Some(dockchannel_hid_tx_filter),
            None,
        )
        .inspect_err(|e| {
            device_println!(dev, "couldn't set up tx irq {e}");
        })?;

        config_intrhook_disestablish(&sc.hook);

        Ok(())
    }
}

extern "C" fn dockchannel_hid_rx_filter(sc: Ref<DockchannelHidSoftc>) -> Filter {
    DockchannelDriver::mask_rx(sc.dev);
    if !sc.rx_avail.load(Ordering::Relaxed) {
        sc.rx_avail.store(true, Ordering::Relaxed);
        wakeup(&sc.rx_avail);

        return FILTER_HANDLED;
    }

    taskqueue_enqueue(&sc.queue, proj!(&sc->task)).unwrap();
    FILTER_HANDLED
}

extern "C" fn dockchannel_hid_tx_filter(sc: Ref<DockchannelHidSoftc>) -> Filter {
    DockchannelDriver::mask_tx(sc.dev);
    sc.tx_avail.store(true, Ordering::Relaxed);
    wakeup(&sc.tx_avail);
    FILTER_HANDLED
}

extern "C" fn dockchannel_hid_task(sc: Ref<DockchannelHidSoftc>, pending: u32) {
    try_dockchannel_hid_task(sc).unwrap();
}

static BUF: Checked<Vec<u8, M_DEVBUF>> = Checked::new(Vec::new());

// This function calls dockchannel_hid_read twice to fill in two buffers. The first is the
// following MtpHdr on the stack and the second is a heap-allocated u8 buffer whose length is
// determined by the packet length in the MtpHdr. The second is then split into an MtpSubHdr and
// another u8 buffer for the rest. The second u8 buffer is then repeatedly split as the
// preceeding parts are read and used to determine how the remaining buffer should be
// interpreted. Note that each "split" is purely a type-system transformation to simplify
// bookkeeping and the buffer itself is not moved or copied after its read into the initial
// Vec<u8>.
fn try_dockchannel_hid_task(sc: Ref<DockchannelHidSoftc>) -> Result<()> {
    let dev = sc.dev;

    let mut hdr = MtpHdr::default();
    dockchannel_hid_read(sc, hdr.as_mut_slice()).inspect_err(|e| {
        device_println!(dev, "failed to read header {e:?}");
        let _ = dockchannel_hid_task_done(sc);
    })?;

    if usize::from(hdr.hdr_len) != size_of::<MtpHdr>() {
        device_println!(dev, "inconsistent header length {:x?}", hdr.hdr_len);
        return dockchannel_hid_task_done(sc);
    }

    let pkt_len = usize::from(hdr.pkt_len) + 4;
    if pkt_len < size_of::<MtpSubHdr>() {
        device_println!(
            dev,
            "packet size of {pkt_len:?} bytes too small for a single subheader",
        );
        return dockchannel_hid_task_done(sc);
    }

    if pkt_len > 1024 {
        device_println!(
            dev,
            "packet size of {pkt_len:?} bytes is unreasonably large"
        );
        return dockchannel_hid_task_done(sc);
    }
    let mut vec = Vec::fill_with_capacity(0u8, pkt_len, M_WAITOK);
    let mut buf: Box<[u8], M_DEVBUF> = vec.into_boxed_slice();

    // Fill in the entire second buffer
    dockchannel_hid_read(sc, &mut buf).inspect_err(|e| {
        device_println!(dev, "failed to read packet of length {pkt_len:?} {e:?}",);
        let _ = dockchannel_hid_task_done(sc);
    })?;
    let checksum = dockchannel_hid_checksum(hdr.as_slice()) + dockchannel_hid_checksum(&buf);

    if checksum != u32::MAX {
        device_println!(dev, "bad checksum {checksum:x?}");
        return dockchannel_hid_task_done(sc);
    }

    // Split the buffer into the MTP subheader and the rest. The MTP subheader will be used to
    // decide how to interpret remaining_buf.
    let (subhdr_buf, remaining_buf) = buf.split_at_mut(size_of::<MtpSubHdr>());

    let subhdr = MtpSubHdr::from_slice(subhdr_buf);
    let group = subhdr
        .mtp_group()
        .inspect_err(|e| device_println!(dev, "unknown MTP group in subhdr {subhdr:x?} {e:?}"))?;
    if group == MtpGroup::Output || group == MtpGroup::Cmd {
        // Check if the iface field in the MtpHdr matches sc.comm.iface or sc.stm.iface after
        // they've been initialized.
        if let Some(iface) = sc.comm.iface.try_get()
            && *iface == hdr.iface
        {
            mtx_lock(&sc.comm.ret).push_back(DockchannelCmdRet {
                code: subhdr.retcode,
                seq: hdr.seq,
            }, M_WAITOK);
            wakeup(&sc.comm.ret);
        } else if let Some(iface) = sc.stm.iface.try_get()
            && *iface == hdr.iface
        {
            mtx_lock(&sc.stm.ret).push_back(DockchannelCmdRet {
                code: subhdr.retcode,
                seq: hdr.seq,
            }, M_WAITOK);
            if subhdr.mtp_req() == Ok(MtpReq::GetReport) {
                let len = usize::from(subhdr.len);
                device_println!(dev, "copying {:?} bytes", len);
                let mut gbuf = BUF.get_mut();
                *gbuf = Vec::fill_with_capacity(0u8, len, M_WAITOK);
                gbuf.copy_from_slice(&remaining_buf[0..len]);
            }
            device_println!(
                sc.dev,
                "waking up sc.stm.ret {:p} for seq {:?}",
                &sc.stm.ret,
                hdr.seq
            );
            wakeup(&sc.stm.ret);
        } else {
            panic!("got unexpected ack from iface {:x?}", hdr.iface);
        };
        return Ok(());
    }

    match hdr.chan {
        MTP_CHAN_CMD => {
            //dockchannel_handle_ack(sc, ...
        }
        MTP_CHAN_REPORT => {
            if hdr.iface == MTP_IFACE_COMM {
                //device_println!(sc.dev, "remaining len: {:?}, subhdr.len: {:?}", remaining_buf.len(), usize::from(subhdr.len));
                let len = usize::from(subhdr.len);
                dockchannel_handle_comm(sc, &mut remaining_buf[..len])?;
            } else if let Some(iface) = sc.kbd.iface.try_get()
                && *iface == hdr.iface
                && sc.kbd.ready.load(Ordering::Relaxed)
            {
                let handler = sc.kbd_hid.handler.get().unwrap();
                let ptr = remaining_buf.as_ptr().cast::<c_void>().cast_mut();
                let buf_len = u32::from(subhdr.len);
                unsafe { handler(sc.kbd_hid.ctx.get().as_ptr(), ptr, buf_len) }
            };
        }
        _ => {
            device_println!(dev, "ignored packet from unknown channel {:x?}", hdr.chan);
        }
    }

    dockchannel_hid_task_done(sc)
}

fn dockchannel_hid_task_done(sc: Ref<DockchannelHidSoftc>) -> Result<()> {
    sc.rx_avail.store(true, Ordering::Relaxed);
    /* unmask rx irq to get ack */
    DockchannelDriver::unmask_rx(sc.dev);
    Ok(())
}

fn dockchannel_handle_comm(sc: Ref<DockchannelHidSoftc>, buf: &mut [u8]) -> Result<()> {
    let len = size_of::<MtpInitHdr>();
    if buf.len() < len {
        return Ok(());
    }
    //device_println!(sc.dev, "splitting buffer at {len:?}");
    let (ihdr_buf, remaining_buf) = buf.split_at_mut(len);
    let ihdr = MtpInitHdr::from_slice(ihdr_buf);
    match ihdr.ty {
        MTP_EVENT_INIT => {
            let dcif = if ihdr.name.starts_with(b"stm") && !sc.stm.iface.is_init() {
                Some(&sc.stm)
            } else if ihdr.name.starts_with(b"keyboard") && !sc.kbd.iface.is_init() {
                Some(&sc.kbd)
            } else if ihdr.name.starts_with(b"multi-touch") && !sc.mt.iface.is_init() {
                Some(&sc.mt)
            } else {
                None
            };
            if let Some(dcif) = dcif {
                dcif.iface.init(ihdr.iface);
                dockchannel_handle_init(sc, ihdr.iface, remaining_buf)?;
                dockchannel_hid_iface_enable(sc, dcif)?;
            };
        }
        MTP_EVENT_READY => {
            if Some(ihdr.unk0) == sc.stm.iface.try_get().cloned() {
                sc.stm.ready.store(true, Ordering::Relaxed);

                dockchannel_hid_get_report2(
                    sc,
                    &sc.stm,
                    STM_REPORT_ID,
                    sc.stm_id.get_mut().as_mut_slice(),
                )?;
                dockchannel_hid_get_report2(
                    sc,
                    &sc.stm,
                    STM_REPORT_SERIAL,
                    sc.serial.get_mut().as_mut_slice(),
                )?;

                if !sc.kbd_hid.hidbus.is_init() && sc.kbd.ready.load(Ordering::Relaxed) {
                    dockchannel_hid_add_child(sc, &sc.kbd_hid)?;
                }
            } else if Some(ihdr.unk0) == sc.kbd.iface.try_get().cloned() {
                sc.kbd.ready.store(true, Ordering::Relaxed);
                device_println!(sc.dev, "adding child device for keyboard");
                if !sc.kbd_hid.hidbus.is_init() {
                    dockchannel_hid_add_child(sc, &sc.kbd_hid)?;
                }
            } else if Some(ihdr.unk0) == sc.mt.iface.try_get().cloned() {
                sc.mt.ready.store(true, Ordering::Relaxed);
            };
            DockchannelDriver::unmask_rx(sc.dev);
        }
        MTP_EVENT_GPIO_CMD => {
            let len = size_of::<MtpGpioCmd>();
            if buf.len() < len {
                return Ok(());
            }
            let (gpio_cmd_buf, _) = buf.split_at_mut(len);
            let gpio_cmd = MtpGpioCmd::from_slice(gpio_cmd_buf);
            device_println!(
                sc.dev,
                "got cmd {:x?} event for gpio {:?} on iface {:?}",
                gpio_cmd.cmd,
                gpio_cmd.gpio,
                gpio_cmd.iface
            );
            if gpio_cmd.iface == *sc.stm.iface.get() {
                let cdev = OF_device_from_xref(*sc.stm.smc.get())?;
                let gpio_pin = *sc.stm.gpio.get();
                AppleSmcDriver::pin_set(cdev, gpio_pin, 1)?;
                unsafe { bindings::DELAY(10000) };
                AppleSmcDriver::pin_set(cdev, gpio_pin, 0)?;
                let mut ack = MtpGpioAck {
                    ty: MTP_CMD_ACK_GPIO_CMD,
                    retcode: 0xe000f00d,
                    cmd: [0; 512],
                };
                let ack_cmd_len = min(512, buf.len());
                (&mut ack.cmd[..ack_cmd_len]).copy_from_slice(&buf[..ack_cmd_len]);
                let mut flags = (MtpGroup::Cmd as u8) << MTP_GROUP_SHIFT;
                flags |= (MtpReq::SetReport as u8) << MTP_REQ_SHIFT;
                dockchannel_hid_cmd(sc, &sc.comm, flags, ack.as_slice())?;
                dockchannel_hid_wait_ack(sc, &sc.comm, sc.comm.seq.load(Ordering::Relaxed) - 1);
            }
        }
        _ => {
            DockchannelDriver::unmask_rx(sc.dev);
        }
    }
    Ok(())
}

fn dockchannel_hid_add_child(
    sc: Ref<DockchannelHidSoftc>,
    child: &DockchannelHidChild,
) -> Result<()> {
    let dev = sc.dev;
    DockchannelDriver::unmask_rx(dev);
    let mut hidbus_hw = child.hw.get_mut();
    let stm_id = *sc.stm_id.get_mut();
    hidbus_hw.idBus = bindings::BUS_HOST as u16;
    hidbus_hw.idVendor = stm_id.vendor_id;
    hidbus_hw.idProduct = stm_id.product_id;
    hidbus_hw.idVersion = stm_id.version;
    hidbus_hw.rdescsize = sc.kbd.desc.get().len() as u32;
    (&mut hidbus_hw.serial[..64]).copy_from_slice(&sc.serial.get_mut()[..]);

    let res = unsafe {
        bindings::hid_add_dynamic_quirk(&raw mut *hidbus_hw, bindings::HQ_NOWRITE as u16)
    };
    if res != 0 {
        return Err(ErrCode::from(res));
    }
    let hidbus = device_add_child(dev, c"hidbus", None)?;
    let ivars = &raw mut *hidbus_hw;
    unsafe { bindings::device_set_ivars(hidbus, ivars.cast::<c_void>()) };
    child.hidbus.init(hidbus);

    bus_topo_lock();
    unsafe {
        bindings::bus_attach_children(dev);
    }
    bus_topo_unlock();
    Ok(())
}

fn dockchannel_hid_get_report2(
    sc: Ref<DockchannelHidSoftc>,
    dcif: &DockchannelHidIface,
    reportnum: u8,
    buf: &mut [u8],
) -> Result<()> {
    let mut flags = (MtpGroup::Cmd as u8) << MTP_GROUP_SHIFT;
    flags |= (MtpReq::GetReport as u8) << MTP_REQ_SHIFT;

    dockchannel_hid_cmd(sc, dcif, flags, &reportnum.to_ne_bytes())?;
    dockchannel_hid_wait_ack(sc, dcif, dcif.seq.load(Ordering::Relaxed) - 1);

    buf.copy_from_slice(&mut BUF.get_mut()[..buf.len()]);

    DockchannelDriver::unmask_rx(sc.dev);
    Ok(())
}

fn dockchannel_handle_init(sc: Ref<DockchannelHidSoftc>, iface: u8, buf: &mut [u8]) -> Result<()> {
    let len = size_of::<MtpInitBlockHdr>();
    if buf.len() < len {
        return Ok(());
    }
    let (mut current_bhdr_buf, mut remaining_buf) = buf.split_at_mut(len);
    let mut bhdr = MtpInitBlockHdr::from_slice(current_bhdr_buf);
    if remaining_buf.len() < usize::from(bhdr.len) {
        return Ok(());
    }
    let mut bhdr_data_buf;
    (bhdr_data_buf, remaining_buf) = remaining_buf.split_at_mut(usize::from(bhdr.len));
    loop {
        match bhdr.ty {
            MTP_BLOCK_DESCRIPTOR => {
                let desc = if Some(iface) == sc.stm.iface.try_get().cloned() {
                    Some(&sc.stm.desc)
                } else if Some(iface) == sc.kbd.iface.try_get().cloned() {
                    Some(&sc.kbd.desc)
                } else if Some(iface) == sc.mt.iface.try_get().cloned() {
                    Some(&sc.mt.desc)
                } else {
                    None
                };
                if let Some(desc) = desc {
                    let desc_len = usize::from(bhdr.len);
                    assert!(desc_len <= HID_DESC_MAX);
                    let mut desc_vec = desc.init(Vec::fill_with_capacity(0u8, desc_len, M_WAITOK));
                    desc_vec.copy_from_slice(bhdr_data_buf);
                };
            }
            MTP_BLOCK_GPIO_REQ => {
                dockchannel_hid_handle_gpio_req(sc, iface, bhdr_data_buf)?;
            }
            MTP_BLOCK_END => {
                return Ok(());
            }
            _ => {
                panic!("uh oh")
            }
        }
        if remaining_buf.len() < size_of::<MtpInitBlockHdr>() {
            return Ok(());
        }
        (current_bhdr_buf, remaining_buf) =
            remaining_buf.split_at_mut(size_of::<MtpInitBlockHdr>());
        bhdr = MtpInitBlockHdr::from_slice(current_bhdr_buf);
        if remaining_buf.len() < usize::from(bhdr.len) {
            return Ok(());
        }
        (bhdr_data_buf, remaining_buf) = remaining_buf.split_at_mut(usize::from(bhdr.len));
    }
    Ok(())
}

fn dockchannel_hid_iface_enable(
    sc: Ref<DockchannelHidSoftc>,
    dcif: &DockchannelHidIface,
) -> Result<()> {
    let cmd = [MTP_CMD_IFACE_ENABLE, *dcif.iface.get()];
    let mut flags = (MtpGroup::Cmd as u8) << MTP_GROUP_SHIFT;
    flags |= (MtpReq::SetReport as u8) << MTP_REQ_SHIFT;

    dockchannel_hid_cmd(sc, &sc.comm, flags, &cmd)?;
    dockchannel_hid_wait_ack(sc, &sc.comm, sc.comm.seq.load(Ordering::Relaxed) - 1);
    Ok(())
}

fn dockchannel_hid_cmd(
    sc: Ref<DockchannelHidSoftc>,
    dcif: &DockchannelHidIface,
    flags: u8,
    cmd: &[u8],
) -> Result<()> {
    let mut hdr = MtpHdr::default();
    hdr.hdr_len = size_of::<MtpHdr>().try_into().unwrap();
    hdr.chan = MTP_CHAN_CMD;
    hdr.pkt_len = cmd.len().next_multiple_of(4) as u16 + size_of::<MtpSubHdr>() as u16;
    // Put old value in header and increment
    hdr.seq = dcif.seq.fetch_add(1, Ordering::Relaxed);
    hdr.iface = *dcif.iface.get();

    let mut shdr = MtpSubHdr::default();
    shdr.flags = flags;
    shdr.len = cmd.len().try_into().unwrap();

    dockchannel_hid_write(sc, hdr.as_slice())?;
    dockchannel_hid_write(sc, shdr.as_slice())?;
    dockchannel_hid_write(sc, &cmd[..cmd.len() & !3])?;
    let mut checksum = u32::MAX
        - dockchannel_hid_checksum(hdr.as_slice())
        - dockchannel_hid_checksum(shdr.as_slice())
        - dockchannel_hid_checksum(&cmd[..cmd.len() & !3]);
    let extra_bytes = cmd.len() & 3;
    if extra_bytes != 0 {
        let extra_start = cmd.len() & !3;
        let mut padding = [0u8; 4];
        padding[..extra_bytes].copy_from_slice(&cmd[extra_start..]);
        dockchannel_hid_write(sc, &padding)?;
        checksum -= dockchannel_hid_checksum(&padding);
    }
    dockchannel_hid_write(sc, &checksum.to_ne_bytes())?;
    Ok(())
}

fn dockchannel_hid_wait_ack(
    sc: Ref<DockchannelHidSoftc>,
    dcif: &DockchannelHidIface,
    wait_seq: u8,
) {
    let dev = sc.dev;
    // Write the rx threshold and immediately drop the regs lock
    bus_write_4!(mtx_lock(&sc.regs).config, CONFIG_RX_THRESH, 8);
    DockchannelDriver::unmask_rx(dev);
    let mut x = mtx_lock(&dcif.ret).retain(|x| x.seq != wait_seq);
    while !x {
        device_println!(
            sc.dev,
            "sleeping on ret field at {:p} for packet #{wait_seq:?}",
            &dcif.ret
        );
        let _ = tsleep(&dcif.ret, Some(PWAIT), c"", hz());
        x = mtx_lock(&dcif.ret).retain(|x| x.seq != wait_seq);
    }
    //let mut ret = dcif.ret.get_mut().take();
    //while ret.map(|r| r.seq) != Some(wait_seq) {
    //    device_println!(
    //        sc.dev,
    //        "sleeping on ret field at {:p} for packet #{wait_seq:?}",
    //        &dcif.ret
    //    );
    //    let _ = tsleep(&dcif.ret, Some(PWAIT), c"", hz());
    //    ret = dcif.ret.get_mut().take();
    //}
    //let ret = ret.unwrap();
    //let iface = *dcif.iface.get();
    //if ret.code != 0 || iface != MTP_IFACE_COMM {
    //    device_println!(
    //        dev,
    //        "got retcode {:?} for packet #{wait_seq:?} on iface {iface:?}",
    //        ret.code
    //    );
    //}
    if *dcif.iface.get() != *sc.stm.iface.get() {
        DockchannelDriver::unmask_rx(dev);
    }
}

fn dockchannel_hid_handle_gpio_req(
    sc: Ref<DockchannelHidSoftc>,
    iface: u8,
    buf: &mut [u8],
) -> Result<()> {
    // TODO: Replace with CString at some point
    let mut name: Vec<u8, M_DEVBUF> = Vec::with_capacity(64, M_WAITOK);
    for &b in b"apple," {
        name.push(b);
    }

    let (req_buf, remaining_buf) = buf.split_at_mut(size_of::<MtpGpioReq>());
    let req = MtpGpioReq::from_slice(req_buf);

    for &b in &req.name {
        if b == 0 {
            break;
        }
        name.push(b);
    }
    for &b in b"-gpios\0" {
        name.push(b);
    }
    let name_cstr = CStr::from_bytes_until_nul(name.as_slice())?;
    //device_println!(sc.dev, "requested gpio {name_cstr:?}");

    let prop = OF_getencprop::<[u32; 3]>(sc.node, name_cstr)?;
    let xref = XRef(prop[0]);

    let cdev = OF_device_from_xref(xref)?;
    AppleSmcDriver::pin_set(cdev, prop[1], 0)?;

    if iface == *sc.stm.iface.get() {
        sc.stm.smc.init(xref);
        sc.stm.gpio.init(prop[1]);
    }
    Ok(())
}

fn dockchannel_hid_checksum(buf: &[u8]) -> u32 {
    let mut checksum = 0u32;
    let mut remaining_buf = buf;
    while remaining_buf.len() >= 4 {
        let mut tmp = u32::from_ne_bytes(remaining_buf[..4].try_into().unwrap());
        checksum = checksum.wrapping_add(tmp);
        remaining_buf = &remaining_buf[4..];
    }
    checksum
}

fn dockchannel_hid_read(sc: Ref<DockchannelHidSoftc>, buf: &mut [u8]) -> Result<()> {
    let mut remaining_buf = buf;
    while !remaining_buf.is_empty() {
        // The lock may already be acquired but its MTX_RECURSE
        let mut regs = mtx_lock(&sc.regs);
        let count = bus_read_4!(regs.data, DATA_RX_COUNT);

        if count == 0 {
            sc.rx_avail.store(false, Ordering::Relaxed);
            /* set the threshold for when to fire the rx irq */
            //bus_write_4(sc->sc_config, CONFIG_RX_THRESH,
            //    min(len, sc->sc_fifo_size / 2));
            bus_write_4!(regs.config, CONFIG_RX_THRESH, 8);

            /* unmask the rx irq */
            DockchannelDriver::unmask_rx(sc.dev);

            /* sleep if the rx irq hasn't fired */
            if !sc.rx_avail.load(Ordering::Relaxed) {
                // Drop the lock before sleeping
                mtx_unlock(regs);
                let _ = tsleep(&sc.rx_avail, Some(PWAIT), c"dcrx", hz());
            }
            if sc.rx_avail.load(Ordering::Relaxed) {
                //device_println!(sc.dev, "waited for read data");
                continue;
            }
            /* give up if sleep timed out */
            device_println!(
                sc.dev,
                "timed out on read with {:?} bytes left",
                remaining_buf.len()
            );
            return Err(ETIMEDOUT);
        }

        /* figure out how much to read */
        let mut read = min(remaining_buf.len(), count as usize);

        while read >= 4 {
            let tmp = bus_read_4!(regs.data, DATA_RX32);
            remaining_buf[..4].copy_from_slice(&tmp.to_ne_bytes());
            remaining_buf = &mut remaining_buf[4..];
            read -= 4;
        }
        while read > 0 {
            remaining_buf[0] = (bus_read_4!(regs.data, DATA_RX8) >> 8) as u8;
            remaining_buf = &mut remaining_buf[1..];
            read -= 1;
        }
    }
    Ok(())
}

fn dockchannel_hid_write(sc: Ref<DockchannelHidSoftc>, buf: &[u8]) -> Result<()> {
    let mut remaining_buf = buf;
    while !remaining_buf.is_empty() {
        let mut regs = mtx_lock(&sc.regs);
        let free = bus_read_4!(regs.data, DATA_TX_FREE);

        if free == 0 {
            sc.tx_avail.store(false, Ordering::Relaxed);
            /* set the threshold for when to fire the tx irq */
            bus_write_4!(
                regs.config,
                CONFIG_TX_THRESH,
                min(remaining_buf.len() as u32, sc.fifo_size / 2)
            );

            /* unmask the tx irq */
            DockchannelDriver::unmask_tx(sc.dev);

            if !sc.tx_avail.load(Ordering::Relaxed) {
                // Drop the lock before sleeping
                mtx_unlock(regs);
                let _ = tsleep(&sc.tx_avail, Some(PWAIT), c"dctx", hz());
            }
            if sc.tx_avail.load(Ordering::Relaxed) {
                device_println!(sc.dev, "waited for write data");
                continue;
            }

            /* give up if sleep timed out */
            device_println!(
                sc.dev,
                "timed out on write with {:?} bytes left",
                remaining_buf.len()
            );
            return Err(ETIMEDOUT);
        }

        let mut write = min(remaining_buf.len(), free as usize);
        while write >= 4 {
            let tmp = u32::from_ne_bytes(remaining_buf[..4].try_into().unwrap());
            bus_write_4!(regs.data, DATA_TX32, tmp);
            remaining_buf = &remaining_buf[4..];
            write -= 4;
        }
        while write > 0 {
            bus_write_1!(regs.data, DATA_TX8, remaining_buf[0]);
            remaining_buf = &remaining_buf[1..];
            write -= 1;
        }
    }
    Ok(())
}

fn dockchannel_hid_get_child(
    sc: &DockchannelHidSoftc,
    child: device_t,
) -> (&DockchannelHidChild, &DockchannelHidIface) {
    if sc.kbd_hid.hidbus.get().as_ptr() == child.as_ptr() {
        (&sc.kbd_hid, &sc.kbd)
    } else if sc.mt_hid.hidbus.get().as_ptr() == child.as_ptr() {
        (&sc.mt_hid, &sc.mt)
    } else {
        panic!("uh oh")
    }
}

impl HidIf for DockchannelHidDriver {
    fn hid_intr_setup(
        sc: Ref<DockchannelHidSoftc>,
        child: device_t,
        intr: hid_intr_t,
        context: *mut c_void,
        rdesc: *mut hid_rdesc_info,
    ) {
        if intr.is_none() {
            return;
        }
        let (dc, _) = dockchannel_hid_get_child(&*sc, child);
        let rdescsize = dc.hw.get_mut().rdescsize;
        unsafe {
            (*rdesc).rdsize = 0; // XXX
            (*rdesc).wrsize = 0; // XXX
            (*rdesc).grsize = rdescsize;
            (*rdesc).srsize = rdescsize;
        }
        dc.handler.init(intr).unwrap();
        dc.ctx.init(Ptr::new(context));
    }
    fn hid_intr_unsetup(sc: Ref<DockchannelHidSoftc>, child: device_t) {}
    fn hid_intr_start(sc: Ref<DockchannelHidSoftc>, child: device_t) -> Result<()> {
        Ok(())
    }
    fn hid_intr_stop(sc: Ref<DockchannelHidSoftc>, child: device_t) -> Result<()> {
        Ok(())
    }
    fn hid_intr_poll(sc: Ref<DockchannelHidSoftc>, child: device_t) {}
    fn hid_get_rdesc(sc: Ref<DockchannelHidSoftc>, child: device_t, buf: &mut [u8]) -> Result<()> {
        let (dc, dcif) = dockchannel_hid_get_child(&*sc, child);
        assert!(dcif.desc.is_init());
        assert!(dcif.ready.load(Ordering::Relaxed));
        buf.copy_from_slice(&dcif.desc.get()[0..buf.len()]);
        Ok(())
    }
    fn hid_get_report(
        sc: Ref<DockchannelHidSoftc>,
        child: device_t,
        data: *mut c_void,
        max_len: hid_size_t,
        actlen: *mut hid_size_t,
        ty: u8,
        id: u8,
    ) -> Result<()> {
        Err(ENOTSUP)
    }
    fn hid_set_report(
        sc: Ref<DockchannelHidSoftc>,
        child: device_t,
        buf: *const c_void,
        len: hid_size_t,
        ty: u8,
        id: u8,
    ) -> Result<()> {
        Err(ENOTSUP)
    }
}

driver! {
    dockchannel_hid_driver, c"dockchannel_hid", DockchannelHidDriver,
    dockchannel_hid_method_table = {
        device_probe dockchannel_hid_probe,
        device_attach dockchannel_hid_attach,

        /* HID interrupt interface */
        hid_intr_setup dockchannel_hid_intr_setup,
        hid_intr_unsetup dockchannel_hid_intr_unsetup,
        hid_intr_start dockchannel_hid_intr_start,
        hid_intr_stop dockchannel_hid_intr_stop,
        hid_intr_poll dockchannel_hid_intr_poll,

        /* HID interface */
        hid_get_rdesc dockchannel_hid_get_rdesc,
        hid_get_report dockchannel_hid_get_report,
        hid_set_report dockchannel_hid_set_report,
    }
    // Required to use interfaces defined outside the KPI crate
    with interfaces from { hid };
}
