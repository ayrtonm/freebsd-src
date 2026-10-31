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
use apple_smc::{AppleSmcDriver, apple_smc_gpio_pin_set};
use core::cmp::min;
use core::ffi::{CStr, c_char, c_void};
use core::mem::MaybeUninit;
use core::slice;
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU64, Ordering, AtomicPtr};
use dockchannel::DockchannelDriver;
use hid::HidIf;
use kpi::bindings::{
    bus_dma_segment_t, bus_size_t, device_t, hid_device_info, hid_intr_t, hid_rdesc_info,
    hid_size_t, phandle_t,
};
use kpi::boxed::Box;
use kpi::bus::dma::{BusDmaMap, BusDmaMem, BusDmaTag};
use kpi::bus::{Filter, Irq, Register, Resource, SysRes};
use kpi::device::{BusProbe, DeviceIf, Device};
use kpi::ffi::{ArrayCString, Ptr, UninitPtr, Softc, PinProject};
use kpi::intr::{Channel, ConfigHook};
gs
use kpi::ofw::{Node, XRef};
use kpi::prelude::*;
use kpi::sync::mtx::Mutex;
use kpi::sync::{Checked, OnceInit};
use kpi::taskqueue::{Task, Taskqueue};
use kpi::vec::Vec;
use kpi::{ErrCode, define_driver, define_projectable};

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

// Header of the multi-touch firmware files extracted from macOS (apple/tpmtfw-*.bin). The data
// sent to the device follows the header.
const MTP_FW_MAGIC: u32 = 0x46444948; // "HIDF"
const MTP_FW_VERSION: u32 = 1;

#[repr(packed)]
struct MtpFwHdr {
    magic: u32,
    version: u32,
    hdr_len: u32,
    data_len: u32,
    // Offset in the data of a byte to overwrite with the interface index, 0 if none
    iface_offset: u32,
}

impl_from_slice!(MtpFwHdr);

#[repr(packed)]
struct MtpSendFwCmd {
    cmd: u8,
    unk0: u8,
    unk1: u8,
    iface: u8,
    // Device address of the firmware data
    addr: u64,
    size: u32,
}

impl_from_slice!(MtpSendFwCmd);

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

define_projectable! {
    #[derive(Debug, Default)]
    struct DockchannelHidIface {
        iface: OnceInit<u8>,
        desc: OnceInit<Vec<u8, M_DEVBUF>>,
        ready: AtomicBool,
        seq: AtomicU8,
        // Last ack received on this iface
        ret: Mutex<Option<DockchannelCmdRet>>,
        ret_chan: Channel,
        smc: OnceInit<XRef>,
        gpio: OnceInit<u32>,
    }
}

#[derive(Debug, Default)]
struct DockchannelHidChild {
    hidbus: OnceInit<device_t>,
    hw: Checked<hid_device_info>,
    handler: OnceInit<hid_intr_t>,
    ctx: AtomicPtr<c_void>,
}

unsafe impl Sync for DockchannelHidSoftc {}
unsafe impl Send for DockchannelHidSoftc {}

#[derive(Debug)]
struct DockchannelHidRegister {
    config: Register,
    data: Register,
}

define_projectable! {
    #[derive(Debug)]
    pub struct DockchannelHidSoftc {
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
        rx_avail_chan: Channel,
        tx_avail: AtomicBool,
        tx_avail_chan: Channel,
        queue: Taskqueue,
        comm: DockchannelHidIface,
        stm: DockchannelHidIface,
        kbd: DockchannelHidIface,
        mt: DockchannelHidIface,
    
        kbd_hid: DockchannelHidChild,
        mt_hid: DockchannelHidChild,
        // Set by whichever task thread gets to add the keyboard's hidbus child first
        kbd_child_added: AtomicBool,
        // Same for the multi-touch child
        mt_child_added: AtomicBool,
        // Set once the multi-touch firmware upload has been started (on first open)
        mt_fw_loaded: AtomicBool,
        // Device address of the multi-touch firmware buffer, set by the dmamap callback
        mt_fw_addr: AtomicU64,

        stm_id: OnceInit<DockchannelHidStmId>,
        serial: OnceInit<[c_char; 64]>,
    }
}

fn dockchannel_alloc_res(
    dev: Device,
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

    fn device_probe(dev: Device) -> Result<BusProbe> {
        if !ofw_bus_status_okay(dev) {
            return Err(ENXIO);
        }
        if !ofw_bus_is_compatible(dev, c"apple,dockchannel-hid") {
            return Err(ENXIO);
        }
        device_set_desc(dev, c"Apple DockChannel HID");
        Ok(BUS_PROBE_DEFAULT)
    }

    fn device_attach(uninit_sc: UninitPtr<DockchannelHidSoftc>) -> Result<()> {
        let dev = uninit_sc.device();
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
        let task = Task::new();
        let hook = ConfigHook::new();
        let queue = Taskqueue::new();

        let sc = uninit_sc.init(DockchannelHidSoftc {
            node,
            mtp,
            fifo_size,
            task,
            hook,
            regs: Mutex::new(DockchannelHidRegister { config, data }),
            rx_irq,
            tx_irq,
            rx_avail: AtomicBool::new(true),
            rx_avail_chan: Channel::new(),
            tx_avail: AtomicBool::new(true),
            tx_avail_chan: Channel::new(),
            queue,
            comm: DockchannelHidIface::default(),
            stm: DockchannelHidIface::default(),
            kbd: DockchannelHidIface::default(),
            mt: DockchannelHidIface::default(),

            kbd_hid: DockchannelHidChild::default(),
            mt_hid: DockchannelHidChild::default(),

            kbd_child_added: AtomicBool::new(false),
            mt_child_added: AtomicBool::new(false),
            mt_fw_loaded: AtomicBool::new(false),
            mt_fw_addr: AtomicU64::new(0),

            stm_id: OnceInit::uninit(),
            serial: OnceInit::uninit(),
        });

        sc.project().hook.init(dockchannel_hid_start_config_hook, sc);
        sc.project().task.init(dockchannel_hid_task, sc.get_ptr())?;

        sc.comm.iface.init(MTP_IFACE_COMM);
        //sc.comm.seq = AtomicU8::new(0);
        //sc.stm.seq = AtomicU8::new(0);
        mtx_init(sc.project().comm.project().ret, c"", None, None);
        mtx_init(sc.project().stm.project().ret, c"", None, None);
        mtx_init(sc.project().kbd.project().ret, c"", None, None);
        mtx_init(sc.project().mt.project().ret, c"", None, None);
        mtx_init(sc.project().regs, c"", None, Some(MTX_RECURSE));

        taskqueue_create_fast(
            ArrayCString::new(c"dockchannel taskq"),
            M_WAITOK,
            sc.project().queue,
        )?;
        taskqueue_start_threads(
            &sc.queue,
            8,
            PWAIT,
            ArrayCString::new(c"dockchannel thead"),
        )?;

        config_intrhook_establish(sc.project().hook).unwrap();

        Ok(())
    }
}

extern "C" fn dockchannel_hid_start_config_hook(sc: &Softc<DockchannelHidSoftc>) {
    DockchannelHidDriver::try_dockchannel_hid_start_config_hook(sc).unwrap()
}

impl DockchannelHidDriver {
    fn try_dockchannel_hid_start_config_hook(sc: &Softc<DockchannelHidSoftc>) -> Result<()> {
        let dev = sc.device();
        AppleRTKitDriver::boot_helper(dev, sc.mtp)?;
        //device_println!(dev, "booted apple_rtkit helper");

        let regs = mtx_lock(&sc.regs);
        let mut config = &regs.config;
        bus_write_4!(config, CONFIG_RX_THRESH, 8);
        bus_write_4!(config, CONFIG_TX_THRESH, 8);
        mtx_unlock(regs);
        //device_println!(dev, "set config rx/tx threshold");

        bus_setup_intr(
            dev,
            sc.project().rx_irq,
            INTR_MPSAFE.0 | INTR_TYPE_MISC.0,
            Some(dockchannel_hid_rx_filter),
            None,
            sc.get_ptr(),
        )
        .inspect_err(|e| {
            device_println!(dev, "couldn't set up rx irq {e}");
        })?;

        bus_setup_intr(
            dev,
            sc.project().tx_irq,
            INTR_MPSAFE.0 | INTR_TYPE_MISC.0,
            Some(dockchannel_hid_tx_filter),
            None,
            sc.get_ptr(),
        )
        .inspect_err(|e| {
            device_println!(dev, "couldn't set up tx irq {e}");
        })?;

        config_intrhook_disestablish(&sc.hook);

        Ok(())
    }
}

extern "C" fn dockchannel_hid_rx_filter(sc: &Softc<DockchannelHidSoftc>) -> Filter {
    DockchannelDriver::mask_rx(sc.device());
    if !sc.rx_avail.load(Ordering::Relaxed) {
        sc.rx_avail.store(true, Ordering::Relaxed);
        wakeup(&sc.rx_avail_chan);

        return FILTER_HANDLED;
    }

    taskqueue_enqueue(&sc.queue, sc.project().task).unwrap();
    FILTER_HANDLED
}

extern "C" fn dockchannel_hid_tx_filter(sc: &Softc<DockchannelHidSoftc>) -> Filter {
    DockchannelDriver::mask_tx(sc.device());
    sc.tx_avail.store(true, Ordering::Relaxed);
    wakeup(&sc.tx_avail_chan);
    FILTER_HANDLED
}

extern "C" fn dockchannel_hid_task(sc: &Softc<DockchannelHidSoftc>, pending: u32) {
    if let Err(e) = try_dockchannel_hid_task(sc) {
        device_println!(sc.device(), "hid_task failed: {e:?}");
        // Unmask rx so we keep getting packets
        let _ = dockchannel_hid_task_done(sc);
    }
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
fn try_dockchannel_hid_task(sc: &Softc<DockchannelHidSoftc>) -> Result<()> {
    let dev = sc.device();

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
    let mut buf: Box<[u8], M_DEVBUF> = vec.into_boxed_slice(M_WAITOK);

    // Fill in the entire second buffer
    dockchannel_hid_read(sc, &mut buf).inspect_err(|e| {
        device_println!(dev, "failed to read packet of length {pkt_len:?} {e:?}",);
        let _ = dockchannel_hid_task_done(sc);
    })?;
    // Use wrapping_add to avoid panic on overflow
    let checksum =
        dockchannel_hid_checksum(hdr.as_slice()).wrapping_add(dockchannel_hid_checksum(&buf));

    if checksum != u32::MAX {
        device_println!(dev, "bad checksum {checksum:x?}");
        return dockchannel_hid_task_done(sc);
    }

    // Split the buffer into the MTP subheader and the rest. The MTP subheader will be used to
    // decide how to interpret remaining_buf.
    let (subhdr_buf, remaining_buf) = buf.split_at_mut(size_of::<MtpSubHdr>());

    let subhdr = MtpSubHdr::from_slice(subhdr_buf);
    let sublen = min(usize::from(subhdr.len), remaining_buf.len());

    // Unknown groups fall through to the channel match
    let group = subhdr.mtp_group().ok();
    if group == Some(MtpGroup::Output) || group == Some(MtpGroup::Cmd) {
        // Check if the iface field in the MtpHdr matches sc.comm.iface or sc.stm.iface after
        // they've been initialized.
        if let Some(iface) = sc.comm.iface.try_get()
            && *iface == hdr.iface
        {
            *mtx_lock(&sc.comm.ret) = Some(DockchannelCmdRet {
		    code: subhdr.retcode,
		    seq: hdr.seq,
	    });
            wakeup(&sc.comm.ret_chan);
        } else if let Some(iface) = sc.stm.iface.try_get()
            && *iface == hdr.iface
        {
            // Copy the report into BUF before setting ret
            if subhdr.mtp_req() == Ok(MtpReq::GetReport) {
                device_println!(dev, "copying {:?} bytes", sublen);
                let mut gbuf = BUF.get_mut();
                *gbuf = Vec::fill_with_capacity(0u8, sublen, M_WAITOK);
                gbuf.copy_from_slice(&remaining_buf[..sublen]);
            }
            *mtx_lock(&sc.stm.ret) = Some(DockchannelCmdRet {
		    code: subhdr.retcode,
		    seq: hdr.seq,
	    });
            device_println!(
                sc.device(),
                "waking up sc.stm.ret {:p} for seq {:?}",
                &sc.stm.ret,
                hdr.seq
            );
            wakeup(&sc.stm.ret_chan);
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
                //device_println!(sc.device(), "remaining len: {:?}, subhdr.len: {:?}", remaining_buf.len(), usize::from(subhdr.len));
                dockchannel_handle_comm(sc, &mut remaining_buf[..sublen])?;
            } else if let Some(iface) = sc.kbd.iface.try_get()
                && *iface == hdr.iface
                && sc.kbd.ready.load(Ordering::Relaxed)
            {
                // Drop reports until hid_intr_setup sets the handler
                if let Some(Some(handler)) = sc.kbd_hid.handler.try_get() {
                    let ptr = remaining_buf.as_mut_ptr().cast::<c_void>();
                    let ctx = sc.kbd_hid.ctx.load(Ordering::Acquire);
                    unsafe { handler(ctx, ptr, sublen as hid_size_t) }
                }
            } else if let Some(iface) = sc.mt.iface.try_get()
                && *iface == hdr.iface
                && sc.mt.ready.load(Ordering::Relaxed)
            {
                // Same as the keyboard, drop reports until the multi-touch child's handler is set
                if let Some(Some(handler)) = sc.mt_hid.handler.try_get() {
                    let ptr = remaining_buf.as_mut_ptr().cast::<c_void>();
                    let ctx = sc.mt_hid.ctx.load(Ordering::Acquire);
                    unsafe { handler(ctx, ptr, sublen as hid_size_t) }
                }
            };
        }
        _ => {
            device_println!(dev, "ignored packet from unknown channel {:x?}", hdr.chan);
        }
    }

    dockchannel_hid_task_done(sc)
}

fn dockchannel_hid_task_done(sc: &Softc<DockchannelHidSoftc>) -> Result<()> {
    sc.rx_avail.store(true, Ordering::Relaxed);
    /* unmask rx irq to get ack */
    DockchannelDriver::unmask_rx(sc.device());
    Ok(())
}

fn dockchannel_handle_comm(sc: &Softc<DockchannelHidSoftc>, buf: &mut [u8]) -> Result<()> {
    // READY and GPIO_CMD events are shorter than MtpInitHdr so only check the type here
    let ty = match buf.first() {
        Some(&ty) => ty,
        None => return Ok(()),
    };
    match ty {
        MTP_EVENT_INIT => {
            let len = size_of::<MtpInitHdr>();
            if buf.len() < len {
                return Ok(());
            }
            let (ihdr_buf, remaining_buf) = buf.split_at_mut(len);
            let ihdr = MtpInitHdr::from_slice(ihdr_buf);
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
                if let Err(e) = dockchannel_handle_init(sc, ihdr.iface, remaining_buf) {
                    device_println!(sc.device(), "failed to handle init blocks {e:?}");
                }
                dockchannel_hid_iface_enable(sc, dcif)?;
            };
        }
        MTP_EVENT_READY => {
            // The iface that became ready is in the second byte (MtpInitHdr::unk0)
            let ready_iface = match buf.get(1) {
                Some(&ready_iface) => ready_iface,
                None => {
                    DockchannelDriver::unmask_rx(sc.device());
                    return Ok(());
                }
            };
            if Some(ready_iface) == sc.stm.iface.try_get().cloned() {
                sc.stm.ready.store(true, Ordering::Relaxed);

                // get_report2 sleeps and other task threads may run meanwhile so read into
                // locals first
                let mut stm_id = DockchannelHidStmId::default();
                if let Err(e) =
                    dockchannel_hid_get_report2(sc, &sc.stm, STM_REPORT_ID, stm_id.as_mut_slice())
                {
                    device_println!(sc.device(), "failed to read stm id {e:?}");
                }
                // Leave the last byte as the NUL terminator
                let mut serial = [0 as c_char; 64];
                if let Err(e) =
                    dockchannel_hid_get_report2(sc, &sc.stm, STM_REPORT_SERIAL, &mut serial[..63])
                {
                    device_println!(sc.device(), "failed to read stm serial {e:?}");
                }
                if !sc.stm_id.is_init() {
                    sc.stm_id.init(stm_id);
                    sc.serial.init(serial);
                }

                if sc.kbd.ready.load(Ordering::Relaxed) {
                    dockchannel_hid_add_kbd_child(sc)?;
                }
                if sc.mt.ready.load(Ordering::Relaxed) {
                    dockchannel_hid_add_mt_child(sc)?;
                }
            } else if Some(ready_iface) == sc.kbd.iface.try_get().cloned() {
                sc.kbd.ready.store(true, Ordering::Relaxed);
                dockchannel_hid_add_kbd_child(sc)?;
            } else if Some(ready_iface) == sc.mt.iface.try_get().cloned() {
                sc.mt.ready.store(true, Ordering::Relaxed);
                dockchannel_hid_add_mt_child(sc)?;
            };
            DockchannelDriver::unmask_rx(sc.device());
        }
        MTP_EVENT_GPIO_CMD => {
            let len = size_of::<MtpGpioCmd>();
            if buf.len() < len {
                return Ok(());
            }
            let (gpio_cmd_buf, _) = buf.split_at_mut(len);
            let gpio_cmd = MtpGpioCmd::from_slice(gpio_cmd_buf);
            device_println!(
                sc.device(),
                "got cmd {:x?} event for gpio {:?} on iface {:?}",
                gpio_cmd.cmd,
                gpio_cmd.gpio,
                gpio_cmd.iface
            );
            // The stm and multi-touch interfaces each request a reset GPIO during init
            let dcif = if Some(&gpio_cmd.iface) == sc.stm.iface.try_get() {
                Some(&sc.stm)
            } else if Some(&gpio_cmd.iface) == sc.mt.iface.try_get() {
                Some(&sc.mt)
            } else {
                None
            };
            if let Some(dcif) = dcif
                && let (Some(smc), Some(gpio_pin)) = (dcif.smc.try_get(), dcif.gpio.try_get())
            {
                let cdev = OF_device_from_xref(*smc)?;
                let gpio_pin = *gpio_pin;
                apple_smc_gpio_pin_set(cdev, gpio_pin, 1)?;
                unsafe { bindings::DELAY(10000) };
                apple_smc_gpio_pin_set(cdev, gpio_pin, 0)?;
                let mut ack = MtpGpioAck {
                    ty: MTP_CMD_ACK_GPIO_CMD,
                    // The pulse was done, report success
                    retcode: 0,
                    cmd: [0; 512],
                };
                let ack_cmd_len = min(512, buf.len());
                (&mut ack.cmd[..ack_cmd_len]).copy_from_slice(&buf[..ack_cmd_len]);
                let mut flags = (MtpGroup::Cmd as u8) << MTP_GROUP_SHIFT;
                flags |= (MtpReq::SetReport as u8) << MTP_REQ_SHIFT;
                dockchannel_hid_cmd(sc, &sc.comm, flags, ack.as_slice())?;
                dockchannel_hid_wait_ack(sc, &sc.comm, sc.comm.seq.load(Ordering::Relaxed) - 1);
            }
            DockchannelDriver::unmask_rx(sc.device());
        }
        _ => {
            DockchannelDriver::unmask_rx(sc.device());
        }
    }
    Ok(())
}

fn dockchannel_hid_add_kbd_child(sc: &Softc<DockchannelHidSoftc>) -> Result<()> {
    if sc.kbd_child_added.swap(true, Ordering::Relaxed) {
        return Ok(());
    }
    device_println!(sc.device(), "adding child device for keyboard");
    dockchannel_hid_add_child(sc, &sc.kbd_hid, &sc.kbd)
}

// Same as the keyboard. The firmware the trackpad needs is only sent when the child is opened
// (hid_intr_start), when the root filesystem with /boot/firmware is available.
fn dockchannel_hid_add_mt_child(sc: &Softc<DockchannelHidSoftc>) -> Result<()> {
    if !sc.mt.desc.is_init() {
        device_println!(sc.device(), "multi-touch has no HID descriptor, not adding it");
        return Ok(());
    }
    if sc.mt_child_added.swap(true, Ordering::AcqRel) {
        return Ok(());
    }
    device_println!(sc.device(), "adding child device for multi-touch");
    dockchannel_hid_add_child(sc, &sc.mt_hid, &sc.mt)
}

fn dockchannel_hid_add_child(
    sc: &Softc<DockchannelHidSoftc>,
    child: &DockchannelHidChild,
    dcif: &DockchannelHidIface,
) -> Result<()> {
    let dev = sc.device();
    DockchannelDriver::unmask_rx(dev);
    let mut hidbus_hw = child.hw.get_mut();
    // stm id may not have been read yet
    let stm_id = sc.stm_id.try_get().copied().unwrap_or_default();
    hidbus_hw.idBus = bindings::BUS_HOST as u16;
    hidbus_hw.idVendor = stm_id.vendor_id;
    hidbus_hw.idProduct = stm_id.product_id;
    hidbus_hw.idVersion = stm_id.version;
    hidbus_hw.rdescsize = dcif.desc.get().len() as u32;
    let serial = sc.serial.try_get().copied().unwrap_or([0; 64]);
    (&mut hidbus_hw.serial[..64]).copy_from_slice(&serial[..]);
    // bcm5974 recognizes the trackpad by this PnP ID (APPLE_MTP_TP there)
    if core::ptr::eq(child, &sc.mt_hid) {
        for (dst, &src) in hidbus_hw.idPnP.iter_mut().zip(b"APPLE-MTP-TP\0") {
            *dst = src as c_char;
        }
    }

    let res = unsafe {
        bindings::hid_add_dynamic_quirk(&raw mut *hidbus_hw, bindings::HQ_NOWRITE as u16)
    };
    if res != 0 {
        return Err(ErrCode::from(res));
    }
    let ivars = &raw mut *hidbus_hw;

    drop(hidbus_hw);

    let hidbus = device_add_child(dev, c"hidbus", None)?.as_ptr();
    unsafe { bindings::device_set_ivars(hidbus, ivars.cast::<c_void>()) };
    child.hidbus.init(hidbus);

    bus_topo_lock();
    unsafe {
        bindings::bus_attach_children(dev.as_ptr());
    }
    bus_topo_unlock();
    Ok(())
}

fn dockchannel_hid_get_report2(
    sc: &Softc<DockchannelHidSoftc>,
    dcif: &DockchannelHidIface,
    reportnum: u8,
    buf: &mut [u8],
) -> Result<()> {
    let mut flags = (MtpGroup::Cmd as u8) << MTP_GROUP_SHIFT;
    flags |= (MtpReq::GetReport as u8) << MTP_REQ_SHIFT;

    dockchannel_hid_cmd(sc, dcif, flags, &reportnum.to_ne_bytes())?;
    dockchannel_hid_wait_ack(sc, dcif, dcif.seq.load(Ordering::Relaxed) - 1);

    // Reports may be shorter than buf
    let src = BUF.get_mut();
    let len = min(buf.len(), src.len());
    if len < buf.len() {
        device_println!(
            sc.device(),
            "short report {reportnum:#x}: got {len:?} of {:?} bytes",
            buf.len()
        );
    }
    buf[..len].copy_from_slice(&src[..len]);
    drop(src);

    DockchannelDriver::unmask_rx(sc.device());
    Ok(())
}

fn dockchannel_handle_init(sc: &Softc<DockchannelHidSoftc>, iface: u8, buf: &mut [u8]) -> Result<()> {
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
                let desc_len = usize::from(bhdr.len);
                if let Some(desc) = desc
                    && desc_len <= HID_DESC_MAX
                    && !desc.is_init()
                {
                    let mut desc_vec = Vec::fill_with_capacity(0u8, desc_len, M_WAITOK);
                    desc_vec.copy_from_slice(bhdr_data_buf);
                    desc.init(desc_vec);
                };
            }
            MTP_BLOCK_GPIO_REQ => {
                // Not fatal, keep parsing the remaining blocks
                if let Err(e) = dockchannel_hid_handle_gpio_req(sc, iface, bhdr_data_buf) {
                    device_println!(sc.device(), "failed to handle gpio request {e:?}");
                }
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

// Send a command on the comm interface and wait for its ack
fn dockchannel_hid_comm_cmd(sc: &Softc<DockchannelHidSoftc>, cmd: &[u8]) -> Result<()> {
    let mut flags = (MtpGroup::Cmd as u8) << MTP_GROUP_SHIFT;
    flags |= (MtpReq::SetReport as u8) << MTP_REQ_SHIFT;

    dockchannel_hid_cmd(sc, &sc.comm, flags, cmd)?;
    dockchannel_hid_wait_ack(sc, &sc.comm, sc.comm.seq.load(Ordering::Relaxed) - 1);
    Ok(())
}

fn dockchannel_hid_iface_enable(
    sc: &Softc<DockchannelHidSoftc>,
    dcif: &DockchannelHidIface,
) -> Result<()> {
    dockchannel_hid_comm_cmd(sc, &[MTP_CMD_IFACE_ENABLE, *dcif.iface.get()])
}

fn dockchannel_hid_iface_reset(
    sc: &Softc<DockchannelHidSoftc>,
    dcif: &DockchannelHidIface,
    state: u8,
) -> Result<()> {
    dockchannel_hid_comm_cmd(sc, &[MTP_CMD_IFACE_RESET, 1, *dcif.iface.get(), state])
}

// Name of the multi-touch firmware from the "multi-touch" child node's firmware-name property,
// NUL-terminated. None if the node or the property doesn't exist (no firmware needed).
fn dockchannel_hid_mt_fw_name(sc: &Softc<DockchannelHidSoftc>) -> Option<[u8; 64]> {
    let mut child = OF_child(sc.node);
    while let Some(node) = child {
        let mut name = [0u8; 32];
        let len = unsafe {
            bindings::OF_getprop(
                node.as_phandle(),
                c"name".as_ptr(),
                name.as_mut_ptr().cast::<c_void>(),
                name.len() - 1,
            )
        };
        if len > 0 && CStr::from_bytes_until_nul(&name).ok() == Some(c"multi-touch") {
            let mut fw_name = [0u8; 64];
            let len = unsafe {
                bindings::OF_getprop(
                    node.as_phandle(),
                    c"firmware-name".as_ptr(),
                    fw_name.as_mut_ptr().cast::<c_void>(),
                    fw_name.len() - 1,
                )
            };
            return if len > 0 { Some(fw_name) } else { None };
        }
        child = OF_peer(node);
    }
    None
}

extern "C" fn dockchannel_hid_mt_fw_cb(
    sc: Ptr<DockchannelHidSoftc>,
    segs: &bus_dma_segment_t,
    nsegs: i32,
    error: i32,
) {
    if error != 0 || nsegs != 1 {
        return;
    }
    sc.as_pin().mt_fw_addr.store(segs.ds_addr, Ordering::Release);
}

// Upload the multi-touch firmware and reset the interface, which the trackpad needs before it
// sends any reports. The firmware data is copied into a DMA buffer and the device is told its
// address.
fn dockchannel_hid_load_mt_fw(sc: &Softc<DockchannelHidSoftc>) -> Result<()> {
    let dev = sc.device();
    let Some(fw_name) = dockchannel_hid_mt_fw_name(sc) else {
        return Ok(());
    };
    let fw_name = CStr::from_bytes_until_nul(&fw_name).map_err(|_| EINVAL)?;
    let iface = *sc.mt.iface.get();

    let fw = unsafe { bindings::firmware_get(fw_name.as_ptr()) };
    if fw.is_null() {
        device_println!(dev, "couldn't load {fw_name:?}, the trackpad won't work");
        return Err(ENOENT);
    }
    // SAFETY: firmware_get returned a valid firmware, which stays valid until firmware_put
    let data = unsafe { slice::from_raw_parts((*fw).data.cast::<u8>(), (*fw).datasize) };

    let res = (|| {
        if data.len() < size_of::<MtpFwHdr>() {
            return Err(EINVAL);
        }
        let hdr = MtpFwHdr::from_slice(data);
        let (magic, version) = (hdr.magic, hdr.version);
        let (hdr_len, data_len, iface_offset) =
            (hdr.hdr_len as usize, hdr.data_len as usize, hdr.iface_offset as usize);
        if magic != MTP_FW_MAGIC
            || version != MTP_FW_VERSION
            || hdr_len < size_of::<MtpFwHdr>()
            || hdr_len.checked_add(data_len).is_none_or(|end| end > data.len())
            || iface_offset >= data_len
        {
            device_println!(dev, "{fw_name:?} has an invalid header");
            return Err(EINVAL);
        }
        let payload = &data[hdr_len..hdr_len + data_len];

        // The device may keep using the firmware after loading it, so the buffer is kept for
        // the lifetime of the driver
        let size = data_len as bus_size_t;
        let tag: BusDmaTag = bus_dma_tag_create(bus_get_dma_tag(dev))
            .alignment(PAGE_SIZE)
            .max_size(size)
            .max_seg_size(size)
            .flags(Some(BUS_DMA_COHERENT))
            .build()?;
        let (map, kva): (BusDmaMap, BusDmaMem) =
            bus_dmamem_alloc(tag, BUS_DMA_WAITOK | BUS_DMA_ZERO)?;
        let buf = kva.as_ptr().cast::<u8>();
        // SAFETY: buf was just allocated with room for size bytes
        unsafe {
            core::ptr::copy_nonoverlapping(payload.as_ptr(), buf, payload.len());
            if iface_offset != 0 {
                *buf.add(iface_offset) = iface;
            }
        }

        sc.mt_fw_addr.store(0, Ordering::Relaxed);
        bus_dmamap_load(
            tag,
            map,
            kva,
            size,
            Some(dockchannel_hid_mt_fw_cb),
            sc.get_ptr(),
            Some(BUS_DMA_NOWAIT),
        )?;
        let addr = sc.mt_fw_addr.load(Ordering::Acquire);
        if addr == 0 {
            device_println!(dev, "couldn't map the multi-touch firmware buffer");
            return Err(ENOMEM);
        }
        bus_dmamap_sync(tag, map, BUS_DMASYNC_PREWRITE);

        device_println!(dev, "sending {size} bytes of multi-touch firmware");
        let cmd = MtpSendFwCmd {
            cmd: MTP_CMD_SEND_FIRMWARE,
            unk0: 2,
            unk1: 0,
            iface,
            addr,
            size: data_len as u32,
        };
        dockchannel_hid_comm_cmd(sc, cmd.as_slice())?;

        // After loading firmware, multi-touch needs a reset
        dockchannel_hid_iface_reset(sc, &sc.mt, 0)?;
        dockchannel_hid_iface_reset(sc, &sc.mt, 2)?;
        Ok(())
    })();

    unsafe { bindings::firmware_put(fw, bindings::FIRMWARE_UNLOAD as i32) };
    res
}

fn dockchannel_hid_cmd(
    sc: &Softc<DockchannelHidSoftc>,
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
        .wrapping_sub(dockchannel_hid_checksum(hdr.as_slice()))
        .wrapping_sub(dockchannel_hid_checksum(shdr.as_slice()))
        .wrapping_sub(dockchannel_hid_checksum(&cmd[..cmd.len() & !3]));
    let extra_bytes = cmd.len() & 3;
    if extra_bytes != 0 {
        let extra_start = cmd.len() & !3;
        let mut padding = [0u8; 4];
        padding[..extra_bytes].copy_from_slice(&cmd[extra_start..]);
        dockchannel_hid_write(sc, &padding)?;
        checksum = checksum.wrapping_sub(dockchannel_hid_checksum(&padding));
    }
    dockchannel_hid_write(sc, &checksum.to_ne_bytes())?;
    Ok(())
}

fn dockchannel_hid_wait_ack(
    sc: &Softc<DockchannelHidSoftc>,
    dcif: &DockchannelHidIface,
    wait_seq: u8,
) {
    let dev = sc.device();
    // Write the rx threshold and immediately drop the regs lock
    bus_write_4!(mtx_lock(&sc.regs).config, CONFIG_RX_THRESH, 8);
    DockchannelDriver::unmask_rx(dev);

    // Sleep until we get the ack for wait_seq
    let mut guard = mtx_lock(&dcif.ret);
    while guard.map(|r| r.seq) != Some(wait_seq) {
        let (g, res) = mtx_sleep(&dcif.ret_chan, guard, PWAIT.0, c"dcack", hz());
        guard = g;
        if res.is_err() {
            //device_println!(dev, "still waiting for ack of packet #{wait_seq:?}");
        }
    }
    let ret = guard.take().unwrap();
    mtx_unlock(guard);

    let iface = *dcif.iface.get();
    if ret.code != 0 || iface != MTP_IFACE_COMM {
        device_println!(
            dev,
            "got retcode {:x?} for packet #{wait_seq:?} on iface {iface:?}",
            ret.code
        );
    }

    // stm iface may not be initialized yet
    if iface != sc.stm.iface.try_get().copied().unwrap_or(0) {
        DockchannelDriver::unmask_rx(dev);
    }
}

fn dockchannel_hid_handle_gpio_req(
    sc: &Softc<DockchannelHidSoftc>,
    iface: u8,
    buf: &mut [u8],
) -> Result<()> {
    // TODO: Replace with CString at some point
    let mut name: Vec<u8, M_DEVBUF> = Vec::with_capacity(64, M_WAITOK);
    for &b in b"apple," {
        name.push(b);
    }

    if buf.len() < size_of::<MtpGpioReq>() {
        return Err(EINVAL);
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
    let xref = unsafe { XRef::new_unchecked(prop[0]) };

    let cdev = OF_device_from_xref(xref)?;
    apple_smc_gpio_pin_set(cdev, prop[1], 0)?;

    let dcif = if Some(&iface) == sc.stm.iface.try_get() {
        Some(&sc.stm)
    } else if Some(&iface) == sc.mt.iface.try_get() {
        Some(&sc.mt)
    } else {
        None
    };
    if let Some(dcif) = dcif
        && !dcif.smc.is_init()
    {
        dcif.smc.init(xref);
        dcif.gpio.init(prop[1]);
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

fn dockchannel_hid_read(sc: &Softc<DockchannelHidSoftc>, buf: &mut [u8]) -> Result<()> {
    let mut remaining_buf = buf;
    while !remaining_buf.is_empty() {
        // The lock may already be acquired but its MTX_RECURSE
        let mut regs = mtx_lock(&sc.regs);
        let count = bus_read_4!(regs.data, DATA_RX_COUNT);

        if count == 0 {
            sc.rx_avail.store(false, Ordering::Relaxed);
            /* set the threshold for when to fire the rx irq */
            bus_write_4!(regs.config, CONFIG_RX_THRESH, 8);

            /* unmask the rx irq */
            DockchannelDriver::unmask_rx(sc.device());

            /* sleep if the rx irq hasn't fired */
            if !sc.rx_avail.load(Ordering::Relaxed) {
                // Drop the lock before sleeping
                mtx_unlock(regs);
                let _ = tsleep(&sc.rx_avail_chan, Some(PWAIT), c"dcrx", hz());
            }
            if sc.rx_avail.load(Ordering::Relaxed) {
                //device_println!(sc.dev, "waited for read data");
                continue;
            }
            /* give up if sleep timed out */
            device_println!(
                sc.device(),
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

fn dockchannel_hid_write(sc: &Softc<DockchannelHidSoftc>, buf: &[u8]) -> Result<()> {
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
            DockchannelDriver::unmask_tx(sc.device());

            if !sc.tx_avail.load(Ordering::Relaxed) {
                // Drop the lock before sleeping
                mtx_unlock(regs);
                let _ = tsleep(&sc.tx_avail_chan, Some(PWAIT), c"dctx", hz());
            }
            if sc.tx_avail.load(Ordering::Relaxed) {
                device_println!(sc.device(), "waited for write data");
                continue;
            }

            /* give up if sleep timed out */
            device_println!(
                sc.device(),
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

fn dockchannel_hid_get_child<'a>(
    sc: &'a DockchannelHidSoftc,
    child: Device,
) -> (&'a DockchannelHidChild, &'a DockchannelHidIface) {
    // Either child may not have been added yet, try_get rather than panicking in get
    if sc.kbd_hid.hidbus.try_get() == Some(&child.as_ptr()) {
        (&sc.kbd_hid, &sc.kbd)
    } else if sc.mt_hid.hidbus.try_get() == Some(&child.as_ptr()) {
        (&sc.mt_hid, &sc.mt)
    } else {
        panic!("uh oh")
    }
}

impl HidIf for DockchannelHidDriver {
    fn hid_intr_setup(
        sc: &Softc<DockchannelHidSoftc>,
        child: Device,
        intr: hid_intr_t,
        context: *mut c_void,
        rdesc: *mut hid_rdesc_info,
    ) {
        if intr.is_none() {
            return;
        }
        let (dc, _) = dockchannel_hid_get_child(&*sc, child);
        unsafe {
            (*rdesc).rdsize = 0; // XXX
            (*rdesc).wrsize = 0; // XXX
            // get/set_report aren't implemented so don't advertise sizes.
            (*rdesc).grsize = 0;
            (*rdesc).srsize = 0;
        }
        // Set ctx before handler since the task only checks handler. This may get called more
        // than once.
        dc.ctx.store(context, Ordering::Release);
        if !dc.handler.is_init() {
            dc.handler.init(intr);
        }
    }
    fn hid_intr_unsetup(sc: &Softc<DockchannelHidSoftc>, child: Device) {}
    fn hid_intr_start(sc: &Softc<DockchannelHidSoftc>, child: Device) -> Result<()> {
        // Send the multi-touch firmware when the trackpad is first opened, once /boot/firmware is
        // available. This runs with only hidbus' sx lock held, so sleeping for the acks is fine.
        let (_, dcif) = dockchannel_hid_get_child(&*sc, child);
        if core::ptr::eq(dcif, &sc.mt) && !sc.mt_fw_loaded.swap(true, Ordering::AcqRel) {
            if let Err(e) = dockchannel_hid_load_mt_fw(sc) {
                device_println!(sc.device(), "failed to load multi-touch firmware {e:?}");
                // Retry on the next open
                sc.mt_fw_loaded.store(false, Ordering::Release);
                return Err(e);
            }
        }
        Ok(())
    }
    fn hid_intr_stop(sc: &Softc<DockchannelHidSoftc>, child: Device) -> Result<()> {
        Ok(())
    }
    fn hid_intr_poll(sc: &Softc<DockchannelHidSoftc>, child: Device) {}
    fn hid_get_rdesc(sc: &Softc<DockchannelHidSoftc>, child: Device, buf: &mut [u8]) -> Result<()> {
        let (dc, dcif) = dockchannel_hid_get_child(&*sc, child);
        assert!(dcif.desc.is_init());
        assert!(dcif.ready.load(Ordering::Relaxed));
        buf.copy_from_slice(&dcif.desc.get()[0..buf.len()]);
        Ok(())
    }
    fn hid_get_report(
        sc: &Softc<DockchannelHidSoftc>,
        child: Device,
        data: *mut c_void,
        max_len: hid_size_t,
        actlen: *mut hid_size_t,
        ty: u8,
        id: u8,
    ) -> Result<()> {
        Err(ENOTSUP)
    }
    fn hid_set_report(
        sc: &Softc<DockchannelHidSoftc>,
        child: Device,
        buf: *const c_void,
        len: hid_size_t,
        ty: u8,
        id: u8,
    ) -> Result<()> {
        Err(ENOTSUP)
    }
}

define_driver! {
    static dockchannel_hid_driver: DockchannelHidDriver = {
        name: c"dockchannel_hid",
    }
    static dockchannel_hid_method_table = {
        device_probe: dockchannel_hid_probe,
        device_attach: dockchannel_hid_attach,

        /* HID interrupt interface */
        hid_intr_setup: dockchannel_hid_intr_setup,
        hid_intr_unsetup: dockchannel_hid_intr_unsetup,
        hid_intr_start: dockchannel_hid_intr_start,
        hid_intr_stop: dockchannel_hid_intr_stop,
        hid_intr_poll: dockchannel_hid_intr_poll,

        /* HID interface */
        hid_get_rdesc: dockchannel_hid_get_rdesc,
        hid_get_report: dockchannel_hid_get_report,
        hid_set_report: dockchannel_hid_set_report,
    }
    // Required to use interfaces defined outside the KPI crate
    with interfaces from { hid }
}
