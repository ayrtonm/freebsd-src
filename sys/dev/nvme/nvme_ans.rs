/* Copyright TBD */
/* derived in part from nvms_ahci.c, also OpenBSD aplns.c */
/*-
 * Copyright (C) 2017 Olivier Houchard
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
/*	$OpenBSD: aplns.c,v 1.12 2022/06/12 16:00:12 kettenis Exp $ */
/*
 * Copyright (c) 2014, 2021 David Gwynne <dlg@openbsd.org>
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

use core::ffi::c_void;
use kpi::kobj::{AsRustType, AsCType};
use kpi::bindings::{
    INTR_MPSAFE, INTR_TYPE_MISC, QUIRK_ANS, bus_addr_t, bus_dma_tag_t, bus_dmamap_t, bus_size_t,
    device_t, nvme_controller, nvme_qpair, nvme_registers, nvme_tracker, phandle_t,
};
use kpi::bus::{Register, Resource};
use kpi::device::{BusProbe, DeviceIf};
use kpi::ffi::SubClass;
use kpi::prelude::*;
use kpi::sync::Mutable;
use kpi::base;
use kpi::{ErrCode, driver};
use kpi::sync::arc::{Arc, ArcRef, UninitArc};
use nvme::NvmeIf;
use rtkit::RTKit;

const ANS_CPU_CTRL: u64 = 0x0044;
const ANS_CPU_CTRL_RUN: u32 = 1 << 4;

const ANS_ACQ_DB: u32 = 0x1004;
const ANS_IOCQ_DB: u32 = 0x100c;

const ANS_MAX_PEND_CMDS_CTRL: u64 = 0x1210;
const ANS_MAX_QUEUE_DEPTH: u32 = 64;

const ANS_BOOT_STATUS: u64 = 0x01300;
const ANS_BOOT_STATUS_OK: u32 = 0xde71ce55;

const ANS_MODESEL_REG: u64 = 0x01304;
const ANS_UNKNOWN_CTRL: u64 = 0x24008;
const ANS_PRP_NULL_CHECK: u32 = 1 << 11;

const ANS_LINEAR_SQ_CTRL: u64 = 0x24908;
const ANS_LINEAR_SQ_CTRL_EN: u32 = 1 << 0;

const ANS_LINEAR_ASQ_DB: u32 = 0x2490c;
const ANS_LINEAR_IOSQ_DB: u32 = 0x24910;

const ANS_NVMMU_NUM: u64 = 0x28100;
//#define	ANS_NVMMU_BASE_ASQ	0x28108
//#define	ANS_NVMMU_BASE_IOSQ	0x28110
//#define	ANS_NVMMU_TCB_INVAL	0x28118
//#define	ANS_NVMMU_TCB_STAT	0x28120

const ANS_NVMMU_TCB_SIZE: u32 = 0x4000;
const ANS_NVMMU_TCB_PITCH: u32 = 0x80;

#[repr(C)]
#[derive(Debug)]
struct NvmeAnsNvmmuTcb {
    opcode: u8,
    flags: u8,
    cid: u8,
    pad0: u8,
    prpl_len: u32,
    pad1: [u16; 2],
    prp: [u64; 2],
}

#[derive(Debug, Default)]
struct NvmeAnsQpair {
    qpair: *mut nvme_qpair,
    tcb: *mut NvmeAnsNvmmuTcb,
    addr: bus_addr_t,
    size: bus_size_t,
    kva: *mut c_void,
    tag: bus_dma_tag_t,
    map: bus_dmamap_t,
    is_admin: bool,
}

unsafe impl Sync for NvmeAnsQpair {}
unsafe impl Send for NvmeAnsQpair {}

pub type NvmeAnsSoftc = SubClass<nvme_controller, NvmeAnsSoftcInternal>;

pub struct NvmeAnsSoftcInternal {
    ans: Mutable<Register>,
    sart: phandle_t,
    rtkit: Arc<RTKit>,
    adminq: Mutable<NvmeAnsQpair>,
    ioq: Mutable<NvmeAnsQpair>,
}

impl DeviceIf for NvmeAnsDriver {
    type Softc = NvmeAnsSoftc;

    fn device_probe(dev: device_t) -> Result<BusProbe> {
        if !ofw_bus_status_okay(dev) {
            return Err(ENXIO);
        }
        if !ofw_bus_is_compatible(dev, c"NOT YET apple,nvme-ans2") {
            return Err(ENXIO);
        }
        device_set_desc(dev, c"Apple NVME Storage controller");
        Ok(BUS_PROBE_DEFAULT)
    }

    fn device_attach(uninit_sc: UninitArc<NvmeAnsSoftc>, dev: device_t) -> Result<()> {
        let node = ofw_bus_get_node(dev);

        let ans_id = ofw_bus_find_string_index(node, c"reg-names", c"ans").map_err(|e| {
            device_println!(dev, "couldn't find 'ans' reg {e}");
            ENXIO
        })?;
        let ans_reg =
            bus_alloc_resource_any(dev, SYS_RES_MEMORY, ans_id, RF_ACTIVE).map_err(|e| {
                device_println!(dev, "couldn't allocate 'ans' mem resource {e}");
                ENOMEM
            })?;
        let ans = Mutable::new(ans_reg.as_register()?);

        let mut ctrlr = nvme_controller::default();
        ctrlr.resource_id =
            ofw_bus_find_string_index(node, c"reg-names", c"nvme").map_err(|e| {
                // TODO: clean up previous allocations
                device_println!(dev, "couldn't find 'nvme' reg {e}");
                ENXIO
            })?;
        let resource = bus_alloc_resource_any(dev, SYS_RES_MEMORY, ctrlr.resource_id, RF_ACTIVE)
            .map_err(|e| {
                device_println!(dev, "couldn't allocate 'nvme' mem resource {e}");
                ENOMEM
            })?;
        ctrlr.bus_tag = rman_get_bustag(&resource);
        ctrlr.bus_handle = rman_get_bushandle(&resource);
        ctrlr.resource = resource.as_c_type();
        ctrlr.regs = ctrlr.bus_handle as *mut nvme_registers;
        ctrlr.rid = 0;
        ctrlr.res = bus_alloc_resource_any(dev, SYS_RES_IRQ, ctrlr.rid, RF_SHAREABLE | RF_ACTIVE)
            .map_err(|e| {
                device_println!(dev, "couldn't allocate irq resource {e}");
                ENOMEM
            })?
            .as_c_type();
        ctrlr.msi_count = 0;
        ctrlr.num_io_queues = 1;
        ctrlr.quirks |= QUIRK_ANS as u32;

        let sart = unsafe {
            OF_getencprop::<phandle_t>(node, c"apple,sart").map_err(|e| {
                device_println!(dev, "couldn't find 'apple,sart' property {e}");
                ENXIO
            })?
        };
        let rtkit = RTKit::new(dev).map_err(|e| {
            device_println!(dev, "error initializing rtkit {e}");
            ENXIO
        })?;
        let rtkit = Arc::new(rtkit, M_DEVBUF, M_WAITOK);
        let adminq = Mutable::new(NvmeAnsQpair::default());
        let ioq = Mutable::new(NvmeAnsQpair::default());
        let sc = NvmeAnsSoftcInternal {
            //ctrlr: UnsafeCell::new(ctrlr),
            ans,
            sart,
            rtkit,
            adminq,
            ioq,
        };
        let sc = uninit_sc.init(SubClass::new_with_base(ctrlr, sc));
        let ctrlr = SubClass::as_base_ptr(&sc);
        let error = unsafe {
            bindings::bus_setup_intr(
                dev,
                (*ctrlr).res,
                (INTR_TYPE_MISC | INTR_MPSAFE) as i32,
                None,
                Some(bindings::nvme_ctrlr_shared_handler),
                ctrlr.cast::<c_void>(),
                &raw const (*ctrlr).tag as *const *mut c_void as *mut *mut c_void,
            )
        };
        if error != 0 {
            device_println!(dev, "couldn't set up interrupt handler {error}");
            return Err(ErrCode::from(error));
        }
        let error = unsafe { bindings::nvme_attach(dev) };
        if error != 0 {
            device_println!(dev, "generic nvme_attach failed {error}");
            return Err(ErrCode::from(error));
        }
        Ok(())
    }

    // FIXME: this is silly since the original version just set nvme_detach in the method table. I
    // should find a way to do that
    fn device_detach(sc: Arc<NvmeAnsSoftc>, dev: device_t) -> Result<()> {
        let res = unsafe { bindings::nvme_detach(dev) };
        if res != 0 {
            return Err(ErrCode::from(res));
        }
        Ok(())
    }
}

impl NvmeAnsDriver {
    fn alloc_qpair(dev: device_t, id: u32) -> Result<NvmeAnsQpair> {
        let is_admin = id == 0;
        Ok(todo!(""))
    }
}

fn nvme_ans_qpair_to_tcb() {}

impl NvmeIf for NvmeAnsDriver {
    fn nvme_delayed_attach(
        sc: ArcRef<NvmeAnsSoftc>,
        dev: device_t,
        ctrlr: *mut nvme_controller,
    ) -> Result<()> {
        let mut ans = sc.ans.get_mut();
        let mut ctrl = bus_read_4!(ans, ANS_CPU_CTRL);
        bus_write_4!(ans, ANS_CPU_CTRL, ctrl | ANS_CPU_CTRL_RUN);

        // SAFETY: TODO: No other thread should be concurrently modifying this field
        let res: Resource = unsafe { base!(sc->res).as_rust_type() };
        let mut nvme_reg = res.as_register()?;
        let mut status = bus_read_4!(nvme_reg, ANS_BOOT_STATUS);
        if status != ANS_BOOT_STATUS_OK {
            device_println!(dev, "booting rtkit");
            let sc_rtkit = sc.clone();
            //rtkit_start(project!(sc_rtkit->rtkit))?;
        }
        for timo in 0..100000 {
            status = bus_read_4!(nvme_reg, ANS_BOOT_STATUS);
            if status != ANS_BOOT_STATUS_OK {
                DELAY(1);
            }
        }
        status = bus_read_4!(nvme_reg, ANS_BOOT_STATUS);
        if status != ANS_BOOT_STATUS_OK {
            device_println!(dev, "timed out waiting for firmware {status}");
            return Err(ENXIO);
        }
        *sc.adminq.get_mut() = Self::alloc_qpair(dev, 0).map_err(|e| {
            device_println!(dev, "unable to allocate dma mem for admin queue {e}");
            ENXIO
        })?;
        *sc.ioq.get_mut() = Self::alloc_qpair(dev, 1).map_err(|e| {
            device_println!(dev, "unable to allocate dma mem for admin queue {e}");
            ENXIO
        })?;
        bus_write_4!(nvme_reg, ANS_LINEAR_SQ_CTRL, ANS_LINEAR_SQ_CTRL_EN);
        bus_write_4!(
            nvme_reg,
            ANS_MAX_PEND_CMDS_CTRL,
            (ANS_MAX_QUEUE_DEPTH << 16) | ANS_MAX_QUEUE_DEPTH
        );
        ctrl = bus_read_4!(nvme_reg, ANS_UNKNOWN_CTRL);
        bus_write_4!(nvme_reg, ANS_UNKNOWN_CTRL, ctrl & !ANS_PRP_NULL_CHECK);
        device_println!(dev, "finished nvme delayed attach");
        Ok(())
    }

    fn nvme_enable(sc: ArcRef<NvmeAnsSoftc>, dev: device_t) {
        // SAFETY: TODO: No other thread should be concurrently modifying this field
        let res: Resource = unsafe { base!(sc->res).as_rust_type() };
        let mut nvme_reg = res.as_register().unwrap();
        bus_write_4!(
            nvme_reg,
            ANS_NVMMU_NUM,
            (ANS_NVMMU_TCB_SIZE / ANS_NVMMU_TCB_PITCH) - 1
        );
        bus_write_4!(nvme_reg, ANS_MODESEL_REG, 0);
    }

    fn nvme_sq_enter(
        sc: ArcRef<NvmeAnsSoftc>,
        dev: device_t,
        qpair: *mut nvme_qpair,
        tr: &nvme_tracker,
    ) -> u32 {
        let id = unsafe { (*tr.req).cmd.cid };
        u32::from(id)
    }

    fn nvme_sq_leave(
        sc: ArcRef<NvmeAnsSoftc>,
        dev: device_t,
        qpair: &nvme_qpair,
        tr: &nvme_tracker,
    ) {
        let mut ans_qpair = &sc.adminq;
        if qpair.id != 0 {
            ans_qpair = &sc.ioq;
        }
        //let tcb =
        let id = unsafe { (*tr.req).cmd.cid };
        let cmd = unsafe { qpair.cmd.add(usize::from(id)) };

        // SAFETY: TODO: No other thread should be concurrently modifying this field
        let res: Resource = unsafe { base!(sc->res).as_rust_type() };
        let mut nvme_reg = res.as_register().unwrap();
        bus_write_4!(nvme_reg, u64::from(qpair.sq_tdbl_off), u32::from(id));
    }

    fn nvme_qpair_construct(
        sc: ArcRef<NvmeAnsSoftc>,
        dev: device_t,
        qpair: &mut nvme_qpair,
        num_entries: u32,
        num_trackers: u32,
        ctrlr: *mut nvme_controller,
    ) -> Result<()> {
        let res = unsafe {
            bindings::nvme_qpair_construct(dev, qpair, num_entries, num_trackers, todo!("sc"))
        };
        if res != 0 {
            return Err(ErrCode::from(res));
        }
        if qpair.id == 0 {
            qpair.sq_tdbl_off = ANS_LINEAR_ASQ_DB;
            qpair.cq_hdbl_off = ANS_ACQ_DB;
            sc.adminq.get_mut().qpair = qpair;
        } else {
            qpair.sq_tdbl_off = ANS_LINEAR_IOSQ_DB;
            qpair.cq_hdbl_off = ANS_IOCQ_DB;
            sc.ioq.get_mut().qpair = qpair;
        }
        Ok(())
    }

    fn nvme_cq_done(
        sc: ArcRef<NvmeAnsSoftc>,
        dev: device_t,
        qpair: &nvme_qpair,
        tr: &nvme_tracker,
    ) {
        let id = unsafe { (*tr.req).cmd.cid };
        let mut ans_qpair = &sc.adminq;
        if qpair.id != 0 {
            ans_qpair = &sc.ioq;
        }

        //bus_write_4!(nvme_reg,
    }
}

driver!(nvme_ans_driver, c"nvme", NvmeAnsDriver,
    nvme_ans_methods = {
        /* Device interface */
        device_probe nvme_ans_probe,
        device_attach nvme_ans_attach,
        device_detach nvme_ans_detach,
        device_shutdown nvme_ans_shutdown,

        /* NVME interface */
        nvme_delayed_attach nvme_ans_delayed_attach,
        nvme_enable nvme_ans_enable,
        nvme_sq_enter nvme_ans_sq_enter,
        nvme_sq_leave nvme_ans_sq_leave,
        nvme_cq_done nvme_ans_cq_done,
        nvme_qpair_construct nvme_ans_qpair_construct,
    }
    with interfaces from { nvme };
);
