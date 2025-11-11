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
use core::ptr::null_mut;
use kpi::bindings::{
    INTR_MPSAFE, INTR_TYPE_MISC, QUIRK_ANS, bus_addr_t, bus_dma_segment_t, bus_dma_tag_t,
    bus_dmamap_t, bus_size_t, device_t, nvme_controller, nvme_qpair, nvme_registers, nvme_tracker,
    phandle_t,
};
use kpi::bus::dma::{BusDmaMap, BusDmaMem, BusDmaTag};
use kpi::bus::{Register, Resource};
use kpi::device::{BusProbe, DeviceIf};
use kpi::ffi::{Ptr, Ref, SubClass, UninitRef};
use kpi::kobj::AsRustType;
use kpi::ofw::XRef;
use kpi::prelude::*;
use kpi::sync::Checked;
use kpi::{base, driver, proj};
use nvme::prelude::*;
use nvme::{NvmeIf, NvmeSoftc};
use rtkit::{RTKit, RTKitDriver, rtkit_boot, rtkit_init, PwrState, rtkit_set_ap};

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
const ANS_NVMMU_BASE_ASQ: u64 = 0x28108;
const ANS_NVMMU_BASE_IOSQ: u64 = 0x28110;
const ANS_NVMMU_TCB_INVAL: u64 = 0x28118;
const ANS_NVMMU_TCB_STAT: u64 = 0x28120;

const ANS_NVMMU_TCB_SIZE: u32 = 0x4000;
const ANS_NVMMU_TCB_PITCH: u32 = 0x80;

const ANS_NVMMU_TCB_WRITE: u8 = 1 << 0;
const ANS_NVMMU_TCB_READ: u8 = 1 << 1;

#[repr(C)]
#[derive(Debug)]
struct NvmeAnsNvmmuTcb {
    opcode: u8,
    flags: u8,
    cid: u8,
    pad0: u8,
    prpl_len: u32,
    pad1: [u8; 16],
    //pad1: [u16; 2],
    prp1: u64,
    prp2: u64,
}

#[derive(Debug, Default)]
struct NvmeAnsQpair {
    qpair: Ptr<nvme_qpair>,
    tcb: Ptr<NvmeAnsNvmmuTcb>,
    addr: bus_addr_t,
    size: bus_size_t,
    kva: BusDmaMem,
    tag: BusDmaTag,
    map: BusDmaMap,
    is_admin: bool,
    sc: Ptr<NvmeAnsSoftc>,
}

fn nvme_ans_sart_map(sc: &NvmeAnsSoftc, addr: bus_addr_t, size: bus_size_t) {
    let xref = sc.sart.0;
    let res = unsafe { bindings::apple_sart_map(xref, addr, size) };
    // This function asserts there is no failure because it is ultimately used as a busdma map
    // callback which does not allow returning an error.
    assert!(res == 0);
}

pub type NvmeAnsSoftc = NvmeSoftc<NvmeAnsSoftcFields>;

pub struct NvmeAnsSoftcFields {
    dev: device_t,
    ans: Checked<Register>,
    sart: XRef,
    rtk: RTKit<NvmeAnsSoftc>,
    adminq: Checked<NvmeAnsQpair>,
    ioq: Checked<NvmeAnsQpair>,
}

impl RTKitDriver for NvmeAnsDriver {
    type CallbackArg = Self::Softc;

    fn get_rtkit(sc: Ref<Self::Softc>) -> Ref<RTKit<Self::Softc>> {
        proj!(&sc->rtk)
    }
}

impl DeviceIf for NvmeAnsDriver {
    type Softc = NvmeAnsSoftc;

    fn device_probe(dev: device_t) -> Result<BusProbe> {
        if !ofw_bus_status_okay(dev) {
            return Err(ENXIO);
        }
        if !ofw_bus_is_compatible(dev, c"apple,nvme-ans2") {
            return Err(ENXIO);
        }
        device_set_desc(dev, c"Apple NVME Storage controller");
        Ok(BUS_PROBE_DEFAULT)
    }

    fn device_attach(uninit_sc: UninitRef<NvmeAnsSoftc>, dev: device_t) -> Result<()> {
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
        let ans = Checked::new(ans_reg.into_register()?);

        // SAFETY: This devicetree property should be interpreted as an XRef and that type has a
        // specified layout
        let sart = unsafe {
            OF_getencprop_unchecked::<XRef>(node, c"apple,sart").map_err(|e| {
                device_println!(dev, "couldn't find 'apple,sart' property {e}");
                ENXIO
            })?
        };
        let rtk = Self::new_rtkit(dev).map_err(|e| {
            device_println!(dev, "error initializing rtkit {e}");
            ENXIO
        })?;
        let adminq = Checked::new(NvmeAnsQpair::default());
        let ioq = Checked::new(NvmeAnsQpair::default());

        // At this point `sc` is a just local variable on the stack that usually gets optimized out
        let sc = NvmeAnsSoftcFields {
            dev,
            ans,
            sart,
            rtk,
            adminq,
            ioq,
        };

        // Only specify the subclass part of the softc here since the base class could overflow the
        // stack if a temporary is constructed on the stack. The base class on the heap is
        // initialized with default values that are later re-initialized in-place to the correct
        // values through a mutable reference. At this point `sc` is a UniqueRef that allows mutably
        // accessing its fields (though SubClass restricts that mutable access to the base class).
        let mut sc = uninit_sc.init(SubClass::new(sc));

        // Open a new scope to initialize the softc base class to limit the extent in which the
        // mutable reference to it (which must be unique) is valid. This helps avoid UB in case we
        // end up creating other references to the softc base class later in this function.
        {
            // Get the a pointer to the base class
            let sc_base_ptr = base!(&sc);

            // SAFETY: There are no other references to the softc base class for the lifetime of
            // this reference (i.e. in this scope) so `as_mut()` will not create UB.
            let mut ctrlr = unsafe { sc_base_ptr.as_mut().unwrap() };

            // Finally initialize the fields we care about
            ctrlr.resource_id =
                ofw_bus_find_string_index(node, c"reg-names", c"nvme").map_err(|e| {
                    // TODO: clean up previous allocations
                    device_println!(dev, "couldn't find 'nvme' reg {e}");
                    ENXIO
                })?;

            // `bus_alloc_resource_any` returns a `Resource` which is the rust KPI wrapper around
            // `*mut resource` so we use `as_ptr()` to get the pointer to actually initialize the
            // field used by the C code.
            ctrlr.resource =
                bus_alloc_resource_any(dev, SYS_RES_MEMORY, ctrlr.resource_id, RF_ACTIVE)
                    .map_err(|e| {
                        device_println!(dev, "couldn't allocate 'nvme' mem resource {e}");
                        ENOMEM
                    })?
                    .as_ptr();
            ctrlr.rid = 0;
            ctrlr.res =
                bus_alloc_resource_any(dev, SYS_RES_IRQ, ctrlr.rid, RF_SHAREABLE | RF_ACTIVE)
                    .map_err(|e| {
                        device_println!(dev, "couldn't allocate irq resource {e}");
                        ENOMEM
                    })?
                    .as_ptr();
            ctrlr.msi_count = 0;
            ctrlr.num_io_queues = 1;
            // We're attached via this funky mechanism. Flag the controller so that
            // it avoids things that can't work when we do that, like asking for
            // PCI config space entries.
            ctrlr.quirks |= QUIRK_ANS as u32;
        }

        // Set the callback for the RTKit module to use when it maps in a buffer. The callback is
        // type-checked using the RTKit's generic parameter
        sc.rtk.map_callback = Some(nvme_ans_sart_map);

        // Turn the UniqueRef into a shared Ref. This allows using the `proj!` macro to project the
        // pointer to a pointer to the struct's fields which is required to initialize the RTKit.
        let sc = sc.into_ref();

        // Initialize the RTKit instance's taskqueue, thread and mailbox
        rtkit_init(proj!(&sc->rtk))?;

        nvme_setup_intr(dev, &sc).map_err(|e| {
            device_println!(dev, "couldn't set up interrupt handler {e}");
            ENXIO
        })?;

        // This sets up the config hook that ends up calling nvme_delayed_attach
        Self::nvme_attach(dev).map_err(|e| {
            device_println!(dev, "generic nvme_attach failed {e}");
            ENXIO
        })?;
        Ok(())
    }
}

impl NvmeIf for NvmeAnsDriver {
    fn nvme_delayed_attach(sc: Ref<NvmeAnsSoftc>) {
        let dev = sc.dev;

        // The ANS register isn't accessed anywhere else so this Checked access can't panic
        let mut ans = sc.ans.get_mut();
        // Now that we have a mutable reference to the Resource do the actual MMIO reads and writes
        let ctrl = bus_read_4!(ans, ANS_CPU_CTRL);
        bus_write_4!(ans, ANS_CPU_CTRL, ctrl | ANS_CPU_CTRL_RUN);

        // SAFETY: This function is called from the config hook and nothing else should be
        // concurrently accessing the nvme register. This won't panic since `sc->resource` was
        // created with SYS_RES_MEMORY.
        let mut nvme_reg = unsafe { Register::from_raw(base!(sc->resource)).unwrap() };

        let mut status = bus_read_4!(nvme_reg, ANS_BOOT_STATUS);
        if status != ANS_BOOT_STATUS_OK {
            rtkit_boot(proj!(&sc->rtk)).unwrap();
        }
        rtkit_set_ap(proj!(&sc->rtk), PwrState::On).unwrap();

        for timo in 0..100000 {
            status = bus_read_4!(nvme_reg, ANS_BOOT_STATUS);
            if status != ANS_BOOT_STATUS_OK {
                DELAY(1);
            }
        }
        status = bus_read_4!(nvme_reg, ANS_BOOT_STATUS);
        if status != ANS_BOOT_STATUS_OK {
            device_println!(dev, "timed out waiting for firmware {status}");
            panic!("uh oh");
        }

        nvme_ans_alloc_qpair(sc, 0, proj!(&sc->adminq))
            .inspect_err(|e| {
                device_println!(dev, "unable to allocate dma mem for admin queue {e}");
            })
            .unwrap();
        nvme_ans_alloc_qpair(sc, 1, proj!(&sc->ioq))
            .inspect_err(|e| {
                device_println!(dev, "unable to allocate dma mem for admin queue {e}");
            })
            .unwrap();

        // wait for dmamap callback instead?
        assert!(sc.adminq.get_mut().addr != 0);
        assert!(sc.ioq.get_mut().addr != 0);

        bus_write_4!(nvme_reg, ANS_LINEAR_SQ_CTRL, ANS_LINEAR_SQ_CTRL_EN);
        bus_write_4!(
            nvme_reg,
            ANS_MAX_PEND_CMDS_CTRL,
            (ANS_MAX_QUEUE_DEPTH << 16) | ANS_MAX_QUEUE_DEPTH
        );
        let ctrl = bus_read_4!(nvme_reg, ANS_UNKNOWN_CTRL);
        bus_write_4!(nvme_reg, ANS_UNKNOWN_CTRL, ctrl & !ANS_PRP_NULL_CHECK);
    }

    fn nvme_enable(sc: Ref<NvmeAnsSoftc>) {
        // SAFETY: The res field is initialized once and no thread should be using the register at
        // this point.
        let mut nvme_reg = unsafe { Register::from_raw(base!(sc->resource)).unwrap() };
        bus_write_4!(
            nvme_reg,
            ANS_NVMMU_NUM,
            (ANS_NVMMU_TCB_SIZE / ANS_NVMMU_TCB_PITCH) - 1
        );
        bus_write_4!(nvme_reg, ANS_MODESEL_REG, 0);
    }

    fn nvme_sq_enter(sc: Ref<Self::Softc>, qpair: *mut nvme_qpair, tr: &nvme_tracker) -> u32 {
        let res = unsafe { (*tr.req).cmd.cid };
        u32::from(res)
    }

    fn nvme_sq_leave(sc: Ref<NvmeAnsSoftc>, qpair: &nvme_qpair, tr: &nvme_tracker) {
        let ans_qpair = match qpair.id {
            0 => &sc.adminq,
            _ => &sc.ioq,
        };
        let ans_qpair = ans_qpair.get_mut();
        let tcb = nvme_ans_qpair_to_tcb(&*ans_qpair, tr);
        let id = unsafe { (*tr.req).cmd.cid };
        let cmd = unsafe { qpair.cmd.add(usize::from(id)) };

        bus_dmamap_sync(
            ans_qpair.tag,
            ans_qpair.map,
            BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE,
        );
        unsafe {
            bindings::memset(tcb.cast(), 0, size_of::<[u8; 128]>());
            (*tcb).opcode = (*cmd).opc;
            (*tcb).flags = ANS_NVMMU_TCB_WRITE | ANS_NVMMU_TCB_READ;
            //(*tcb).flags = if (*cmd).opc & 1 != 0 {
            //    ANS_NVMMU_TCB_WRITE
            //} else {
            //    ANS_NVMMU_TCB_READ
            //};
            (*tcb).cid = id.try_into().unwrap();
            // intentionally truncate
            (*tcb).prpl_len = (*cmd).cdw12 & u32::from(u16::MAX);
            (*tcb).prp1 = (*cmd).__bindgen_anon_1.bindgen_union_field[0];
            (*tcb).prp2 = (*cmd).__bindgen_anon_1.bindgen_union_field[1];
        };
        bus_dmamap_sync(
            ans_qpair.tag,
            ans_qpair.map,
            BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE,
        );
        unsafe {
            bindings::bus_dmamap_sync(
                qpair.dma_tag,
                qpair.queuemem_map,
                BUS_DMASYNC_PREREAD.0 | BUS_DMASYNC_PREWRITE.0,
            )
        };

        // SAFETY: TODO: No other thread should be concurrently modifying this field
        let mut nvme_reg = unsafe { Register::from_raw(base!(sc->resource)).unwrap() };
        bus_write_4!(nvme_reg, u64::from(qpair.sq_tdbl_off), u32::from(id));
    }

    fn nvme_qpair_construct(
        sc: Ref<NvmeAnsSoftc>,
        qpair: *mut nvme_qpair,
        num_entries: u32,
        num_trackers: u32,
        ctrlr: *mut nvme_controller,
    ) -> Result<()> {
        let id = unsafe { (*qpair).id };
        nvme_qpair_construct(sc.dev, qpair, num_entries, num_trackers, sc).map_err(|e| {
            device_println!(sc.dev, "failed to construct qpair");
            ENXIO
        })?;
        unsafe {
            if (*qpair).id == 0 {
                (*qpair).sq_tdbl_off = ANS_LINEAR_ASQ_DB;
                (*qpair).cq_hdbl_off = ANS_ACQ_DB;
                sc.adminq.get_mut().qpair = Ptr::new(qpair);
            } else {
                (*qpair).sq_tdbl_off = ANS_LINEAR_IOSQ_DB;
                (*qpair).cq_hdbl_off = ANS_IOCQ_DB;
                sc.ioq.get_mut().qpair = Ptr::new(qpair);
            }
        }
        Ok(())
    }

    fn nvme_cq_done(sc: Ref<NvmeAnsSoftc>, qpair: &nvme_qpair, tr: &nvme_tracker) {
        let id = unsafe { (*tr.req).cmd.cid };
        let ans_qpair = if qpair.id == 0 {
            &*sc.adminq.get_mut()
        } else {
            &*sc.ioq.get_mut()
        };
        let tcb = nvme_ans_qpair_to_tcb(ans_qpair, tr);

        bus_dmamap_sync(
            ans_qpair.tag,
            ans_qpair.map,
            BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE,
        );
        unsafe { bindings::memset(tcb.cast(), 0, size_of::<[u8; 128]>()) };
        bus_dmamap_sync(
            ans_qpair.tag,
            ans_qpair.map,
            BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE,
        );
        let mut nvme_reg = unsafe { Register::from_raw(base!(sc->resource)).unwrap() };
        bus_write_4!(nvme_reg, ANS_NVMMU_TCB_INVAL, u32::from(id));
        let stat = bus_read_4!(nvme_reg, ANS_NVMMU_TCB_STAT);
        if stat != 0 {
            device_println!(sc.dev, "nvmmu tcb stat is non-zero {stat:x?}");
        }
    }
}

fn nvme_ans_alloc_qpair(
    sc: Ref<NvmeAnsSoftc>,
    id: u32,
    qpair: Ref<Checked<NvmeAnsQpair>>,
) -> Result<()> {
    let is_admin = id == 0;

    let dev = sc.dev;
    let parent_tag = bus_get_dma_tag(dev);
    let tcb_size = u64::from(ANS_NVMMU_TCB_SIZE);
    // TODO: alignment, bounds
    let tag = bus_dma_tag_create(parent_tag)
        .alignment(PAGE_SIZE)
        .bounds(0)
        .max_size(tcb_size)
        .num_segments(1)
        .max_seg_size(tcb_size)
        .flags(Some(BUS_DMA_COHERENT))
        .build()?;

    let (map, kva) = bus_dmamem_alloc(tag, BUS_DMA_WAITOK | BUS_DMA_ZERO | BUS_DMA_COHERENT)?;

    let mut qpair_guard = qpair.get_mut();
    qpair_guard.is_admin = is_admin;
    qpair_guard.tag = tag;
    qpair_guard.map = map;
    qpair_guard.kva = kva;
    qpair_guard.sc = Ref::into_ptr(sc);
    drop(qpair_guard);

    let res = bus_dmamap_load(
        tag,
        map,
        kva,
        tcb_size,
        Some(nvme_ans_dmamap_cb),
        qpair,
        None,
    );
    match res {
        Ok(_) | Err(EINPROGRESS) => {}
        Err(e) => {
            device_println!(dev, "bus_dmamap_load failed {res:?}");
            return Err(e);
        }
    }
    bus_dmamap_sync(tag, map, BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

    Ok(())
}

extern "C" fn nvme_ans_dmamap_cb(
    qpair: Ref<Checked<NvmeAnsQpair>>,
    segs: &bus_dma_segment_t,
    nsegs: i32,
    error: i32,
) {
    assert!(error == 0);
    assert!(nsegs == 1);

    let mut qpair = qpair.get_mut();
    qpair.addr = segs.ds_addr;
    qpair.size = segs.ds_len;
    let base = if qpair.is_admin {
        ANS_NVMMU_BASE_ASQ
    } else {
        ANS_NVMMU_BASE_IOSQ
    };
    let sc = unsafe { qpair.sc.get() };
    let mut nvme_reg = unsafe { Register::from_raw(base!(sc->resource)).unwrap() };
    bus_write_8!(nvme_reg, base, qpair.addr);
}

fn nvme_ans_qpair_to_tcb(ans_qpair: &NvmeAnsQpair, tr: &nvme_tracker) -> *mut NvmeAnsNvmmuTcb {
    let cid = usize::from(unsafe { (*tr.req).cmd.cid });
    unsafe {
        ans_qpair
            .kva
            .as_ptr()
            .byte_add(cid * ANS_NVMMU_TCB_PITCH as usize)
            .cast::<NvmeAnsNvmmuTcb>()
    }
}

driver!(nvme_ans_driver, c"nvme", NvmeAnsDriver,
    nvme_ans_methods = {
        /* Device interface */
        // These functions are implemented in rust so this macro creates extern "C" functions with
        // the second identifier in each entry. This driver uses nvme_ans_ as prefix to ensure the
        // ELF symbols are unique.
        device_probe nvme_ans_probe,
        device_attach nvme_ans_attach,

        // These functions are implemented in C so the method table points to the second identifier
        // in each entry. The functions must be in the source file generated by bindgen to allow
        // creating this reference. If the functions are manually added there but not implemented
        // this leads to a link-time missing symbol error.
        device_detach nvme_detach defined in C,
        device_shutdown nvme_shutdown defined in C,

        /* NVME interface */
        nvme_delayed_attach nvme_ans_delayed_attach,
        nvme_enable nvme_ans_enable,
        nvme_sq_enter nvme_ans_sq_enter,
        nvme_sq_leave nvme_ans_sq_leave,
        nvme_cq_done nvme_ans_cq_done,
        nvme_qpair_construct nvme_ans_qpair_construct,
    }
    // Required to use interfaces defined outside the KPI crate
    with interfaces from { nvme };
);
