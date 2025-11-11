/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2021 Andrew Turner
 * Copyright (c) 2022 Michael J. Karels <karels@freebsd.org>
 * Copyright (c) 2022 Kyle Evans <kevans@FreeBSD.org>
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

use core::array;
use core::arch::asm;
use core::sync::atomic::AtomicU32;
use kpi::bindings::{cpuset_t, device_t, intr_polarity, intr_trigger, trapframe, u_int};
use kpi::boxed::Box;
use kpi::bus::{Filter, Register, Resource};
use kpi::device::{BusProbe, DeviceIf};
use kpi::ffi::{Ref, ToArrayCString, UninitRef};
use kpi::intr::{IntrRoot, IrqSrc, MapData, PicIf};
use kpi::ofw::OfwCompatData;
use kpi::prelude::*;
use kpi::sync::Checked;
use kpi::vec::Vec;
use kpi::{base, driver, proj, gen_newtype};

const AIC_INFO: u64 = 0x0004;

const AIC_EVENT: u64 = 0x2004;
const AIC_EVENT_TYPE_NONE: u32 = 0;
const AIC_EVENT_TYPE_IRQ: u32 = 1;
const AIC_EVENT_TYPE_IPI: u32 = 4;

const AIC_MAXDIES: usize = 4;
const NUM_FIQS: usize = 6;
const NUM_IPIS: usize = bindings::INTR_IPI_COUNT as usize;

const AIC2_CONFIG: u64 = 0x0014;
const AIC2_CONFIG_ENABLE: u32 = 1 << 0;

const AIC_IPI_SR_EL1_PENDING: u64 = 1 << 0;

const AIC_FIQ_VM_TIMER_VEN: u64 = 1 << 0;
const AIC_FIQ_VM_TIMER_PEN: u64 = 1 << 1;
const AIC_FIQ_VM_TIMER_BITS: u64 = AIC_FIQ_VM_TIMER_VEN | AIC_FIQ_VM_TIMER_PEN;

const CNTV_CTL_ENABLE: u64 = 1 << 0;
const CNTV_CTL_IMASK: u64 = 1 << 1;
const CNTV_CTL_ISTATUS: u64 = 1 << 2;
const CNTV_CTL_BITS: u64 = CNTV_CTL_ENABLE | CNTV_CTL_IMASK | CNTV_CTL_ISTATUS;

const AIC_IRQ_CR_EL1_DISABLE: u64 = 3 << 0;

// Since this macro is only visible in this crate keep the name short
macro_rules! reg {
    (AIC_IPI_SR_EL1) => {
        "s3_5_c15_c1_1"
    };
    (AIC_IPI_LOCAL_RR_EL1) => {
        "s3_5_c15_c0_0"
    };
    (AIC_IPI_GLOBAL_RR_EL1) => {
        "s3_5_c15_c0_1"
    };
    (AIC_TMR_CTL_GUEST_PHYS) => {
        "s3_5_c14_c2_1"
    };
    (AIC_TMR_CTL_GUEST_VIRT) => {
        "s3_5_c14_c3_1"
    };
    (AIC_FIQ_VM_TIMER) => {
        "s3_5_c15_c1_3"
    };
    (AIC_IRQ_CR_EL1) => {
        "s3_4_c15_c10_4"
    };
}

fn info_nirqs(info: u32) -> usize {
    (info & 0xffff) as usize
}

fn info_ndie(info: u32) -> usize {
    ((info >> 24) & 0xf) as usize
}

fn event_type(event: u32) -> u32 {
    (event >> 16) & 0xff
}

fn event_die(event: u32) -> usize {
    ((event >> 24) & 0xff) as usize
}

fn event_irq(event: u32) -> usize {
    (event & 0xffff) as usize
}

fn irq_mask(irq: u64) -> u32 {
    1 << (irq & 0x1f)
}

#[derive(Debug)]
struct AppleIntData {
    version: u32,
    //irq_cfg: Option<u64>,
    sw_set: u64,
    sw_clear: u64,
    mask_set: u64,
    mask_clear: u64,
    die_stride: u64,
}

static AIC_V1: AppleIntData = AppleIntData {
    version: 1,
    //irq_cfg: None,
    sw_set: 0x4000,
    sw_clear: 0x4080,
    mask_set: 0x4100,
    mask_clear: 0x4180,
    die_stride: 0,
};

static AIC_V2: AppleIntData = AppleIntData {
    version: 2,
    //irq_cfg: Some(0x2000),
    sw_set: 0x6000,
    sw_clear: 0x6200,
    mask_set: 0x06400,
    mask_clear: 0x6600,
    die_stride: 0x4a00,
};

static COMPAT_DATA: OfwCompatData<AppleIntData, 2> =
    OfwCompatData::new([(c"apple,aic", &AIC_V1), (c"apple,aic2", &AIC_V2)]);

fn apple_aic_mask_set(sc: &AppleIntSoftc, die: usize, irq: usize) {
    let die = die as u64;
    let irq = irq as u64;
    let mut offset = sc.cfg.mask_set;
    offset += sc.cfg.die_stride * die;
    offset += (irq >> 5) * 4;
    bus_write_4!(sc.mem.get_mut(), offset, irq_mask(irq));
}

fn apple_aic_mask_clear(sc: &AppleIntSoftc, die: usize, irq: usize) {
    let die = die as u64;
    let irq = irq as u64;
    let mut offset = sc.cfg.mask_clear;
    offset += sc.cfg.die_stride * die;
    offset += (irq >> 5) * 4;
    bus_write_4!(sc.mem.get_mut(), offset, irq_mask(irq));
}

fn apple_aic_sw_set(sc: &AppleIntSoftc, die: usize, irq: usize) {
    let die = die as u64;
    let irq = irq as u64;
    let mut offset = sc.cfg.sw_set;
    offset += sc.cfg.die_stride * die;
    offset += (irq >> 5) * 4;
    bus_write_4!(sc.mem.get_mut(), offset, irq_mask(irq));
}

fn apple_aic_sw_clear(sc: &AppleIntSoftc, die: usize, irq: usize) {
    let die = die as u64;
    let irq = irq as u64;
    let mut offset = sc.cfg.sw_clear;
    offset += sc.cfg.die_stride * die;
    offset += (irq >> 5) * 4;
    bus_write_4!(sc.mem.get_mut(), offset, irq_mask(irq));
}

fn apple_aic_fiq_unmask(fiq: AppleFiqKind) {
    match fiq {
        AIC_TMR_GUEST_PHYS => {
            let old = read_specialreg!(reg!(AIC_FIQ_VM_TIMER));
            write_specialreg!(reg!(AIC_FIQ_VM_TIMER), old | AIC_FIQ_VM_TIMER_PEN);
            isb!();
        }
        AIC_TMR_GUEST_VIRT => {
            let old = read_specialreg!(reg!(AIC_FIQ_VM_TIMER));
            write_specialreg!(reg!(AIC_FIQ_VM_TIMER), old | AIC_FIQ_VM_TIMER_VEN);
            isb!();
        }
        _ => { /* no mask bits for the hypervisor timers */ }
    }
    //unsafe { bindings::apple_aic_fiq_unmask(fiq.0 as u32) }
}

fn apple_aic_fiq_mask(fiq: AppleFiqKind) {
    match fiq {
        AIC_TMR_GUEST_PHYS => {
            let old = read_specialreg!(reg!(AIC_FIQ_VM_TIMER));
            write_specialreg!(reg!(AIC_FIQ_VM_TIMER), old & !AIC_FIQ_VM_TIMER_PEN);
            isb!();
        }
        AIC_TMR_GUEST_VIRT => {
            let old = read_specialreg!(reg!(AIC_FIQ_VM_TIMER));
            write_specialreg!(reg!(AIC_FIQ_VM_TIMER), old & !AIC_FIQ_VM_TIMER_VEN);
            isb!();
        }
        _ => { /* no mask bits for the hypervisor timers */ }
    }
    //unsafe { bindings::apple_aic_fiq_mask(fiq.0 as u32) }
}

// IrqSrc<F> is a subclass of intr_irqsrc with extra fields F
pub type AppleIrqSrc = IrqSrc<AppleIrqSrcFields>;

#[derive(Debug)]
pub struct AppleIrqSrcFields {
    kind: AppleIntrKind,
    pol: intr_polarity,
    trig: intr_trigger,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct AppleFiqKind(usize);

gen_newtype! {
    AppleFiqKind as usize,
    AIC_TMR_HV_PHYS,
    AIC_TMR_HV_VIRT,
    AIC_TMR_GUEST_PHYS,
    AIC_TMR_GUEST_VIRT,
    AIC_CPU_PMU_E,
    AIC_CPU_PMU_P,
}

impl AppleFiqKind {
    fn all_fiqs() -> [AppleFiqKind; NUM_FIQS] {
        [
            AIC_TMR_HV_PHYS,
            AIC_TMR_HV_VIRT,
            AIC_TMR_GUEST_PHYS,
            AIC_TMR_GUEST_VIRT,
            AIC_CPU_PMU_E,
            AIC_CPU_PMU_P,
        ]
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
enum AppleIntrKind {
    Irq { die: usize, irq: usize },
    Fiq(AppleFiqKind),
    Ipi,
}

fn new_irq_src(kind: AppleIntrKind) -> AppleIrqSrc {
    AppleIrqSrc::new(AppleIrqSrcFields {
        kind,
        pol: bindings::INTR_POLARITY_CONFORM,
        trig: bindings::INTR_TRIGGER_CONFORM,
    })
}

#[derive(Debug)]
pub struct AppleIntSoftc {
    dev: device_t,
    cfg: &'static AppleIntData,
    mem: Checked<Register>,
    event: Option<Checked<Register>>,
    irq_srcs: Box<[Box<[AppleIrqSrc]>]>,
    fiq_srcs: [AppleIrqSrc; NUM_FIQS],
    ipi_srcs: [AppleIrqSrc; NUM_IPIS],
    ndie: usize,
    nirqs: usize,
    ipimasks: Vec<AtomicU32>,
}

impl DeviceIf for AppleIntDriver {
    type Softc = AppleIntSoftc;

    fn device_probe(dev: device_t) -> Result<BusProbe> {
        if !ofw_bus_status_okay(dev) {
            return Err(ENXIO);
        }
        if ofw_bus_search_compatible(dev, &COMPAT_DATA).is_err() {
            return Err(ENXIO);
        }
        device_set_desc(dev, c"Apple Interrupt Controller");
        return Ok(BUS_PROBE_DEFAULT);
    }

    fn device_attach(uninit_sc: UninitRef<AppleIntSoftc>, dev: device_t) -> Result<()> {
        // Can't fail since it must have been called successfully in device_probe
        let cfg = ofw_bus_search_compatible(dev, &COMPAT_DATA).unwrap();

        let mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, 0, RF_ACTIVE).map_err(|e| {
            device_println!(dev, "unable to allocate memory register {e}");
            return ENXIO;
        })?;
        // SYS_RES_MEMORY was specified above so this can't fail
        let mut mem = Register::new(mem_res).unwrap();
        let mut event = None;
        if cfg.version == 2 {
            let event_res =
                bus_alloc_resource_any(dev, SYS_RES_MEMORY, 1, RF_ACTIVE).map_err(|e| {
                    device_println!(dev, "unable to allocate event register {e}");
                    return ENXIO;
                })?;
            event = Some(Checked::new(Register::new(event_res)?));
        };
        let info = bus_read_4!(mem, AIC_INFO);
        let nirqs = info_nirqs(info);
        let ndie = info_ndie(info) + 1;
        device_println!(dev, "Found {nirqs} interrupts, {ndie} die");

        let mut ndie_vec = Vec::with_capacity(ndie, M_WAITOK);
        for die in 0..ndie {
            let mut irq_vec = Vec::with_capacity(nirqs, M_WAITOK);
            for irq in 0..nirqs {
                irq_vec.push(new_irq_src(AppleIntrKind::Irq { die, irq }));
            }
            ndie_vec.push(irq_vec.into_boxed_slice());
        }
        let irq_srcs = ndie_vec.into_boxed_slice();
        let fiq_srcs = AppleFiqKind::all_fiqs().map(|fiq| new_irq_src(AppleIntrKind::Fiq(fiq)));
        let ipi_srcs = array::from_fn(|_| new_irq_src(AppleIntrKind::Ipi));

        let num_masks = mp_maxid() + 1;
        let mut ipimasks = Vec::with_capacity(num_masks, M_WAITOK);
        for _ in 0..num_masks {
            ipimasks.push(AtomicU32::new(0));
        }
        if cfg.version == 1 {
            todo!("set cpuids");
        }
        let sc = AppleIntSoftc {
            dev,
            cfg,
            mem: Checked::new(mem),
            event,
            irq_srcs,
            fiq_srcs,
            ipi_srcs,
            ndie,
            nirqs,
            ipimasks,
        };
        let sc = uninit_sc.init(sc).into_ref();

        // name is a buffer with a copy of the nameunit so we can push/pop extra characters
        let mut name = device_get_nameunit(dev);
        for die in 0..sc.ndie {
            for irq in 0..sc.nirqs {
                // Turn the die and irq numbers into strings to append them to the name buffer.
                let mut len_pushed = name.push_c_str(c",d");
                len_pushed += name.push_c_str(die.to_array_cstring().as_c_str());
                len_pushed += name.push_c_str(c",i");
                len_pushed += name.push_c_str(irq.to_array_cstring().as_c_str());

                let isrc = proj!(&sc->irq_srcs[die][irq]);
                let isrc_flags = if sc.cfg.version == 2 {
                    //Some(INTR_ISRCF_BOUND)
                    None
                } else {
                    None
                };
                intr_isrc_register(isrc, sc.dev, isrc_flags, &name).map_err(|e| {
                    device_println!(dev, "unable to register irq {irq} for die {die} {e}");
                    ENXIO
                })?;

                name.pop(len_pushed);
            }
        }
        for fiq in 0..NUM_FIQS {
            let prefix = name.push_c_str(c",f");
            let fiq_len = name.push_c_str(fiq.to_array_cstring().as_c_str());

            let isrc = proj!(&sc->fiq_srcs[fiq]);
            intr_isrc_register(isrc, sc.dev, Some(INTR_ISRCF_PPI), &name).map_err(|e| {
                device_println!(dev, "unable to register fiq {:?} {e}", sc.fiq_srcs[fiq]);
                ENXIO
            })?;

            name.pop(prefix + fiq_len);
        }

        for ipi in 0..NUM_IPIS {
            let prefix = name.push_c_str(c",ipi");
            let ipi_len = name.push_c_str(ipi.to_array_cstring().as_c_str());

            let isrc = proj!(&sc->ipi_srcs[ipi]);
            intr_isrc_register(isrc, sc.dev, Some(INTR_ISRCF_IPI), &name).map_err(|e| {
                device_println!(dev, "unable to register ipi {ipi} {e}");
                ENXIO
            })?;

            name.pop(prefix + ipi_len);
        }

        let xref = OF_xref_from_node(ofw_bus_get_node(dev));
        intr_pic_register(dev, xref).map_err(|e| {
            device_println!(dev, "unable to register interrupt handler {e}");
            ENXIO
        })?;

        intr_pic_claim_root(dev, xref, apple_aic_irq, sc, INTR_ROOT_IRQ).map_err(|e| {
            device_println!(dev, "unable to set root interrupt controller {e}");
            //intr_pic_deregister(dev, xref);
            ENXIO
        })?;

        intr_pic_claim_root(dev, xref, apple_aic_fiq, sc, INTR_ROOT_FIQ).map_err(|e| {
            device_println!(dev, "unable to set root fiq controller {e}");
            //intr_pic_deregister(dev, xref);
            ENXIO
        })?;

        intr_ipi_pic_register(dev, 0).map_err(|e| {
            device_println!(dev, "could not register for IPIs {e}");
            ENXIO
        })?;

        OF_device_register_xref(xref, dev);

        device_println!(dev, "masking all {:?} irqs", sc.nirqs);
        for irq in 0..sc.nirqs {
            for die in 0..sc.ndie {
                apple_aic_mask_set(&sc, die, irq);
                apple_aic_sw_clear(&sc, die, irq);
                //if let Some(irq_cfg_offset) = sc.cfg.irq_cfg {
                //    let mut mem = sc.mem.get_mut();
                //    bus_write_4!(mem, irq_cfg_offset + (die as u64 * sc.cfg.die_stride) + (4 * irq as u64), 0);
                //}
            }
        }

        if sc.cfg.version == 2 {
            let mut mem = sc.mem.get_mut();
            let config = bus_read_4!(mem, AIC2_CONFIG);
            bus_write_4!(mem, AIC2_CONFIG, config | AIC2_CONFIG_ENABLE);
        }

        apple_aic_init_cpu();

        Ok(())
    }
}

fn apple_aic_init_cpu() {
    /* mask pending IPI FIQs */
    write_specialreg!(reg!(AIC_IPI_SR_EL1), AIC_IPI_SR_EL1_PENDING);

    /* mask timer FIQs */
    write_specialreg!(
        cntp_ctl_el0,
        read_specialreg!(cntp_ctl_el0) | CNTV_CTL_IMASK
    );
    write_specialreg!(
        cntv_ctl_el0,
        read_specialreg!(cntv_ctl_el0) | CNTV_CTL_IMASK
    );
    if in_vhe() {
        /* mask guest timer FIQs */
        write_specialreg!(
            reg!(AIC_FIQ_VM_TIMER),
            read_specialreg!(reg!(AIC_FIQ_VM_TIMER)) & !AIC_FIQ_VM_TIMER_BITS
        );
    }
    isb!();
}

extern "C" fn apple_aic_irq(sc: Ref<AppleIntSoftc>) -> Filter {
    let event = match &sc.event {
        Some(reg) => bus_read_4!(reg.get_mut(), 0),
        None => bus_read_4!(sc.mem.get_mut(), AIC_EVENT),
    };
    let ty = event_type(event);
    assert!(ty != AIC_EVENT_TYPE_IPI);
    assert!(ty != AIC_EVENT_TYPE_NONE);

    let die = event_die(event);
    let irq = event_irq(event);

    if die >= sc.ndie {
        panic!("unexpected die {die}");
    }
    if irq >= sc.nirqs {
        panic!("unexpected irq {irq}");
    }

    let tf = curthread!(td_intr_frame);

    if intr_isrc_dispatch(&sc.irq_srcs[die][irq], tf) != 0 {
        device_println!(sc.dev, "disabling stray irq {irq:?}");
        return FILTER_STRAY;
    }

    return FILTER_HANDLED;
}

extern "C" fn apple_aic_fiq(sc: Ref<AppleIntSoftc>) -> Filter {
    let tf = curthread!(td_intr_frame);

    // #ifdef SMP
    if read_specialreg!(reg!(AIC_IPI_SR_EL1)) & AIC_IPI_SR_EL1_PENDING != 0 {
        write_specialreg!(reg!(AIC_IPI_SR_EL1), AIC_IPI_SR_EL1_PENDING);
        isb!();
        //device_println!(sc.dev, "recv'd ipi");
        apple_aic_ipi_received(&sc, tf);
    }

    /*
     * FIQs don't store any state in the interrupt controller at all outside
     * of IPI handling, so we have to probe around outside of AIC to
     * determine if we might have been fired off due to a timer.
     */
    let reg = read_specialreg!(cntp_ctl_el0);
    if (reg & CNTV_CTL_BITS) == (CNTV_CTL_ENABLE | CNTV_CTL_ISTATUS) {
        //device_println!(sc.dev, "recv'd guest phys");
        intr_isrc_dispatch(&sc.fiq_srcs[AIC_TMR_HV_PHYS.0], tf);
    };

    let reg = read_specialreg!(cntv_ctl_el0);
    if (reg & CNTV_CTL_BITS) == (CNTV_CTL_ENABLE | CNTV_CTL_ISTATUS) {
        //device_println!(sc.dev, "recv'd guest virt");
        intr_isrc_dispatch(&sc.fiq_srcs[AIC_TMR_HV_VIRT.0], tf);
    };

    if in_vhe() {
        let reg = read_specialreg!(reg!(AIC_FIQ_VM_TIMER));

        if reg & AIC_FIQ_VM_TIMER_PEN != 0 {
            let reg = read_specialreg!(reg!(AIC_TMR_CTL_GUEST_PHYS));
            if (reg & CNTV_CTL_BITS) == (CNTV_CTL_ENABLE | CNTV_CTL_ISTATUS) {
                //device_println!(sc.dev, "recv'd hy phys");
                intr_isrc_dispatch(&sc.fiq_srcs[AIC_TMR_GUEST_PHYS.0], tf);
            }
        };
        if reg & AIC_FIQ_VM_TIMER_VEN != 0 {
            let reg = read_specialreg!(reg!(AIC_TMR_CTL_GUEST_VIRT));
            if (reg & CNTV_CTL_BITS) == (CNTV_CTL_ENABLE | CNTV_CTL_ISTATUS) {
                //device_println!(sc.dev, "recv'd hy virt");
                intr_isrc_dispatch(&sc.fiq_srcs[AIC_TMR_GUEST_VIRT.0], tf);
            }
        };
    }
    return FILTER_HANDLED;
}

fn apple_aic_ipi_received(sc: &AppleIntSoftc, tf: *mut trapframe) {
    let cpu = pcpu_get!(pc_cpuid) as usize;
    // TODO: is the rmb necessary? The C version doesn't have this
    //rmb!();
    let mut mask = atomic_readandclear_32(&sc.ipimasks[cpu]);

    while mask != 0 {
        let ipi = mask.trailing_zeros();
        mask &= !(1 << ipi);

        intr_ipi_dispatch(ipi);
    }
}

fn get_fdt_intr_data(dev: device_t, data: &MapData) -> Result<(AppleIntrKind, i32)> {
    match data {
        MapData::FDT(fdt_data) => {
            /*
             * The first cell is the interrupt type:
             *   0 = IRQ
             *   1 = FIQ
             * The optional next cell is the die ID
             * The next cell is the interrupt number
             * The last cell is the flags
             */
            let [ty, die, num, flags] = match fdt_data.cells() {
                [ty, num, flags] => [*ty, 0, *num, *flags],
                [ty, die, num, flags] => [*ty, *die, *num, *flags],
                cells => {
                    device_println!(
                        dev,
                        "devicetree has unexpected #interrupt-cells {}",
                        cells.len()
                    );
                    return Err(EINVAL);
                }
            };
            let die = die as usize;
            let num = num as usize;
            let intr = match ty {
                0 => AppleIntrKind::Irq { die, irq: num },
                1 => AppleIntrKind::Fiq(AppleFiqKind::all_fiqs()[num]),
                _ => {
                    return Err(EINVAL);
                }
            };
            Ok((intr, flags as i32))
        }
        _ => Err(ENOTSUP),
    }
}

impl PicIf for AppleIntDriver {
    type IrqSrcFields = AppleIrqSrcFields;

    fn pic_map_intr<'sc>(sc: &'sc Ref<AppleIntSoftc>, data: MapData) -> Result<&'sc AppleIrqSrc> {
        let (kind, _flags) = get_fdt_intr_data(sc.dev, &data)?;
        //device_println!(sc.dev, "called pic_map_intr for {kind:?}");

        match kind {
            AppleIntrKind::Irq { die, irq } => {
                Ok(&sc.irq_srcs[die][irq])
            }
            AppleIntrKind::Fiq(fiq) => {
                Ok(&sc.fiq_srcs[fiq.0])
            }
            AppleIntrKind::Ipi => {
                device_println!(sc.dev, "mapping IPIs not supported yet");
                Err(ENOTSUP)
            }
        }
    }

    fn pic_setup_intr(
        sc: Ref<AppleIntSoftc>,
        isrc: &AppleIrqSrc,
        res: Resource,
        data: MapData,
    ) -> Result<()> {
        let (kind, flags) = get_fdt_intr_data(sc.dev, &data)?;
        //device_println!(sc.dev, "called pic_setup_intr for {kind:?}");
        if isrc.kind != kind {
            device_println!(sc.dev, "pic_setup_intr called with mismatched isrc and FDT data");
            return Err(EINVAL);
        }
        if unsafe { base!(isrc->isrc_handlers) } != 0 {
            return Ok(());
        }
        // SAFETY: No other thread is concurrently modifying this field
        let isrc_flags = unsafe { base!(isrc->isrc_flags) };
        if isrc_flags & INTR_ISRCF_PPI.0 as u32 != 0 {
            let cpuid = pcpu_get!(pc_cpuid);
            CPU_SET(cpuid, base!(&isrc->isrc_cpu));
        }
        match sc.cfg.version {
            1 => {
                if let AppleIntrKind::Irq { die, irq } = isrc.kind {
                    assert!(isrc_flags == 0);
                    todo!("")
                    //aic_next_cpu = intr_irq_next_cpu(aic_next_cpu, &all_cpus);
                    //bus_write_4!(sc.mem AIC_
                }
            },
            2 => {
                if let AppleIntrKind::Irq { die, irq } = isrc.kind {
                    let cpuid = pcpu_get!(pc_cpuid);
                    //device_println!(sc.dev, "binding irq {irq:?} to cpuid {cpuid:?}");
                    CPU_SET(cpuid, base!(&isrc->isrc_cpu));
                }
            },
            other => panic!("unknown AIC version {other:?}"),
        }
        Ok(())
    }

    fn pic_enable_intr(sc: Ref<AppleIntSoftc>, isrc: &AppleIrqSrc) {
        //device_println!(sc.dev, "called pic_enable_intr for {:?}", isrc.kind);
        match isrc.kind {
            AppleIntrKind::Irq { die, irq } => {
                apple_aic_mask_clear(&sc, die, irq);
            }
            AppleIntrKind::Fiq(fiq) => {
                apple_aic_fiq_unmask(fiq);
            }
            AppleIntrKind::Ipi => (),
        }
    }

    fn pic_disable_intr(sc: Ref<AppleIntSoftc>, isrc: &AppleIrqSrc) {
        //device_println!(sc.dev, "called pic_disable_intr for {:?}", isrc.kind);
        match isrc.kind {
            AppleIntrKind::Irq { die, irq } => {
                apple_aic_mask_set(&sc, die, irq);
            }
            AppleIntrKind::Fiq(fiq) => {
                apple_aic_fiq_mask(fiq);
            }
            AppleIntrKind::Ipi => (),
        }
    }

    fn pic_post_filter(sc: Ref<AppleIntSoftc>, isrc: &AppleIrqSrc) {
        match isrc.kind {
            AppleIntrKind::Irq { die, irq } => {
                apple_aic_sw_clear(&sc, die, irq);
                apple_aic_mask_clear(&sc, die, irq);
            }
            AppleIntrKind::Fiq(fiq) => {
                apple_aic_fiq_unmask(fiq);
            }
            AppleIntrKind::Ipi => (),
        }
    }

    fn pic_pre_ithread(sc: Ref<AppleIntSoftc>, isrc: &AppleIrqSrc) {
        match isrc.kind {
            AppleIntrKind::Irq { die, irq } => {
                //device_println!(sc.dev, "called pic_pre_ithread for {:?}", isrc.kind);
                apple_aic_sw_clear(&sc, die, irq);
                // reading the event register automatically sets the irq mask
                Self::pic_disable_intr(sc, isrc);
            }
            bad_isrc => panic!("registered an ithread for non-IRQ interrupt {bad_isrc:?}"),
        }
    }

    fn pic_post_ithread(sc: Ref<AppleIntSoftc>, isrc: &AppleIrqSrc) {
        match isrc.kind {
            AppleIntrKind::Irq { die, irq } => {
                //device_println!(sc.dev, "called pic_post_ithread for {:?}", isrc.kind);
                apple_aic_sw_clear(&sc, die, irq);
                Self::pic_enable_intr(sc, isrc);
            }
            bad_isrc => panic!("registered an ithread for non-IRQ interrupt {bad_isrc:?}"),
        }
    }

    fn pic_bind_intr(sc: Ref<AppleIntSoftc>, isrc: &AppleIrqSrc) -> Result<()> {
        return Err(ENOTSUP);
    }

    fn pic_init_secondary(sc: Ref<AppleIntSoftc>, root: IntrRoot) {
        match root {
            INTR_ROOT_FIQ => {
                apple_aic_init_cpu();
            }
            INTR_ROOT_IRQ => {
                let cpuid = pcpu_get!(pc_cpuid);
                if sc.cfg.version == 2 {
                    //device_println!(sc.dev, "disabling IRQ delivery to CPU {cpuid:?}");
                    /*
                     * AIC2 doesn't provide a way to target external interrupt to a
                     * particular core so disable IRQ delivery to secondary CPUs
                     */
                    write_specialreg!(reg!(AIC_IRQ_CR_EL1), AIC_IRQ_CR_EL1_DISABLE);
                    return;
                }
                todo!("")
            }
            invalid_root => unreachable!(""),
        }
    }

    fn pic_ipi_setup<'sc>(sc: &'sc Ref<AppleIntSoftc>, ipi: u32) -> Result<&'sc AppleIrqSrc> {
        let ipi = ipi as usize;
        if ipi >= NUM_IPIS {
            panic!("ipi {ipi} too high");
        }
        let cpuid = pcpu_get!(pc_cpuid);
        let isrc = &sc.ipi_srcs[ipi];
        CPU_SET(cpuid, base!(&isrc->isrc_cpu));

        Ok(&sc.ipi_srcs[ipi])
    }

    fn pic_ipi_send(sc: Ref<AppleIntSoftc>, isrc: &AppleIrqSrc, cpus: cpuset_t, ipi: u32) {
        let localgrp = CPU_AFF1(CPU_AFFINITY(pcpu_get!(pc_cpuid)));
        for cpu in 0..=mp_maxid() as u_int {
            if CPU_ISSET(cpu, &cpus) {
                let aff = CPU_AFFINITY(cpu);
                let mut sendmask = CPU_AFF0(aff);
                atomic_set_32(&sc.ipimasks[cpu as usize], 1 << ipi);
                // TODO: C version uses dsb ishst instead of this dmb st
                //wmb!();
                unsafe {
                    asm!("dsb ishst");
                }

                if CPU_AFF1(aff) == localgrp {
                    write_specialreg!(reg!(AIC_IPI_LOCAL_RR_EL1), sendmask);
                } else {
                    sendmask |= CPU_AFF1(aff) << 16;
                    write_specialreg!(reg!(AIC_IPI_GLOBAL_RR_EL1), sendmask);
                }
                isb!();
            }
        }
    }
}
driver!(apple_aic_driver, c"aic", AppleIntDriver,
    apple_aic_methods = {
        /* device_t interface */
        device_probe apple_aic_probe,
        device_attach apple_aic_attach,

        /* Interrupt controller interface */
        pic_disable_intr apple_aic_disable_intr,
        pic_enable_intr apple_aic_enable_intr,
        pic_map_intr apple_aic_map_intr,
        pic_setup_intr apple_aic_setup_intr,
        pic_teardown_intr apple_aic_teardown_intr,
        pic_post_filter apple_aic_post_filter,
        pic_post_ithread apple_aic_post_ithread,
        pic_pre_ithread apple_aic_pre_ithread,

        /* TODO: #ifdef SMP */
        pic_bind_intr apple_aic_bind_intr,
        pic_init_secondary apple_aic_init_secondary,

        pic_ipi_setup apple_aic_ipi_setup,
        pic_ipi_send apple_aic_ipi_send,
    }
);
