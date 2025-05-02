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
#![feature(macro_metavar_expr_concat)]

use core::mem::MaybeUninit;
use kpi::bindings::{intr_irqsrc, intr_polarity, intr_trigger};
use kpi::bus::{Register, Resource};
use kpi::cell::SubClass;
use kpi::cell::{Checked, ManagedPtr};
use kpi::device::{BusProbe, Device};
use kpi::driver;
use kpi::enum_c_macros;
use kpi::intr::{IntrRoot, IrqSrc, MapData};
use kpi::ofw::OfwCompatData;

type Box<T> = kpi::boxed::Box<T, M_DEVBUF>;
type Vec<T, M> = kpi::vec::Vec<T, M>;

const AIC_INFO: u64 = 0x0004;

const AIC_MAXDIES: usize = 4;
const NUM_FIQS: usize = 6;
const NUM_IPIS: usize = bindings::INTR_IPI_COUNT as usize;

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
    sw_set: u64,
    sw_clear: u64,
    mask_set: u64,
    mask_clear: u64,
    die_stride: u64,
}

static AIC_V1: AppleIntData = AppleIntData {
    version: 1,
    sw_set: 0x4000,
    sw_clear: 0x4080,
    mask_set: 0x4100,
    mask_clear: 0x4180,
    die_stride: 0,
};

static AIC_V2: AppleIntData = AppleIntData {
    version: 2,
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

fn apple_aic_fiq_unmask(sc: &AppleIntSoftc, fiq: AppleFiqKind) {
    match fiq {
        AppleFiqKind::AIC_TMR_GUEST_PHYS => {
            write_specialreg!(
                reg!(AIC_FIQ_VM_TIMER),
                read_specialreg!(reg!(AIC_FIQ_VM_TIMER)) & !AIC_FIQ_VM_TIMER_PEN
            );
            isb!();
        }
        AppleFiqKind::AIC_TMR_GUEST_VIRT => {
            write_specialreg!(
                reg!(AIC_FIQ_VM_TIMER),
                read_specialreg!(reg!(AIC_FIQ_VM_TIMER)) & !AIC_FIQ_VM_TIMER_VEN
            );
            isb!();
        }
        _ => { /* no mask bits for the hypervisor timers */ }
    }
}
fn apple_aic_fiq_mask(sc: &AppleIntSoftc, fiq: AppleFiqKind) {
    match fiq {
        AppleFiqKind::AIC_TMR_GUEST_PHYS => {
            write_specialreg!(
                reg!(AIC_FIQ_VM_TIMER),
                read_specialreg!(reg!(AIC_FIQ_VM_TIMER)) | AIC_FIQ_VM_TIMER_PEN
            );
            isb!();
        }
        AppleFiqKind::AIC_TMR_GUEST_VIRT => {
            write_specialreg!(
                reg!(AIC_FIQ_VM_TIMER),
                read_specialreg!(reg!(AIC_FIQ_VM_TIMER)) | AIC_FIQ_VM_TIMER_VEN
            );
            isb!();
        }
        _ => { /* no mask bits for the hypervisor timers */ }
    }
}

pub type AppleIrqSrc = IrqSrc<AppleIrqSrcFields>;

#[derive(Debug)]
pub struct AppleIrqSrcFields {
    kind: AppleIntrKind,
    pol: intr_polarity,
    trig: intr_trigger,
}

enum_c_macros! {
    #[repr(i32)]
    #[derive(Debug, Copy, Clone, PartialEq, Eq)]
    pub enum AppleFiqKind {
        AIC_TMR_HV_PHYS,
        AIC_TMR_HV_VIRT,
        AIC_TMR_GUEST_PHYS,
        AIC_TMR_GUEST_VIRT,
        AIC_CPU_PMU_E,
        AIC_CPU_PMU_P,
    }
}

impl AppleFiqKind {
    fn all_fiqs() -> [AppleFiqKind; NUM_FIQS] {
        [
            Self::AIC_TMR_HV_PHYS,
            Self::AIC_TMR_HV_VIRT,
            Self::AIC_TMR_GUEST_PHYS,
            Self::AIC_TMR_GUEST_VIRT,
            Self::AIC_CPU_PMU_E,
            Self::AIC_CPU_PMU_P,
        ]
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
enum AppleIntrKind {
    Irq { die: usize, irq: usize },
    Fiq(AppleFiqKind),
    Ipi,
}

pub const fn new_irq_src(kind: AppleIntrKind) -> AppleIrqSrc {
    AppleIrqSrc::new(AppleIrqSrcFields {
        kind,
        pol: bindings::INTR_POLARITY_CONFORM,
        trig: bindings::INTR_TRIGGER_CONFORM,
    })
}

#[derive(Debug)]
pub struct AppleIntSoftc {
    dev: Device,
    cfg: &'static AppleIntData,
    mem: Checked<Register>,
    event: Option<Checked<Register>>,
    irq_srcs: Box<[Box<[AppleIrqSrc]>]>,
    fiq_srcs: [AppleIrqSrc; NUM_FIQS],
    ipi_srcs: [AppleIrqSrc; NUM_IPIS],
    ndie: usize,
    nirqs: usize,
}

impl DeviceIf for AppleIntDriver {
    type Softc = AppleIntSoftc;

    fn device_probe(dev: Device) -> Result<BusProbe> {
        if !ofw_bus_status_okay(dev) {
            return Err(ENXIO);
        }
        if ofw_bus_search_compatible(dev, &COMPAT_DATA).is_err() {
            return Err(ENXIO);
        }
        device_set_desc(dev, c"Apple Interrupt Controller");
        return Ok(BUS_PROBE_DEFAULT);
    }

    fn device_attach(dev: Device) -> Result<()> {
        // Cannot fail since it must have been called successfully in device_probe
        let cfg = ofw_bus_search_compatible(dev, &COMPAT_DATA).unwrap();

        let mem_res = bus_alloc_resource(dev, SYS_RES_MEMORY, 0, RF_ACTIVE).map_err(|e| {
            device_println!(dev, "unable to allocate memory register {e}");
            return ENXIO;
        })?;
        // SYS_RES_MEMORY was specified above so this shouldn't fail
        let mut mem = Register::new(mem_res)?;
        let mut event = None;
        if cfg.version == 2 {
            let event_res = bus_alloc_resource(dev, SYS_RES_MEMORY, 1, RF_ACTIVE).map_err(|e| {
                device_println!(dev, "unable to allocate event register {e}");
                return ENXIO;
            })?;
            event = Some(Checked::new(Register::new(event_res)?));
        };
        let info = bus_read_4!(mem, AIC_INFO);
        let nirqs = info_nirqs(info);
        let ndie = info_ndie(info) + 1;
        device_println!(dev, "Found {nirqs} interrupts, {ndie} die");

        // Create an empty Vec with space for up to `ndie` elements
        let mut ndie_vec = Vec::try_with_capacity(ndie, M_WAITOK | M_ZERO).map_err(|e| {
            device_println!(dev, "failed to allocate memory for irqs {e}");
            return ENXIO;
        })?;

        // Start populating `ndie_vec`
        for die in 0..ndie {
            // Create an empty Vec with space for up to `nirqs` elements
            let mut nirqs_vec = Vec::try_with_capacity(nirqs, M_WAITOK | M_ZERO).map_err(|e| {
                device_println!(dev, "failed to allocate memory for irqs {e}");
                return ENXIO;
            })?;
            // Populate `nirqs_vec`
            for irq in 0..nirqs {
                let isrc = new_irq_src(AppleIntrKind::Irq { die, irq });
                // Cannot fail since we preallocated `nirqs` entries
                nirqs_vec.push(isrc);
            }
            // `nirqs_vec`'s size won't change so turn it into a slice. into_boxed_slice() drops excess capacity which is why this is called after `nirqs_vec` is populated.
            let nirqs_slice = nirqs_vec.into_boxed_slice();
            // Populate this die's entry in `ndie_vec`. This cannot fail since we preallocated `ndie` entries
            ndie_vec.push(nirqs_slice);
        }
        // `ndie_vec`'s size won't change so turn it into a slice
        let irq_srcs = ndie_vec.into_boxed_slice();
        let fiq_srcs = AppleFiqKind::all_fiqs().map(|fiq| new_irq_src(AppleIntrKind::Fiq(fiq)));
        let ipi_srcs = [const { new_irq_src(AppleIntrKind::Ipi) }; NUM_IPIS];

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
        };
        let sc = device_init_softc!(dev, sc);
        let name = device_get_nameunit(dev);
        for die in 0..ndie {
            for irq in 0..nirqs {
                let isrc = project!(sc->irq_srcs[die][irq]);
                if let Err(e) = intr_isrc_register(
                    isrc,
                    sc.dev,
                    None,
                    c"%s,d%u,i%u",
                    name,
                    die as u32,
                    irq as u32,
                ) {
                    device_println!(dev, "unable to register irq {irq} for die {die} {e}");
                    return Err(ENXIO);
                };
            }
        }
        for fiq in 0..NUM_FIQS {
            let isrc = project!(sc->fiq_srcs[fiq]);
            if let Err(e) = intr_isrc_register(
                isrc,
                sc.dev,
                Some(INTR_ISRCF_PPI),
                c"%s,f%u",
                name,
                fiq as u32,
                0,
            ) {
                device_println!(dev, "unable to register fiq {:?} {e}", sc.fiq_srcs[fiq]);
                return Err(ENXIO);
            }
        }

        if sc.cfg.version == 1 {
            todo!("")
        }
        for ipi in 0..NUM_IPIS {
            let isrc = project!(sc->ipi_srcs[ipi]);
            if let Err(e) = intr_isrc_register(
                isrc,
                sc.dev,
                Some(INTR_ISRCF_IPI),
                c"%s,ipi%u",
                name,
                ipi as u32,
                0,
            ) {
                device_println!(dev, "unable to register ipi {ipi} {e}");
                return Err(ENXIO);
            }
        }

        let xref = OF_xref_from_node(ofw_bus_get_node(dev));
        if let Err(e) = intr_pic_register(dev, xref) {
            device_println!(dev, "unable to register interrupt handler {e}");
            return Err(ENXIO);
        };

        if let Err(e) = intr_pic_claim_root(dev, xref, Self::apple_aic_irq, &sc, INTR_ROOT_IRQ) {
            device_println!(dev, "unable to set root interrupt controller {e}");
            //intr_pic_deregister(dev, xref);
            return Err(ENXIO);
        }

        if let Err(e) = intr_pic_claim_root(dev, xref, Self::apple_aic_fiq, &sc, INTR_ROOT_FIQ) {
            device_println!(dev, "unable to set root fiq controller {e}");
            //intr_pic_deregister(dev, xref);
            return Err(ENXIO);
        }

        if let Err(e) = intr_ipi_pic_register(dev, 0) {
            device_println!(dev, "could not register for IPIs {e}");
            return Err(ENXIO);
        }

        OF_device_register_xref(xref, dev);

        if sc.cfg.version == 2 {
            const AIC2_CONFIG: u64 = 0x0014;
            const AIC2_CONFIG_ENABLE: u32 = 1 << 0;
            let mut mem = sc.mem.get_mut();
            let config = bus_read_4!(mem, AIC2_CONFIG);
            bus_write_4!(mem, AIC2_CONFIG, config | AIC2_CONFIG_ENABLE);
        }

        apple_aic_init_cpu();

        Ok(())
    }
}

const CNTV_CTL_ENABLE: u64 = 1 << 0;
const CNTV_CTL_IMASK: u64 = 1 << 1;
const CNTV_CTL_ISTATUS: u64 = 1 << 2;
const CNTV_CTL_BITS: u64 = CNTV_CTL_ENABLE | CNTV_CTL_IMASK | CNTV_CTL_ISTATUS;

const AIC_IPI_SR_EL1_PENDING: u64 = 1 << 0;
const AIC_FIQ_VM_TIMER_VEN: u64 = 1 << 0;
const AIC_FIQ_VM_TIMER_PEN: u64 = 1 << 1;
const AIC_FIQ_VM_TIMER_BITS: u64 = AIC_FIQ_VM_TIMER_VEN | AIC_FIQ_VM_TIMER_PEN;

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

impl AppleIntDriver {
    extern "C" fn apple_aic_irq(sc: &AppleIntSoftc) -> i32 {
        const AIC_EVENT: u64 = 0x2004;
        let event = match &sc.event {
            Some(reg) => bus_read_4!(reg.get_mut(), 0),
            None => bus_read_4!(sc.mem.get_mut(), AIC_EVENT),
        };
        let ty = event_type(event);
        const AIC_EVENT_TYPE_NONE: u32 = 0;
        const AIC_EVENT_TYPE_IRQ: u32 = 1;
        const AIC_EVENT_TYPE_IPI: u32 = 4;
        assert!(ty != AIC_EVENT_TYPE_IPI);

        let die = event_die(event);
        let irq = event_irq(event);

        if die >= sc.ndie {
            panic!("unexpected die {die}");
        }
        if irq >= sc.nirqs {
            panic!("unexpected irq {irq}");
        }

        let tf = curthread!(td_intr_frame);

        let aisrc = &sc.irq_srcs[die][irq];
        if intr_isrc_dispatch(aisrc, tf) != 0 {
            return -1;
        }

        0
    }
    extern "C" fn apple_aic_fiq(sc: &AppleIntSoftc) -> i32 {
        0
    }
}

fn get_fdt_intr_data(dev: Device, data: &MapData) -> Result<(AppleIntrKind, i32)> {
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

    fn pic_setup_intr(
        dev: Device,
        isrc: &mut AppleIrqSrc,
        res: Resource,
        data: MapData,
    ) -> Result<()> {
        let (kind, flags) = get_fdt_intr_data(dev, &data)?;
        let sc = device_get_softc!(dev);
        if isrc.kind != kind {
            device_println!(
                dev,
                "input irq doesn't match dt {kind:?} vs {:?}",
                isrc.kind
            );
            return Err(EINVAL);
        }
        let base_isrc = unsafe { SubClass::get_base_ref(isrc) };
        if base_isrc.isrc_flags & bindings::INTR_ISRCF_PPI as u32 != 0 {
            let cpuid = pcpu_get!(pc_cpuid);
            // CPU_SET(cpuid, &isrc->isrc_cpu);
        }
        if sc.cfg.version == 1 {
            if let AppleIntrKind::Irq { die, irq } = isrc.kind {
                assert!(base_isrc.isrc_flags == 0);
                todo!("")
                //aic_next_cpu = intr_irq_next_cpu(aic_next_cpu, &all_cpus);
                //bus_write_4!(sc.mem AIC_
            }
        }
        Ok(())
    }

    fn pic_map_intr(dev: Device, data: MapData, isrcp: *mut *mut intr_irqsrc) -> Result<()> {
        let (kind, _flags) = get_fdt_intr_data(dev, &data)?;

        let sc = device_get_softc!(dev);
        match kind {
            AppleIntrKind::Irq { die, irq } => unsafe {
                *isrcp = SubClass::get_base_ptr(&sc.irq_srcs[die][irq]);
            },
            AppleIntrKind::Fiq(fiq) => unsafe {
                *isrcp = SubClass::get_base_ptr(&sc.fiq_srcs[fiq as usize]);
            },
            AppleIntrKind::Ipi => (),
        }
        Ok(())
    }

    fn pic_enable_intr(dev: Device, isrc: &mut AppleIrqSrc) {
        let sc = device_get_softc!(dev);
        match isrc.kind {
            AppleIntrKind::Irq { die, irq } => {
                apple_aic_mask_clear(&sc, die, irq);
            }
            AppleIntrKind::Fiq(fiq) => {
                apple_aic_fiq_unmask(&sc, fiq);
            }
            AppleIntrKind::Ipi => (),
        }
    }

    fn pic_disable_intr(dev: Device, isrc: &mut AppleIrqSrc) {
        let sc = device_get_softc!(dev);
        match isrc.kind {
            AppleIntrKind::Irq { die, irq } => {
                apple_aic_mask_set(&sc, die, irq);
            }
            AppleIntrKind::Fiq(fiq) => {
                apple_aic_fiq_mask(&sc, fiq);
            }
            AppleIntrKind::Ipi => (),
        }
    }

    fn pic_teardown_intr(
        dev: Device,
        isrc: &mut AppleIrqSrc,
        res: Resource,
        data: MapData,
    ) -> Result<()> {
        todo!("")
    }

    fn pic_post_filter(dev: Device, isrc: &mut AppleIrqSrc) {
        let sc = device_get_softc!(dev);
        match isrc.kind {
            AppleIntrKind::Irq { die, irq } => {
                apple_aic_sw_clear(&sc, die, irq);
                apple_aic_mask_clear(&sc, die, irq);
            }
            AppleIntrKind::Fiq(fiq) => {
                apple_aic_fiq_unmask(&sc, fiq);
            }
            AppleIntrKind::Ipi => (),
        }
    }

    fn pic_pre_ithread(dev: Device, isrc: &mut AppleIrqSrc) {
        let sc = device_get_softc!(dev);
        match isrc.kind {
            AppleIntrKind::Irq { die, irq } => {
                apple_aic_sw_clear(&sc, die, irq);
                // reading the event register automatically sets the irq mask
                Self::pic_disable_intr(dev, isrc);
            }
            bad_isrc => panic!("registered an ithread for non-IRQ interrupt {bad_isrc:?}"),
        }
    }

    fn pic_post_ithread(dev: Device, isrc: &mut AppleIrqSrc) {
        let sc = device_get_softc!(dev);
        match isrc.kind {
            AppleIntrKind::Irq { die, irq } => {
                apple_aic_sw_clear(&sc, die, irq);
                Self::pic_enable_intr(dev, isrc);
            }
            bad_isrc => panic!("registered an ithread for non-IRQ interrupt {bad_isrc:?}"),
        }
    }

    fn pic_bind_intr(dev: Device, isrc: &mut AppleIrqSrc) -> Result<()> {
        return Err(ENOTSUP);
    }

    fn pic_init_secondary(dev: Device, root: IntrRoot) {
        match root {
            INTR_ROOT_FIQ => {
                apple_aic_init_cpu();
            }
            INTR_ROOT_IRQ => {
                let sc = device_get_softc!(dev);
                if sc.cfg.version == 2 {
                    return;
                }
                let cpuid = pcpu_get!(pc_cpuid);
                todo!("")
            }
        }
    }
    fn pic_ipi_setup(dev: Device, ipi: u32, isrcp: *mut *mut intr_irqsrc) -> Result<()> {
        let sc = device_get_softc!(dev);
        let ipi = ipi as usize;
        if ipi >= NUM_IPIS {
            panic!("ipi {ipi} too high");
        }
        let cpuid = pcpu_get!(pc_cpuid);
        // CPU_SET(cpuid, ...
        unsafe {
            *isrcp = SubClass::get_base_ptr(&sc.ipi_srcs[ipi]);
        }

        Ok(())
    }
}
driver!(apple_aic_driver, c"aic", AppleIntDriver, apple_aic_methods,
    INTERFACES {
        /* Device interface */
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
        /*
        pic_ipi_send apple_aic_ipi_send,
        */
    }
);
