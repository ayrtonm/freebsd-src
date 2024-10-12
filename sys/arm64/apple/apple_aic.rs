/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2021 Andrew Turner
 * All rights reserved.
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
#![feature(try_with_capacity, concat_idents, variant_count)]
#![deny(unused_must_use)]

extern crate alloc;
extern crate kpi;

use alloc::boxed::Box;
use alloc::vec::Vec;
use core::mem::{transmute, variant_count, MaybeUninit};
use core::ptr::{addr_of_mut, null_mut};
use core::pin::Pin;
use core::ops::{Index, IndexMut};
use kpi::arm64::in_vhe;
use kpi::bindings::{
    intr_irqsrc, intr_polarity, intr_trigger, FDT_INTR_EDGE_MASK, FDT_INTR_LEVEL_HIGH,
    INTR_IPI_COUNT, INTR_ISRCF_IPI, INTR_ISRCF_PPI, INTR_POLARITY_CONFORM, INTR_POLARITY_HIGH,
    INTR_POLARITY_LOW, INTR_TRIGGER_CONFORM, INTR_TRIGGER_EDGE, INTR_TRIGGER_LEVEL,
};
use kpi::bus::Resource;
use kpi::device::{Device, DeviceIf, ProbeRes};
use kpi::intr::{FilterRes, IrqSrc, MapData, Pic, PicIf};
use kpi::ofw::{CompatData, CompatEntry};
use kpi::{
    bindings, curthread, driver, enum_c_macros, isb, pcpu_get, read_reg, write_reg, PointsTo, Ptr,
    Ref, SharedValue, SubClass,
};

#[derive(Copy, Clone, Debug)]
struct Cfg {
    version: u32,
    sw_set: u32,
    sw_clear: u32,
    mask_set: u32,
    mask_clear: u32,
    die_stride: u32,
}

static AIC1: Cfg = Cfg {
    version: 1,
    sw_set: 0x4000,
    sw_clear: 0x4080,
    mask_set: 0x4100,
    mask_clear: 0x4180,
    die_stride: 0,
};

static AIC2: Cfg = Cfg {
    version: 2,
    sw_set: 0x6000,
    sw_clear: 0x6200,
    mask_set: 0x6400,
    mask_clear: 0x6600,
    die_stride: 0x4a00,
};

static COMPAT: CompatData<Cfg, 3> = CompatData::new(|| {
    [
        CompatEntry::new(c"apple,aic", &AIC1),
        CompatEntry::new(c"apple,aic2", &AIC2),
        CompatEntry::null(),
    ]
});

macro_rules! aic_reg {
    (AIC_IPI_SR_EL1) => {
        "s3_5_c15_c1_1"
    };
    (AIC_FIQ_VM_TIMER) => {
        "s3_5_c15_c1_3"
    };
    (AIC_IRQ_CR_EL1) => {
        "s3_4_c15_c10_4"
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
}

macro_rules! aic_read_reg {
    ($reg:ident) => {
        read_reg!(aic_reg!($reg))
    };
}
macro_rules! aic_write_reg {
    ($reg:ident, $value:expr) => {
        write_reg!(aic_reg!($reg), $value)
    };
}

const AIC_INFO: u64 = 0x0004;

fn info_ndie(info: u32) -> u32 {
    ((info >> 24) & 0xf) + 1
}

fn info_nirqs(info: u32) -> u32 {
    info & 0xffff
}

const AIC_WHOAMI: u64 = 0x2000;
const AIC_EVENT: u64 = 0x2004;
const AIC_EVENT_TYPE_NONE: u8 = 0;
const AIC_EVENT_TYPE_IRQ: u8 = 1;
const AIC_EVENT_TYPE_IPI: u8 = 4;
const AIC_EVENT_IPI_OTHER: u64 = 1;
const AIC_EVENT_IPI_SELF: u64 = 2;

const AIC_IPI_SR_EL1_PENDING: u64 = 1 << 0;
const AIC_FIQ_VM_TIMER_PEN: u64 = 1 << 1;
const AIC_FIQ_VM_TIMER_VEN: u64 = 1 << 0;
const AIC_FIQ_VM_TIMER_BITS: u64 = AIC_FIQ_VM_TIMER_VEN | AIC_FIQ_VM_TIMER_PEN;

const CNTV_CTL_ENABLE: u64 = 1 << 0;
const CNTV_CTL_IMASK: u64 = 1 << 1;
const CNTV_CTL_ISTATUS: u64 = 1 << 2;
const CNTV_CTL_BITS: u64 = CNTV_CTL_ENABLE | CNTV_CTL_IMASK | CNTV_CTL_ISTATUS;

const AIC2_CONFIG: u64 = 0x0014;
const AIC2_CONFIG_ENABLE: u32 = 1 << 0;
//const AIC2_EVENT: u64 = 0xc000;

const AIC_IRQ_CR_EL1_DISABLE: u32 = 3 << 0;

const AIC_MAXCPUS: u64 = 32;
const AIC_MAXDIES: usize = 4;

enum_c_macros! {
    #[repr(i32)]
    #[derive(Debug, Copy, Clone)]
    pub enum FiqKind {
        AIC_TMR_HV_PHYS,
        AIC_TMR_HV_VIRT,
        AIC_TMR_GUEST_PHYS,
        AIC_TMR_GUEST_VIRT,
        AIC_CPU_PMU_E,
        AIC_CPU_PMU_P,
    }
}

impl FiqKind {
    pub fn fiqs() -> [Self; NUM_FIQS] {
        [
            FiqKind::AIC_TMR_HV_PHYS,
            FiqKind::AIC_TMR_HV_VIRT,
            FiqKind::AIC_TMR_GUEST_PHYS,
            FiqKind::AIC_TMR_GUEST_VIRT,
            FiqKind::AIC_CPU_PMU_E,
            FiqKind::AIC_CPU_PMU_P,
        ]
    }
}


const NUM_FIQS: usize = variant_count::<FiqKind>();
const NUM_IPIS: usize = INTR_IPI_COUNT as usize;

#[derive(Debug, Copy, Clone)]
enum IntrKind {
    Irq { die: u32, irq: u32 },
    Fiq(FiqKind),
    Ipi,
}

type AppleIrqSrc = SubClass<IrqSrc, AppleIrqSrcFields>;

#[derive(Debug)]
struct AppleIrqSrcFields {
    kind: IntrKind,
    pol: intr_polarity,
    trig: intr_trigger,
}

impl AppleIrqSrcFields {
    pub const fn irq(die: u32, irq: u32) -> AppleIrqSrc {
        AppleIrqSrc::new(AppleIrqSrcFields {
            kind: IntrKind::Irq { die, irq },
            pol: INTR_POLARITY_CONFORM,
            trig: INTR_TRIGGER_CONFORM,
        })
    }

    pub const fn fiq(fiq: FiqKind) -> AppleIrqSrc {
        AppleIrqSrc::new(AppleIrqSrcFields {
            kind: IntrKind::Fiq(fiq),
            pol: INTR_POLARITY_CONFORM,
            trig: INTR_TRIGGER_CONFORM,
        })
    }

    pub const fn ipi() -> AppleIrqSrc {
        AppleIrqSrc::new(AppleIrqSrcFields {
            kind: IntrKind::Ipi,
            pol: INTR_POLARITY_CONFORM,
            trig: INTR_TRIGGER_CONFORM,
        })
    }
}

#[derive(Debug)]
struct Softc {
    dev: Device,
    mem: Resource,
    event: Option<Resource>,
    nirqs: u32,
    ndie: u32,
    irq_srcs: [Option<Box<[AppleIrqSrc]>>; AIC_MAXDIES],
    fiq_srcs: [AppleIrqSrc; NUM_FIQS],
    ipi_srcs: [AppleIrqSrc; NUM_IPIS],
    cfg: &'static Cfg,
}

fn get_irq_src(sc: &Ref<Softc>, die: u32, irq: u32) -> &AppleIrqSrc {
    /*unsafe { pinned_sc.map_unchecked(|sc|*/ &sc.irq_srcs[die as usize].as_ref().unwrap()[irq as usize]//) }
}

fn get_fiq_src(sc: &Ref<Softc>, fiq: FiqKind) -> &AppleIrqSrc {
    /*unsafe { pinned_sc.map_unchecked(|sc|*/ &sc.fiq_srcs[fiq as i32 as u32 as usize]//) }
}

fn irq_bitmask(irq: u32) -> u32 {
    let res = 1 << (irq & 0x1f);
    res as u32
}

fn irq_mask_set(sc: &Softc, die: u32, irq: u32) {
    let mut offset = sc.cfg.mask_set;
    offset += sc.cfg.die_stride * die;
    offset += (irq >> 5) * 4;

    sc.mem.write_4(offset as u64, irq_bitmask(irq))
}

fn irq_mask_clear(sc: Ref<Softc>, die: u32, irq: u32) {
    let mut offset = sc.cfg.mask_clear;
    offset += sc.cfg.die_stride * die;
    offset += (irq >> 5) * 4;

    sc.mem.write_4(offset as u64, irq_bitmask(irq))
}

fn irq_sw_set(sc: Ref<Softc>, die: u32, irq: u32) {
    let mut offset = sc.cfg.sw_set;
    offset += sc.cfg.die_stride * die;
    offset += (irq >> 5) * 4;

    sc.mem.write_4(offset as u64, irq_bitmask(irq))
}

fn irq_sw_clear(sc: Ref<Softc>, die: u32, irq: u32) {
    let mut offset = sc.cfg.sw_clear;
    offset += sc.cfg.die_stride * die;
    offset += (irq >> 5) * 4;

    sc.mem.write_4(offset as u64, irq_bitmask(irq))
}

fn fiq_unmask(fiq: FiqKind) {
    match fiq {
        FiqKind::AIC_TMR_GUEST_PHYS => {
            aic_write_reg!(
                AIC_FIQ_VM_TIMER,
                aic_read_reg!(AIC_FIQ_VM_TIMER) & !AIC_FIQ_VM_TIMER_PEN
            );
            isb!();
        }
        FiqKind::AIC_TMR_GUEST_VIRT => {
            aic_write_reg!(
                AIC_FIQ_VM_TIMER,
                aic_read_reg!(AIC_FIQ_VM_TIMER) & !AIC_FIQ_VM_TIMER_VEN
            );
            isb!();
        }
        _ => (),
    }
}

fn fiq_mask(fiq: FiqKind) {
    match fiq {
        FiqKind::AIC_TMR_GUEST_PHYS => {
            aic_write_reg!(
                AIC_FIQ_VM_TIMER,
                aic_read_reg!(AIC_FIQ_VM_TIMER) | AIC_FIQ_VM_TIMER_PEN
            );
            isb!();
        }
        FiqKind::AIC_TMR_GUEST_VIRT => {
            aic_write_reg!(
                AIC_FIQ_VM_TIMER,
                aic_read_reg!(AIC_FIQ_VM_TIMER) | AIC_FIQ_VM_TIMER_VEN
            );
            isb!();
        }
        _ => (),
    }
}

fn init_cpu() {
    /* mask pending IPI FIQs */
    aic_write_reg!(AIC_IPI_SR_EL1, AIC_IPI_SR_EL1_PENDING);

    write_reg!(
        stringify!(cntp_ctl_el0),
        read_reg!(stringify!(cntp_ctl_el0)) | CNTV_CTL_IMASK
    );
    write_reg!(
        stringify!(cntv_ctl_el0),
        read_reg!(stringify!(cntv_ctl_el0)) | CNTV_CTL_IMASK
    );

    if in_vhe() {
        aic_write_reg!(
            AIC_FIQ_VM_TIMER,
            aic_read_reg!(AIC_FIQ_VM_TIMER) & !AIC_FIQ_VM_TIMER_BITS
        );
    }
    isb!();
}

impl Driver {
    fn do_map_intr(&self, dev: Device, data: MapData) -> Result<AppleIrqSrcFields> {
        if let MapData::FDT(fdt) = data {
            /*
             * The first cell is the interrupt type:
             *   0 = IRQ
             *   1 = FIQ
             * The optional next cell is the die ID
             * The next cell is the interrupt number
             * The last cell is the flags
             */
            let (kind, die, num, flags) = match fdt.cells() {
                [k, n, f] => {
                    // bindgen generates i32 for macros so cast kind and flags to i32 for easy
                    // comparison later
                    let k = *k as i32;
                    let f = *f as i32;
                    (k, 0, *n, f)
                }
                [k, d, n, f] => {
                    let k = *k as i32;
                    let f = *f as i32;
                    (k, *d, *n, f)
                }
                _ => return Err(EINVAL),
            };
            let sc = self.share_softc(dev)?;
            if die >= sc.ndie {
                return Err(EINVAL);
            }
            let kind = match kind {
                bindings::AIC_IRQ => {
                    if num >= sc.nirqs {
                        return Err(EINVAL);
                    }
                    IntrKind::Irq { die, irq: num }
                }
                bindings::AIC_FIQ => {
                    if num as usize >= NUM_FIQS {
                        return Err(EINVAL);
                    };
                    IntrKind::Fiq(FiqKind::try_from(num as i32)?)
                }
                _ => return Err(EINVAL),
            };
            let pol = if (flags & FDT_INTR_LEVEL_HIGH) != 0 {
                INTR_POLARITY_HIGH
            } else {
                INTR_POLARITY_LOW
            };
            let trig = if (flags & FDT_INTR_EDGE_MASK) != 0 {
                INTR_TRIGGER_EDGE
            } else {
                INTR_TRIGGER_LEVEL
            };
            return Ok(AppleIrqSrcFields { kind, pol, trig });
        };
        Err(ENOTSUP)
    }
}

impl PicIf<AppleIrqSrcFields> for Driver {
    fn pic_setup_intr(
        &self,
        dev: Device,
        isrc: &mut AppleIrqSrc,
        res: Resource,
        data: MapData,
    ) -> Result<()> {
        isrc.sub = self.do_map_intr(dev, data)?;
        Ok(())
    }

    fn pic_disable_intr(&self, dev: Device, isrc: &AppleIrqSrc) {
        let sc = self.share_softc(dev).unwrap();
        match isrc.kind {
            IntrKind::Irq { die, irq } => irq_mask_set(&sc, die, irq),
            IntrKind::Fiq(fiq) => fiq_mask(fiq),
            IntrKind::Ipi => {}
        }
    }

    fn pic_enable_intr(&self, dev: Device, isrc: &AppleIrqSrc) {
        let sc = self.share_softc(dev).unwrap();
        match isrc.kind {
            IntrKind::Irq { die, irq } => irq_mask_clear(sc, die, irq),
            IntrKind::Fiq(fiq) => fiq_unmask(fiq),
            IntrKind::Ipi => {}
        }
    }
    fn pic_map_intr(&self, dev: Device, data: MapData, isrcp: &mut *mut IrqSrc) -> Result<()> {
        let aic_isrc = self.do_map_intr(dev, data)?;
        match aic_isrc.kind {
            IntrKind::Irq { die, irq } => {
                //let sc = self.share_softc(dev)?;
                //let irq_src = &sc.irq_srcs[die];
            }
            IntrKind::Fiq(fiq) => {}
            IntrKind::Ipi => {}
        }
        Ok(())
    }
    fn pic_post_filter(&self, dev: Device, isrc: &AppleIrqSrc) {
        let sc = self.share_softc(dev).unwrap();
        match isrc.kind {
            IntrKind::Irq { die, irq } => {
                irq_sw_clear(sc, die, irq);
                irq_mask_clear(sc, die, irq);
            },
            IntrKind::Fiq(fiq) => {
                fiq_unmask(fiq);
            },
            _ => (),
        }
    }
    fn pic_pre_ithread(&self, dev: Device, isrc: &AppleIrqSrc) {
        let sc = self.share_softc(dev).unwrap();
        match isrc.kind {
            IntrKind::Irq { die, irq } => {
                irq_sw_clear(sc, die, irq);
                self.pic_disable_intr(dev, isrc);
            },
            _ => panic!("registered an ithread for a non-IRQ interrupt"),
        }
    }
    fn pic_post_ithread(&self, dev: Device, isrc: &AppleIrqSrc) {
        let sc = self.share_softc(dev).unwrap();
        match isrc.kind {
            IntrKind::Irq { die, irq } => {
                self.pic_enable_intr(dev, isrc);
            },
            _ => panic!("registered an ithread for a non-IRQ interrupt"),
        }
    }
}

impl DeviceIf for Driver {
    fn device_probe(&self, dev: Device) -> Result<ProbeRes> {
        if !dev.ofw_bus_status_okay() {
            return Err(ENXIO);
        }

        if dev.ofw_bus_search_compatible(&*COMPAT.0).is_err() {
            return Err(ENXIO);
        }

        dev.set_desc(c"Apple Interrupt Controller");

        Ok(BUS_PROBE_SPECIFIC)
    }

    fn device_attach(&self, mut dev: Device) -> Result<()> {
        let cfg = dev.ofw_bus_search_compatible(&*COMPAT.0)?;

        let mem = dev.bus_alloc_resource(SYS_RES_MEMORY, 0)?;

        let event = if cfg.version == 2 {
            Some(dev.bus_alloc_resource(SYS_RES_MEMORY, 1)?)
        } else {
            None
        };

        let info = mem.read_4(AIC_INFO);
        let nirqs = info_nirqs(info);
        let ndie = info_ndie(info);

        let name = dev.get_nameunit();
        let mut irq_srcs = [const { None }; AIC_MAXDIES];

        for die in 0..ndie {
            let mut irqs = Vec::try_with_capacity(nirqs as usize)?;
            for irq in 0..nirqs {
                irqs.push(AppleIrqSrcFields::irq(die, irq));
            }
            irq_srcs[die as usize] = Some(irqs.into_boxed_slice());
        }
        let fiq_srcs = FiqKind::fiqs().map(|f| AppleIrqSrcFields::fiq(f));
        let ipi_srcs = [const { AppleIrqSrcFields::ipi() }; NUM_IPIS];

        let sc = Softc {
            dev,
            mem,
            event,
            nirqs,
            ndie,
            irq_srcs,
            fiq_srcs,
            ipi_srcs,
            cfg,
        };
        self.init_softc(dev, sc)?;
        let sc = self.share_softc(dev)?;

        // TODO: isrc_register doesn't force the isrc to have a fixed address yet
        for die in 0..ndie {
            for irq in 0..nirqs {
                self.isrc_register(dev, get_irq_src(&sc, die, irq), 0, c"%s,die%d,irq%d", name, die, irq)?;
            }
        }
        for fiq in FiqKind::fiqs() {
            self.isrc_register(dev, get_fiq_src(&sc, fiq), INTR_ISRCF_PPI, c"%s,fiq%d", name, fiq as i32 as u32, 0)?;
        }
        /*
        for die in 0..ndie {
            let mut irqs = Vec::try_with_capacity(nirqs as usize)?;
            for irq in 0..nirqs {
                irqs.push(new_irq(die, irq));
                self.isrc_register(dev, &irqs[irq], 0, c"irq")?;
            }
            irq_srcs[die] = Some(irqs.into_boxed_slice());

        }
        let fiq_srcs = fiqs();
        for fiq in *fiq_srcs {
            self.isrc_register(dev, fiq, INTR_ISRCF_PPI, c"fiq")?;
        }

        let ipi_srcs = [const { new_ipi() }; NUM_IPIS];
        */
        /*#ifdef SMP
            sc->sc_ipimasks = malloc(sizeof(*sc->sc_ipimasks) * mp_maxid + 1,
                M_DEVBUF, M_WAITOK | M_ZERO);
            if (sc->sc_cfg->version == 1) {
                sc->sc_cpuids = malloc(sizeof(*sc->sc_cpuids) * mp_maxid + 1,
                    M_DEVBUF, M_WAITOK | M_ZERO);
                cpu = PCPU_GET(cpuid);
                sc->sc_cpuids[cpu] = bus_read_4(sc->sc_mem, AIC_WHOAMI);
            }
        #endif*/

        //for ipi in &ipi_srcs {
        //    self.isrc_register(dev, ipi, INTR_ISRCF_IPI, c"ipi")?;
        //}

        dev.ipi_pic_register(0)?;

        let xref = dev.ofw_bus_get_node().xref_from_node();
        let mut pic = dev.pic_register(xref)?;

        dev.register_xref(xref);

        //let sc = self.share_softc(dev)?;

        pic.claim_root(irq_handler, sc, INTR_ROOT_IRQ)?;
        pic.claim_root(fiq_handler, sc, INTR_ROOT_IRQ)?;

        if cfg.version == 2 {
            let config = sc.mem.read_4(AIC2_CONFIG);
            sc.mem.write_4(AIC2_CONFIG, config | AIC2_CONFIG_ENABLE);
        }

        init_cpu();

        Ok(())
    }

    fn device_detach(&self, dev: Device) -> Result<()> {
        panic!("not yet")
    }
}

fn event_die(event: u32) -> u32 {
    (event >> 24) & 0xff
}

enum Event {
    Unused,
    Irq,
    Ipi,
}

fn event_type(event: u32) -> Option<Event> {
    let res = (event >> 16) & 0xff;
    match res {
        0 => Some(Event::Unused),
        1 => Some(Event::Irq),
        4 => Some(Event::Ipi),
        _ => None,
    }
}

fn event_irq(event: u32) -> u32 {
    event & 0xffff
}

extern "C" fn irq_handler(sc: Ref<Softc>) -> FilterRes {
    let tf = curthread!(td_intr_frame);

    let event = match &sc.event {
        Some(reg) => reg.read_4(0),
        None => sc.mem.read_4(AIC_EVENT),
    };
    match event_type(event) {
        Some(Event::Irq) => {
            let die = event_die(event);
            let irq = event_irq(event);

            let irq_src = get_irq_src(&sc, die, irq);
            if irq_src.isrc_dispatch(tf) != 0 {
                dprintln!(sc.dev, "Stray irq {irq:} disabled");
                return FILTER_STRAY;
            }
        }
        Some(Event::Ipi) => {
            panic!("received IPI in irq handler");
        }
        _ => return FILTER_STRAY,
    }
    FILTER_HANDLED
}

extern "C" fn fiq_handler(sc: Ref<Softc>) -> FilterRes {
    FILTER_HANDLED
}

driver!(apple_aic_driver, c"aic", apple_aic_methods, Softc,
    device_probe apple_aic_probe,
    device_attach apple_aic_attach,
    device_detach apple_aic_detach,

    pic_setup_intr apple_aic_setup_intr,
    pic_map_intr apple_aic_map_intr,
    pic_disable_intr apple_aic_disable_intr,
    pic_enable_intr apple_aic_enable_intr,

    pic_post_filter apple_aic_post_filter,
    pic_pre_ithread apple_aic_pre_ithread,
    pic_post_ithread apple_aic_post_ithread,
);
/*
    pic_if pic_teardown_intr apple_aic_teardown_intr,

#ifdef SMP
    DEVMETHOD(pic_bind_intr,	apple_aic_bind_intr),
    DEVMETHOD(pic_init_secondary,	apple_aic_init_secondary),
    DEVMETHOD(pic_ipi_send,		apple_aic_ipi_send),
    DEVMETHOD(pic_ipi_setup,	apple_aic_ipi_setup),
#endif
*/
