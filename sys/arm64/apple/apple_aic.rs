#![no_std]
#![feature(try_with_capacity, concat_idents, variant_count)]
#![deny(unused_must_use)]

extern crate alloc;
extern crate kpi;

use alloc::boxed::Box;
use alloc::vec::Vec;
use core::mem::{transmute, variant_count, MaybeUninit};
use core::ptr::{addr_of_mut, null_mut};
use kpi::arm64::in_vhe;
use kpi::bus::Resource;
use kpi::device::{Device, DeviceIf, ProbeRes};
use kpi::intr::{FilterRes, IrqSrc, Pic, PicIf};
use kpi::ofw::{CompatData, CompatEntry};
use kpi::{bindings, curthread, driver, isb, pcpu_get, read_reg, write_reg, PointsTo, Ref};

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

fn info_ndie(info: u32) -> usize {
    let res = ((info >> 24) & 0xf) + 1;
    res as usize
}

fn info_nirqs(info: u32) -> usize {
    (info & 0xffff) as usize
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

static COMPAT: CompatData<Cfg, 3> = CompatData::new(|| {
    [
        CompatEntry::new(c"apple,aic", &AIC1),
        CompatEntry::new(c"apple,aic2", &AIC2),
        CompatEntry::null(),
    ]
});

#[derive(Debug)]
enum FiqKind {
    TmrHvPhys,
    TmrHvVirt,
    TmrGuestPhys,
    TmrGuestVirt,
    CpuPmuE,
    CpuPmuU,
}

const NUM_FIQS: usize = variant_count::<FiqKind>();
const NUM_IPIS: usize = bindings::INTR_IPI_COUNT as usize;

#[derive(Debug)]
enum IntrKind {
    Irq { die: usize, irq: usize },
    Fiq(FiqKind),
    Ipi,
}

#[derive(Debug)]
struct Intr {
    isrc: IrqSrc,
    kind: IntrKind,
    pol: bindings::intr_polarity,
    trig: bindings::intr_trigger,
}

impl Intr {
    pub fn irq(die: usize, irq: usize) -> Self {
        Self {
            isrc: IrqSrc::new(),
            kind: IntrKind::Irq { die, irq },
            pol: bindings::INTR_POLARITY_CONFORM,
            trig: bindings::INTR_TRIGGER_CONFORM,
        }
    }

    pub fn fiq(kind: FiqKind) -> Self {
        Self {
            isrc: IrqSrc::new(),
            kind: IntrKind::Fiq(kind),
            pol: bindings::INTR_POLARITY_CONFORM,
            trig: bindings::INTR_TRIGGER_CONFORM,
        }
    }

    pub fn fiqs() -> [Self; NUM_FIQS] {
        let kinds = [
            FiqKind::TmrHvPhys,
            FiqKind::TmrHvVirt,
            FiqKind::TmrGuestPhys,
            FiqKind::TmrGuestVirt,
            FiqKind::CpuPmuE,
            FiqKind::CpuPmuU,
        ];
        kinds.map(|k| Self::fiq(k))
    }

    pub const fn ipi() -> Self {
        Self {
            isrc: IrqSrc::new(),
            kind: IntrKind::Ipi,
            pol: bindings::INTR_POLARITY_CONFORM,
            trig: bindings::INTR_TRIGGER_CONFORM,
        }
    }
}

#[derive(Debug)]
struct Softc {
    dev: Device,
    mem: Resource,
    event: Option<Resource>,
    nirqs: usize,
    ndie: usize,
    irq_srcs: [Option<Box<[Intr]>>; AIC_MAXDIES],
    fiq_srcs: [Option<[Intr; NUM_FIQS]>; AIC_MAXDIES],
    ipi_srcs: [Intr; NUM_IPIS],
    cfg: &'static Cfg,
}

//extern "C" fn irq_handler(sc: &mut Softc) -> FilterRes {
//    let tf = curthread!(td_intr_frame);
//    #[repr(C)]
//    struct Event {
//        irq: u16,
//        ty: u8,
//        die: u8,
//    }
//
//    let event = if sc.cfg.version == 1 {
//        sc.mem.read_4(AIC_EVENT)
//    } else {
//        sc.event.read_4(0)
//    };
//
//    let event: Event = unsafe { transmute(event) };
//
//    let ty = event.ty;
//
//    if ty != AIC_EVENT_TYPE_IRQ {
//        if ty != AIC_EVENT_TYPE_NONE {}
//    }
//    //    // get curthread td_intr_frame from x18 PCPU
//    //    let tf = unsafe { (*curthread!()).td_intr_frame };
//    //    #[repr(C)]
//    //    struct Event {
//    //        irq: u16,
//    //        ty: u8,
//    //        die: u8,
//    //    }
//    //    let event = if sc.cfg.version == 1 {
//    //        sc.mem.read_4(bindings::AIC_EVENT as u64)
//    //    } else {
//    //        sc.event.read_4(0)
//    //    };
//    //    // TODO: there's probably a cleaner way to do this...
//    //    let event: Event = unsafe { transmute(event) };
//    //
//    //    let ty = event.ty;
//    //
//    //    if ty != bindings::AIC_EVENT_TYPE_IRQ as u8 {
//    //        if ty != bindings::AIC_EVENT_TYPE_NONE as u8 {
//    //            dprintln!(sc.dev, "unexpected event type {ty}");
//    //            return FILTER_STRAY;
//    //        }
//    //    }
//    //
//    //    let die = event.die as usize;
//    //    let irq = event.irq as usize;
//    //    let idx = (irq + (die * sc.nirqs as usize));
//    //    if Pic::isrc_dispatch(&mut sc.irq_srcs[idx].isrc, tf).is_err() {
//    //        dprintln!(sc.dev, "Stray irq {die}:{irq} disabled");
//    //        return FILTER_STRAY;
//    //    }
//    //
//    FILTER_HANDLED
//}

//extern "C" fn fiq_handler(sc: &mut Softc) -> FilterRes {
//    FILTER_STRAY
//}

fn irq_bitmask(irq: u32) -> u32 {
    1 << (irq & 0x1f)
}

/*
fn irq_mask_set(sc: &mut Softc, die: u32, irq: u32) {
    let mut offset = sc.cfg.mask_set;
    offset += sc.cfg.die_stride * die;
    offset += (irq >> 5) * 4;

    sc.mem.write_4(offset as u64, irq_bitmask(irq))
}

fn irq_mask_clear(sc: &mut Softc, die: u32, irq: u32) {
    let mut offset = sc.cfg.mask_clear;
    offset += sc.cfg.die_stride * die;
    offset += (irq >> 5) * 4;

    sc.mem.write_4(offset as u64, irq_bitmask(irq))
}

fn irq_sw_set(sc: &mut Softc, die: u32, irq: u32) {
    let mut offset = sc.cfg.sw_set;
    offset += sc.cfg.die_stride * die;
    offset += (irq >> 5) * 4;

    sc.mem.write_4(offset as u64, irq_bitmask(irq))
}

fn irq_sw_clear(sc: &mut Softc, die: u32, irq: u32) {
    let mut offset = sc.cfg.sw_clear;
    offset += sc.cfg.die_stride * die;
    offset += (irq >> 5) * 4;

    sc.mem.write_4(offset as u64, irq_bitmask(irq))
}
*/

fn fiq_unmask(fiq: i32) {
    match fiq {
        bindings::AIC_TMR_GUEST_PHYS => {
            aic_write_reg!(
                AIC_FIQ_VM_TIMER,
                aic_read_reg!(AIC_FIQ_VM_TIMER) & !AIC_FIQ_VM_TIMER_PEN
            );
            isb!();
        }
        bindings::AIC_TMR_GUEST_VIRT => {
            aic_write_reg!(
                AIC_FIQ_VM_TIMER,
                aic_read_reg!(AIC_FIQ_VM_TIMER) & !AIC_FIQ_VM_TIMER_VEN
            );
            isb!();
        }
        _ => (),
    }
}

fn fiq_mask(fiq: i32) {
    match fiq {
        bindings::AIC_TMR_GUEST_PHYS => {
            aic_write_reg!(
                AIC_FIQ_VM_TIMER,
                aic_read_reg!(AIC_FIQ_VM_TIMER) | AIC_FIQ_VM_TIMER_PEN
            );
            isb!();
        }
        bindings::AIC_TMR_GUEST_VIRT => {
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

impl PicIf for Driver {
    fn pic_disable_intr(&self, dev: Device, isrc: IrqSrc) {}
    fn pic_enable_intr(&self, dev: Device, isrc: IrqSrc) {}
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

        let mem = dev
            .bus_alloc_resource(SYS_RES_MEMORY, 0)
            .inspect_err(|e| dprintln!(dev, "could not allocate 'core' register {e:?}"))?;

        let event = if cfg.version == 2 {
            let reg = dev
                .bus_alloc_resource(SYS_RES_MEMORY, 1)
                .inspect_err(|e| dprintln!(dev, "coult not allocate 'event' register {e:?}"))?;
            Some(reg)
        } else {
            None
        };

        let info = mem.read_4(AIC_INFO);
        let nirqs = info_nirqs(info);
        let ndie = info_ndie(info);

        let name = dev.get_nameunit();
        let mut irq_srcs = [const { None }; AIC_MAXDIES];
        let mut fiq_srcs = [const { None }; AIC_MAXDIES];

        for die in 0..ndie {
            let mut irqs: Vec<Intr> = Vec::try_with_capacity(nirqs)?;
            for irq in 0..nirqs {
                irqs.push(Intr::irq(die, irq));

                let isrc_ptr = addr_of_mut!(irqs[irq].isrc);
                let rc = unsafe {
                    bindings::intr_isrc_register(
                        isrc_ptr,
                        dev.as_ptr(),
                        0,
                        c"%s,die%d,irq%d".as_ptr(),
                        name.as_ptr(),
                        die,
                        irq,
                    )
                };
                if rc != 0 {
                    return Err(ENXIO);
                }
            }
            irq_srcs[die] = Some(irqs.into_boxed_slice());

            let mut fiqs = Intr::fiqs();
            for fiq in &mut fiqs {
                // TODO: Not sure I can expose fiq.isrc's temporary address to C but whatever
                let isrc_ptr = addr_of_mut!(fiq.isrc);
                let rc = unsafe {
                    bindings::intr_isrc_register(
                        isrc_ptr,
                        dev.as_ptr(),
                        bindings::INTR_ISRCF_PPI as u32,
                        c"%s,die%d,fiq".as_ptr(),
                        name.as_ptr(),
                        die,
                    )
                };
                if rc != 0 {
                    return Err(ENXIO);
                }
            }
            fiq_srcs[die] = Some(fiqs);
        }

        let ipi_srcs = [const { Intr::ipi() }; NUM_IPIS];

        let rc = unsafe { bindings::intr_ipi_pic_register(dev.as_ptr(), 0) };
        if rc != 0 {
            return Err(ENXIO);
        }
        /*#ifdef SMP
            sc->sc_ipimasks = malloc(sizeof(*sc->sc_ipimasks) * mp_maxid + 1,
                M_DEVBUF, M_WAITOK | M_ZERO);
            if (sc->sc_cfg->version == 1) {
                sc->sc_cpuids = malloc(sizeof(*sc->sc_cpuids) * mp_maxid + 1,
                    M_DEVBUF, M_WAITOK | M_ZERO);
                cpu = PCPU_GET(cpuid);
                sc->sc_cpuids[cpu] = bus_read_4(sc->sc_mem, AIC_WHOAMI);
            }
            for (ipi = 0; ipi < AIC_NIPIS; ipi++) {
                isrc->ai_irq = ipi;
                isrc = &sc->sc_ipi_srcs[ipi];
                isrc->ai_type = AIC_TYPE_IPI;
                error = intr_isrc_register(&isrc->ai_isrc, dev, INTR_ISRCF_IPI,
                    "%s,ipi%d", name, ipi);
                if (error != 0) {
                    return (ENXIO);
                }
            }
            error = intr_ipi_pic_register(dev, 0);
            if (error != 0) {
                return (ENXIO);
            }
        #endif*/

        let xref = dev.ofw_bus_get_node().xref_from_node();
        let mut pic = dev.pic_register(xref)?;

        dev.register_xref(xref);

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

        self.softc_init(dev, sc)?;
        let sc = self.softc_share(dev)?;

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

extern "C" fn irq_handler(sc: Ref<Softc>) -> FilterRes {
    FILTER_HANDLED
}

extern "C" fn fiq_handler(sc: Ref<Softc>) -> FilterRes {
    FILTER_HANDLED
}

driver!(apple_aic_driver, c"aic", apple_aic_methods, Softc,
    device_if device_probe apple_aic_probe,
    device_if device_attach apple_aic_attach,
    device_if device_detach apple_aic_detach,

    pic_if pic_disable_intr apple_aic_disable_intr,
    pic_if pic_enable_intr apple_aic_enable_intr,

);
/*
    pic_if pic_map_intr apple_aic_map_intr,
    pic_if pic_setup_intr apple_aic_setup_intr,
    pic_if pic_teardown_intr apple_aic_teardown_intr,
    pic_if pic_post_filter apple_aic_post_filter,
    pic_if pic_post_ithread apple_aic_post_ithread,
    pic_if pic_pre_ithread apple_aic_pre_ithread,
#ifdef SMP
    DEVMETHOD(pic_bind_intr,	apple_aic_bind_intr),
    DEVMETHOD(pic_init_secondary,	apple_aic_init_secondary),
    DEVMETHOD(pic_ipi_send,		apple_aic_ipi_send),
    DEVMETHOD(pic_ipi_setup,	apple_aic_ipi_setup),
#endif
    */
