#![no_std]
#![feature(try_with_capacity, concat_idents)]
#![deny(unused_must_use)]

extern crate alloc;
extern crate kpi;

use alloc::boxed::Box;
use alloc::vec::Vec;
use core::mem::{transmute, MaybeUninit};
use kpi::arm64::in_vhe;
use kpi::bus::Resource;
use kpi::device::{Device, DeviceIf, ProbeRes};
use kpi::intr::{FilterRes, IrqSrc, Pic, PicIf};
use kpi::ofw::{CompatData, CompatEntry};
// This glob imports KPI Result, error codes, BUS_PROBE_* and relevant SYS_RES_* macros
use kpi::prelude::*;
use kpi::{bindings, curthread, driver, isb, pcpu_get, read_reg, write_reg, Unique};

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

//const AIC_MAXCPUS: u64 = 32;
//const AIC_MAXDIES: u64 = 4;

static COMPAT: CompatData<Cfg, 3> = CompatData::new(|| {
    [
        CompatEntry::new(c"apple,aic", &AIC1),
        CompatEntry::new(c"apple,aic2", &AIC2),
        CompatEntry::null(),
    ]
});

#[derive(Debug)]
enum IntrKind {
    Irq { die: u32, irq: u32 },
    Fiq(u32),
    Ipi,
}

#[derive(Debug)]
struct Intr {
    isrc: IrqSrc,
    kind: IntrKind,
    pol: bindings::intr_polarity,
    trig: bindings::intr_trigger,
}

#[derive(Debug)]
struct Softc {
    mem: Resource,
    event: Resource,
    nirqs: u32,
    ndie: u32,
    irq_srcs: Option<Box<[Intr]>>,
    //fiq_srcs: Box<[Intr]>,
    cfg: Cfg,
}

extern "C" fn irq_handler(sc: &mut Softc) -> FilterRes {
    let tf = curthread!(td_intr_frame);
    #[repr(C)]
    struct Event {
        irq: u16,
        ty: u8,
        die: u8,
    }

    let event = if sc.cfg.version == 1 {
        sc.mem.read_4(AIC_EVENT)
    } else {
        sc.event.read_4(0)
    };

    let event: Event = unsafe { transmute(event) };

    let ty = event.ty;

    if ty != AIC_EVENT_TYPE_IRQ {
        if ty != AIC_EVENT_TYPE_NONE {}
    }
    //    // get curthread td_intr_frame from x18 PCPU
    //    let tf = unsafe { (*curthread!()).td_intr_frame };
    //    #[repr(C)]
    //    struct Event {
    //        irq: u16,
    //        ty: u8,
    //        die: u8,
    //    }
    //    let event = if sc.cfg.version == 1 {
    //        sc.mem.read_4(bindings::AIC_EVENT as u64)
    //    } else {
    //        sc.event.read_4(0)
    //    };
    //    // TODO: there's probably a cleaner way to do this...
    //    let event: Event = unsafe { transmute(event) };
    //
    //    let ty = event.ty;
    //
    //    if ty != bindings::AIC_EVENT_TYPE_IRQ as u8 {
    //        if ty != bindings::AIC_EVENT_TYPE_NONE as u8 {
    //            dprintln!(sc.dev, "unexpected event type {ty}");
    //            return FILTER_STRAY;
    //        }
    //    }
    //
    //    let die = event.die as usize;
    //    let irq = event.irq as usize;
    //    let idx = (irq + (die * sc.nirqs as usize));
    //    if Pic::isrc_dispatch(&mut sc.irq_srcs[idx].isrc, tf).is_err() {
    //        dprintln!(sc.dev, "Stray irq {die}:{irq} disabled");
    //        return FILTER_STRAY;
    //    }
    //
    FILTER_HANDLED
}

extern "C" fn fiq_handler(sc: &mut Softc) -> FilterRes {
    FILTER_STRAY
}

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
        if dev.ofw_bus_search_compatible(&*COMPAT.0).is_none() {
            return Err(ENXIO);
        }
        dev.set_desc(c"Apple Interrupt Controller");
        Ok(BUS_PROBE_DEFAULT)
    }

    fn device_attach(&self, dev: Device<Unique>) -> Result<()> {
        let mut sc = self.claim_softc(dev)?;
        sc.cfg = *dev
            .ofw_bus_search_compatible(&*COMPAT.0)
            .expect("reached device_attach with missing config");

        sc.mem = dev
            .bus_alloc_resource(SYS_RES_MEMORY, 0)
            .inspect_err(|e| dprintln!(dev, "could not allocate 'core' memory resource {:?}", e))?;
        if sc.cfg.version == 2 {
            sc.event = dev.bus_alloc_resource(SYS_RES_MEMORY, 1).inspect_err(|e| {
                dprintln!(dev, "could not allocate 'event' memory resource {:?}", e)
            })?;
        }

        let info = sc.mem.read_4(AIC_INFO);
        sc.nirqs = info_nirqs(info);
        sc.ndie = info_ndie(info);

        let name = dev.get_nameunit();

        for die in 0..sc.ndie {
            for irq in 0..sc.nirqs {}
        }

        let node = dev.ofw_bus_get_node();
        let xref = node.xref_from_node();

        let pic = Pic::register(dev, xref)?;

        dev.register_xref(xref);

        pic.claim_root(dev, irq_handler, sc, INTR_ROOT_IRQ)?;
        pic.claim_root(dev, fiq_handler, sc, INTR_ROOT_FIQ)?;

        if sc.cfg.version == 2 {
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
