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

use kpi::bindings::{intr_polarity, intr_trigger};
use kpi::bus::Register;
use kpi::cell::SubClass;
use kpi::device::{BusProbe, Device};
use kpi::driver;
use kpi::enum_c_macros;
use kpi::intr::IrqSrc;
use kpi::ofw::OfwCompatData;

const AIC_INFO: u64 = 0x0004;

const AIC_MAXDIES: usize = 4;
const NUM_FIQS: usize = 6;
const NUM_IPIS: usize = bindings::INTR_IPI_COUNT as usize;

fn info_nirqs(info: u32) -> u32 {
    info & 0xffff
}

fn info_ndie(info: u32) -> u32 {
    (info >> 24) & 0xf
}

#[derive(Debug)]
struct AppleIntData {
    version: u32,
}

static COMPAT_DATA: OfwCompatData<AppleIntData, 2> = OfwCompatData::new([
    (c"apple,aic", &AppleIntData { version: 1 }),
    (c"apple,aic2", &AppleIntData { version: 1 }),
]);

type AppleIrqSrc = SubClass<IrqSrc, AppleIrqFields>;

#[derive(Debug)]
struct AppleIrqFields {
    kind: AppleIntrKind,
    pol: intr_polarity,
    trig: intr_trigger,
}

enum_c_macros! {
    #[repr(i32)]
    #[derive(Debug, Copy, Clone)]
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

#[derive(Debug)]
enum AppleIntrKind {
    Irq { die: u32, irq: u32 },
    Fiq(AppleFiqKind),
    Ipi,
}

#[derive(Debug)]
pub struct AppleIntSoftc {
    dev: Device,
    cfg: &'static AppleIntData,
    mem: Register,
    event: Option<Register>,
    irq_srcs: [Option<Box<[AppleIrqSrc], M_DEVBUF>>; AIC_MAXDIES],
    fiq_srcs: [AppleIrqSrc; NUM_FIQS],
    ipi_srcs: [AppleIrqSrc; NUM_IPIS],
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
            event = Some(Register::new(event_res)?)
        };
        let info = bus_read_4!(mem, AIC_INFO);
        let nirqs = info_nirqs(info);
        let ndie = info_ndie(info) + 1;
        let name = device_get_nameunit(&dev);

        let mut irq_srcs = [const { None }; AIC_MAXDIES];
        for die in 0..ndie {
            let mut irqs_vec =
                Vec::try_with_capacity(nirqs as usize, M_WAITOK | M_ZERO).map_err(|e| {
                    device_println!(dev, "failed to allocate memory for irqs {e}");
                    return ENXIO;
                })?;
            for irq in 0..nirqs {
                let irq_src = AppleIrqSrc::new(AppleIrqFields {
                    kind: AppleIntrKind::Irq { die, irq },
                    pol: bindings::INTR_POLARITY_CONFORM,
                    trig: bindings::INTR_TRIGGER_CONFORM,
                });
                // Cannot fail since we preallocated enough capacity
                irqs_vec.try_push(irq_src).unwrap();
            }
            irq_srcs[die as usize] = Some(irqs_vec.into_boxed_slice());
        }
        let fiq_srcs = AppleFiqKind::all_fiqs().map(|fiq| {
            AppleIrqSrc::new(AppleIrqFields {
                kind: AppleIntrKind::Fiq(fiq),
                pol: bindings::INTR_POLARITY_CONFORM,
                trig: bindings::INTR_TRIGGER_CONFORM,
            })
        });
        let ipi_srcs = [const {
            AppleIrqSrc::new(AppleIrqFields {
                kind: AppleIntrKind::Ipi,
                pol: bindings::INTR_POLARITY_CONFORM,
                trig: bindings::INTR_TRIGGER_CONFORM,
            })
        }; NUM_IPIS];

        let sc = AppleIntSoftc {
            dev,
            cfg,
            mem,
            event,
            irq_srcs,
            fiq_srcs,
            ipi_srcs,
        };
        let pinned_sc = device_init_softc!(dev, sc);
        fn foo(x: ()) {
        }
        for pinned_fiq in pinned_sc.fiq_srcs {
            foo(pinned_fiq);
        }
        //for die in 0..ndie {
        //    for die_irqs in pinned_sc.irq_srcs {
        //        if let Some(irqs) = die_irqs {
        //            for pinned_irq in irqs {
        //                foo(pinned_irq);
        //            }
        //        };
        //    }
        //}
        //    for irq in 0..nirqs {
        //        //foo(pinned_sc.irq_srcs);
        //    }
        Ok(())
    }
}

driver!(apple_aic_driver, c"aic", AppleIntDriver, apple_aic_methods,
    INTERFACES {
        device_probe apple_aic_probe,
        device_attach apple_aic_attach,
    }
);
