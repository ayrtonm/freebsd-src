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

use core::array;
use kpi::bindings::device_t;
use kpi::bus::{Filter, Irq, Register, Resource, ResourceSpec};
use kpi::device::{BusProbe, DeviceIf};
use kpi::ffi::{Ref, ToArrayCString, UninitRef};
use kpi::intr::{IrqSrc, MapData, PicIf};
use kpi::prelude::*;
use kpi::sync::{Checked, OnceInit};
use kpi::{driver, proj};
use simplebus::{SimpleBusDriver, SimpleBusSoftc, SimpleBusSoftcBase};

const MAX_INTR: usize = 32;
const IRQ_MASK: u64 = 0x0000;
const IRQ_STAT: u64 = 0x0004;

const SPEC: [ResourceSpec; 2] = [
    ResourceSpec::new(SYS_RES_MEMORY, 0),
    ResourceSpec::new(SYS_RES_IRQ, 0),
];

pub type DockchannelIrqSrc = IrqSrc<OnceInit<DockchannelIrqSrcFields>>;

#[derive(Debug)]
pub struct DockchannelIrqSrcFields {
    irq: u32,
    level: u32,
}

pub type DockchannelSoftc = SimpleBusSoftc<DockchannelSoftcFields>;

#[derive(Debug)]
pub struct DockchannelSoftcFields {
    dev: device_t,
    irq: Irq,
    isrcs: [DockchannelIrqSrc; MAX_INTR],
    stat_reg: Checked<Register>,
    mask_reg: Checked<Register>,
}

impl SimpleBusDriver for DockchannelDriver {}

impl DeviceIf for DockchannelDriver {
    type Softc = DockchannelSoftc;

    fn device_probe(dev: device_t) -> Result<BusProbe> {
        if !ofw_bus_status_okay(dev) {
            return Err(ENXIO);
        }

        if !ofw_bus_is_compatible(dev, c"apple,dockchannel") {
            return Err(ENXIO);
        }
        device_set_desc(dev, c"Apple DockChannel");
        Ok(BUS_PROBE_DEFAULT)
    }

    fn device_attach(uninit_sc: UninitRef<DockchannelSoftc>, dev: device_t) -> Result<()> {
        let [mem_res, irq_res] = bus_alloc_resources(dev, SPEC).map_err(|e| {
            device_println!(dev, "cannot allocate device resources {e}");
            ENXIO
        })?;
        let irq = irq_res.into_irq()?;
        let mut regs = mem_res.split_registers::<2>()?;
        let mut mask_reg = regs.take_register(IRQ_MASK, 4)?;
        let mut stat_reg = regs.take_register(IRQ_STAT, 4)?;

        /* mask and clear interrupts */
        bus_write_4!(mask_reg, IRQ_MASK, 0);
        bus_write_4!(stat_reg, IRQ_STAT, u32::MAX);

        let isrcs = array::from_fn(|_| DockchannelIrqSrc::new(OnceInit::uninit()));
        let sc = uninit_sc
            .init(DockchannelSoftc::new(DockchannelSoftcFields {
                dev,
                irq,
                isrcs,
                stat_reg: Checked::new(stat_reg),
                mask_reg: Checked::new(mask_reg),
            }))
            .into_ref();

        bus_setup_intr!(
            dev,
            proj!(&sc->irq),
            INTR_TYPE_CLK.0 | INTR_MPSAFE.0,
            Some(dockchannel_intr),
            None,
        )
        .unwrap();

        let mut name = device_get_nameunit(dev);
        for irq in 0..MAX_INTR {
            let mut pushed = name.push(b',');
            pushed += name.push_c_str(irq.to_array_cstring().as_c_str());
            intr_isrc_register(proj!(&sc->isrcs[irq]), dev, None, &name)?;
            name.pop(pushed);
        }

        let node = ofw_bus_get_node(dev);
        intr_pic_register(dev, OF_xref_from_node(node))?;

        Self::simplebus_attach(dev, |simplebus_sc| {
            simplebus_sc.dev = dev;
            simplebus_sc.node = node.0;
        })
        .map_err(|e| {
            device_println!(dev, "simplebus_attach failed {e}");
            ENXIO
        })?;

        let children = device_get_children(dev)?;
        for child in &children {
            device_probe_and_attach(*child)?;
        }
        Ok(())
    }
}

extern "C" fn dockchannel_intr(sc: Ref<DockchannelSoftc>) -> Filter {
    let tf = curthread!(td_intr_frame);

    let mut stat_reg = sc.stat_reg.get_mut();
    let stat = bus_read_4!(stat_reg, IRQ_STAT);
    let mut pending = stat;
    while pending != 0 {
        let irq = pending.trailing_zeros();
        if intr_isrc_dispatch(&sc.isrcs[irq as usize], tf) != 0 {
            device_println!(sc.dev, "Stray irq {irq} disabled");
            return FILTER_STRAY;
        }
        pending &= !(1 << irq);
    }
    bus_write_4!(stat_reg, IRQ_STAT, stat);
    FILTER_HANDLED
}

impl DockchannelDriver {
    // TODO: derive the 2 and 3 from the IRQ struct resources
    pub fn mask_rx(dev: device_t) {
        let parent = device_get_parent(dev).unwrap();
        let sc = device_get_softc!(parent);
        dockchannel_mask_irq(sc, 3)
    }

    pub fn mask_tx(dev: device_t) {
        let parent = device_get_parent(dev).unwrap();
        let sc = device_get_softc!(parent);
        dockchannel_mask_irq(sc, 2)
    }

    pub fn unmask_rx(dev: device_t) {
        let parent = device_get_parent(dev).unwrap();
        let sc = device_get_softc!(parent);
        dockchannel_unmask_irq(sc, 3)
    }

    pub fn unmask_tx(dev: device_t) {
        let parent = device_get_parent(dev).unwrap();
        let sc = device_get_softc!(parent);
        dockchannel_unmask_irq(sc, 2)
    }
}

fn dockchannel_mask_irq(sc: Ref<DockchannelSoftc>, irq: u32) {
    let mut mask_reg = sc.mask_reg.get_mut();
    let old_value = bus_read_4!(mask_reg, IRQ_MASK);
    bus_write_4!(mask_reg, IRQ_MASK, old_value & !(1 << irq));
}

fn dockchannel_unmask_irq(sc: Ref<DockchannelSoftc>, irq: u32) {
    let mut mask_reg = sc.mask_reg.get_mut();
    let old_value = bus_read_4!(mask_reg, IRQ_MASK);
    bus_write_4!(mask_reg, IRQ_MASK, old_value | (1 << irq));
}

fn do_dockchannel_irq_and_level(data: MapData) -> Result<(u32, u32)> {
    if let MapData::FDT(fdt_data) = data {
        let cells = fdt_data.cells();
        if cells.len() != 2 {
            return Err(EINVAL);
        }
        /* The first cell is the interrupt number. The second cells is the level. */
        let irq = cells[0];
        if irq as usize >= MAX_INTR {
            return Err(EINVAL);
        }
        return Ok((irq, cells[1]));
    };
    Err(EINVAL)
}

impl PicIf for DockchannelDriver {
    type IrqSrcFields = OnceInit<DockchannelIrqSrcFields>;

    fn pic_enable_intr(sc: Ref<DockchannelSoftc>, isrc: &DockchannelIrqSrc) {
        dockchannel_unmask_irq(sc, isrc.get().irq);
    }

    fn pic_disable_intr(sc: Ref<DockchannelSoftc>, isrc: &DockchannelIrqSrc) {
        dockchannel_mask_irq(sc, isrc.get().irq);
    }

    fn pic_map_intr<'sc>(
        sc: &'sc Ref<DockchannelSoftc>,
        data: MapData,
    ) -> Result<&'sc DockchannelIrqSrc> {
        let (irq, _level) = do_dockchannel_irq_and_level(data)?;
        Ok(&sc.isrcs[irq as usize])
    }

    fn pic_setup_intr(
        _sc: Ref<DockchannelSoftc>,
        isrc: &DockchannelIrqSrc,
        _res: Resource,
        data: MapData,
    ) -> Result<()> {
        let (irq, level) = do_dockchannel_irq_and_level(data)?;
        isrc.init(DockchannelIrqSrcFields { irq, level });
        Ok(())
    }

    fn pic_teardown_intr(
        sc: Ref<DockchannelSoftc>,
        _isrc: &DockchannelIrqSrc,
        _res: Resource,
        data: MapData,
    ) -> Result<()> {
        let (irq, _level) = do_dockchannel_irq_and_level(data)?;
        dockchannel_mask_irq(sc, irq);
        Ok(())
    }
}

driver! {
    dockchannel_driver, c"dockchannel", DockchannelDriver,
    dockchannel_methods = {
        device_probe dockchannel_probe,
        device_attach dockchannel_attach,

        pic_disable_intr dockchannel_disable_intr,
        pic_enable_intr dockchannel_enable_intr,
        pic_map_intr dockchannel_map_intr,
        pic_setup_intr dockchannel_setup_intr,
        pic_teardown_intr dockchannel_teardown_intr,
    },
    inherit from simplebus_driver,
}
