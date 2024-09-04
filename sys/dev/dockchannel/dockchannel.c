/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2024 Ayrton Muñoz
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

#include "opt_platform.h"

#include <sys/cdefs.h>

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/proc.h>
#include <sys/rman.h>
#include <sys/smp.h>

#include <machine/bus.h>
#include <machine/machdep.h>
#ifdef SMP
#include <machine/intr.h>
#include <machine/smp.h>
#endif

#include <dev/fdt/fdt_intr.h>
#include <dev/fdt/simplebus.h>

#include <dev/hid/hid.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "dockchannel.h"
#include "pic_if.h"

#define IRQ_MASK	0x0000
#define IRQ_STAT	0x0004

#define MAX_INTR	32

#define DC_READ4(sc, reg) \
	bus_read_4((sc)->sc_res[DOCKCHANNEL_MEMRES], reg)
#define DC_WRITE4(sc, reg, val) \
	bus_write_4((sc)->sc_res[DOCKCHANNEL_MEMRES], reg, val)

#define DC_SET4(sc, reg, bit) DC_WRITE4(sc, reg, DC_READ4(sc, reg) | (1 << bit))

#define DC_CLEAR4(sc, reg, bit) \
	DC_WRITE4(sc, reg, DC_READ4(sc, reg) & ~(1 << bit))

enum {
	DOCKCHANNEL_MEMRES = 0,
	DOCKCHANNEL_IRQRES,
	DOCKCHANNEL_NRES,
};

struct dockchannel_irqsrc {
	struct intr_irqsrc	di_isrc; /* base class must be first */
	uint32_t			di_irq;
	uint32_t			di_level;
};

struct dockchannel_softc {
	struct simplebus_softc simplebus_sc; /* base class must be first */
	device_t		sc_dev;

	struct resource	*sc_res[DOCKCHANNEL_NRES];
	void			*sc_intrhand;

	struct dockchannel_irqsrc	sc_isrcs[MAX_INTR];

	struct dockchannel_hid_softc *sc_hid;
};

static struct resource_spec dockchannel_res_spec[] = {
	{ SYS_RES_MEMORY, 0, RF_ACTIVE },
	{ SYS_RES_IRQ, 0, RF_ACTIVE },
	{ -1, 0, 0 },
};

static device_probe_t dockchannel_probe;
static device_attach_t dockchannel_attach;
static device_detach_t dockchannel_detach;

static pic_disable_intr_t dockchannel_disable_intr;
static pic_enable_intr_t dockchannel_enable_intr;
static pic_map_intr_t dockchannel_map_intr;
static pic_setup_intr_t dockchannel_setup_intr;
static pic_teardown_intr_t dockchannel_teardown_intr;

bool
dockchannel_is_masked(struct dockchannel_softc *sc, uint32_t irq)
{
	MPASS(irq < MAX_INTR);
	uint32_t mask = DC_READ4(sc, IRQ_MASK);
	return (mask & (1 << irq));
}
void
dockchannel_mask_irq(struct dockchannel_softc *sc, uint32_t irq)
{
	MPASS(irq < MAX_INTR);
	DC_CLEAR4(sc, IRQ_MASK, irq);
}

void
dockchannel_unmask_irq(struct dockchannel_softc *sc, uint32_t irq)
{
	MPASS(irq < MAX_INTR);
	DC_SET4(sc, IRQ_MASK, irq);
}

static int dockchannel_intr(void *);

static int
dockchannel_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev)) {
		return (ENXIO);
	}

	if (!ofw_bus_is_compatible(dev, "apple,dockchannel")) {
		return (ENXIO);
	}

	device_set_desc(dev, "Apple DockChannel");
	return (BUS_PROBE_DEFAULT);
}

static int
dockchannel_attach(device_t dev)
{
	int irq, rc;
	struct dockchannel_softc *sc;
	device_t *cdev;
	int ncdev;
	const char *name;
	phandle_t node;

	sc = device_get_softc(dev);

	sc->sc_dev = dev;
	node = ofw_bus_get_node(sc->sc_dev);

	sc->simplebus_sc.dev = dev;
	sc->simplebus_sc.node = node;

	if (bus_alloc_resources(dev, dockchannel_res_spec, sc->sc_res) != 0) {
		device_printf(dev, "cannot allocate device resources\n");
		return (ENXIO);
	}

	/* mask and clear interrupts */
	DC_WRITE4(sc, IRQ_MASK, 0);
	DC_WRITE4(sc, IRQ_STAT, 0xffffffff);

	if (bus_setup_intr(dev, sc->sc_res[DOCKCHANNEL_IRQRES],
		INTR_TYPE_CLK | INTR_MPSAFE, dockchannel_intr, NULL, sc,
		&sc->sc_intrhand)) {
		device_printf(dev, "can't establish interrupt\n");
		goto err_setup_intr;
	}

	name = device_get_nameunit(sc->sc_dev);
	for (irq = 0; irq < MAX_INTR; irq++) {
		sc->sc_isrcs[irq].di_irq = irq;
		sc->sc_isrcs[irq].di_level = 0; // ???
		rc = intr_isrc_register(&sc->sc_isrcs[irq].di_isrc,
			sc->sc_dev, 0, "%s,%d", name, irq);
		if (rc != 0) {
			device_printf(dev, "failed to register irq %d\n", irq);
			goto err_register_intr;
		}
	}

	if (intr_pic_register(sc->sc_dev, OF_xref_from_node(node)) == NULL) {
		device_printf(dev, "failed to register pic\n");
		goto err_pic_register;
	}

	rc = simplebus_attach(dev);
	if (rc != 0) {
		device_printf(dev, "simplebus_attach failed %d\n", rc);
		goto err_add_child;
	}
	device_get_children(dev, &cdev, &ncdev);
	MPASS(ncdev == 1);
	MPASS(*cdev != NULL);
	return device_probe_and_attach(*cdev);

err_add_child:
err_pic_register:
	// need to unregister isrcs here?
err_register_intr:
	//bus_teardown_intr(
err_setup_intr:
	bus_release_resources(dev, dockchannel_res_spec, sc->sc_res);
	return (ENXIO);
}

static int
dockchannel_detach(device_t dev)
{
	panic("uh oh");
}

static int
dockchannel_intr(void *arg)
{
	struct dockchannel_softc *sc;
	struct dockchannel_irqsrc *disrc;
	struct trapframe *tf;
	uint32_t irq, stat, pending;

	sc = arg;
	tf = curthread->td_intr_frame;

	stat = DC_READ4(sc, IRQ_STAT);
	pending = stat;
	while (pending) {
		irq = ffs(pending) - 1;

		disrc = &sc->sc_isrcs[irq];

		if (intr_isrc_dispatch(&disrc->di_isrc, tf) != 0) {
			device_printf(sc->sc_dev, "Stray irq %d disabled\n", irq);
			return FILTER_STRAY;
		}

		pending &= ~(1 << irq);
	}
	DC_WRITE4(sc, IRQ_STAT, stat);

	return FILTER_HANDLED;
}

static void
dockchannel_enable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	struct dockchannel_softc *sc = device_get_softc(dev);
	struct dockchannel_irqsrc *di = (struct dockchannel_irqsrc *)isrc;
	uint32_t irq = di->di_irq;

	dockchannel_unmask_irq(sc, irq);
}

static void
dockchannel_disable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	struct dockchannel_softc *sc = device_get_softc(dev);
	struct dockchannel_irqsrc *di = (struct dockchannel_irqsrc *)isrc;
	uint32_t irq = di->di_irq;

	dockchannel_mask_irq(sc, irq);
}

static int
do_dockchannel_map_intr(device_t dev, struct intr_map_data *data,
    uint32_t *irqp, uint32_t *levelp)
{
	if (data->type != INTR_MAP_DATA_FDT) {
		return ENOTSUP;
	}
	struct intr_map_data_fdt *daf = (struct intr_map_data_fdt *)data;

	if (daf->ncells != 2) {
		return EINVAL;
	}
	/* The first cell is the interrupt number. The second cells is the level. */
	uint32_t irq = daf->cells[0];
	if (irq >= MAX_INTR) {
		return EINVAL;
	}
	if (irqp) {
		*irqp = irq;
	}
	uint32_t level = daf->cells[1];
	if (levelp) {
		*levelp = level;
	}
	return 0;
}

static int
dockchannel_map_intr(device_t dev, struct intr_map_data *data,
    struct intr_irqsrc **isrcp)
{
	int rc;
	uint32_t irq;
	struct dockchannel_softc *sc = device_get_softc(dev);

	if (isrcp == NULL) {
		return EINVAL;
	}

	rc = do_dockchannel_map_intr(dev, data, &irq, NULL);
	if (rc != 0) {
		return rc;
	}

	*isrcp = &sc->sc_isrcs[irq].di_isrc;
	return 0;
}

static int
dockchannel_setup_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{
	int rc;
	uint32_t irq, level;
	struct dockchannel_irqsrc *di;

	di = (struct dockchannel_irqsrc *)isrc;

	if (isrc->isrc_handlers != 0) {
		return 0;
	}
	if (data == NULL) {
		return ENOTSUP;
	}
	rc = do_dockchannel_map_intr(dev, data, &irq, &level);
	if (rc != 0) {
		return rc;
	}
	di->di_irq = irq;
	di->di_level = level;
	return 0;
}

static int
dockchannel_teardown_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{
	int rc;
	uint32_t irq, level;
	struct dockchannel_softc *sc = device_get_softc(dev);

	rc = do_dockchannel_map_intr(dev, data, &irq, &level);
	if (rc != 0) {
		return rc;
	}
	dockchannel_mask_irq(sc, irq);
	return 0;
}

static device_method_t dockchannel_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, dockchannel_probe),
	DEVMETHOD(device_attach, dockchannel_attach),
	DEVMETHOD(device_detach, dockchannel_detach),

	DEVMETHOD(pic_disable_intr, dockchannel_disable_intr),
	DEVMETHOD(pic_enable_intr, dockchannel_enable_intr),
	DEVMETHOD(pic_map_intr ,dockchannel_map_intr),
	DEVMETHOD(pic_setup_intr, dockchannel_setup_intr),
	DEVMETHOD(pic_teardown_intr, dockchannel_teardown_intr),

	DEVMETHOD_END
};

DEFINE_CLASS_1(dockchannel, dockchannel_driver, dockchannel_methods,
    sizeof(struct dockchannel_softc), simplebus_driver);

DRIVER_MODULE(dockchannel, simplebus, dockchannel_driver, 0, 0);
