/*-
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

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "pic_if.h"

#define IRQ_MASK	0x0000
#define IRQ_STAT	0x0004

#define CONFIG_TX_THRESH 0x0
#define CONFIG_RX_THRESH 0x4

#define DATA_TX8		0x4
#define DATA_TX32		0x10
#define DATA_TX_FREE	0x14
#define DATA_RX8		0x1c
#define DATA_RX32		0x28
#define DATA_RX_COUNT	0x2c

#define MAX_INTR	32

#define DC_READ4(sc, reg) \
	bus_read_4((sc)->sc_res[APPLE_DC_MEMRES], reg)
#define DC_WRITE4(sc, reg, val) \
	bus_write_4((sc)->sc_res[APPLE_DC_MEMRES], reg, val)

#define DC_SET4(sc, reg, bit) DC_WRITE4(sc, reg, DC_READ4(sc, reg) | (1 << bit))

#define DC_CLEAR4(sc, reg, bit) \
	DC_WRITE4(sc, reg, DC_READ4(sc, reg) & ~(1 << bit))

#define HID_READ4(sc, memres, reg) bus_read_4((sc)->sc_res[memres], reg)
#define HID_WRITE4(sc, memres, reg, val) \
	bus_write_4((sc)->sc_res[memres], reg, val)

enum {
	APPLE_DC_MEMRES = 0,
	APPLE_DC_IRQRES,
	APPLE_DC_NRES,
};

struct apple_dc_irqsrc {
	struct intr_irqsrc	di_isrc; /* base class must be first */
	uint32_t			di_irq;
	uint32_t			di_level;
};

struct apple_dc_softc {
	device_t		sc_dev;

	struct resource	*sc_res[APPLE_DC_NRES];
	void			*sc_intrhand;

	struct apple_dc_irqsrc	sc_isrcs[MAX_INTR];
};

static struct resource_spec apple_dc_res_spec[] = {
	{ SYS_RES_MEMORY, 0, RF_ACTIVE },
	{ SYS_RES_IRQ, 0, RF_ACTIVE },
	{ -1, 0, 0 },
};

static device_probe_t apple_dc_probe;
static device_attach_t apple_dc_attach;
static device_detach_t apple_dc_detach;

static pic_disable_intr_t apple_dc_disable_intr;
static pic_enable_intr_t apple_dc_enable_intr;
static pic_map_intr_t apple_dc_map_intr;
static pic_setup_intr_t apple_dc_setup_intr;
static pic_teardown_intr_t apple_dc_teardown_intr;

static void
apple_dc_mask_irq(struct apple_dc_softc *sc, uint32_t irq)
{
	MPASS(irq < MAX_INTR);
	DC_CLEAR4(sc, IRQ_MASK, irq);
}

static void
apple_dc_unmask_irq(struct apple_dc_softc *sc, uint32_t irq)
{
	MPASS(irq < MAX_INTR);
	DC_SET4(sc, IRQ_MASK, irq);
}

static int apple_dc_intr(void *);

static int
apple_dc_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "apple,dockchannel"))
		return (ENXIO);

	device_set_desc(dev, "Apple DockChannel");
	return (BUS_PROBE_DEFAULT);
}

static int
apple_dc_attach(device_t dev)
{
	int irq, rc;
	struct apple_dc_softc *sc = device_get_softc(dev);
	const char *name;
	phandle_t node;

	device_printf(dev, "attaching dockchannel...\n");
	DELAY(1000000);
	sc->sc_dev = dev;
	node = ofw_bus_get_node(sc->sc_dev);

	if (bus_alloc_resources(dev, apple_dc_res_spec, sc->sc_res) != 0) {
		device_printf(dev, "cannot allocate device resources\n");
		return (ENXIO);
	}

	/* mask and clear interrupts */
	DC_WRITE4(sc, IRQ_MASK, 0);
	DC_WRITE4(sc, IRQ_STAT, 0xffffffff);

	if (bus_setup_intr(dev, sc->sc_res[APPLE_DC_IRQRES],
		INTR_TYPE_CLK | INTR_MPSAFE, apple_dc_intr, NULL, sc,
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

	return (0);

err_pic_register:
	// need to unregister isrcs here?
err_register_intr:
	//bus_teardown_intr(
err_setup_intr:
	bus_release_resources(dev, apple_dc_res_spec, sc->sc_res);
	return (ENXIO);
}

static int
apple_dc_detach(device_t dev)
{
	panic("uh oh");
}

static int
apple_dc_intr(void *arg)
{
	struct apple_dc_softc *sc;
	struct apple_dc_irqsrc *disrc;
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
apple_dc_enable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	struct apple_dc_softc *sc = device_get_softc(dev);
	struct apple_dc_irqsrc *di = (struct apple_dc_irqsrc *)isrc;
	uint32_t irq = di->di_irq;

	apple_dc_unmask_irq(sc, irq);
}

static void
apple_dc_disable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	struct apple_dc_softc *sc = device_get_softc(dev);
	struct apple_dc_irqsrc *di = (struct apple_dc_irqsrc *)isrc;
	uint32_t irq = di->di_irq;

	apple_dc_mask_irq(sc, irq);
}

static int
do_apple_dc_map_intr(device_t dev, struct intr_map_data *data,
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
apple_dc_map_intr(device_t dev, struct intr_map_data *data,
    struct intr_irqsrc **isrcp)
{
	int rc;
	uint32_t irq;
	struct apple_dc_softc *sc = device_get_softc(dev);

	if (isrcp == NULL) {
		return EINVAL;
	}

	rc = do_apple_dc_map_intr(dev, data, &irq, NULL);
	if (rc != 0) {
		return rc;
	}

	*isrcp = &sc->sc_isrcs[irq].di_isrc;
	return 0;
}

static int
apple_dc_setup_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{
	int rc;
	uint32_t irq, level;
	struct apple_dc_irqsrc *di;

	di = (struct apple_dc_irqsrc *)isrc;

	if (isrc->isrc_handlers != 0) {
		return 0;
	}
	if (data == NULL) {
		return ENOTSUP;
	}
	rc = do_apple_dc_map_intr(dev, data, &irq, &level);
	if (rc != 0) {
		return rc;
	}
	di->di_irq = irq;
	di->di_level = level;
	return 0;
}

static int
apple_dc_teardown_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{
	int rc;
	uint32_t irq, level;
	struct apple_dc_softc *sc = device_get_softc(dev);

	rc = do_apple_dc_map_intr(dev, data, &irq, &level);
	if (rc != 0) {
		return rc;
	}
	apple_dc_mask_irq(sc, irq);
	return 0;
}

static device_method_t apple_dc_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, apple_dc_probe),
	DEVMETHOD(device_attach, apple_dc_attach),
	DEVMETHOD(device_detach, apple_dc_detach),

	DEVMETHOD(pic_disable_intr, apple_dc_disable_intr),
	DEVMETHOD(pic_enable_intr, apple_dc_enable_intr),
	DEVMETHOD(pic_map_intr ,apple_dc_map_intr),
	DEVMETHOD(pic_setup_intr, apple_dc_setup_intr),
	DEVMETHOD(pic_teardown_intr, apple_dc_teardown_intr),

	DEVMETHOD_END
};

static driver_t apple_dc_driver = {
	.name = "dockchannel",
	.methods = apple_dc_methods,
	.size = sizeof(struct apple_dc_softc),
};

EARLY_DRIVER_MODULE(apple_dc, simplebus, apple_dc_driver, 0, 0, BUS_PASS_DEFAULT-1);

enum {
	APPLE_DC_HID_CONFIG = 0,
	APPLE_DC_HID_DATA,
	APPLE_DC_HID_RMT_CONFIG,
	APPLE_DC_HID_RMT_DATA,
	APPLE_DC_HID_NMEMRES,
};

enum {
	APPLE_DC_HID_TX = 0,
	APPLE_DC_HID_RX,
	APPLE_DC_HID_NIRQRES,
};

struct apple_dc_hid_softc {
	device_t		sc_dev;
	struct resource *sc_res[APPLE_DC_HID_NMEMRES + APPLE_DC_HID_NIRQRES];
	void			*sc_intrhand;
	struct intr_config_hook	config_hook;
	phandle_t		sc_mtp;
};

static struct resource *
apple_dc_hid_irq_res(struct apple_dc_hid_softc *sc, int idx)
{
	return sc->sc_res[idx + APPLE_DC_HID_NMEMRES];
}

static struct resource_spec apple_dc_hid_res_spec[] = {
	{ SYS_RES_MEMORY, APPLE_DC_HID_CONFIG, RF_ACTIVE },
	{ SYS_RES_MEMORY, APPLE_DC_HID_DATA, RF_ACTIVE },
	{ SYS_RES_MEMORY, APPLE_DC_HID_RMT_CONFIG, RF_ACTIVE },
	{ SYS_RES_MEMORY, APPLE_DC_HID_RMT_DATA, RF_ACTIVE },
	{ SYS_RES_IRQ, APPLE_DC_HID_TX, RF_ACTIVE },
	{ SYS_RES_IRQ, APPLE_DC_HID_RX, RF_ACTIVE },
	{ -1, 0, 0 },
};

static device_probe_t apple_dc_hid_probe;
static device_attach_t apple_dc_hid_attach;
static device_detach_t apple_dc_hid_detach;
static int apple_dc_hid_rx_intr(void *arg);

static int
apple_dc_hid_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "apple,dockchannel-hid"))
		return (ENXIO);

	device_set_desc(dev, "Apple DockChannel HID");
	return (BUS_PROBE_DEFAULT);
}

static int
apple_dc_hid_attach(device_t dev)
{
	int rc;
	struct apple_dc_hid_softc *sc;
	phandle_t node;

	device_printf(dev, "attaching dockchannel HID...\n");
	DELAY(1000000);

	sc = device_get_softc(dev);
	sc->sc_dev = dev;

	node = ofw_bus_get_node(dev);

	if (bus_alloc_resources(dev, apple_dc_hid_res_spec, sc->sc_res) != 0) {
		device_printf(dev, "cannot allocate device resources\n");
		return ENXIO;
	}

	rc = bus_setup_intr(dev, apple_dc_hid_irq_res(sc, APPLE_DC_HID_RX),
		INTR_MPSAFE | INTR_TYPE_MISC, apple_dc_hid_rx_intr, NULL, sc,
		&sc->sc_intrhand);
	if (rc != 0) {
		device_printf(dev, "cannot setup intr (%d)\n", rc);
		return rc;
	}

	rc = OF_getencprop(node, "apple,helper-cpu", &sc->sc_mtp,
		sizeof(sc->sc_mtp));
	if (rc != sizeof(sc->sc_mtp)) {
		device_printf(dev, "couldn't find 'apple-helper-cpu' prop %d\n", rc);
		return rc;
	}

	//sc->config_hook.ich_func = apple_dc_hid_start_config_hook;
	//sc->config_hook.ich_arg = sc;

	//if (config_intrhook_establish(&ctrlr->config_hook) != 0)
	//	return (ENOMEM);

	return 0;
}

static int
apple_dc_hid_detach(device_t dev)
{
	panic("uh oh");
}

static int
apple_dc_hid_rx_intr(void *arg)
{
	//struct apple_dc_hid_softc *sc = arg;
	return FILTER_HANDLED;
}

static device_method_t apple_dc_hid_methods[] = {
	DEVMETHOD(device_probe, apple_dc_hid_probe),
	DEVMETHOD(device_attach, apple_dc_hid_attach),
	DEVMETHOD(device_detach, apple_dc_detach),

	DEVMETHOD_END
};

static driver_t apple_dc_hid_driver = {
	.name = "dockchannel_HID",
	.methods = apple_dc_hid_methods,
	.size = sizeof(struct apple_dc_hid_softc),
};
DRIVER_MODULE(apple_dc_hid, simplebus, apple_dc_hid_driver, 0, 0);
