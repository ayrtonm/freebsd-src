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

#define MAX_INTR	32

#define HREAD4(sc, reg)	\
	bus_read_4((sc)->res[APPLE_DOCKCHANNEL_MEMRES], reg)
#define HWRITE4(sc, reg, val)	\
	bus_write_4((sc)->res[APPLE_DOCKCHANNEL_MEMRES], reg, val)

enum {
	APPLE_DOCKCHANNEL_MEMRES = 0,
	APPLE_DOCKCHANNEL_IRQRES,
	APPLE_DOCKCHANNEL_NRES,
};

struct apple_dc_irqsrc {
	struct intr_irqsrc	isrc; /* base class must be first */
	int					irq;
};

struct apple_dc_softc {
	device_t		dev;

	struct resource	*res[APPLE_DOCKCHANNEL_NRES];
	void			*intrhand;

	struct apple_dc_irqsrc	dcisrcs[MAX_INTR];
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

static void apple_dc_intr(void *);

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
	struct apple_dc_softc *sc = device_get_softc(dev);
	sc->dev = dev;

	if (bus_alloc_resources(dev, apple_dc_res_spec, sc->res) != 0) {
		device_printf(dev, "cannot allocate device resources\n");
		return (ENXIO);
	}

	/* disable and clear interrupts */
	HWRITE4(sc, IRQ_MASK, 0);
	HWRITE4(sc, IRQ_STAT, 0xffffffff);

	if (bus_setup_intr(dev, sc->res[APPLE_DOCKCHANNEL_IRQRES],
		INTR_TYPE_CLK | INTR_MPSAFE, NULL, apple_dc_intr, sc,
		&sc->intrhand)) {
		device_printf(dev, "can't establish interrupt\n");
		goto error;
	}

	return (0);

error:
	bus_release_resources(dev, apple_dc_res_spec, sc->res);
	return (ENXIO);
}

static int
apple_dc_detach(device_t dev)
{
	panic("uh oh");
}

static void
apple_dc_intr(void *arg)
{
	uint32_t irq;
	uint32_t stat, pending;

	struct apple_dc_softc *sc = arg;
	struct trapframe *tf = curthread->td_intr_frame;
	struct apple_dc_irqsrc *dcisrc;

	stat = HREAD4(sc, IRQ_STAT);
	pending = stat;
	while (pending) {
		irq = ffs(pending) - 1;

		dcisrc = &sc->dcisrcs[irq];

		if (intr_isrc_dispatch(&dcisrc->isrc, tf) != 0) {
		}

		pending &= ~(1 << irq);
	}
	HWRITE4(sc, IRQ_STAT, stat);
}

static void
apple_dc_enable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	struct apple_dc_softc *sc = device_get_softc(dev);
	struct apple_dc_irqsrc *dcisrc = (struct apple_dc_irqsrc *)isrc;
	uint32_t irq = dcisrc->irq;
	uint32_t mask;

	MPASS(irq < MAX_INTR);
	mask = HREAD4(sc, IRQ_MASK);
	mask |= 1 << irq;
	HWRITE4(sc, IRQ_MASK, mask);
}

static void
apple_dc_disable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	struct apple_dc_softc *sc = device_get_softc(dev);
	struct apple_dc_irqsrc *dcisrc = (struct apple_dc_irqsrc *)isrc;
	uint32_t irq = dcisrc->irq;
	uint32_t mask;

	MPASS(irq < MAX_INTR);
	mask = HREAD4(sc, IRQ_MASK);
	mask &= ~(1 << irq);
	HWRITE4(sc, IRQ_MASK, mask);
}

static int
apple_dc_map_intr(device_t dev, struct intr_map_data *data,
    struct intr_irqsrc **isrcp)
{
	struct apple_dc_softc *sc = device_get_softc(dev);
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
	*isrcp = &sc->dcisrcs[irq].isrc;
	return 0;
}

#if 0
static int
apple_dc_setup_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{
	uint32_t irq;
	struct apple_dc_softc *sc = device_get_softc(dev);
	struct apple_dc_irqsrc *dcisrc = (struct apple_dc_irqsrc *)isrc;

	if (data == NULL) {
		return ENOTSUP;
	}
}

static int
apple_dc_teardown_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{
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
	.name = "apple dockchannel",
	.methods = apple_dc_methods,
	.size = sizeof(struct apple_dc_softc),
};

DRIVER_MODULE(apple_dc, simplebus, apple_dc_driver, 0, 0);

#define APPLE_DOCKCHANNEL_HID_NRES 4
//#define APPLE_DOCKCHANNEL_HID_IRQRES 4
#define APPLE_DOCKCHANNEL_HID_CONFIG 0
#define APPLE_DOCKCHANNEL_HID_DATA 1
#define APPLE_DOCKCHANNEL_HID_RMT_CONFIG 2
#define APPLE_DOCKCHANNEL_HID_RMT_DATA 3

struct apple_dc_hid_softc {
	device_t dev;
	struct resource *sc_mem_res[APPLE_DOCKCHANNEL_HID_NRES];
	struct resource *sc_irq_res;

	void *intrhand;
};

static struct resource_spec apple_dc_hid_res_spec[] = {
	{ SYS_RES_MEMORY, APPLE_DOCKCHANNEL_HID_CONFIG, RF_ACTIVE },
	{ SYS_RES_MEMORY, APPLE_DOCKCHANNEL_HID_DATA, RF_ACTIVE },
	{ SYS_RES_MEMORY, APPLE_DOCKCHANNEL_HID_RMT_CONFIG, RF_ACTIVE },
	{ SYS_RES_MEMORY, APPLE_DOCKCHANNEL_HID_RMT_DATA, RF_ACTIVE },
//	{ SYS_RES_IRQ, 0, RF_ACTIVE },
	{ -1, 0, 0 },
};

static device_probe_t apple_dc_hid_probe;
static device_attach_t apple_dc_hid_attach;
static device_detach_t apple_dc_hid_detach;
static void apple_dc_hid_intr(void *);

static int
apple_dc_hid_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "apple,dc-hid"))
		return (ENXIO);

	device_set_desc(dev, "Apple Dockchannel HID");
	return (BUS_PROBE_DEFAULT);
}

static int
apple_dc_hid_attach(device_t dev)
{
	phandle_t node;
	int rid;
	struct apple_dc_hid_softc *sc;

	sc = device_get_softc(dev);
	sc->dev = dev;

	node = ofw_bus_get_node(dev);

	if (bus_alloc_resources(dev, apple_dc_hid_res_spec, sc->sc_mem_res) != 0) {
		device_printf(dev, "cannot allocate device memory resources\n");
		return (ENXIO);
	}

	if (ofw_bus_find_string_index(node, "interrupt-names", "rx", &rid) != 0) {
		device_printf(dev, "could not find rx irq\n");
		goto error;
	}

	sc->sc_irq_res = bus_alloc_resources_any(dev, SYS_RES_IRQ, &rid, RF_ACTIVE);
	if (sc->sc_irq_res == NULL) {
		device_printf(dev, "cannot allocate device irq resource\n");
		goto error
	}

	bus_setup_intr(dev, sc->sc_irq_res, INTR_MPSAFE | INTR_TYPE_MISC /* INTR_TYPE_TTY? */,
		apple_dc_hid_intr, NULL, sc, &sc->intrhand);

	phandle = OF_getpropint(faa->fa_node, "apple,helper-cpu", 0);
	if (phandle) {
		error = aplrtk_start(phandle);
		if (error) {
			printf(": can't start helper CPU\n");
			return;
		}
	}

	return (0);
error:
	bus_release_resources(dev, apple_dc_hid_res_spec, sc->res);
	return (ENXIO);
}

struct mtp_hdr {
	uint8_t hdr_len;
	uint8_t chan;
	uint16_t pkt_len;
	uint8_t seq;
	uint8_t iface;
	uint16_t pad;
} __packed;

static void
apple_dc_hid_read(struct apple_dc_hid_softc *sc, void *buf, size_t len,
	uint32_t *cksum)
{
}

// this is the rx interrupt handler... do we need a tx handler at some point?
static void
apple_dc_hid_intr(void *arg)
{
	struct apple_dc_hid_softc *sc = arg;
	struct mtp_hdr hdr;
	uint32_t cksum = 0;
	char buf[1024];

	apple_dc_hid_read(sc, &hdr, sizeof(hdr), &cksum);
	apple_dc_hid_read(sc, buf, hdr.pkt_len + 4, &cksum);
	if (cksum != 0xffffffff) {
		return;
	}
}

static device_method_t apple_dc_hid_methods[] = {
	DEVMETHOD(device_probe, apple_dc_hid_probe),
	DEVMETHOD(device_attach, apple_dc_hid_attach),
	DEVMETHOD(device_detach, apple_dc_detach),

	DEVMETHOD_END
};

static driver_t apple_dc_hid_driver = {
	.name = "apple dc hid",
	.methods = apple_dc_hid_methods,
	.size = sizeof(struct apple_dc_hid_softc),
};
DRIVER_MODULE(apple_dc_hid, simplebus, apple_dc_hid_driver, 0, 0);
#endif
