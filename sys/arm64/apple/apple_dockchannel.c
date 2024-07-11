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
	bus_read_4((sc)->sc_res[APPLE_DOCKCHANNEL_MEMRES], reg)
#define HWRITE4(sc, reg, val)	\
	bus_write_4((sc)->sc_res[APPLE_DOCKCHANNEL_MEMRES], reg, val)

enum {
	APPLE_DOCKCHANNEL_MEMRES = 0,
	APPLE_DOCKCHANNEL_IRQRES,
	APPLE_DOCKCHANNEL_NRES,
};

struct apple_dockchannel_softc {
	device_t sc_dev;

	struct resource *sc_res[APPLE_DOCKCHANNEL_NRES];

	void *sc_intrhand[MAX_INTR];
};

static struct resource_spec apple_dockchannel_res_spec[] = {
	{ SYS_RES_MEMORY, 0, RF_ACTIVE },
	{ SYS_RES_IRQ, 0, RF_ACTIVE },
	{ -1, 0, 0 },
};

static device_probe_t apple_dockchannel_probe;
static device_attach_t apple_dockchannel_attach;
#if 0
static device_detach_t apple_dockchannel_detach;

static pic_disable_intr_t apple_dockchannel_disable_intr;
static pic_enable_intr_t apple_dockchannel_enable_intr;
static pic_map_intr_t apple_dockchannel_map_intr;
static pic_setup_intr_t apple_dockchannel_setup_intr;
static pic_teardown_intr_t apple_dockchannel_teardown_intr;

#endif
static void apple_dockchannel_intr(void *);

static int
apple_dockchannel_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "apple,dockchannel"))
		return (ENXIO);

	device_set_desc(dev, "Apple Dockchannel");
	return (BUS_PROBE_DEFAULT);
}

static int
apple_dockchannel_attach(device_t dev)
{
	//phandle_t node;
	struct apple_dockchannel_softc *sc;
	//int error;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;

	//node = ofw_bus_get_node(dev);

	if (bus_alloc_resources(dev, apple_dockchannel_res_spec, sc->sc_res) != 0) {
		device_printf(dev, "cannot allocate device resources\n");
		return (ENXIO);
	}

	/* disable and clear interrupts */
	HWRITE4(sc, IRQ_MASK, 0);
	HWRITE4(sc, IRQ_STAT, 0xffffffff);

	if (bus_setup_intr(dev, sc->sc_res[APPLE_DOCKCHANNEL_IRQRES],
		INTR_TYPE_CLK | INTR_MPSAFE, NULL, apple_dockchannel_intr, sc,
		sc->sc_intrhand)) {
		device_printf(dev, "can't establish interrupt\n");
		goto error;
	}

	return (0);

error:
	bus_release_resources(dev, apple_dockchannel_res_spec, sc->sc_res);
	return (ENXIO);
}

static void
apple_dockchannel_intr(void *arg)
{
	struct apple_dockchannel_softc *sc = arg;
	void *ih;
	uint32_t stat, pending;
	int irq;

	stat = HREAD4(sc, IRQ_STAT);
	pending = stat;
	while (pending) {
		irq = ffs(pending) - 1;

		ih = sc->sc_intrhand[irq];
		if (ih) {
		}

		pending &= ~(1 << irq);
	}

	HWRITE4(sc, IRQ_STAT, stat);
	return;
}
#if 0

static void
apple_dockchannel_enable_intr(device_t dev, struct intr_irqsrc *isrc)
{
}

static void
apple_dockchannel_disable_intr(device_t dev, struct intr_irqsrc *isrc)
{
}

static int
apple_dockchannel_map_intr(device_t dev, struct intr_map_data *data,
    struct intr_irqsrc **isrcp)
{
}

static int
apple_dockchannel_setup_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{
}

static int
apple_dockchannel_teardown_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{
}
#endif

static device_method_t apple_dockchannel_methods[] = {
	DEVMETHOD(device_probe, apple_dockchannel_probe),
	DEVMETHOD(device_attach, apple_dockchannel_attach),
#if 0
	DEVMETHOD(device_detach, apple_dockchannel_detach),

	DEVMETHOD(pic_disable_intr, apple_dockchannel_disable_intr),
	DEVMETHOD(pic_enable_intr, apple_dockchannel_enable_intr),
	DEVMETHOD(pic_map_intr ,apple_dockchannel_map_intr),
	DEVMETHOD(pic_setup_intr, apple_dockchannel_setup_intr),
	DEVMETHOD(pic_teardown_intr, apple_dockchannel_teardown_intr),
#endif

	DEVMETHOD_END
};

static driver_t apple_dockchannel_driver = {
	.name = "apple dockchannel",
	.methods = apple_dockchannel_methods,
	.size = sizeof(struct apple_dockchannel_softc),
};

DRIVER_MODULE(apple_dockchannel, simplebus, apple_dockchannel_driver, 0, 0);

#define APPLE_DOCKCHANNEL_HID_NRES 4
//#define APPLE_DOCKCHANNEL_HID_IRQRES 4
#define APPLE_DOCKCHANNEL_HID_CONFIG 0
#define APPLE_DOCKCHANNEL_HID_DATA 1
#define APPLE_DOCKCHANNEL_HID_RMT_CONFIG 2
#define APPLE_DOCKCHANNEL_HID_RMT_DATA 3

struct apple_dockchannel_hid_softc {
	device_t sc_dev;
	struct resource *sc_mem_res[APPLE_DOCKCHANNEL_HID_NRES];
	struct resource *sc_irq_res;

	void *sc_intrhand;
}

static struct resource_spec apple_dockchannel_hid_res_spec[] = {
	{ SYS_RES_MEMORY, APPLE_DOCKCHANNEL_HID_CONFIG, RF_ACTIVE },
	{ SYS_RES_MEMORY, APPLE_DOCKCHANNEL_HID_DATA, RF_ACTIVE },
	{ SYS_RES_MEMORY, APPLE_DOCKCHANNEL_HID_RMT_CONFIG, RF_ACTIVE },
	{ SYS_RES_MEMORY, APPLE_DOCKCHANNEL_HID_RMT_DATA, RF_ACTIVE },
//	{ SYS_RES_IRQ, 0, RF_ACTIVE },
	{ -1, 0, 0 },
};

static device_probe_t apple_dockchannel_hid_probe;
static device_attach_t apple_dockchannel_hid_attach;
#if 0
static device_detach_t apple_dockchannel_hid_detach;
#endif
static void apple_dockchannel_hid_intr(void *);

static int
apple_dockchannel_hid_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_is_compatible(dev, "apple,dockchannel-hid"))
		return (ENXIO);

	device_set_desc(dev, "Apple Dockchannel HID");
	return (BUS_PROBE_DEFAULT);
}

static int
apple_dockchannel_hid_attach(device_t dev)
{
	phandle_t node;
	int rid;
	struct apple_dockchannel_hid_softc *sc;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;

	node = ofw_bus_get_node(dev);

	if (bus_alloc_resources(dev, apple_dockchannel_hid_res_spec, sc->sc_mem_res) != 0) {
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
		apple_dockchannel_hid_intr, NULL, sc, &sc->sc_intrhand);

#if 0
	phandle = OF_getpropint(faa->fa_node, "apple,helper-cpu", 0);
	if (phandle) {
		error = aplrtk_start(phandle);
		if (error) {
			printf(": can't start helper CPU\n");
			return;
		}
	}
#endif

	return (0);
error:
	bus_release_resources(dev, apple_dockchannel_hid_res_spec, sc->sc_res);
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
apple_dockchannel_hid_read(struct apple_dockchannel_hid_softc *sc, void *buf, size_t len,
	uint32_t *cksum)
{
}

// this is the rx interrupt handler... do we need a tx handler at some point?
static void
apple_dockchannel_hid_intr(void *arg)
{
	struct apple_dockchannel_hid_softc *sc = arg;
	struct mtp_hdr hdr;
	uint32_t cksum = 0;
	char buf[1024];

	apple_dockchannel_hid_read(sc, &hdr, sizeof(hdr), &cksum);
	apple_dockchannel_hid_read(sc, buf, hdr.pkt_len + 4, &cksum);
	if (cksum != 0xffffffff) {
		return;
	}
}

static device_method_t apple_dockchannel_hid_methods[] = {
	DEVMETHOD(device_probe, apple_dockchannel_hid_probe),
	DEVMETHOD(device_attach, apple_dockchannel_hid_attach),
#if 0
	DEVMETHOD(device_detach, apple_dockchannel_detach),
#endif

	DEVMETHOD_END
};

static driver_t apple_dockchannel_hid_driver = {
	.name = "apple dockchannel hid",
	.methods = apple_dockchannel_hid_methods,
	.size = sizeof(struct apple_dockchannel_hid_softc),
};
DRIVER_MODULE(apple_dockchannel_hid, simplebus, apple_dockchannel_hid_driver, 0, 0);
