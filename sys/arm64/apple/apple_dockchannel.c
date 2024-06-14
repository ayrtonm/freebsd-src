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

static struct ofw_compat_data compat_data[] = {
	{ "apple,dockchannel",	1 },
	{ NULL,		0 },
};

static struct resource_spec apple_dockchannel_res_spec[] = {
	{ SYS_RES_MEMORY, 0, RF_ACTIVE },
	{ SYS_RES_IRQ, 0, RF_ACTIVE },
	{ -1, 0, 0 },
};

static device_probe_t apple_dockchannel_probe;
static device_attach_t apple_dockchannel_attach;
static device_detach_t apple_dockchannel_detach;

static pic_disable_intr_t apple_dockchannel_disable_intr;
static pic_enable_intr_t apple_dockchannel_enable_intr;
static pic_map_intr_t apple_dockchannel_map_intr;
static pic_setup_intr_t apple_dockchannel_setup_intr;
static pic_teardown_intr_t apple_dockchannel_teardown_intr;

static void apple_dockchannel_intr(void *);

static int
apple_dockchannel_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Apple Dockchannel");
	return (BUS_PROBE_DEFAULT);
}

static int
apple_dockchannel_attach(device_t dev)
{
	phandle_t node;
	struct apple_dockchannel_softc *sc;
	int error;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;

	node = ofw_bus_get_node(dev);

	if (bus_alloc_resources(dev, apple_dockchannel_res_spec, sc->sc_res) != 0) {
		device_printf(dev, "cannot allocate device resources\n");
		return (ENXIO);
	}

	/* disable and clear interrupts */
	HWRITE4(sc, IRQ_MASK, 0);
	HWRITE4(sc, IRQ_STAT, 0xffffffff);

	if (bus_setup_intr(dev, sc->sc_res[APPLE_PINCTRL_IRQRES],
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
