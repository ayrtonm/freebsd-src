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

struct apple_dockchannel_softc {
};

static device_probe_t apple_dockchannel_probe;
static device_attach_t apple_dockchannel_attach;
static device_detach_t apple_dockchannel_detach;

static pic_disable_intr apple_dockchannel_disable_intr;
static pic_enable_intr apple_dockchannel_enable_intr;
static pic_map_intr apple_dockchannel_map_intr;
static pic_setup_intr apple_dockchannel_setup_intr;
static pic_teardown_intr apple_dockchannel_teardown_intr;

static struct ofw_compat_data compat_data[] = {
	{ "apple,dockchannel",	1 },
	{ NULL,		0 },
};

static int
apple_dockchannel_probe(device_t dev)
{
	struct apple_dockchannel_softc *sc;

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
	struct apple_dockchannel_softc *sc;

	sc = device_get_softc(dev);
}

static device_method_t apple_dockchannel_methods[] = {
	DEVMETHOD(device_probe, apple_dockchannel_probe),
	DEVMETHOD(device_attach, apple_dockchannel_attach),
	DEVMETHOD(device_detach, apple_dockchannel_detach),

	DEVMETHOD(pic_disable_intr, apple_dockchannel_disable_intr),
	DEVMETHOD(pic_enable_intr, apple_dockchannel_enable_intr),
	DEVMETHOD(pic_map_intr ,apple_dockchannel_map_intr),
	DEVMETHOD(pic_setup_intr, apple_dockchannel_setup_intr),
	DEVMETHOD(pic_teardown_intr, apple_dockchannel_teardown_intr),

	DEVMETHOD_END
};

static driver_t apple_dockchannel_driver = {
	.name = "apple dockchannel",
	.methods = apple_dockchannel_methods,
	.size = sizeof(struct apple_dockchannel_softc),
};

DRIVER_MODULE(apple_dockchannel, simplebus, apple_dockchannel_driver, 0, 0);
