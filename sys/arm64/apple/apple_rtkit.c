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
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <arm64/apple/rtkit.h>

#define CPU_CTRL		0x44
#define CPU_CTRL_RUN	(1 << 4)

#define CPU_CTRL_READ4(sc) bus_read_4((sc)->sc_res[APPLE_RTKIT_ASC], CPU_CTRL)
#define CPU_CTRL_WRITE4(sc, val) \
	bus_write_4((sc)->sc_res[APPLE_RTKIT_ASC], CPU_CTRL, (val))

enum {
	APPLE_RTKIT_ASC = 0,
	APPLE_RTKIT_SRAM,
	APPLE_RTKIT_NRES,
};

struct apple_rtkit_softc {
	device_t			sc_dev;
	struct resource		*sc_res[APPLE_RTKIT_NRES];
	phandle_t			sc_self;
	struct rtkit_state	*sc_rtkit_state;
	struct intr_config_hook sc_config_hook;
};

static struct resource_spec apple_rtkit_res_spec[] = {
	// asc
	{ SYS_RES_MEMORY, 0, RF_ACTIVE },
	// sram
	{ SYS_RES_MEMORY, 1, RF_ACTIVE },
	{ -1, 0, 0 },
};

static device_probe_t apple_rtkit_probe;
static device_attach_t apple_rtkit_attach;
static device_detach_t apple_rtkit_detach;

static void apple_rtkit_start_config_hook(void *arg);

static int
apple_rtkit_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev)) {
		return ENXIO;
	}

	if (!ofw_bus_is_compatible(dev, "apple,rtk-helper-asc4")) {
		return ENXIO;
	}

	device_set_desc(dev, "Apple RTKit helper");
	return BUS_PROBE_DEFAULT;
}

static int
apple_rtkit_attach(device_t dev)
{
	int rc;
	phandle_t node;
	struct apple_rtkit_softc *sc;

	device_printf(dev, "attaching apple RTKit helper...\n");
	DELAY(1000000);

	sc = device_get_softc(dev);
	sc->sc_dev = dev;
	node = ofw_bus_get_node(dev);

	// openbsd gets this from the implicit phandle prop, but it probably doesn't matter
	sc->sc_self = node;

	if (bus_alloc_resources(dev, apple_rtkit_res_spec, sc->sc_res) != 0) {
		device_printf(dev, "cannot allocate device resources\n");
		return ENXIO;
	}

	rc = OF_device_register_xref(node, dev);
	if (rc != 0) {
		device_printf(dev, "failed to register xref %d\n", rc);
		return ENXIO;
	}

	rc = rtkit_init(dev, &sc->sc_rtkit_state);
	if (rc != 0) {
		device_printf(dev, "error initializing RTKit %d\n", rc);
		return ENXIO;
	}

	sc->sc_config_hook.ich_func = apple_rtkit_start_config_hook;
	sc->sc_config_hook.ich_arg = sc;

	if (config_intrhook_establish(&sc->sc_config_hook) != 0) {
		return ENOMEM;
	}
	return 0;

}

static int
apple_rtkit_detach(device_t dev)
{
	panic("uh oh");
}

static void
apple_rtkit_start_config_hook(void *arg)
{
	int rc;
	uint32_t ctrl;
	struct apple_rtkit_softc *sc = arg;
	device_t dev = sc->sc_dev;

	device_printf(dev, "setting CPU_CTRL RUN bit\n");
	ctrl = CPU_CTRL_READ4(sc);
	ctrl |= CPU_CTRL_RUN;
	CPU_CTRL_WRITE4(sc, ctrl);

	device_printf(dev, "booting RTKit\n");
	rc = rtkit_boot(sc->sc_rtkit_state);
	if (rc != 0) {
		device_printf(dev, "failed to boot RTKit %d\n", rc);
		panic("uh oh");
	}
	device_printf(dev, "setting RTKit AP power state to ON\n");
	rtkit_set_ap_pwrstate(sc->sc_rtkit_state, RTKIT_MGMT_PWR_STATE_ON);
}

static device_method_t apple_rtkit_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, apple_rtkit_probe),
	DEVMETHOD(device_attach, apple_rtkit_attach),
	DEVMETHOD(device_detach, apple_rtkit_detach),

	DEVMETHOD_END
};

static driver_t apple_rtkit_driver = {
	.name = "rtkit_helper",
	.methods = apple_rtkit_methods,
	.size = sizeof(struct apple_rtkit_softc),
};

EARLY_DRIVER_MODULE(apple_rtkit, simplebus, apple_rtkit_driver, 0, 0, BUS_PASS_DEFAULT-2);
