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
#include <sys/bus.h>

enum {
	RTKIT_HELPER_ASC = 0,
	RTKIT_HELPER_SRAM,
	RTKIT_HELPER_NRES,
};

struct rtkit_helper_softc {
	device_t			sc_dev;
	struct resource		*sc_res[RTKIT_HELPER_NRES];
	phandle_t			sc_self;
};

static struct resource_spec rtkit_helper_res_spec[] = {
	// asc
	{ SYS_RES_MEMORY, 0, RF_ACTIVE },
	// sram
	{ SYS_RES_MEMORY, 1, RF_ACTIVE },
	{ -1, 0, 0 },
};

static device_probe_t rtkit_helper_probe;
static device_attach_t rtkit_helper_attach;
static device_detach_t rtkit_helper_detach;

static int
rtkit_helper_probe(device_t dev)
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
rtkit_helper_attach(device_t dev)
{
	int rc;
	phandle node;
	struct rtkit_helper_softc *sc = device_get_softc(dev);
	sc->sc_dev = dev;

	// openbsd gets this from the implicit phandle prop, but it probably doesn't matter
	sc->sc_self = node;

	if (bus_alloc_resource(dev, rtkit_helper_res_spec, sc->sc_res) != 0) {
		device_printf(dev, "cannot allocate device resources\n");
		return ENXIO;
	}

	return 0;
}

static int
rtkit_helper_detach(device_t dev)
{
	panic("uh oh");
}

int
rtkit_helper_start(phandle_t node)
{
	// dockchannel-hid dt node has apple,helper-cpu prop that references rtkit
	// helper node which should start this, but it might be ok to call
	// rtkit_helper_start from attach
}
