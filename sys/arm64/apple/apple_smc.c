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
#include <sys/gpio.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/fdt/simplebus.h>

#include <dev/gpio/gpiobusvar.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <arm64/apple/rtkit.h>

#include "gpio_if.h"

#define	SMC_EP	0x20

#define	SMC_CMD(d)	((d) & 0xff)
#define	SMC_READ_KEY		0x10
#define	SMC_WRITE_KEY		0x11
#define	SMC_MSG_INIT		0x17
#define	SMC_NOTIFICATION	0x18

#define	SMC_GPIO_CMD_OUTPUT	(1 << 24)

#define	SMC_KEY(s)	((s[0] << 24) | (s[1] << 16) | (s[2] << 8) | s[3])

struct apple_smc_softc {
	struct simplebus_softc simplebus_sc; /* base class */
	device_t sc_gpiobus;

	struct resource *sc_sram;

	struct rtkit_state	*sc_rtkit_state;
	struct intr_config_hook	sc_config_hook;

	uint8_t sc_msgid;
	uint64_t sc_data;
};

static device_probe_t apple_smc_probe;
static device_attach_t apple_smc_attach;
static device_detach_t apple_smc_detach;
static void apple_smc_start_config_hook(void *arg);
static int apple_smc_write_key(struct apple_smc_softc *sc, uint32_t key,
    void *data, size_t len);

static int
apple_smc_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev)) {
		return (ENXIO);
	}

	if (!ofw_bus_is_compatible(dev, "apple,smc")) {
		return (ENXIO);
	}

	device_set_desc(dev, "Apple SMC");
	return (BUS_PROBE_DEFAULT);
}

static int
apple_smc_attach(device_t dev)
{
	struct apple_smc_softc *sc;
	int error;

	sc = device_get_softc(dev);

	sc->simplebus_sc.flags |= SB_FLAG_NO_RANGES;

	error = simplebus_attach(dev);
	if (error != 0) {
		device_printf(dev, "simplebus_attach failed %d\n", error);
		return (ENXIO);
	}

	//apple_smc_send_cmd(sc, SMC_MSG_INIT, 0, 0);
	//error = ofw_bus_find_string_index(node, "reg-names", "sram", &rid);
	//if (error != 0) {
	//	device_printf(dev, "cannot find 'sram' reg\n");
	//	return (ENXIO);
	//}
	//sc->sc_sram = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	//if (sc->sc_sram == NULL) {
	//	device_printf(dev, "cannot allocate 'sram' reg\n");
	//	return (ENXIO);
	//}

	sc->sc_config_hook.ich_func = apple_smc_start_config_hook;
	sc->sc_config_hook.ich_arg = sc;

	if (config_intrhook_establish(&sc->sc_config_hook) != 0) {
		return (ENOMEM);
	}

	return (0);
}

static int
apple_smc_detach(device_t dev)
{
	panic("not yet");
}

static int
apple_smc_send_cmd(struct apple_smc_softc *sc, uint16_t cmd, uint32_t key,
    uint16_t len)
{
	uint64_t data;

	data = cmd;
	data |= (uint64_t)len << 16;
	data |= (uint64_t)key << 32;
	data |= (sc->sc_msgid++ & 0xf) << 12;

	return rtkit_send_endpoint(sc->sc_rtkit_state, SMC_EP, data);
}

static void
apple_smc_handle_notification(struct apple_smc_softc *sc, uint64_t data)
{
}

static void
apple_smc_callback(void *arg, uint64_t data)
{
	struct apple_smc_softc *sc = arg;
	if (SMC_CMD(data) == SMC_NOTIFICATION) {
		apple_smc_handle_notification(sc, data);
		return;
	}
	sc->sc_data = data;
	wakeup(&sc->sc_data);
}

extern struct bus_space memmap_bus;

static void
apple_smc_start_config_hook(void *arg)
{
	struct apple_smc_softc *sc = arg;
	int error;
	device_t dev = sc->simplebus_sc.dev;
	phandle_t node, self;

	node = ofw_bus_get_node(dev);

	error = rtkit_init(dev, &sc->sc_rtkit_state, true);
	if (error != 0) {
		device_printf(dev, "error initializing RTKit %d\n", error);
		panic("uh oh");
	}


	rtkit_boot(sc->sc_rtkit_state);
	error = rtkit_start_endpoint(sc->sc_rtkit_state, SMC_EP, apple_smc_callback, sc);
	if (error != 0) {
		device_printf(dev, "failed to start SMC RTKit endpoint %d\n", error);
		panic("uh oh");
	}
	error = apple_smc_send_cmd(sc, SMC_MSG_INIT, 0, 0);
	MPASS(error == 0);
	device_printf(dev, "waiting up to 5s for command response\n");
	tsleep(&sc->sc_data, PWAIT, "apple,smc", 5 * hz);
	device_printf(dev, "got command response %lx\n", sc->sc_data);

	int rid = 1;
	sc->sc_sram = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE | RF_UNMAPPED);
	MPASS(sc->sc_sram != NULL);

	bus_space_handle_t handle;
	rman_set_bustag(sc->sc_sram, &memmap_bus);
	error = bus_space_map(&memmap_bus, sc->sc_data, 0x4000, BUS_SPACE_MAP_NONPOSTED, &handle);
	rman_set_bushandle(sc->sc_sram, handle);
	MPASS(error == 0);

	for (node = OF_child(node); node > 0; node = OF_peer(node)) {
		if (!OF_hasprop(node, "gpio-controller")) {
			continue;
		}
		error = OF_getencprop(node, "phandle", &self, sizeof(self));
		MPASS(error == sizeof(self));
		device_printf(dev, "registering phandle %x\n", self);
		OF_device_register_xref(self, dev);

		sc->sc_gpiobus = gpiobus_attach_bus(dev);
		MPASS(sc->sc_gpiobus != NULL);
	}

	device_printf(dev, "enabling notifications\n");
	uint8_t ntap = 1;
	error = apple_smc_write_key(sc, SMC_KEY("NTAP"), &ntap, sizeof(ntap));
	MPASS(error == 0);
	device_printf(dev, "enabled notifications\n");
	config_intrhook_disestablish(&sc->sc_config_hook);
}

static device_t
apple_smc_get_bus(device_t dev)
{
	struct apple_smc_softc *sc;

	sc = device_get_softc(dev);

	return (sc->sc_gpiobus);
}

static int
apple_smc_write_key(struct apple_smc_softc *sc, uint32_t key, void *data,
    size_t len)
{
	bus_write_region_1(sc->sc_sram, 0, data, len);
	bus_barrier(sc->sc_sram, 0, len, BUS_SPACE_BARRIER_WRITE);
	return apple_smc_send_cmd(sc, SMC_WRITE_KEY, key, len);
}

int apple_smc_pin_set(device_t dev, uint32_t pin, uint32_t val);

int
apple_smc_pin_set(device_t dev, uint32_t pin, uint32_t val)
{
	struct apple_smc_softc *sc;
	static char *digits = "0123456789abcdef";
	uint32_t data;
	uint32_t key;
	int error;

	sc = device_get_softc(dev);
	key = SMC_KEY("gP\0\0");
	key |= (digits[(pin >> 0) & 0xf] << 0);
	key |= (digits[(pin >> 4) & 0xf] << 8);

	//assume active low
	val = !val;

	data = SMC_GPIO_CMD_OUTPUT | !!val;

	error = apple_smc_write_key(sc, key, &data, sizeof(data));
	MPASS(error == 0);

	return (error);
}


static device_method_t apple_smc_methods[] = {
	DEVMETHOD(device_probe, apple_smc_probe),
	DEVMETHOD(device_attach, apple_smc_attach),
	DEVMETHOD(device_detach, apple_smc_detach),

	DEVMETHOD(gpio_get_bus, apple_smc_get_bus),
	//DEVMETHOD(gpio_pin_get, apple_smc_pin_get),
	DEVMETHOD(gpio_pin_set, apple_smc_pin_set),
	//DEVMETHOD(gpio_foo, apple_smc_foo),

	DEVMETHOD_END
};

DEFINE_CLASS_1(apple_smc, apple_smc_driver, apple_smc_methods,
    sizeof(struct apple_smc_softc), simplebus_driver);

DRIVER_MODULE(apple_smc, simplebus, apple_smc_driver, 0, 0);
