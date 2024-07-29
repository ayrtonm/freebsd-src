/*-
 * Copyright (c) 2022 Kyle Evans <kevans@FreeBSD.org>
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

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <machine/bus.h>

#define SART2_CONFIG(idx)	(0x0000 + 4 * (idx))
#define  SART2_CONFIG_FLAGS_MASK	0xff000000
#define  SART2_CONFIG_FLAGS_ALLOW	0xff000000
#define SART2_ADDR(idx)		(0x0040 + 4 * (idx))

#define SART3_CONFIG(idx)	(0x0000 + 4 * (idx))
#define  SART3_CONFIG_FLAGS_MASK	0x000000ff
#define  SART3_CONFIG_FLAGS_ALLOW	0x000000ff
#define SART3_ADDR(idx)		(0x0040 + 4 * (idx))
#define SART3_SIZE(idx)		(0x0080 + 4 * (idx))

#define SART_NUM_ENTRIES	16
#define SART_ADDR_SHIFT		12
#define SART_SIZE_SHIFT		12

#define HREAD4(sc, reg)							\
	(bus_read_4((sc)->res, (reg)))
#define HWRITE4(sc, reg, val)						\
	bus_write_4((sc)->res, (reg), (val))

/* XXX */
int apple_sart_map(phandle_t node, bus_addr_t addr, bus_size_t size);
int apple_sart_unmap(phandle_t node, bus_addr_t addr, bus_size_t size);

struct apple_sart_softc;

typedef int (apple_sart_map_fn)(struct apple_sart_softc *, bus_addr_t,
    bus_size_t);

struct apple_sart_cfg {
	apple_sart_map_fn	*cfg_map;
	apple_sart_map_fn	*cfg_unmap;
};

struct apple_sart_softc {
	struct mtx			 mtx;
	device_t			 dev;
	struct resource			*res;
	const struct apple_sart_cfg	*cfg;
};

#define	SART_LOCK(sc)		mtx_lock(&(sc)->mtx)
#define	SART_UNLOCK(sc)		mtx_unlock(&(sc)->mtx)
#define	SART_LOCK_ASSERT(sc)	mtx_assert(&(sc)->mtx, MA_OWNED)

static apple_sart_map_fn	apple_sart2_map;
static apple_sart_map_fn	apple_sart2_unmap;
static apple_sart_map_fn	apple_sart3_map;
static apple_sart_map_fn	apple_sart3_unmap;

static const struct apple_sart_cfg apple_sart2_cfg = {
	.cfg_map = &apple_sart2_map,
	.cfg_unmap = &apple_sart2_unmap,
};

static const struct apple_sart_cfg apple_sart3_cfg = {
	.cfg_map = &apple_sart3_map,
	.cfg_unmap = &apple_sart3_unmap,
};

static struct ofw_compat_data compat_data[] = {
	{ "apple,t8103-sart", (uintptr_t)&apple_sart2_cfg },
	{ "apple,t6000-sart", (uintptr_t)&apple_sart3_cfg },
	{ NULL, 0 },
};

static int
apple_sart_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Apple SART");
	return (BUS_PROBE_DEFAULT);
}

static int
apple_sart_attach(device_t dev)
{
	struct apple_sart_softc *sc;
	uintptr_t cfg_addr;
	phandle_t phandle;
	int rid;

	sc = device_get_softc(dev);
	rid = 0;
	sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->res == NULL) {
		device_printf(dev, "cannot allocate memory resource\n");
		return (ENXIO);
	}

	cfg_addr = ofw_bus_search_compatible(dev, compat_data)->ocd_data;
	MPASS(cfg_addr != 0);

	sc->cfg = (const struct apple_sart_cfg *)cfg_addr;

	mtx_init(&sc->mtx, "apple_sart", NULL, MTX_DEF);
	/* XXX power_domain_enable_all */

	phandle = ofw_bus_get_node(dev);
	OF_device_register_xref(OF_xref_from_node(phandle), dev);

	return (0);
}

static int
apple_sart2_map(struct apple_sart_softc *sc, bus_addr_t addr, bus_size_t size)
{
	uint32_t cfg;

	SART_LOCK_ASSERT(sc);
	for (int i = 0; i < SART_NUM_ENTRIES; i++) {
		cfg = HREAD4(sc, SART2_CONFIG(i));
		if ((cfg & SART2_CONFIG_FLAGS_MASK) != 0)
			continue;

		HWRITE4(sc, SART2_ADDR(i), addr >> SART_ADDR_SHIFT);
		HWRITE4(sc, SART2_CONFIG(i),
		    (size >> SART_SIZE_SHIFT) | SART2_CONFIG_FLAGS_ALLOW);

		return (0);
	}

	return (ENOENT);
}

static int
apple_sart2_unmap(struct apple_sart_softc *sc, bus_addr_t addr, bus_size_t size)
{
	SART_LOCK_ASSERT(sc);
	for (int i = 0; i < SART_NUM_ENTRIES; i++) {
		if (HREAD4(sc, SART2_ADDR(i)) != (addr >> SART_ADDR_SHIFT))
			continue;

		HWRITE4(sc, SART2_ADDR(i), 0);
		HWRITE4(sc, SART2_CONFIG(i), 0);

		return (0);
	}

	return (ENOENT);
}

static int
apple_sart3_map(struct apple_sart_softc *sc, bus_addr_t addr, bus_size_t size)
{
	uint32_t cfg;

	SART_LOCK_ASSERT(sc);
	for (int i = 0; i < SART_NUM_ENTRIES; i++) {
		cfg = HREAD4(sc, SART3_CONFIG(i));
		if ((cfg & SART3_CONFIG_FLAGS_MASK) != 0)
			continue;

		HWRITE4(sc, SART3_ADDR(i), addr >> SART_ADDR_SHIFT);
		HWRITE4(sc, SART3_SIZE(i), size >> SART_SIZE_SHIFT);
		HWRITE4(sc, SART3_CONFIG(i), SART3_CONFIG_FLAGS_ALLOW);

		return (0);
	}

	return (ENOENT);
}

static int
apple_sart3_unmap(struct apple_sart_softc *sc, bus_addr_t addr, bus_size_t size)
{
	SART_LOCK_ASSERT(sc);
	for (int i = 0; i < SART_NUM_ENTRIES; i++) {
		if (HREAD4(sc, SART3_ADDR(i)) != (addr >> SART_ADDR_SHIFT))
			continue;

		HWRITE4(sc, SART3_ADDR(i), 0);
		HWRITE4(sc, SART3_SIZE(i), 0);
		HWRITE4(sc, SART3_CONFIG(i), 0);

		return (0);
	}

	return (ENOENT);
}

int
apple_sart_map(phandle_t node, bus_addr_t addr, bus_size_t size)
{
	struct apple_sart_softc *sc;
	device_t dev;
	int error;

	dev = OF_device_from_xref(node);
	if (dev == NULL)
		return (ENXIO);
	sc = device_get_softc(dev);

	SART_LOCK(sc);
	error = (*sc->cfg->cfg_map)(sc, addr, size);
	SART_UNLOCK(sc);
	return (error);
}

int
apple_sart_unmap(phandle_t node, bus_addr_t addr, bus_size_t size)
{
	struct apple_sart_softc *sc;
	device_t dev;
	int error;

	dev = OF_device_from_xref(node);
	if (dev == NULL)
		return (ENXIO);
	sc = device_get_softc(dev);

	SART_LOCK(sc);
	error = (*sc->cfg->cfg_unmap)(sc, addr, size);
	SART_UNLOCK(sc);
	return (error);
}

static device_method_t apple_sart_methods[] = {
	DEVMETHOD(device_probe, apple_sart_probe),
	DEVMETHOD(device_attach, apple_sart_attach),

	DEVMETHOD_END
};

static driver_t apple_sart_driver = {
	"apple_sart",
	apple_sart_methods,
	sizeof(struct apple_sart_softc),
};

DRIVER_MODULE(apple_sart, simplebus, apple_sart_driver, 0, 0);
