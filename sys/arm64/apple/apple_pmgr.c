/*	$OpenBSD: aplpmgr.c,v 1.1 2021/12/09 11:38:27 kettenis Exp $	*/
/*
 * SPDX-License-Identifier: ISC
 *
 * Copyright (c) 2021 Mark Kettenis <kettenis@openbsd.org>
 * Copyright (c) 2022 Kyle Evans <kevans@FreeBSD.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/powerdom/powerdom.h>

#include "powerdom_if.h"

#define PMGR_PS_TARGET_MASK	0x0000000f
#define PMGR_PS_TARGET_SHIFT	0
#define PMGR_PS_ACTUAL_MASK	0x000000f0
#define PMGR_PS_ACTUAL_SHIFT	4
#define  PMGR_PS_ACTIVE		0xf
#define  PMGR_PS_CLKGATE	0x4
#define  PMGR_PS_PWRGATE	0x0

#define HREAD4(sc, reg)							\
	bus_read_4((sc)->sc_res, (reg))
#define HWRITE4(sc, reg, val)						\
	bus_write_4((sc)->sc_res, (reg), (val))

MALLOC_DEFINE(M_APLPMGR, "apple_pmgr", "Apple Power Controller");

struct apple_pmgr_softc;

struct apple_pmgr_pwrstate {
	struct apple_pmgr_softc		*ps_sc;
	powerdom_t			*ps_parents;
	size_t				ps_nparents;
	char				*ps_label;
	bus_size_t			ps_offset;
	phandle_t			ps_xref;
	int				ps_min_state;
	int				ps_width;
	bool				ps_always_on;
};

struct apple_pmgr_softc {
	device_t		sc_dev;
	struct resource		*sc_res;

	struct mtx		sc_mtx;

	struct apple_pmgr_pwrstate	*sc_pwrstate;
	int			sc_npwrstate;
};

#define	APPLE_PMGR_LOCK(sc)	mtx_lock_spin(&(sc)->sc_mtx)
#define	APPLE_PMGR_UNLOCK(sc)	mtx_unlock_spin(&(sc)->sc_mtx)
#define	APPLE_PMGR_LOCK_ASSERT(sc)	mtx_assert(&(sc)->sc_mtx, MA_OWNED)

static int	apple_pmgr_probe(device_t);
static int	apple_pmgr_attach(device_t);

static struct ofw_compat_data compat_data[] = {
	{ "apple,pmgr",		1 },
	{ NULL,				0 },
};

static int
apple_pmgr_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Apple Power Controller");

	return (BUS_PROBE_DEFAULT);
}

static int
apple_pmgr_attach(device_t dev)
{
	struct apple_pmgr_softc *sc;
	struct apple_pmgr_pwrstate *ps;
	ssize_t sz;
	pcell_t reg[2];
	phandle_t parent, node;
	int error, i, j, nstates, rid;

	nstates = rid = 0;
	sc = device_get_softc(dev);
	parent = ofw_bus_get_node(dev);
	sc->sc_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->sc_res == NULL) {
		device_printf(dev, "could not allocate memory resource\n");
		return (ENXIO);
	}

	mtx_init(&sc->sc_mtx, "aaplpmgr", NULL, MTX_SPIN);

	for (node = OF_child(parent); node; node = OF_peer(node)) {
		if (ofw_bus_node_is_compatible(node, "apple,pmgr-pwrstate"))
			sc->sc_npwrstate++;
	}

	sc->sc_pwrstate = mallocarray(sc->sc_npwrstate,
	    sizeof(*sc->sc_pwrstate), M_APLPMGR, M_WAITOK | M_ZERO);

	ps = sc->sc_pwrstate;
	for (node = OF_child(parent); node; node = OF_peer(node)) {
		if (!ofw_bus_node_is_compatible(node, "apple,pmgr-pwrstate"))
			continue;

		sz = OF_getencprop(node, "reg", reg, sizeof(reg));

		if (sz < sizeof(reg)) {
			device_printf(dev, "invalid reg property\n");
			sc->sc_npwrstate--;
			continue;
		}

		sz = OF_getproplen(node, "power-domains");
		if (sz > 0) {
			ps->ps_nparents = sz / sizeof(phandle_t);
			ps->ps_parents = mallocarray(ps->ps_nparents,
			    sizeof(*ps->ps_parents), M_APLPMGR,
			    M_WAITOK | M_ZERO);
		}

		sz = OF_getprop_alloc(node, "label", (void **)&ps->ps_label);
		if (sz <= 0) {
			/* label is a required property... */
			device_printf(dev, "error reading label\n");
			free(ps->ps_parents, M_APLPMGR);
			goto error;
		}

		nstates++;
		ps->ps_sc = sc;
		ps->ps_xref = OF_xref_from_node(node);
		ps->ps_always_on = OF_hasprop(node, "apple,always-on");

		/* XXX Auto-PM not yet supported. */
		OF_getencprop(node, "apple,min-state", &ps->ps_min_state,
		    sizeof(ps->ps_min_state));

		ps->ps_offset = reg[0];
		ps->ps_width = reg[1];
		MPASS(ps->ps_width == 4);
		OF_device_register_xref(ps->ps_xref, dev);

		ps++;
	}

	powerdom_register_ofw_provider(dev);

	/*
	 * Resolve dependencies between power domains now that we're registered
	 * as a provider.  Hopefully there's no cross-controller dependencies
	 * with one that we haven't probed yet.
	 */
	for (i = 0; i < sc->sc_npwrstate; i++) {
		ps = &sc->sc_pwrstate[i];

		/* Many will have none. */
		if (ps->ps_nparents == 0)
			continue;

		node = OF_node_from_xref(ps->ps_xref);
		for (j = 0; j < ps->ps_nparents; j++) {
			error = powerdom_get_by_ofw_idx(dev, node, j,
			    &ps->ps_parents[j]);
			if (error != 0) {
				device_printf(dev,
				    "failed to get power-domain, %s idx %d\n",
				    ps->ps_label, j);
				goto error;
			}
		}
	}

	return (0);
error:
	while (nstates > 0) {
		ps = &sc->sc_pwrstate[--nstates];

		while (ps->ps_nparents > 0) {
			powerdom_release(ps->ps_parents[--ps->ps_nparents]);
		}

		OF_prop_free(ps->ps_label);
		free(ps->ps_parents, M_APLPMGR);

		OF_device_unregister_xref(ps->ps_xref, dev);
	}

	powerdom_unregister_ofw_provider(dev);
	free(sc->sc_pwrstate, M_APLPMGR);
	bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->sc_res);

	mtx_destroy(&sc->sc_mtx);

	return (ENXIO);
}

static int
apple_pmgr_map(device_t dev, phandle_t xref, int ncells __unused,
    pcell_t *cells __unused, intptr_t *id)
{
	struct apple_pmgr_softc *sc;
	struct apple_pmgr_pwrstate *ps;
	int i;

	sc = device_get_softc(dev);
	for (i = 0; i < sc->sc_npwrstate; i++) {
		ps = &sc->sc_pwrstate[i];

		if (ps->ps_xref == xref) {
			*id = (intptr_t)ps;
			return (0);
		}
	}

	return (ENOENT);
}

static bool
apple_pmgr_state(struct apple_pmgr_softc *sc, struct apple_pmgr_pwrstate *ps)
{
	int reg;

	APPLE_PMGR_LOCK_ASSERT(sc);

	reg = HREAD4(sc, ps->ps_offset);
	reg &= PMGR_PS_ACTUAL_MASK;
	return ((reg >> PMGR_PS_ACTUAL_SHIFT) == PMGR_PS_ACTIVE);
}

static int
apple_pmgr_set(device_t dev, intptr_t id, bool on)
{
	struct apple_pmgr_softc *sc;
	struct apple_pmgr_pwrstate *ps;
	uint32_t pstate;
	uint32_t val;
	int i, timo;

	sc = device_get_softc(dev);
	ps = (struct apple_pmgr_pwrstate *)id;
	pstate = on ? PMGR_PS_ACTIVE : PMGR_PS_PWRGATE;

	if (!on && ps->ps_always_on)
		return (EPERM);
	else if (ps->ps_always_on)
		return (0);

	if (on && ps->ps_nparents > 0) {
		/*
		 * Parent may be in a different pmgr block entirely, so we can't
		 * necessarily do any nice locking here.
		 */
		for (i = 0; i < ps->ps_nparents; i++) {
			powerdom_enable(ps->ps_parents[i]);
		}
	}

	APPLE_PMGR_LOCK(sc);
	val = HREAD4(sc, ps->ps_offset);
	val &= ~PMGR_PS_TARGET_MASK;
	val |= (pstate << PMGR_PS_TARGET_SHIFT);
	HWRITE4(sc, ps->ps_offset, val);

	for (timo = 0; timo < 100; timo++) {
		if (apple_pmgr_state(sc, ps) == on)
			break;
		DELAY(1);
	}
	APPLE_PMGR_UNLOCK(sc);

	return (0);
}

static int
apple_pmgr_is_enabled(device_t dev, intptr_t id, bool *value)
{
	struct apple_pmgr_softc *sc;
	struct apple_pmgr_pwrstate *ps;

	sc = device_get_softc(dev);
	ps = (struct apple_pmgr_pwrstate *)id;

	APPLE_PMGR_LOCK(sc);
	*value = apple_pmgr_state(sc, ps);
	APPLE_PMGR_UNLOCK(sc);

	return (0);
}

static int
apple_pmgr_enable(device_t dev, intptr_t id)
{

	return (apple_pmgr_set(dev, id, true));
}

static int
apple_pmgr_disable(device_t dev, intptr_t id)
{

	return (apple_pmgr_set(dev, id, false));
}

static device_method_t apple_pmgr_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		apple_pmgr_probe),
	DEVMETHOD(device_attach,	apple_pmgr_attach),

	/* Power domain interface */
	DEVMETHOD(powerdom_map,		apple_pmgr_map),
	DEVMETHOD(powerdom_is_enabled,	apple_pmgr_is_enabled),
	DEVMETHOD(powerdom_enable,	apple_pmgr_enable),
	DEVMETHOD(powerdom_disable,	apple_pmgr_disable),

	DEVMETHOD_END
};

static driver_t apple_pmgr_driver = {
	"pmgr",
	apple_pmgr_methods,
	sizeof(struct apple_pmgr_softc),
};

/* Competing with simple_mfd */
EARLY_DRIVER_MODULE(apple_pmgr, simplebus, apple_pmgr_driver,
    0, 0, BUS_PASS_BUS + BUS_PASS_ORDER_LATE);
