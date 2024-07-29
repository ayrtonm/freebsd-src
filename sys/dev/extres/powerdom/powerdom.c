/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright 2016 Michal Meloun <mmel@FreeBSD.org>
 * All rights reserved.
 *
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
 *
 * $FreeBSD$
 */
#include "opt_platform.h"
#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/kobj.h>
#include <sys/malloc.h>
#include <sys/systm.h>

#ifdef FDT
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#endif

#include <dev/extres/powerdom/powerdom.h>

#include "powerdom_if.h"

struct powerdom {
	device_t	consumer_dev;		/* consumer device */
	device_t	provider_dev;		/* provider device */
	intptr_t	pd_id;			/* power domain id */
};

MALLOC_DEFINE(M_POWERDOM, "powerdom", "Power domain framework");

int
powerdom_enable(powerdom_t pd)
{

	return (POWERDOM_ENABLE(pd->provider_dev, pd->pd_id));
}

int
powerdom_disable(powerdom_t pd)
{

	return (POWERDOM_DISABLE(pd->provider_dev, pd->pd_id));
}

int
powerdom_is_enabled(powerdom_t pd, bool *value)
{

	return (POWERDOM_IS_ENABLED(pd->provider_dev, pd->pd_id, value));
}

void
powerdom_release(powerdom_t pd)
{

	free(pd, M_POWERDOM);
}

int
powerdom_get_by_id(device_t consumer_dev, device_t provider_dev, intptr_t id,
    powerdom_t *pd_out)
{
	powerdom_t pd;

	/* Create handle */
	pd = malloc(sizeof(struct powerdom), M_POWERDOM,
	    M_WAITOK | M_ZERO);
	pd->consumer_dev = consumer_dev;
	pd->provider_dev = provider_dev;
	pd->pd_id = id;
	*pd_out = pd;
	return (0);
}

#ifdef FDT
int
powerdom_default_ofw_map(device_t provider_dev, phandle_t xref, int ncells,
    pcell_t *cells, intptr_t *id)
{
	if (ncells == 0)
		*id = 1;
	else if (ncells == 1)
		*id = cells[0];
	else
		return  (ERANGE);

	return (0);
}

int
powerdom_get_by_ofw_idx(device_t consumer_dev, phandle_t cnode, int idx,
    powerdom_t *pd)
{
	phandle_t xnode;
	pcell_t *cells;
	device_t pddev;
	int ncells, rv;
	intptr_t id;

	if (cnode <= 0)
		cnode = ofw_bus_get_node(consumer_dev);
	if (cnode <= 0) {
		device_printf(consumer_dev,
		    "%s called on not ofw based device\n", __func__);
		return (ENXIO);
	}

	rv = ofw_bus_parse_xref_list_alloc(cnode, "power-domains",
	    "#power-domain-cells",
	    idx, &xnode, &ncells, &cells);
	if (rv != 0)
		return (rv);

	/* Tranlate provider to device */
	pddev = OF_device_from_xref(xnode);
	if (pddev == NULL) {
		OF_prop_free(cells);
		return (ENODEV);
	}
	/* Map power domain to number */
	rv = POWERDOM_MAP(pddev, xnode, ncells, cells, &id);
	OF_prop_free(cells);
	if (rv != 0)
		return (rv);

	return (powerdom_get_by_id(consumer_dev, pddev, id, pd));
}

int
powerdom_get_by_ofw_name(device_t consumer_dev, phandle_t cnode, char *name,
    powerdom_t *pd)
{
	int rv, idx;

	if (cnode <= 0)
		cnode = ofw_bus_get_node(consumer_dev);
	if (cnode <= 0) {
		device_printf(consumer_dev,
		    "%s called on not ofw based device\n",  __func__);
		return (ENXIO);
	}
	rv = ofw_bus_find_string_index(cnode, "power-domain-names", name, &idx);
	if (rv != 0)
		return (rv);
	return (powerdom_get_by_ofw_idx(consumer_dev, cnode, idx, pd));
}

void
powerdom_register_ofw_provider(device_t provider_dev)
{
	phandle_t xref, node;

	node = ofw_bus_get_node(provider_dev);
	if (node <= 0)
		panic("%s called on not ofw based device.\n", __func__);

	xref = OF_xref_from_node(node);
	OF_device_register_xref(xref, provider_dev);
}

void
powerdom_unregister_ofw_provider(device_t provider_dev)
{
	phandle_t xref;

	xref = OF_xref_from_device(provider_dev);
	OF_device_unregister_xref(xref, provider_dev);
}
#endif
