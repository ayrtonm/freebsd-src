/*	$OpenBSD: aplmbox.c,v 1.2 2022/01/04 20:55:48 kettenis Exp $	*/
/*
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
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/intr.h>
#include <machine/resource.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/mbox/mbox.h>

#include "apple_mboxvar.h"

#include "mbox_if.h"

#define MBOX_A2I_CTRL		0x110
#define  MBOX_A2I_CTRL_FULL	(1 << 16)
#define MBOX_I2A_CTRL		0x114
#define  MBOX_I2A_CTRL_EMPTY	(1 << 17)
#define MBOX_A2I_SEND0		0x800
#define MBOX_A2I_SEND1		0x808
#define MBOX_I2A_RECV0		0x830
#define MBOX_I2A_RECV1		0x838

#define HREAD4(sc, reg)							\
	(bus_read_4((sc)->sc_mem_res, (reg)))
#define HREAD8(sc, reg)							\
	(bus_read_8((sc)->sc_mem_res, (reg)))
#define HWRITE4(sc, reg, val)						\
	bus_write_4((sc)->sc_mem_res, (reg), (val))
#define HWRITE8(sc, reg, val)						\
	bus_write_8((sc)->sc_mem_res, (reg), (val))

struct apple_mbox_softc {
	device_t		sc_dev;
	struct resource		*sc_mem_res;
	struct resource		*sc_irq_res;

	void			*sc_intrhand;
	mbox_rx_fn		*sc_rx_callback;
	void			*sc_rx_arg;

	struct mtx		sc_mtx;
};

static int	apple_mbox_probe(device_t dev);
static int	apple_mbox_attach(device_t dev);

static struct ofw_compat_data compat_data[] = {
	{ "apple,asc-mailbox-v4",	1 },
	{ NULL, 0 },
};

static void	apple_mbox_intr(void *);
static int	apple_mbox_setup_channel(device_t, int, mbox_rx_fn *, void *);
static int	apple_mbox_write(device_t, int, const void *, size_t);
static int	apple_mbox_read(device_t, int, void *, size_t);

static int
apple_mbox_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Apple Mailbox");
	return (0);
}

static int
apple_mbox_attach(device_t dev)
{
	struct apple_mbox_softc *sc;
	phandle_t node;
	int error, rid;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;

	rid = 0;
	node = ofw_bus_get_node(dev);
	sc->sc_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->sc_mem_res == NULL) {
		device_printf(dev, "cannot map regs\n");
		return (ENXIO);
	}

	error = ofw_bus_find_string_index(node, "interrupt-names",
	    "recv-not-empty", &rid);
	if (error == 0) {
		sc->sc_irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
		    RF_ACTIVE);
		if (sc->sc_irq_res == NULL) {
			device_printf(dev, "cannot allocate recv irq\n");
			error = ENXIO;
			goto out;
		}
	}

	mtx_init(&sc->sc_mtx, "apple_mbox", NULL, MTX_DEF);

	error = bus_setup_intr(dev, sc->sc_irq_res,
	    INTR_MPSAFE | INTR_TYPE_MISC, NULL, apple_mbox_intr, sc,
	    &sc->sc_intrhand);

	mbox_register_ofw_provider(dev);
out:
	if (error != 0) {
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->sc_mem_res);
		if (sc->sc_irq_res != NULL)
			bus_release_resource(dev, SYS_RES_IRQ,
			    rman_get_rid(sc->sc_irq_res), sc->sc_irq_res);
	}

	return (error);
}

static void
apple_mbox_intr(void *arg)
{
	struct apple_mbox_softc *sc = arg;
	uint32_t ctrl;

	mtx_lock(&sc->sc_mtx);
	ctrl = HREAD4(sc, MBOX_I2A_CTRL);
	if (ctrl & MBOX_I2A_CTRL_EMPTY)
		return;

	if (sc->sc_rx_callback) {
		(*sc->sc_rx_callback)(sc->sc_rx_arg, -1);
	} else {
		device_printf(sc->sc_dev, "0x%016jx 0x%016jx\n",
		    HREAD8(sc, MBOX_I2A_RECV0), HREAD8(sc, MBOX_I2A_RECV1));
	}
	mtx_unlock(&sc->sc_mtx);

	return;
}

static int
apple_mbox_setup_channel(device_t dev, int channel, mbox_rx_fn *rx_callback,
    void *rx_data)
{
	struct apple_mbox_softc *sc = device_get_softc(dev);

	/* XXX LOCKING */
	if (channel != -1)
		return (EINVAL);

	sc->sc_rx_callback = rx_callback;
	sc->sc_rx_arg = rx_data;

	return (0);
}

static int
apple_mbox_write(device_t dev, int channel, const void *data, size_t datasz)
{
	struct apple_mbox_softc *sc = device_get_softc(dev);
	const struct apple_mbox_msg *msg = data;
	uint32_t ctrl;

	/* XXX LOCKING */
	if (channel != -1)
		return (EINVAL);
	if (datasz != sizeof(struct apple_mbox_msg))
		return (EINVAL);

	ctrl = HREAD4(sc, MBOX_A2I_CTRL);
	if (ctrl & MBOX_A2I_CTRL_FULL)
		return (EBUSY);

	HWRITE8(sc, MBOX_A2I_SEND0, msg->data0);
	HWRITE8(sc, MBOX_A2I_SEND1, msg->data1);

	return (0);
}

static int
apple_mbox_read(device_t dev, int channel, void *data, size_t datasz)
{
	struct apple_mbox_softc *sc = device_get_softc(dev);
	struct apple_mbox_msg *msg = data;
	uint32_t ctrl;

	/* XXX LOCKING */
	if (channel != -1)
		return (EINVAL);
	if (datasz != sizeof(struct apple_mbox_msg))
		return (EINVAL);

	ctrl = HREAD4(sc, MBOX_I2A_CTRL);
	if (ctrl & MBOX_I2A_CTRL_EMPTY)
		return (EAGAIN);

	msg->data0 = HREAD8(sc, MBOX_I2A_RECV0);
	msg->data1 = HREAD8(sc, MBOX_I2A_RECV1);

	return (0);
}

static device_method_t apple_mbox_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		apple_mbox_probe),
	DEVMETHOD(device_attach,	apple_mbox_attach),

	/* Mbox interface */
	DEVMETHOD(mbox_setup_channel,	apple_mbox_setup_channel),
	DEVMETHOD(mbox_read,		apple_mbox_read),
	DEVMETHOD(mbox_write,		apple_mbox_write),

	DEVMETHOD_END
};

static driver_t apple_mbox_driver = {
	"mbox",
	apple_mbox_methods,
	sizeof(struct apple_mbox_softc),
};

DRIVER_MODULE(apple_mbox, simplebus, apple_mbox_driver, 0, 0);
