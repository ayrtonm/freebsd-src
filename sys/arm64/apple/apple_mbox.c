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

#include "apple_mbox.h"

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
	apple_mbox_rx	sc_rx_cb;
	void			*sc_rx_arg;

	struct mtx		sc_tx_mtx;
};

static int	apple_mbox_probe(device_t dev);
static int	apple_mbox_attach(device_t dev);

static struct ofw_compat_data compat_data[] = {
	{ "apple,asc-mailbox-v4",	1 },
	{ NULL, 0 },
};

static void	apple_mbox_intr(void *);

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
	phandle_t node, xref;
	int error, rid;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;

	rid = 0;
	node = ofw_bus_get_node(dev);

	sc->sc_mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->sc_mem_res == NULL) {
		device_printf(dev, "cannot map regs\n");
		return ENXIO;
	}

	error = ofw_bus_find_string_index(node, "interrupt-names",
	    "recv-not-empty", &rid);
	if (error != 0) {
		error = ENXIO;
		goto err_find_recv_irq;
	}

	sc->sc_irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid, RF_ACTIVE);
	if (sc->sc_irq_res == NULL) {
		error = ENXIO;
		goto err_alloc_irq;
	}

	error = bus_setup_intr(dev, sc->sc_irq_res,
		INTR_MPSAFE | INTR_TYPE_MISC, NULL, apple_mbox_intr, sc,
		&sc->sc_intrhand);
	if (error != 0) {
		goto err_setup_intr;
	}

	xref = OF_xref_from_node(node);
	OF_device_register_xref(xref, dev);

	// TODO: witness code might not like this name when using more than one mbox
	mtx_init(&sc->sc_tx_mtx, "apple mbox tx", NULL, MTX_SPIN);

	return 0;

err_setup_intr:
	bus_release_resource(dev, SYS_RES_IRQ, rid, sc->sc_irq_res);
err_alloc_irq:
err_find_recv_irq:
	bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->sc_mem_res);
	return (error);
}

static void
apple_mbox_intr(void *arg)
{
	struct apple_mbox_softc *sc = arg;
	struct apple_mbox_msg msg;

	// TODO: Add filter to mask mailbox interrupt
	while ((HREAD4(sc, MBOX_I2A_CTRL) & MBOX_I2A_CTRL_EMPTY) == 0) {
		msg.data0 = HREAD8(sc, MBOX_I2A_RECV0);
		msg.data1 = HREAD8(sc, MBOX_I2A_RECV1);

		if (sc->sc_rx_cb) {
			// TODO: what to do when this returns ENOMEM?
			(sc->sc_rx_cb)(sc->sc_rx_arg, msg);
		} else {
			device_printf(sc->sc_dev,
				"Received RTKit msg w/o callback installed\n");
		}
	}

	// TODO: unmask mailbox interrupt here
	return;
}

static int
apple_mbox_write_locked(device_t dev, const struct apple_mbox_msg *msg)
{
	struct apple_mbox_softc *sc = device_get_softc(dev);
	uint32_t ctrl;

	ctrl = HREAD4(sc, MBOX_A2I_CTRL);
	if (ctrl & MBOX_A2I_CTRL_FULL)
		return (EBUSY);

	HWRITE8(sc, MBOX_A2I_SEND0, msg->data0);
	HWRITE8(sc, MBOX_A2I_SEND1, msg->data1);

	return (0);
}

int
apple_mbox_write(struct apple_mbox *mbox, const struct apple_mbox_msg *msg)
{
	int rc;
	device_t dev = mbox->dev;
	struct apple_mbox_softc *sc = device_get_softc(dev);

	mtx_lock_spin(&sc->sc_tx_mtx);

	rc = apple_mbox_write_locked(dev, msg);

	mtx_unlock_spin(&sc->sc_tx_mtx);

	return rc;
}
//
//static int
//apple_mbox_read(device_t dev, int channel, void *data, size_t datasz)
//{
//	struct apple_mbox_softc *sc = device_get_softc(dev);
//	struct apple_mbox_msg *msg = data;
//	uint32_t ctrl;
//
//	/* XXX LOCKING */
//	if (channel != -1)
//		return (EINVAL);
//	if (datasz != sizeof(struct apple_mbox_msg))
//		return (EINVAL);
//
//	ctrl = HREAD4(sc, MBOX_I2A_CTRL);
//	if (ctrl & MBOX_I2A_CTRL_EMPTY)
//		return (EAGAIN);
//
//	msg->data0 = HREAD8(sc, MBOX_I2A_RECV0);
//	msg->data1 = HREAD8(sc, MBOX_I2A_RECV1);
//
//	return (0);
//}

static device_method_t apple_mbox_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		apple_mbox_probe),
	DEVMETHOD(device_attach,	apple_mbox_attach),

	DEVMETHOD_END
};

static driver_t apple_mbox_driver = {
	"mbox",
	apple_mbox_methods,
	sizeof(struct apple_mbox_softc),
};

EARLY_DRIVER_MODULE(apple_mbox, simplebus, apple_mbox_driver, 0, 0, BUS_PASS_DEFAULT-3);

int
apple_mbox_get(device_t client_dev, device_t *mboxp)
{
	int error;
	phandle_t client_node, mbox_node;

	if (mboxp == NULL)
		return EINVAL;

	client_node = ofw_bus_get_node(client_dev);
	error = OF_getencprop(client_node, "mboxes", &mbox_node,
		sizeof(mbox_node));

	if (error != sizeof(mbox_node))
		return error;

	*mboxp = OF_device_from_xref(mbox_node);
	if (*mboxp == NULL)
		return ENODEV;

	return 0;
}

void
apple_mbox_set_rx(struct apple_mbox *mbox, apple_mbox_rx rx_cb, void *rx_arg)
{
	struct apple_mbox_softc *sc;
	sc = device_get_softc(mbox->dev);
	sc->sc_rx_cb = rx_cb;
	sc->sc_rx_arg = rx_arg;
}
