/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
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
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/proc.h>
#include <sys/rman.h>
#include <sys/smp.h>
#include <sys/taskqueue.h>

#include <machine/bus.h>
#include <machine/machdep.h>
#ifdef SMP
#include <machine/intr.h>
#include <machine/smp.h>
#endif

#include <dev/evdev/input.h>
#include <dev/fdt/fdt_intr.h>
#include <dev/fdt/simplebus.h>

#include <dev/hid/hid.h>
#include <dev/hid/hidquirk.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <arm64/apple/apple_mbox.h>
#include <arm64/apple/rtkit.h>

#include "dockchannel.h"
#include "hid_if.h"

#define	CONFIG_TX_THRESH	0x0
#define	CONFIG_RX_THRESH	0x4

#define	DATA_TX8		0x4
#define	DATA_TX16		0x8
#define	DATA_TX24		0xc
#define	DATA_TX32		0x10
#define	DATA_TX_FREE	0x14
#define	DATA_RX8		0x1c
#define	DATA_RX16		0x20
#define	DATA_RX24		0x24
#define	DATA_RX32		0x28
#define	DATA_RX_COUNT	0x2c

#define	DATA_RX8_DATA(d)	(((d) >> 8) & 0xff)

struct mtp_hdr {
	uint8_t hdr_len;
	uint8_t chan;
#define	MTP_CHAN_CMD	0x11
#define	MTP_CHAN_REPORT	0x12
	uint16_t pkt_len;
	uint8_t seq;
	uint8_t iface;
#define	MTP_IFACE_COMM		0
	uint16_t pad;
} __packed;

struct mtp_subhdr {
	uint8_t flags;
#define	MTP_GROUP_SHIFT		6
#define	MTP_GROUP(x)		(((x) >> 6) & 0x3)
#define	MTP_GROUP_INPUT		0
#define	MTP_GROUP_OUTPUT	1
#define	MTP_GROUP_CMD		2
#define	MTP_REQ(x)			((x) & 0x3f)
#define	MTP_REQ_SHIFT		0
#define	MTP_REQ_SET_REPORT	0
#define	MTP_REQ_GET_REPORT	1
	uint8_t unk;
	uint16_t len;
	uint32_t retcode;
} __packed;

struct mtp_init_hdr {
	uint8_t type;
#define	MTP_EVENT_GPIO_CMD	0xa0
#define	MTP_EVENT_INIT		0xf0
#define	MTP_EVENT_READY		0xf1
	uint8_t unk0;
	uint8_t unk1;
	uint8_t iface;
	char name[16];
} __packed;

struct mtp_init_block_hdr {
	uint16_t type;
#define	MTP_BLOCK_DESCRIPTOR	0
#define	MTP_BLOCK_GPIO_REQ		1
#define	MTP_BLOCK_END			2
	uint16_t subtype;
	uint16_t len;
} __packed;

struct mtp_gpio_req {
	uint16_t unk;
	uint16_t id;
	char name[32];
} __packed;

struct mtp_gpio_cmd {
	uint8_t type;
	uint8_t iface;
	uint8_t gpio;
	uint8_t unk;
	uint8_t cmd;
} __packed;

struct mtp_gpio_ack {
	uint8_t type;
	uint32_t retcode;
	uint8_t cmd[512];
} __packed;

#define	MTP_CMD_IFACE_RESET		0x40
#define	MTP_CMD_SEND_FIRMWARE	0x95
#define	MTP_CMD_IFACE_ENABLE	0xb4
#define	MTP_CMD_ACK_GPIO_CMD	0xa1
#define	MTP_CMD_GET_DIMENSIONS	0xd9

#define	STM_REPORT_ID		0x10
#define	STM_REPORT_SERIAL	0x11

struct dockchannel_hid_stm_id {
	uint8_t unk0;
	uint16_t vendor_id;
	uint16_t product_id;
	uint16_t version;
	uint16_t unk1;
	uint8_t keyboard_type;
	uint8_t serial_len;
} __packed;

#define	HID_DESC_MAX	512

struct dockchannel_hid_iface {
	uint8_t		iface;
	uint8_t		desc[HID_DESC_MAX];
	size_t		desc_len;
	bool		ready;
	uint8_t		seq;
	uint8_t		retseq;
	uint32_t	retcode;
	phandle_t	smc;
	phandle_t	gpio;
};

struct dockchannel_hid_child {
	device_t			hidbus;
	struct hid_device_info	hw;
	hid_intr_t			*intr_handler;
	void				*intr_ctx;
};

struct dockchannel_hid_softc {
	device_t		sc_dev;

	phandle_t		sc_node;
	phandle_t		sc_mtp;
	uint32_t		sc_fifo_size;

	struct resource *sc_rx_irq;
	struct resource *sc_tx_irq;
	struct resource *sc_config;
	struct resource *sc_data;

	void			*sc_rx_intrhand;
	void			*sc_tx_intrhand;

	bool			sc_rx_avail;
	bool			sc_tx_avail;
	struct task		sc_task;

	struct dockchannel_hid_iface	sc_comm;
	struct dockchannel_hid_iface	sc_stm;
	struct dockchannel_hid_iface	sc_kbd;
	struct dockchannel_hid_iface	sc_mt;

	struct dockchannel_hid_child	sc_kbd_hid;
	struct dockchannel_hid_child	sc_mt_hid;

	struct dockchannel_hid_stm_id	sc_stm_id;
	char	sc_serial[64];
};

static device_probe_t dockchannel_hid_probe;
static device_attach_t dockchannel_hid_attach;
static device_detach_t dockchannel_hid_detach;

static void dockchannel_hid_start_config_hook(void *arg);

/* Helper functions */
static int dockchannel_get_prop(struct dockchannel_hid_softc *sc,
    const char *name, uint32_t *propp, size_t len);
static int dockchannel_alloc_res(struct dockchannel_hid_softc *sc, int type,
    const char *name, struct resource **resp);

/* irqs */
static int dockchannel_hid_tx_filter(void *arg);
static int dockchannel_hid_rx_filter(void *arg);
static void dockchannel_hid_task(void *arg, int pending);

/* iface */
static void dockchannel_hid_iface_enable(struct dockchannel_hid_softc *sc,
    const struct dockchannel_hid_iface *dcif);

/* hidbus */
static int dockchannel_hid_add_child(struct dockchannel_hid_softc *sc,
    struct dockchannel_hid_child *child);

static void dockchannel_hid_cmd(struct dockchannel_hid_softc *sc,
    struct dockchannel_hid_iface *dcif, uint8_t flags, void *cmd, size_t len);
static void dockchannel_hid_wait_ack(struct dockchannel_hid_softc *sc,
    struct dockchannel_hid_iface *dcif, uint8_t seq);

static int dockchannel_hid_write(struct dockchannel_hid_softc *sc,
    const void *buf, size_t len);
static int dockchannel_hid_read(struct dockchannel_hid_softc *sc, void *buf,
    size_t len);

static void dockchannel_hid_handle_gpio_req(struct dockchannel_hid_softc *sc,
    uint8_t iface, void *buf, size_t len);
static void dockchannel_handle_init(struct dockchannel_hid_softc *sc,
    uint8_t iface, void *buf, size_t len);
static void dockchannel_handle_comm(struct dockchannel_hid_softc *sc,
    struct mtp_subhdr *shdr, size_t len);

// TODO: move to header or interface
extern int apple_smc_pin_set(device_t dev, uint32_t pin, uint32_t val);


static int
dockchannel_hid_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev)) {
		return (ENXIO);
	}

	if (!ofw_bus_is_compatible(dev, "apple,dockchannel-hid")) {
		return (ENXIO);
	}

	device_set_desc(dev, "Apple DockChannel HID");
	return (BUS_PROBE_DEFAULT);
}

static int
dockchannel_get_prop(struct dockchannel_hid_softc *sc, const char *name,
    uint32_t *propp, size_t len)
{
	int error;
	device_t dev = sc->sc_dev;

	error = OF_getencprop(sc->sc_node, name, propp, len);
	if (error != len) {
		device_printf(dev, "couldn't find '%s' prop (%d)\n", name, error);
		return (EINVAL);
	}
	return (0);
}

static int
dockchannel_alloc_res(struct dockchannel_hid_softc *sc, int type,
    const char *name, struct resource **resp)
{
	int error, rid;
	struct resource *res;
	device_t dev = sc->sc_dev;
	phandle_t node = sc->sc_node;
	const char *res_names = "reg-names";

	if (resp == NULL) {
		return (EINVAL);
	}

	if (type == SYS_RES_IRQ) {
		res_names = "interrupt-names";
	} else if (type != SYS_RES_MEMORY) {
		return (EINVAL);
	}

	error = ofw_bus_find_string_index(node, res_names, name, &rid);
	if (error != 0) {
		device_printf(dev, "couldn't find '%s' in %s\n", name, res_names);
		return (EINVAL);
	}

	res = bus_alloc_resource_any(dev, type, &rid, RF_ACTIVE);
	if (res == NULL) {
		device_printf(dev, "couldn't allocate '%s' resource\n", name);
		return (ENOMEM);
	}

	*resp = res;

	return (0);
}

static int
dockchannel_hid_attach(device_t dev)
{
	int error;
	struct dockchannel_hid_softc *sc = device_get_softc(dev);

	sc->sc_dev = dev;
	sc->sc_node = ofw_bus_get_node(dev);

	error = dockchannel_get_prop(sc, "apple,helper-cpu", &sc->sc_mtp,
	    sizeof(sc->sc_mtp));
	if (error != 0) {
		return (ENXIO);
	}

	error = dockchannel_get_prop(sc, "apple,fifo-size", &sc->sc_fifo_size,
	    sizeof(sc->sc_fifo_size));
	if (error != 0) {
		return (ENXIO);
	}

	error = dockchannel_alloc_res(sc, SYS_RES_IRQ, "rx", &sc->sc_rx_irq);
	if (error != 0) {
		return (ENXIO);
	}

	error = dockchannel_alloc_res(sc, SYS_RES_IRQ, "tx", &sc->sc_tx_irq);
	if (error != 0) {
		goto err_alloc_tx;
	}

	error = dockchannel_alloc_res(sc, SYS_RES_MEMORY, "config", &sc->sc_config);
	if (error != 0) {
		goto err_alloc_config;
	}

	error = dockchannel_alloc_res(sc, SYS_RES_MEMORY, "data", &sc->sc_data);
	if (error != 0) {
		goto err_alloc_data;
	}

	TASK_INIT(&sc->sc_task, 0, dockchannel_hid_task, sc);
	sc->sc_rx_avail = true;
	sc->sc_tx_avail = true;

	config_intrhook_oneshot(dockchannel_hid_start_config_hook, sc);

	return (0);

err_alloc_data:
	bus_free_resource(dev, SYS_RES_MEMORY, sc->sc_config);
err_alloc_config:
	bus_free_resource(dev, SYS_RES_IRQ, sc->sc_tx_irq);
err_alloc_tx:
	bus_free_resource(dev, SYS_RES_IRQ, sc->sc_rx_irq);
	return (ENXIO);
}

int apple_rtkit_boot(device_t dev, phandle_t helper);

static void
dockchannel_hid_start_config_hook(void *arg)
{
	int error;
	struct dockchannel_hid_softc *sc = arg;
	device_t dev = sc->sc_dev;

	error = apple_rtkit_boot(dev, sc->sc_mtp);
	if (error != 0) {
		device_printf(dev, "RTKit failed to boot (%d)\n", error);
		return;
	}

	bus_write_4(sc->sc_config, CONFIG_RX_THRESH, 8);
	error = bus_setup_intr(dev, sc->sc_rx_irq, INTR_MPSAFE | INTR_TYPE_MISC,
	    dockchannel_hid_rx_filter, NULL, sc,
	    &sc->sc_rx_intrhand);

	if (error != 0) {
		device_printf(dev, "couldn't set up rx irq (%d)\n", error);
		return;
	}

	bus_write_4(sc->sc_config, CONFIG_TX_THRESH, 8);
	error = bus_setup_intr(dev, sc->sc_tx_irq, INTR_MPSAFE | INTR_TYPE_MISC,
	    dockchannel_hid_tx_filter, NULL, sc, &sc->sc_tx_intrhand);

	if (error != 0) {
		device_printf(dev, "couldn't set up tx irq (%d)\n", error);
		return;
	}

	sc->sc_comm.iface = MTP_IFACE_COMM;
	sc->sc_comm.seq = 0;
	sc->sc_comm.retseq = -1;
	sc->sc_stm.seq = 0;
	sc->sc_stm.retseq = -1;
	sc->sc_kbd_hid.hidbus = NULL;

	return;
}

TASKQUEUE_FAST_DEFINE(dockchannel, taskqueue_thread_enqueue,
    &taskqueue_dockchannel, taskqueue_start_threads(&taskqueue_dockchannel, 8, PWAIT, "dockchannel taskq"));

static int
dockchannel_hid_tx_filter(void *arg)
{
	struct dockchannel_hid_softc *sc = arg;
	device_t dev = sc->sc_dev;

	/* mask tx irq */
	dockchannel_mask_tx(dev);
	sc->sc_tx_avail = true;
	wakeup(&sc->sc_tx_irq);

	return (FILTER_HANDLED);
}

static int
dockchannel_hid_rx_filter(void *arg)
{
	struct dockchannel_hid_softc *sc = arg;
	device_t dev = sc->sc_dev;

	/* mask rx irq */
	dockchannel_mask_rx(dev);
	if (!sc->sc_rx_avail) {
		sc->sc_rx_avail = true;
		wakeup(&sc->sc_rx_irq);

		return (FILTER_HANDLED);
	}

	taskqueue_enqueue(taskqueue_dockchannel, &sc->sc_task);
	return (FILTER_HANDLED);
}

static int
dockchannel_hid_read(struct dockchannel_hid_softc *sc, void *buf, size_t len)
{
	device_t dev = sc->sc_dev;
	uint32_t count, read, tmp;
	uint8_t *dst = buf;

	while (len > 0) {
		/* check how much data is available */
		count = bus_read_4(sc->sc_data, DATA_RX_COUNT);

		if (count == 0) {
			sc->sc_rx_avail = false;
			/* set the threshold for when to fire the rx irq */
			//bus_write_4(sc->sc_config, CONFIG_RX_THRESH,
			//    min(len, sc->sc_fifo_size / 2));
			bus_write_4(sc->sc_config, CONFIG_RX_THRESH, 8);

			/* unmask the rx irq */
			dockchannel_unmask_rx(dev);

			/* sleep if the rx irq hasn't fired */
			if (!sc->sc_rx_avail) {
				tsleep(&sc->sc_rx_irq, PWAIT, "dcrx", hz);
			}
			if (sc->sc_rx_avail) {
				device_printf(dev, "waited for read data\n");
				continue;
			}
			/* give up if sleep timed out */
			device_printf(dev, "timed out on read with %zu bytes left\n", len);
			return (ETIMEDOUT);
		}

		/* figure out how much to read */
		read = min(len, count);

		/* read in u32 increments while there is enough data */
		while (read >= 4) {
			tmp = bus_read_4(sc->sc_data, DATA_RX32);
			memcpy(dst, &tmp, sizeof(tmp));
			dst += 4;
			len -= 4;
			read -= 4;
		}

		/* read the rest of the available data in u8 increments */
		while (read > 0) {
			*dst++ = DATA_RX8_DATA(bus_read_4(sc->sc_data, DATA_RX8));
			len--;
			read--;
		}
	}
	return (0);
}

static int
dockchannel_hid_write(struct dockchannel_hid_softc *sc, const void *buf,
    size_t len)
{
	device_t dev = sc->sc_dev;
	uint32_t free, write, tmp;
	const uint8_t *src = buf;

	while (len > 0) {
		free = bus_read_4(sc->sc_data, DATA_TX_FREE);

		if (free == 0) {
			sc->sc_tx_avail = false;
			/* set the threshold for when to fire the tx irq */
			bus_write_4(sc->sc_config, CONFIG_TX_THRESH,
			    min(len, sc->sc_fifo_size / 2));

			/* unmask the tx irq */
			dockchannel_unmask_tx(dev);

			/* sleep if the tx irq hasn't fired */
			if (!sc->sc_tx_avail) {
				tsleep(&sc->sc_tx_irq, PWAIT, "dctx", hz);
			}
			if (sc->sc_tx_avail) {
				device_printf(dev, "waited for write data\n");
				continue;
			}

			/* give up if sleep timed out */
			device_printf(dev, "timed out on write with %zu bytes left\n", len);
			return (ETIMEDOUT);
		}

		write = min(len, free);

		while (write >= 4) {
			memcpy(&tmp, src, sizeof(tmp));
			bus_write_4(sc->sc_data, DATA_TX32, tmp);
			src += 4;
			len -= 4;
			write -= 4;
		}

		while (write > 0) {
			bus_write_1(sc->sc_data, DATA_TX8, *src++);
			len--;
			write--;
		}
	}
	return (0);
}

static uint32_t
dockchannel_hid_cksum(const void *buf, size_t len)
{
	uint32_t tmp;
	uint32_t cksum = 0;
	const uint8_t *stream = buf;

	while (len >= 4) {
		memcpy(&tmp, stream, sizeof(tmp));
		cksum += tmp;
		stream += 4;
		len -= 4;
	}
	return cksum;
}

static uint8_t gbuf[0xffff + 4];

static void
dockchannel_hid_task(void *arg, int pending)
{
	struct dockchannel_hid_softc *sc = arg;
	device_t dev = sc->sc_dev;
	struct mtp_hdr hdr;
	struct mtp_subhdr *shdr;
	int error;
	uint32_t cksum;
	char buf[1024];

	//device_printf(dev, "started task with %d pending\n", pending);

	error = dockchannel_hid_read(sc, &hdr, sizeof(hdr));
	if (error != 0) {
		device_printf(dev, "failed to read header\n");
		goto done;
	}

	if (hdr.hdr_len != sizeof(hdr)) {
		device_printf(dev, "inconsistent header length 0x%x\n", hdr.hdr_len);
		goto done;
	}

	if (hdr.pkt_len < sizeof(*shdr)) {
		device_printf(dev,
		    "packet size 0x%x too small for a single subheader\n", hdr.pkt_len);
		goto done;
	}

	error = dockchannel_hid_read(sc, buf, hdr.pkt_len + 4);
	if (error != 0) {
		device_printf(dev, "failed to read packet\n");
		goto done;
	}

	cksum = dockchannel_hid_cksum(&hdr, sizeof(hdr));
	cksum += dockchannel_hid_cksum(buf, hdr.pkt_len + 4);

	if (cksum != 0xffffffff) {
		device_printf(dev, "bad checksum %x\n", cksum);
		goto done;
	}

	shdr = (struct mtp_subhdr *)buf;
	if (MTP_GROUP(shdr->flags) == MTP_GROUP_OUTPUT ||
	    MTP_GROUP(shdr->flags) == MTP_GROUP_CMD) {
		if (hdr.iface == sc->sc_comm.iface) {
			sc->sc_comm.retcode = shdr->retcode;
			sc->sc_comm.retseq = hdr.seq;
			wakeup(&sc->sc_comm.retseq);
		} else if (hdr.iface == sc->sc_stm.iface) {
			sc->sc_stm.retcode = shdr->retcode;
			sc->sc_stm.retseq = hdr.seq;
			if (MTP_REQ(shdr->flags) == MTP_REQ_GET_REPORT) {
				device_printf(sc->sc_dev, "copying %d bytes\n", shdr->len);
				memcpy(gbuf, shdr + 1, shdr->len);
			}
			wakeup(&sc->sc_stm.retseq);
		} else {
			panic("got unexpected ack from iface 0x%x\n", hdr.iface);
		}


		return;
	}

	switch (hdr.chan) {
	case MTP_CHAN_CMD:
		//dockchannel_handle_ack(sc, ...
		break;
	case MTP_CHAN_REPORT:
		if (hdr.iface == MTP_IFACE_COMM) {
			dockchannel_handle_comm(sc, shdr + 1, shdr->len);
		} else if (sc->sc_kbd.ready && hdr.iface == sc->sc_kbd.iface) {
			//MPASS(hdr.iface == sc->sc_kbd.iface);
			//MPASS(sc->sc_kbd.ready);
			MPASS(sc->sc_kbd_hid.intr_handler != NULL);
			//device_printf(dev, "calling intr handler\n");
			(*sc->sc_kbd_hid.intr_handler)(sc->sc_kbd_hid.intr_ctx, shdr + 1,
			    shdr->len);
			//dockchannel_handle_report(sc, ...
		}
		break;
	default:
		device_printf(dev, "ignored packet from unknown channel 0x%x\n",
		    hdr.chan);
		break;
	}
done:
	sc->sc_rx_avail = true;
	/* unmask rx irq to get ack */
	dockchannel_unmask_rx(dev);
}

static void
dockchannel_hid_iface_enable(struct dockchannel_hid_softc *sc,
    const struct dockchannel_hid_iface *dcif)
{
	uint8_t flags;
	uint8_t cmd[2];

	MPASS(dcif != NULL);

	cmd[0] = MTP_CMD_IFACE_ENABLE;
	cmd[1] = dcif->iface;

	flags = MTP_GROUP_CMD << MTP_GROUP_SHIFT;
	flags |= MTP_REQ_SET_REPORT << MTP_REQ_SHIFT;

	dockchannel_hid_cmd(sc, &sc->sc_comm, flags, cmd, sizeof(cmd));
	dockchannel_hid_wait_ack(sc, &sc->sc_comm, sc->sc_comm.seq - 1);
}

static void
dockchannel_hid_get_report2(struct dockchannel_hid_softc *sc,
    struct dockchannel_hid_iface *dcif, uint8_t reportnum, void *buf,
    size_t len)
{
	device_t dev = sc->sc_dev;
	uint8_t flags = MTP_GROUP_CMD << MTP_GROUP_SHIFT;
	flags |= MTP_REQ_GET_REPORT << MTP_REQ_SHIFT;
	dockchannel_hid_cmd(sc, dcif, flags, &reportnum, 1);
	dockchannel_hid_wait_ack(sc, dcif, dcif->seq - 1);
	memcpy(buf, gbuf, len);
	dockchannel_unmask_rx(dev);
}

static int
dockchannel_hid_add_child(struct dockchannel_hid_softc *sc,
    struct dockchannel_hid_child *child)
{
	device_t dev, hidbus;

	dev = sc->sc_dev;

	dockchannel_unmask_rx(dev);

	child->hw.idBus = BUS_HOST;
	child->hw.idVendor = sc->sc_stm_id.vendor_id;
	child->hw.idProduct = sc->sc_stm_id.product_id;
	child->hw.idVersion = sc->sc_stm_id.version;
	child->hw.rdescsize = sc->sc_kbd.desc_len;
	memcpy(child->hw.serial, sc->sc_serial, sizeof(sc->sc_serial));

	/* turn write into set_report */
	hid_add_dynamic_quirk(&child->hw, HQ_NOWRITE);

	hidbus = device_add_child(dev, "hidbus", DEVICE_UNIT_ANY);
	if (hidbus == NULL) {
		device_printf(dev, "couldn't add child device for HID\n");
		return (ENOMEM);
	}
	device_set_ivars(hidbus, &child->hw);
	child->hidbus = hidbus;

    // XXX: there's probably a better way of attaching than grabbing Giant...
	mtx_lock(&Giant);
	bus_attach_children(dev);
	mtx_unlock(&Giant);

	return (0);
}

static int
dockchannel_hid_detach(device_t dev)
{
	panic("uh oh");
}

static void
dockchannel_hid_wait_ack(struct dockchannel_hid_softc *sc,
    struct dockchannel_hid_iface *dcif, uint8_t seq)
{
	device_t dev = sc->sc_dev;

	bus_write_4(sc->sc_config, CONFIG_RX_THRESH, 8);
	dockchannel_unmask_rx(dev);
	while (seq != dcif->retseq) {
		tsleep(&dcif->retseq, PWAIT, "", hz);
		if (seq == dcif->retseq) {
			break;
		}
	}
	if (dcif->retcode != 0 || dcif->iface != MTP_IFACE_COMM) {
		device_printf(dev, "got retcode %x for packet #%d on iface %d\n",
		    dcif->retcode, dcif->retseq, dcif->iface);
	}
	if (dcif->iface != sc->sc_stm.iface) {
		dockchannel_unmask_rx(dev);
	}
}

static void
dockchannel_hid_cmd(struct dockchannel_hid_softc *sc,
    struct dockchannel_hid_iface *dcif, uint8_t flags, void *cmd, size_t len)
{
	struct mtp_hdr hdr;
	struct mtp_subhdr shdr;
	uint32_t cksum = 0xffffffff;
	uint8_t *data = cmd;
	uint8_t padding[4] = { 0, 0, 0, 0 };
	int error;

	MPASS(dcif != NULL);

	memset(&hdr, 0, sizeof(hdr));
	hdr.hdr_len = sizeof(hdr);
	hdr.chan = MTP_CHAN_CMD;
	hdr.pkt_len = roundup(len, 4) + sizeof(shdr);
	hdr.seq = dcif->seq;
	dcif->seq++;
	hdr.iface = dcif->iface;

	memset(&shdr, 0, sizeof(shdr));
	shdr.flags = flags;
	shdr.len = len;
	error = dockchannel_hid_write(sc, &hdr, sizeof(hdr));
	MPASS(error == 0);
	error = dockchannel_hid_write(sc, &shdr, sizeof(shdr));
	MPASS(error == 0);
	error = dockchannel_hid_write(sc, cmd, len & ~3);
	MPASS(error == 0);

	cksum -= dockchannel_hid_cksum(&hdr, sizeof(hdr));
	cksum -= dockchannel_hid_cksum(&shdr, sizeof(shdr));
	cksum -= dockchannel_hid_cksum(cmd, len & ~3);

	if (len & 3) {
		memset(padding, 0, sizeof(padding));
		memcpy(padding, &data[len & ~3], len & 3);
		error = dockchannel_hid_write(sc, padding, sizeof(padding));
		MPASS(error == 0);
		cksum -= dockchannel_hid_cksum(padding, sizeof(padding));
	}
	error = dockchannel_hid_write(sc, &cksum, sizeof(cksum));
	MPASS(error == 0);
}

// XXX
static void
dockchannel_hid_handle_gpio_req(struct dockchannel_hid_softc *sc, uint8_t iface,
    void *buf, size_t len)
{
	struct mtp_gpio_req *req = buf;
	phandle_t node = sc->sc_node;
	char name[64];
	uint32_t prop[3];
	int error;
	device_t dev = sc->sc_dev;

	snprintf(name, sizeof(name), "apple,%s-gpios", req->name);
	device_printf(dev, "requested gpio '%s' #%d\n", req->name, req->id);

	error = dockchannel_get_prop(sc, name, prop, sizeof(prop));
	if (error != 0) {
		return;
	}

	len = OF_getproplen(node, name);
	MPASS(len > 0);

	error = OF_getencprop(node, name, prop, sizeof(prop));
	MPASS(error == (sizeof(uint32_t) * 3));

	device_t cdev = OF_device_from_xref(prop[0]);
	apple_smc_pin_set(cdev, prop[1], 0);

	if (iface == sc->sc_stm.iface) {
		sc->sc_stm.smc = prop[0];
		sc->sc_stm.gpio = prop[1];
	}
}

static void
dockchannel_handle_init(struct dockchannel_hid_softc *sc, uint8_t iface,
    void *buf, size_t len)
{
	struct mtp_init_block_hdr *bhdr = buf;

	for (;;) {
		if (len < sizeof(*bhdr))
			return;

		len -= sizeof(*bhdr);

		if (len < bhdr->len)
			return;
		len -= bhdr->len;

		switch(bhdr->type) {
		case MTP_BLOCK_DESCRIPTOR:
			if (iface == sc->sc_stm.iface &&
			    bhdr->len <= sizeof(sc->sc_stm.desc)) {
				memcpy(sc->sc_stm.desc, bhdr + 1, bhdr->len);
				sc->sc_stm.desc_len = bhdr->len;
			} else if (iface == sc->sc_kbd.iface &&
			    bhdr->len <= sizeof(sc->sc_kbd.desc)) {
				memcpy(sc->sc_kbd.desc, bhdr + 1, bhdr->len);
				sc->sc_kbd.desc_len = bhdr->len;
			} else if (iface == sc->sc_mt.iface &&
			    bhdr->len <= sizeof(sc->sc_mt.desc)) {
				memcpy(sc->sc_mt.desc, bhdr + 1, bhdr->len);
				sc->sc_mt.desc_len = bhdr->len;
			}
			break;
		case MTP_BLOCK_GPIO_REQ:
			dockchannel_hid_handle_gpio_req(sc, iface, bhdr + 1, bhdr->len);
			break;
		case MTP_BLOCK_END:
			return;
		default:
			panic("uh oh");
			break;
		}

		bhdr = (void *)((uint8_t *)(bhdr + 1) + bhdr->len);
	}
}

static void
dockchannel_handle_comm(struct dockchannel_hid_softc *sc,
    struct mtp_subhdr *shdr, size_t len)
{
	device_t dev = sc->sc_dev;
	struct mtp_init_hdr *ihdr = (struct mtp_init_hdr *)shdr;
	uint8_t iface;
	int error;

	switch (ihdr->type) {
	case MTP_EVENT_INIT:
		if (strcmp(ihdr->name, "stm") == 0) {
			sc->sc_stm.iface = ihdr->iface;
			dockchannel_handle_init(sc, ihdr->iface, ihdr + 1, len - sizeof(*ihdr));

			dockchannel_hid_iface_enable(sc, &sc->sc_stm);
		}
		if (strcmp(ihdr->name, "keyboard") == 0) {
			sc->sc_kbd.iface = ihdr->iface;
			dockchannel_handle_init(sc, ihdr->iface, ihdr + 1, len - sizeof(*ihdr));

			dockchannel_hid_iface_enable(sc, &sc->sc_kbd);
		}
		if (strcmp(ihdr->name, "multi-touch") == 0) {
			sc->sc_mt.iface = ihdr->iface;
			dockchannel_handle_init(sc, ihdr->iface, ihdr + 1, len - sizeof(*ihdr));

			dockchannel_hid_iface_enable(sc, &sc->sc_mt);
		}
		break;
	case MTP_EVENT_READY:
		iface = ihdr->unk0;
		if (iface == sc->sc_stm.iface) {
			sc->sc_stm.ready = true;
			dockchannel_hid_get_report2(sc, &sc->sc_stm, STM_REPORT_ID,
			    &sc->sc_stm_id, sizeof(sc->sc_stm_id));

			dockchannel_hid_get_report2(sc, &sc->sc_stm, STM_REPORT_SERIAL,
			    &sc->sc_serial, sizeof(sc->sc_serial) - 1);
			if (sc->sc_kbd_hid.hidbus == NULL && sc->sc_kbd.ready) {
				error = dockchannel_hid_add_child(sc, &sc->sc_kbd_hid);
				MPASS(error == 0);
			}
			//if (sc->sc_mt_hid.hidbus == NULL) {
			//	error = dockchannel_hid_add_child(sc, &sc->sc_mt_hid);
			//	MPASS(error == 0);
			//}
		} else if (iface == sc->sc_kbd.iface) {
			sc->sc_kbd.ready = true;
			if (sc->sc_kbd_hid.hidbus == NULL) {
				error = dockchannel_hid_add_child(sc, &sc->sc_kbd_hid);
				MPASS(error == 0);
			}
		} else if (iface == sc->sc_mt.iface) {
			sc->sc_mt.ready = true;
		}
		dockchannel_unmask_rx(sc->sc_dev);
		break;
	case MTP_EVENT_GPIO_CMD: {
		struct mtp_gpio_cmd *gpio_cmd = (void *)ihdr;
		device_printf(dev,"got cmd 0x%x event for gpio %d on iface %d\n", gpio_cmd->cmd, gpio_cmd->gpio, gpio_cmd->iface);
		if (gpio_cmd->iface == sc->sc_stm.iface) {
			device_t cdev = OF_device_from_xref(sc->sc_stm.smc);
			apple_smc_pin_set(cdev, sc->sc_stm.gpio, 1);
			DELAY(10000);
			apple_smc_pin_set(cdev, sc->sc_stm.gpio, 0);
			struct mtp_gpio_ack ack;
			ack.type = MTP_CMD_ACK_GPIO_CMD;
			ack.retcode = 0xe000f00d;
			memcpy(&ack.cmd, shdr, min(512, len));
			uint8_t flags = MTP_GROUP_CMD << MTP_GROUP_SHIFT;
			flags |= MTP_REQ_SET_REPORT << MTP_REQ_SHIFT;
			dockchannel_hid_cmd(sc, &sc->sc_comm, flags, &ack, sizeof(ack));
			dockchannel_hid_wait_ack(sc, &sc->sc_comm, sc->sc_comm.seq - 1);
		}
		// fallthrough
	}
	default:
		dockchannel_unmask_rx(sc->sc_dev);
		break;
	}
}

static struct dockchannel_hid_child *
dockchannel_hid_get_child(device_t child, struct dockchannel_hid_iface **dcif)
{
	device_t parent = device_get_parent(child);
	struct dockchannel_hid_softc *sc = device_get_softc(parent);
	if (child == sc->sc_kbd_hid.hidbus) {
		if (dcif) {
			*dcif = &sc->sc_kbd;
		}
		return (&sc->sc_kbd_hid);
	} else {
		MPASS(child == sc->sc_mt_hid.hidbus);
		if (dcif) {
			*dcif = &sc->sc_mt;
		}
		return (&sc->sc_mt_hid);
	}
}

static void
dockchannel_hid_intr_setup(device_t dev, device_t child, hid_intr_t intr,
    void *context, struct hid_rdesc_info *rdesc)
{
	struct dockchannel_hid_child *dc = dockchannel_hid_get_child(child, NULL);
	if (intr == NULL) {
		return;
	}
	rdesc->rdsize = 0; // XXX
	rdesc->wrsize = 0; // XXX
	rdesc->grsize = 0; // dc->hw.rdescsize;
	rdesc->srsize = 0; // dc->hw.rdescsize;

	dc->intr_handler = intr;
	dc->intr_ctx = context;
}

static void
dockchannel_hid_intr_unsetup(device_t dev, device_t child)
{
}

static int
dockchannel_hid_intr_start(device_t dev, device_t child)
{
	return (0);
}

static int
dockchannel_hid_intr_stop(device_t dev, device_t child)
{
	return (0);
}

static void
dockchannel_hid_intr_poll(device_t dev, device_t child)
{
}

static int
dockchannel_hid_get_rdesc(device_t dev, device_t child, void *buf,
    hid_size_t len)
{
	struct dockchannel_hid_iface *dcif = NULL;
	dockchannel_hid_get_child(child, &dcif);

	MPASS(dcif->desc_len != 0);
	MPASS(dcif->ready);
	memcpy(buf, dcif->desc, len);

	return (0);
}

static int
dockchannel_hid_get_report(device_t dev, device_t child, void *data,
    hid_size_t maxlen, hid_size_t *actlen, uint8_t type, uint8_t id)
{
	return (ENOTSUP);
}

static int
dockchannel_hid_set_report(device_t dev, device_t child, const void *buf,
    hid_size_t len, uint8_t type, uint8_t id)
{
	return (ENOTSUP);
}



static device_method_t dockchannel_hid_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, dockchannel_hid_probe),
	DEVMETHOD(device_attach, dockchannel_hid_attach),
	DEVMETHOD(device_detach, dockchannel_hid_detach),

	/* HID interrupt interface */
	DEVMETHOD(hid_intr_setup,		dockchannel_hid_intr_setup),
	DEVMETHOD(hid_intr_unsetup,		dockchannel_hid_intr_unsetup),
	DEVMETHOD(hid_intr_start,		dockchannel_hid_intr_start),
	DEVMETHOD(hid_intr_stop,		dockchannel_hid_intr_stop),
	DEVMETHOD(hid_intr_poll,		dockchannel_hid_intr_poll),

	/* HID interface */
	DEVMETHOD(hid_get_rdesc,	dockchannel_hid_get_rdesc),
	DEVMETHOD(hid_get_report,	dockchannel_hid_get_report),
	DEVMETHOD(hid_set_report,	dockchannel_hid_set_report),

	DEVMETHOD_END
};

static driver_t dockchannel_hid_driver = {
	.name = "dockchannel_hid",
	.methods = dockchannel_hid_methods,
	.size = sizeof(struct dockchannel_hid_softc),
};
DRIVER_MODULE(dockchannel_hid, dockchannel, dockchannel_hid_driver, 0, 0);
MODULE_DEPEND(dockchannel_hid, hid, 1, 1, 1);
MODULE_DEPEND(dockchannel_hid, hidbus, 1, 1, 1);
