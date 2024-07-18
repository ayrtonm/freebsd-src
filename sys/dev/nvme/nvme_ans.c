/* Copyright TBD */
/* derived in part from nvms_ahci.c, also OpenBSD aplns.c */
/*-
 * Copyright (C) 2017 Olivier Houchard
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
/*	$OpenBSD: aplns.c,v 1.12 2022/06/12 16:00:12 kettenis Exp $ */
/*
 * Copyright (c) 2014, 2021 David Gwynne <dlg@openbsd.org>
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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/buf.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/proc.h>
#include <sys/smp.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <arm64/apple/rtkit.h>

#include "nvme_private.h"
#include "nvme_if.h"

#define ANS_CPU_CTRL		0x0044
#define ANS_CPU_CTRL_RUN	(1 << 4)

#define ANS_MAX_PEND_CMDS_CTRL	0x01210
#define  ANS_MAX_QUEUE_DEPTH	64
#define ANS_BOOT_STATUS		0x01300
#define  ANS_BOOT_STATUS_OK	0xde71ce55
#define ANS_MODESEL_REG		0x01304
#define ANS_UNKNOWN_CTRL	0x24008
#define  ANS_PRP_NULL_CHECK	(1 << 11)
#define ANS_LINEAR_SQ_CTRL	0x24908
#define  ANS_LINEAR_SQ_CTRL_EN	(1 << 0)
#define ANS_LINEAR_ASQ_DB	0x2490c
#define ANS_LINEAR_IOSQ_DB	0x24910

#define ANS_NVMMU_NUM		0x28100
#define ANS_NVMMU_BASE_ASQ	0x28108
#define ANS_NVMMU_BASE_IOSQ	0x28110
#define ANS_NVMMU_TCB_INVAL	0x28118
#define ANS_NVMMU_TCB_STAT	0x28120

#define ANS_NVMMU_TCB_SIZE	0x4000
#define ANS_NVMMU_TCB_PITCH	0x80

#define LOCAL_TRACE 1
#include <sys/ltrace.h>

struct ans_nvmmu_tcb {
	uint8_t		tcb_opcode;
	uint8_t		tcb_flags;
#define ANS_NVMMU_TCB_WRITE		(1 << 0)
#define ANS_NVMMU_TCB_READ		(1 << 1)
	uint8_t		tcb_cid;
	uint8_t		tcb_pad0[1];

	uint32_t	tcb_prpl_len;
	uint8_t		tcb_pad1[16];

	uint64_t	tcb_prp[2];
};

struct nvme_ans_controller {
	struct nvme_controller	nvme;		/* base class, must be first */
	/* SART info */
	bus_space_tag_t		bus_tag;
	bus_space_handle_t	bus_handle;
	int			resource_id;
	struct resource		*resource;

	uint32_t		 sart;
	struct rtkit		 rtkit;
	struct rtkit_state	*rtkit_state;
	struct nvme_dmamem	*nvmmu;
	mbox_t			mbox;
};

#define ANSDEVICE2SOFTC(dev) \
	((struct nvme_ans_controller *) device_get_softc(dev))

/*
 * The following two macros work on an nvme_ans_controller (for SART)
 * as well as nvme_controller (for NVME and Apple/ANS) by virtue of
 * common structure element names.
 */
#define NVME_ANS_READ_4(_sc, reg) \
    bus_space_read_4((_sc)->bus_tag, (_sc)->bus_handle, (reg))
#define NVME_ANS_WRITE_4(_sc, reg, val) \
    bus_space_write_4((_sc)->bus_tag, (_sc)->bus_handle, (reg), (val))

static int	nvme_ans_probe(device_t dev);
static int	nvme_ans_attach(device_t dev);
//static int    nvme_ans_detach(device_t dev);

static int	nvme_ans_sart_map(void *, bus_addr_t, bus_size_t);
//extern int	apple_sart_map(uint32_t, bus_addr_t, bus_size_t);
void		nvme_ans_enable(device_t dev, struct nvme_controller *ctrlr);
uint32_t	nvme_ans_sq_enter(device_t dev,
				  struct nvme_controller *ctrlr,
				  struct nvme_qpair *qpair);
void		nvme_ans_sq_leave(device_t dev,
				  struct nvme_controller *ctrlr,
				  struct nvme_qpair *qpair);

static device_method_t nvme_ans_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,     nvme_ans_probe),
	DEVMETHOD(device_attach,    nvme_ans_attach),
	DEVMETHOD(device_detach,    nvme_detach),
	DEVMETHOD(device_shutdown,  nvme_shutdown),

	/* NVME interface */
	DEVMETHOD(nvme_enable,      nvme_ans_enable),
	DEVMETHOD(nvme_sq_enter,    nvme_ans_sq_enter),
	DEVMETHOD(nvme_sq_leave,    nvme_ans_sq_leave),

	{ 0, 0 }
};

static driver_t nvme_ans_driver = {
	"nvme",
	nvme_ans_methods,
	sizeof(struct nvme_ans_controller),
};

DRIVER_MODULE(nvme, simplebus, nvme_ans_driver, NULL, NULL);

static struct ofw_compat_data compat_data[] = {
	{"apple,nvme-m1",		1},
	{"apple,nvme-ans2",		1},
	{NULL,				0}
};

static int
nvme_ans_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
		return (ENXIO);

	device_set_desc(dev, "Apple NVME Storage controller");
	return (BUS_PROBE_DEFAULT);
}

static int
nvme_ans_attach(device_t dev)
{
	struct nvme_ans_controller *sc = ANSDEVICE2SOFTC(dev);
	struct nvme_controller *ctrlr = &sc->nvme;
	phandle_t node;
	uint32_t ctrl, status;
	ssize_t sret;
	int ret;

	/* need registers for NVME, SART */

	ltracef("ANS attach");
	/* Map NVME registers */
	node = ofw_bus_get_node(dev);
	if (ofw_bus_find_string_index(node, "reg-names", "nvme",
	    &ctrlr->resource_id) != 0) {
		device_printf(dev, "couldn't get \"nvme\" regs\n");
		ret = ENXIO;
		goto bad;
	}
	ltracef("found nvme");
	ctrlr->resource = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &ctrlr->resource_id, RF_ACTIVE);

	if (ctrlr->resource == NULL) {
		device_printf(dev, "unable to allocate NVME mem resource\n");
		ret = ENOMEM;
		goto bad;
	}
	ctrlr->bus_tag = rman_get_bustag(ctrlr->resource);
	ctrlr->bus_handle = rman_get_bushandle(ctrlr->resource);
	ctrlr->regs = (struct nvme_registers *)ctrlr->bus_handle;

	ltracef("SART");
	/* Map SART registers */
	if (ofw_bus_find_string_index(node, "reg-names", "ans",
	    &sc->resource_id) != 0) {
		device_printf(dev, "couldn't get \"ans\" regs\n");
		ret = ENXIO;
		goto bad;
	}
	sc->resource = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &sc->resource_id, RF_ACTIVE);

	if (sc->resource == NULL) {
		device_printf(dev, "unable to allocate SART mem resource\n");
		ret = ENOMEM;
		goto bad;
	}
	sc->bus_tag = rman_get_bustag(sc->resource);
	sc->bus_handle = rman_get_bushandle(sc->resource);

	//power_domain_enable(faa->fa_node);

	ltracef("IRQ");
	/* Allocate and setup IRQ */
	ctrlr->rid = 0;
	ctrlr->res = bus_alloc_resource_any(dev, SYS_RES_IRQ,
	    &ctrlr->rid, RF_SHAREABLE | RF_ACTIVE);
	if (ctrlr->res == NULL) {
		device_printf(dev, "unable to allocate interrupt\n");
		ret = ENOMEM;
		goto bad;
	}

	ctrlr->msi_count = 0;
	ctrlr->num_io_queues = 1;

	/*
	 * We're attached via this funky mechanism. Flag the controller so that
	 * it avoids things that can't work when we do that, like asking for
	 * PCI config space entries.
	 */
	ctrlr->quirks |= QUIRK_ANS;

	sret = OF_getencprop(node, "apple,sart", &sc->sart,
	    sizeof(sc->sart)); /* XXX ??? */
	if (sret != sizeof(sc->sart))
		device_printf(dev, "OF_getprop apple,sart %jd\n",
		    (intmax_t) sret);
	sc->rtkit.rk_cookie = sc;
	// TODO: figure out a reasonable way to set a maximum. this is the value returned on my m2 macbook air
	sc->rtkit.rk_dma_maxsize = 32 * 1024;
	ltracef("cretaing dma tag with alignment/PAGE_SIZE: %x", PAGE_SIZE);
	ret = bus_dma_tag_create(bus_get_dma_tag(dev),	/* parent? */
		       PAGE_SIZE, 0,		/* alignment, bounds */
		       BUS_SPACE_MAXADDR,	/* lowaddr */
		       BUS_SPACE_MAXADDR,	/* highaddr */
		       NULL, NULL,		/* filter, filterarg not supported by arm64 dma bounce anyway */
		       sc->rtkit.rk_dma_maxsize,		/* maxsize */
		       1,			/* nsegments */
		       64 * 1024,		/* maxsegsize ??? */
		       0 /*BUS_DMA_ALLOCNOW*/,	/* flags */
		       NULL,			/* lockfunc */
		       NULL,			/* lockarg */
		       &sc->rtkit.rk_dmat);
	if (ret != 0) {
		device_printf(dev, "bus_dma_tag_create failed %d\n", ret);
		goto bad;
	}
	sc->rtkit.rk_map = nvme_ans_sart_map;

	ret = mbox_get_by_ofw_idx(dev, node, 0, &sc->mbox);
	if (ret < 0) {
		device_printf(dev, "can't set up rtkit mailbox ret %d\n", ret);
		ret = ENXIO;
		goto bad;
	}
	sc->rtkit_state = rtkit_init(node, NULL, &sc->rtkit, sc->mbox);
	if (sc->rtkit_state == NULL) {
		device_printf(dev, "can't set up rtkit\n");
		ret = ENXIO;
		goto bad;
	}
	/* XXX how do we set up mbox callback? */

	ltracef("hit regs");
	ctrl = NVME_ANS_READ_4(sc, ANS_CPU_CTRL);
	NVME_ANS_WRITE_4(sc, ANS_CPU_CTRL, ctrl | ANS_CPU_CTRL_RUN);

	status = NVME_ANS_READ_4(ctrlr, ANS_BOOT_STATUS);
	if (status != ANS_BOOT_STATUS_OK)
		rtkit_boot(sc->rtkit_state);

	status = NVME_ANS_READ_4(ctrlr, ANS_BOOT_STATUS);
	if (status != ANS_BOOT_STATUS_OK) {
		device_printf(dev, "firmware not ready\n");
		ret = ENXIO;
		goto bad;
	}

	if (bus_setup_intr(dev, ctrlr->res,
	    INTR_TYPE_MISC | INTR_MPSAFE, NULL, nvme_ctrlr_shared_handler,
	    ctrlr, &ctrlr->tag) != 0) {
		device_printf(dev, "unable to setup interrupt\n");
		ret = ENOMEM;
		goto bad;
	}
	ctrlr->tag = (void *)0x1;

	NVME_ANS_WRITE_4(ctrlr, ANS_LINEAR_SQ_CTRL, ANS_LINEAR_SQ_CTRL_EN);
	NVME_ANS_WRITE_4(ctrlr, ANS_MAX_PEND_CMDS_CTRL,
	    (ANS_MAX_QUEUE_DEPTH << 16) | ANS_MAX_QUEUE_DEPTH);

	ctrl = NVME_ANS_READ_4(ctrlr, ANS_UNKNOWN_CTRL);
	NVME_ANS_WRITE_4(ctrlr, ANS_UNKNOWN_CTRL, ctrl & ~ANS_PRP_NULL_CHECK);

	//ctrlr->sc_ios = faa->fa_reg[0].size;	/* XXX */
	//ctrlr->sc_openings = 1;

	ltracef("nvme_attach");
	return (nvme_attach(dev));	/* Note: failure frees resources */
bad:
	ltracef("bad:");
	if (ctrlr->resource != NULL) {
		bus_release_resource(dev, SYS_RES_MEMORY,
		    ctrlr->resource_id, ctrlr->resource);
	}
	if (sc->resource != NULL) {
		bus_release_resource(dev, SYS_RES_MEMORY,
		    sc->resource_id, sc->resource);
	}
	if (ctrlr->res) {
		bus_release_resource(dev, SYS_RES_IRQ,
		    rman_get_rid(ctrlr->res), ctrlr->res);
	}

	return (ret);
}

static int
nvme_ans_sart_map(void *cookie, bus_addr_t addr, bus_size_t size)
{
	struct nvme_ans_controller *sc = cookie;
	
	return (apple_sart_map(sc->sart, addr, size));
}

void
nvme_ans_enable(device_t dev, struct nvme_controller *ctrlr)
{
nvme_printf(ctrlr, "enable\n");
	bus_space_write_4(ctrlr->bus_tag, ctrlr->bus_handle, ANS_NVMMU_NUM,
			  (ANS_NVMMU_TCB_SIZE / ANS_NVMMU_TCB_PITCH) - 1);
	bus_space_write_4(ctrlr->bus_tag, ctrlr->bus_handle, ANS_MODESEL_REG, 0);
}

uint32_t
nvme_ans_sq_enter(device_t dev, struct nvme_controller *ctrlr,
    struct nvme_qpair *qpair)
{
nvme_printf(ctrlr, "sq_enter\n");
	return (0/*notyet*/);
}

void
nvme_ans_sq_leave(device_t dev, struct nvme_controller *ctrlr,
    struct nvme_qpair *qpair)
{
nvme_printf(ctrlr, "sq_leave\n");
}
