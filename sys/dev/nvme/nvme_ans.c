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
#include <sys/taskqueue.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

//#include <arm64/apple/apple_mbox.h>
#include <arm64/apple/rtkit.h>

#include "nvme_private.h"
#include "nvme_if.h"

#define	ANS_CPU_CTRL		0x0044
#define	ANS_CPU_CTRL_RUN	(1 << 4)

#define	ANS_ACQ_DB	0x1004
#define	ANS_IOCQ_DB	0x100c

#define	ANS_MAX_PEND_CMDS_CTRL	0x1210
#define	ANS_MAX_QUEUE_DEPTH	64

#define	ANS_BOOT_STATUS		0x01300
#define	ANS_BOOT_STATUS_OK	0xde71ce55

#define	ANS_MODESEL_REG		0x01304
#define	ANS_UNKNOWN_CTRL	0x24008
#define	ANS_PRP_NULL_CHECK	(1 << 11)

#define	ANS_LINEAR_SQ_CTRL	0x24908
#define	ANS_LINEAR_SQ_CTRL_EN	(1 << 0)

#define	ANS_LINEAR_ASQ_DB	0x2490c
#define	ANS_LINEAR_IOSQ_DB	0x24910

#define	ANS_NVMMU_NUM		0x28100
#define	ANS_NVMMU_BASE_ASQ	0x28108
#define	ANS_NVMMU_BASE_IOSQ	0x28110
#define	ANS_NVMMU_TCB_INVAL	0x28118
#define	ANS_NVMMU_TCB_STAT	0x28120

#define	ANS_NVMMU_TCB_SIZE	0x4000
#define	ANS_NVMMU_TCB_PITCH	0x80

struct nvme_ans_qpair {
	struct nvme_qpair	*qpair;
	struct nvme_ans_nvmmu_tcb *tcb;

	bus_addr_t			addr;
	bus_size_t			size;
	void				*kva;

	bus_dma_tag_t		tag;
	bus_dmamap_t		map;

	struct nvme_ans_controller *sc;
	bool is_admin;
};

struct nvme_ans_nvmmu_tcb {
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
	struct nvme_controller	sc_ctrlr;		/* base class, must be first */

	struct resource			*sc_ans;
	int						sc_ans_id;
	bus_space_tag_t			sc_ans_bus_tag;
	bus_space_handle_t		sc_ans_bus_handle;

	phandle_t				sc_sart;
	struct rtkit_state		*sc_rtkit_state;

	struct nvme_ans_qpair	sc_adminq;
	struct nvme_ans_qpair	sc_ioq;
};

#define ANSDEVICE2SOFTC(dev) \
	((struct nvme_ans_controller *) device_get_softc(dev))

#define NVME_READ_4(_sc, _reg) \
	bus_space_read_4((_sc)->bus_tag, (_sc)->bus_handle, (_reg))

#define NVME_WRITE_4(_sc, _reg, _val) \
	bus_space_write_4((_sc)->bus_tag, (_sc)->bus_handle, (_reg), (_val))

#define NVME_WRITE_8(_sc, _reg, _val) \
	do { \
		bus_space_write_4((_sc)->bus_tag, (_sc)->bus_handle, (_reg), (_val & 0xffffffff)); \
		bus_space_write_4((_sc)->bus_tag, (_sc)->bus_handle, (_reg) + 4, (_val >> 32)); \
	} while (0)

#define NVME_ANS_READ_4(_sc, _reg) \
	bus_space_read_4((_sc)->sc_ans_bus_tag, (_sc)->sc_ans_bus_handle, (_reg))
#define NVME_ANS_WRITE_4(_sc, _reg, _val) \
	bus_space_write_4((_sc)->sc_ans_bus_tag, (_sc)->sc_ans_bus_handle, (_reg), (_val))

static int nvme_ans_probe(device_t dev);
static int nvme_ans_attach(device_t dev);
//static int nvme_ans_detach(device_t dev);

static int	nvme_ans_sart_map(void *, bus_addr_t, bus_size_t);

static int nvme_ans_delayed_attach(device_t dev, struct nvme_controller *ctrlr);
static void nvme_ans_enable(device_t dev);
static uint32_t nvme_ans_sq_enter(device_t dev, struct nvme_qpair *qpair,
	struct nvme_tracker *tr);
static void nvme_ans_sq_leave(device_t dev, struct nvme_qpair *qpair,
	struct nvme_tracker *tr);
static void nvme_ans_cq_done(device_t dev, struct nvme_qpair *qpair,
	struct nvme_tracker *tr);

static int nvme_ans_qpair_construct(device_t dev, struct nvme_qpair *qpair,
	uint32_t num_entries, uint32_t num_trackers, struct nvme_controller *ctrlr);

static device_method_t nvme_ans_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,     nvme_ans_probe),
	DEVMETHOD(device_attach,    nvme_ans_attach),
	DEVMETHOD(device_detach,    nvme_detach),
	DEVMETHOD(device_shutdown,  nvme_shutdown),

	/* NVME interface */
	DEVMETHOD(nvme_delayed_attach,	nvme_ans_delayed_attach),
	DEVMETHOD(nvme_enable,			nvme_ans_enable),
	DEVMETHOD(nvme_sq_enter,		nvme_ans_sq_enter),
	DEVMETHOD(nvme_sq_leave,		nvme_ans_sq_leave),
	DEVMETHOD(nvme_cq_done,			nvme_ans_cq_done),
	DEVMETHOD(nvme_qpair_construct, nvme_ans_qpair_construct),

	DEVMETHOD_END,
};

static driver_t nvme_ans_driver = {
	"nvme",
	nvme_ans_methods,
	sizeof(struct nvme_ans_controller),
};

DRIVER_MODULE(nvme, simplebus, nvme_ans_driver, NULL, NULL);

static int
nvme_ans_sart_map(void *cookie, bus_addr_t addr, bus_size_t size)
{
	struct nvme_ans_controller *sc = cookie;
	
	return (apple_sart_map(sc->sc_sart, addr, size));
}

static struct ofw_compat_data compat_data[] = {
	{"apple,nvme-ans2",	1},
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
	int error;
	struct nvme_ans_controller *sc = ANSDEVICE2SOFTC(dev);
	struct nvme_controller *ctrlr = &sc->sc_ctrlr;
	phandle_t node;

	node = ofw_bus_get_node(dev);

	error = ofw_bus_find_string_index(node, "reg-names", "ans",
		&sc->sc_ans_id);
	if (error != 0) {
		device_printf(dev, "couldn't find 'ans' reg %d\n", error);
		return ENXIO;
	}

	sc->sc_ans = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
		&sc->sc_ans_id, RF_ACTIVE);
	if (sc->sc_ans == NULL) {
		device_printf(dev, "couldn't allocate 'ans' mem resource\n");
		return ENOMEM;
	}

	sc->sc_ans_bus_tag = rman_get_bustag(sc->sc_ans);
	sc->sc_ans_bus_handle = rman_get_bushandle(sc->sc_ans);


	error = ofw_bus_find_string_index(node, "reg-names", "nvme",
		&ctrlr->resource_id);
	if (error != 0) {
		device_printf(dev, "couldn't find 'nvme' reg %d\n", error);
		goto err_find_nvme;
	}

	ctrlr->resource = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
		&ctrlr->resource_id, RF_ACTIVE);
	if (ctrlr->resource == NULL) {
		device_printf(dev, "couldn't allocate 'nvme' mem resource\n");
		error = ENOMEM;
		goto err_alloc_nvme;
	}
	ctrlr->bus_tag = rman_get_bustag(ctrlr->resource);
	ctrlr->bus_handle = rman_get_bushandle(ctrlr->resource);
	ctrlr->regs = (struct nvme_registers *)ctrlr->bus_handle;

	ctrlr->rid = 0;
	ctrlr->res = bus_alloc_resource_any(dev, SYS_RES_IRQ,
		&ctrlr->rid, RF_SHAREABLE | RF_ACTIVE);
	if (ctrlr->res == NULL) {
		device_printf(dev, "couldn't allocate irq resource\n");
		error = ENOMEM;
		goto err_alloc_irq;
	}
	ctrlr->msi_count = 0;
	ctrlr->num_io_queues = 1;
	/*
	 * We're attached via this funky mechanism. Flag the controller so that
	 * it avoids things that can't work when we do that, like asking for
	 * PCI config space entries.
	 */
	ctrlr->quirks |= QUIRK_ANS;

	error = OF_getencprop(node, "apple,sart", &sc->sc_sart,
		sizeof(sc->sc_sart));
	if (error != sizeof(sc->sc_sart)) {
		device_printf(dev, "couldn't find 'apple,sart' property %d\n", error);
		goto err_find_sart;
	}

	sc->sc_rtkit_state = rtkit_init(dev, false);
	if (sc->sc_rtkit_state == NULL) {
		device_printf(dev, "error initializing RTKit\n");
		goto err_rtkit_init;
	}
	rtkit_set_map_callback(sc->sc_rtkit_state, nvme_ans_sart_map, sc);

	error = bus_setup_intr(dev, ctrlr->res, INTR_TYPE_MISC| INTR_MPSAFE, NULL,
		nvme_ctrlr_shared_handler, ctrlr, &ctrlr->tag);
	if (error != 0) {
		device_printf(dev, "couldn't set up interrupt handler %d\n", error);
		goto err_setup_intr;
	}

	// This frees ctrlr resources on failure
	error = nvme_attach(dev);
	if (error != 0) {
		device_printf(dev, "generic nvme_attach failed %d\n", error);
		goto err_ctrlr_attach;
	}

	return 0;

err_rtkit_init:
err_setup_intr:
err_find_sart:
	bus_release_resource(dev, SYS_RES_IRQ, ctrlr->rid, ctrlr->res);
err_alloc_irq:
	bus_release_resource(dev, SYS_RES_MEMORY, ctrlr->resource_id,
		ctrlr->resource);
// This label is out of order since nvme_attach frees ctrlr resources on failure
// TODO: double check nvme_attach
err_ctrlr_attach:
err_alloc_nvme:
err_find_nvme:
	bus_release_resource(dev, SYS_RES_MEMORY, sc->sc_ans_id,
		sc->sc_ans);
	return error;
}

static void
nvme_ans_dmamap_cb(void *arg, bus_dma_segment_t *segs, int nsegs, int error)
{
	struct nvme_ans_qpair *ans_qpair = arg;
	struct nvme_controller *ctrlr = &ans_qpair->sc->sc_ctrlr;

	MPASS(error == 0);
	MPASS(nsegs == 1);

	ans_qpair->addr = segs->ds_addr;
	ans_qpair->size = segs->ds_len;

	bus_size_t base;
	if (ans_qpair->is_admin) {
		base = ANS_NVMMU_BASE_ASQ;
	} else {
		base = ANS_NVMMU_BASE_IOSQ;
	}

	NVME_WRITE_8(ctrlr, base, ans_qpair->addr);

	wakeup(&ans_qpair->addr);
}

static int
nvme_ans_alloc_qpair(device_t dev, uint32_t id, struct nvme_ans_qpair *ans_qpair)
{
	int rc;
	struct nvme_ans_controller *sc = ANSDEVICE2SOFTC(dev);
	struct nvme_controller *ctrlr = &sc->sc_ctrlr;

	if (id == 0) {
		ans_qpair->is_admin = true;
	} else {
		ans_qpair->is_admin = false;
	}
	ans_qpair->addr = 0;
	ans_qpair->size = 0;
	ans_qpair->sc = sc;

	rc = bus_dma_tag_create(bus_get_dma_tag(dev),
		PAGE_SIZE, /* TODO: alignment */
		0, /* TODO: bounds */
		BUS_SPACE_MAXADDR, BUS_SPACE_MAXADDR,
		NULL, NULL,
		ANS_NVMMU_TCB_SIZE,
		1, ANS_NVMMU_TCB_SIZE, /* 1 segment */
		BUS_DMA_COHERENT,
		NULL, NULL,
		&ans_qpair->tag);
	if (rc != 0) {
		nvme_printf(ctrlr, "unable to create dma tag %d\n", rc);
		return rc;
	}

	rc = bus_dmamem_alloc(ans_qpair->tag, &ans_qpair->kva,
		BUS_DMA_WAITOK | BUS_DMA_ZERO | BUS_DMA_COHERENT, &ans_qpair->map);
	if (rc != 0) {
		nvme_printf(ctrlr, "unable to allocate dma mem %d\n", rc);
		return rc;
	}

	rc = bus_dmamap_load(ans_qpair->tag, ans_qpair->map,
		ans_qpair->kva, ANS_NVMMU_TCB_SIZE, nvme_ans_dmamap_cb, ans_qpair, 0);
	if ((rc != 0) && (rc != EINPROGRESS)) {
		nvme_printf(ctrlr, "dma map load failed %d\n", rc);
		return rc;
	}

	bus_dmamap_sync(ans_qpair->tag, ans_qpair->map,
		BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);
	return 0;
}

static int
nvme_ans_delayed_attach(device_t dev, struct nvme_controller *ctrlr)
{
	uint32_t ctrl, status;
	struct nvme_ans_controller *sc = ANSDEVICE2SOFTC(dev);

	ctrl = NVME_ANS_READ_4(sc, ANS_CPU_CTRL);
	NVME_ANS_WRITE_4(sc, ANS_CPU_CTRL, ctrl | ANS_CPU_CTRL_RUN);

	status = NVME_READ_4(ctrlr, ANS_BOOT_STATUS);
	if (status != ANS_BOOT_STATUS_OK) {
		device_printf(dev, "booting rtkit\n");
		rtkit_boot(sc->sc_rtkit_state);
	}

	for (int timo = 0; timo < 100000; timo++) {
		status = NVME_READ_4(ctrlr, ANS_BOOT_STATUS);
		if (status != ANS_BOOT_STATUS_OK) {
			DELAY(1);
		}
	}
	status = NVME_READ_4(ctrlr, ANS_BOOT_STATUS);
	if (status != ANS_BOOT_STATUS_OK) {
		device_printf(dev, "timed out waiting for firmware\n");
		return ENXIO;
	}

	if (nvme_ans_alloc_qpair(dev, 0, &sc->sc_adminq)) {
		device_printf(dev, "unable to allocate dma mem for admin q\n");
		return ENXIO;
	}
	if (nvme_ans_alloc_qpair(dev, 1, &sc->sc_ioq)) {
		device_printf(dev, "unable to allocate dma mem for IO q\n");
		return ENXIO;
	}

	// wait for dmamap callback, probably not necessary
	if (sc->sc_adminq.addr == 0) {
		tsleep(&sc->sc_adminq.addr, PWAIT, "", 5 * hz);
	}
	if (sc->sc_ioq.addr == 0) {
		tsleep(&sc->sc_ioq.addr, PWAIT, "", 5 * hz);
	}

	NVME_WRITE_4(ctrlr, ANS_LINEAR_SQ_CTRL, ANS_LINEAR_SQ_CTRL_EN);
	NVME_WRITE_4(ctrlr, ANS_MAX_PEND_CMDS_CTRL,
		(ANS_MAX_QUEUE_DEPTH << 16) | ANS_MAX_QUEUE_DEPTH);

	ctrl = NVME_READ_4(ctrlr, ANS_UNKNOWN_CTRL);
	NVME_WRITE_4(ctrlr, ANS_UNKNOWN_CTRL, ctrl & ~ANS_PRP_NULL_CHECK);
	device_printf(dev, "finished nvme delayed attach\n");

	return 0;
}

static void
nvme_ans_enable(device_t dev)
{
	struct nvme_ans_controller *sc = ANSDEVICE2SOFTC(dev);
	struct nvme_controller *ctrlr = &sc->sc_ctrlr;

	NVME_WRITE_4(ctrlr, ANS_NVMMU_NUM,
		(ANS_NVMMU_TCB_SIZE / ANS_NVMMU_TCB_PITCH) - 1);
	NVME_WRITE_4(ctrlr, ANS_MODESEL_REG, 0);
}

static uint32_t
nvme_ans_sq_enter(device_t dev, struct nvme_qpair *qpair,
	struct nvme_tracker *tr)
{
	return tr->req->cmd.cid;
}

static struct nvme_ans_nvmmu_tcb *
nvme_ans_qpair_to_tcb(struct nvme_ans_qpair *ans_qpair, struct nvme_tracker *tr)
{
	return (struct nvme_ans_nvmmu_tcb *)(((char *)ans_qpair->kva) + (tr->req->cmd.cid * ANS_NVMMU_TCB_PITCH));
}

static void
nvme_ans_sq_leave(device_t dev, struct nvme_qpair *qpair, struct nvme_tracker *tr)
{
	struct nvme_ans_controller *sc = ANSDEVICE2SOFTC(dev);
	struct nvme_controller *ctrlr = &sc->sc_ctrlr;

	uint16_t id = tr->req->cmd.cid;
	struct nvme_ans_qpair *ans_qpair = &sc->sc_adminq;
	if (qpair->id != 0) {
		ans_qpair = &sc->sc_ioq;
	}
	//MPASS(ans_qpair->qpair == qpair);
	struct nvme_ans_nvmmu_tcb *tcb = nvme_ans_qpair_to_tcb(ans_qpair, tr);

	struct nvme_command *cmd = qpair->cmd;
	cmd += id;

	bus_dmamap_sync(ans_qpair->tag, ans_qpair->map, BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);

	memset(tcb, 0, sizeof(*tcb));
	tcb->tcb_opcode = cmd->opc;
	tcb->tcb_flags = ANS_NVMMU_TCB_WRITE | ANS_NVMMU_TCB_READ;
	tcb->tcb_cid = id;
	tcb->tcb_prpl_len = cmd->cdw12;
	tcb->tcb_prp[0] = cmd->prp1;
	tcb->tcb_prp[1] = cmd->prp2;

	bus_dmamap_sync(ans_qpair->tag, ans_qpair->map, BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

	NVME_WRITE_4(ctrlr, qpair->sq_tdbl_off, id);
}

static void
nvme_ans_cq_done(device_t dev, struct nvme_qpair *qpair, struct nvme_tracker *tr)
{
	struct nvme_ans_controller *sc = ANSDEVICE2SOFTC(dev);
	struct nvme_controller *ctrlr = &sc->sc_ctrlr;

	uint16_t id = tr->req->cmd.cid;
	struct nvme_ans_qpair *ans_qpair = &sc->sc_adminq;
	if (qpair->id != 0) {
		ans_qpair = &sc->sc_ioq;
	}
	//MPASS(ans_qpair->qpair == qpair);
	struct nvme_ans_nvmmu_tcb *tcb = nvme_ans_qpair_to_tcb(ans_qpair, tr);
	uint32_t stat;

	bus_dmamap_sync(ans_qpair->tag, ans_qpair->map, BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);
	memset(tcb, 0, sizeof(*tcb));
	bus_dmamap_sync(ans_qpair->tag, ans_qpair->map, BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

	NVME_WRITE_4(ctrlr, ANS_NVMMU_TCB_INVAL, id);
	stat = NVME_READ_4(ctrlr, ANS_NVMMU_TCB_STAT);
	if (stat != 0) {
		nvme_printf(ctrlr, "nvmmu tcb stat is non-zero %d\n", stat);
	}
}

static int nvme_ans_qpair_construct(device_t dev, struct nvme_qpair *qpair,
	uint32_t num_entries, uint32_t num_trackers, struct nvme_controller *ctrlr)
{
	struct nvme_ans_controller *sc = ANSDEVICE2SOFTC(dev);
	int rc = nvme_qpair_construct(dev, qpair, num_entries, num_trackers, ctrlr);
	if (rc != 0) {
		return rc;
	}
	if (qpair->id == 0) {
		qpair->sq_tdbl_off = ANS_LINEAR_ASQ_DB;
		qpair->cq_hdbl_off = ANS_ACQ_DB;
		sc->sc_adminq.qpair = qpair;
	} else {
		qpair->sq_tdbl_off = ANS_LINEAR_IOSQ_DB;
		qpair->cq_hdbl_off = ANS_IOCQ_DB;
		sc->sc_ioq.qpair = qpair;
	}
	return 0;
}
