/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019-2020 Ruslan Bukin <br@bsdpad.com>
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
#include <sys/gtaskqueue.h>	/* iommu/_task.h */
#include <sys/intr.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <sys/systm.h>
#include <sys/tree.h>
#include <sys/vmem.h>

#include <vm/vm.h>
#include <vm/vm_page.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>
#include <dev/iommu/iommu.h>
#include <arm64/iommu/iommu.h>
#include <arm64/iommu/iommu_pmap.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "apple_dartreg.h"

#include "bus_if.h"
#include "iommu_if.h"

MALLOC_DEFINE(M_APLDART, "apple_dart", "Apple DART (IOMMU)");

#define	DART_MAXADDR	(1ul << 48) - 1

struct dart_cfg {
	u_int shift;
	uint64_t nsid;
};

static const struct dart_cfg t8103_dart_cfg = {
	.shift = 0,
	.nsid = DART_STREAM_MAX,
};

static const struct dart_cfg t6000_dart_cfg = {
	.shift = 4,
	.nsid = DART_STREAM_MAX,
};

static const struct ofw_compat_data compat_data[] = {
	{ "apple,t6000-dart",	(uintptr_t)&t6000_dart_cfg },
	{ "apple,t8103-dart",	(uintptr_t)&t8103_dart_cfg },
	{ NULL,			0  },
};

static struct resource_spec dart_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE | RF_SHAREABLE},
	{ -1, 0 },
};

struct apple_dart_domain;
struct apple_dart_ctx;

struct apple_dart_unit {
	struct iommu_unit		iommu;
	LIST_HEAD(, apple_dart_domain)	domain_list;
};

#if 0
struct apple_dart_ttbr {
	uintptr_t
};
#endif

struct apple_dart_domain {
	struct iommu_domain		iodom;
	struct apple_dart_ctx		*ctx;
	LIST_ENTRY(apple_dart_domain)	next;
	struct apple_dart_softc		*sc;

	size_t				nl1;
	uint64_t			*dom_l1;
	size_t				nl2;
	uint64_t			**dom_l2;
	size_t				ntte;

	uint16_t			sid;
	bool				bypass;
	bool				ready;
};

struct apple_dart_ctx {
	struct iommu_ctx		ioctx;
	struct apple_dart_domain	*domain;
	LIST_ENTRY(apple_dart_ctx)	next;

	device_t			dev;
	bool				bypass;
};

struct apple_dart_softc {
	device_t sc_dev;
	struct mtx sc_mtx;
	struct apple_dart_unit sc_unit;
	struct resource *sc_res[2];
	phandle_t sc_phandle;
	bus_space_tag_t sc_bst;
	bus_space_handle_t sc_bsh;
	bus_dma_tag_t sc_dmat;

	u_int sc_sid_mask;
	u_int sc_nsid;
	u_int sc_shift;
	u_int sc_sid_allocated;
	struct mtx sc_sid_mtx;

	void *sc_intr_cookie;
};

#define	DART_LOCK(_sc)		mtx_lock(&(_sc)->sc_mtx)
#define DART_UNLOCK(_sc)	mtx_unlock(&(_sc)->sc_mtx)

#define	DART_SID_LOCK(_sc)	mtx_lock_spin(&(_sc)->sc_sid_mtx)
#define DART_SID_UNLOCK(_sc)	mtx_unlock_spin(&(_sc)->sc_sid_mtx)

#define DART_READ(sc, reg) \
	bus_space_read_4((sc)->sc_bst, (sc)->sc_bsh, (reg))
#define	DART_WRITE(sc, reg, val) \
	bus_space_write_4((sc)->sc_bst, (sc)->sc_bsh, (reg), (val))

static void
apple_dart_flush_tlb_sid(struct apple_dart_softc *sc, u_int sid)
{
	dsb(sy);
	isb();

	DART_LOCK(sc);
	DART_WRITE(sc, DART_TLB_OP_SIDMASK, sid);
	DART_WRITE(sc, DART_TLB_OP, DART_TLB_OP_FLUSH);
	while ((DART_READ(sc, DART_TLB_OP) & DART_TLB_OP_BUSY) != 0) {
		__asm volatile ("yield" ::: "memory");
	}
	DART_UNLOCK(sc);
}

static void
apple_dart_flush_tlb(struct apple_dart_softc *sc)
{

	return (apple_dart_flush_tlb_sid(sc, sc->sc_sid_mask));
}

#if 0
static struct apple_dart_dma *
apple_dart_dma_alloc(bus_dma_tag_t dmat, bus_size_t size, bus_size_t align)
{
	struct apple_dart_dma *dma;
	int nsegs, error;

	dma = malloc(sizeof(*dma), M_APLDART, M_WAITOK | M_ZERO);
	dma->dma_size = size;

	error = bus_dmamem_alloc(dmat, size, align, 0, &dma->dma_seg, 1,
	    &nsegs, BUS_DMA_WAITOK);
	if (error != 0) {
		goto destroy;
	}

	error = bus_dmamem_map(dmat, &dma->dma_seg, nsegs, size,
	    &dma->dma_kva, BUS_DMA_WAITOK | BUS_DMA_NOCACHE);
	if (error != 0) {
		goto free;
	}

	error = bus_dmamap_create(dmat, size, 1, size, 0,
	    BUS_DMA_WAITOK | BUS_DMA_ALLOCNOW, &dma->dma_map);
	if (error != 0) {
		goto dmafree;
	}

	error = bus_dmamap_load(dmat, dma->dma_map, dma->dma_kva, size,
	    NULL, BUS_DMA_WAITOK);
	if (error != 0) {
		goto unmap;
	}

	memset(dma->dma_kva, 0, size);

	return dma;

destroy:
	bus_dmamap_destroy(dmat, dma->dma_map);
unmap:
	bus_dmamem_unmap(dmat, dma->dma_kva, size);
free:
	bus_dmamem_free(dmat, &dma->dma_seg, 1);
dmafree:
	free(dma, M_APLDART);
	return NULL;
}
#endif

static int
apple_dart_intr(void *priv)
{
	struct apple_dart_softc * const sc = priv;
	uint64_t addr;
	uint32_t status;

	status = DART_READ(sc, DART_ERR_STATUS);
	addr = DART_READ(sc, DART_ERR_ADDRL);
	addr |= (uint64_t)DART_READ(sc, DART_ERR_ADDRH) << 32;
	DART_WRITE(sc, DART_ERR_STATUS, status);

	device_printf(sc->sc_dev, "error addr 0x%016lx status 0x%08x\n",
	    addr, status);

	return 1;
}

static volatile uint64_t *
apple_dart_lookup_tte(struct apple_dart_domain *dom, bus_addr_t dva)
{
	int idx = dva / DART_PAGE_SIZE;
	int l2_idx = idx / (DART_PAGE_SIZE / sizeof(uint64_t));
	int tte_idx = idx % (DART_PAGE_SIZE / sizeof(uint64_t));
	volatile uint64_t *l2;

	l2 = dom->dom_l2[l2_idx];
	return (&l2[tte_idx]);
}

static int
apple_dart_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Apple DART");

	return (BUS_PROBE_DEFAULT);
}

static int
apple_dart_attach_iommu(struct apple_dart_softc *const sc)
{
	struct apple_dart_unit *unit;
	struct iommu_unit *iommu;

	unit = &sc->sc_unit;
	LIST_INIT(&unit->domain_list);

	iommu = &unit->iommu;
	iommu->dev = sc->sc_dev;

	return (iommu_register(iommu));
}

static int
apple_dart_attach(device_t dev)
{
	struct apple_dart_softc * const sc = device_get_softc(dev);
	const phandle_t phandle = ofw_bus_get_node(dev);
	const struct dart_cfg *dcfg;
	u_int config, idx, params2, sid;
	int error;
#if 1
	bool bypass;
#endif

	error = 0;
	if (bus_alloc_resources(dev, dart_spec, sc->sc_res) != 0) {
		device_printf(dev, "cannot allocate resources for device\n");
		return (ENXIO);
	}

	sc->sc_dev = dev;
	sc->sc_phandle = phandle;
	sc->sc_bst = rman_get_bustag(sc->sc_res[0]);
	sc->sc_bsh = rman_get_bushandle(sc->sc_res[0]);
	sc->sc_dmat = bus_get_dma_tag(dev);

	dcfg = (const struct dart_cfg *)ofw_bus_search_compatible(dev,
	    compat_data)->ocd_data;

	sc->sc_shift = dcfg->shift;
	sc->sc_nsid = dcfg->nsid;

	config = DART_READ(sc, DART_CONFIG);
	if ((config & DART_CONFIG_LOCK) != 0) {
		device_printf(dev, "locked\n");
		bus_release_resources(dev, dart_spec, sc->sc_res);
		return (ENXIO);
	}

	params2 = DART_READ(sc, DART_PARAMS2);
#if 1
	bypass = ((params2 & DART_PARAMS2_BYPASS_SUPPORT) != 0);
	if (bypass) {
		for (sid = 0; sid < sc->sc_nsid; sid++) {
			DART_WRITE(sc, DART_TCR(sid),
			    DART_TCR_BYPASS_DART | DART_TCR_BYPASS_DAPF);
			for (idx = 0; idx < DART_L1_IDX_MAX; idx++) {
				DART_WRITE(sc, DART_TTBR(sid, idx), 0);
			}
		}

		device_printf(dev, "configured in bypass mode\n");
		/* XXX */
		return (0);

	} else {
		/* IOMMU bits don't work yet. */
		return (ENXIO);
	}
#endif

	mtx_init(&sc->sc_mtx, device_get_nameunit(dev), "apple_dart", MTX_DEF);
	mtx_init(&sc->sc_sid_mtx, "asid alloc", NULL, MTX_SPIN);

#if 0
	if (bus_space_map(sc->sc_bst, addr, size, 0, &sc->sc_bsh) != 0) {
		aprint_error(": couldn't map registers\n");
		return;
	}
#endif
	if (OF_getencprop(phandle, "sid-mask", &sc->sc_sid_mask,
	    sizeof(sc->sc_sid_mask)) <= 0)
		sc->sc_sid_mask = DART_STREAM_MASK;

	if (bootverbose)
		device_printf(dev, "%u SIDs (mask 0x%x)\n", sc->sc_nsid,
		    sc->sc_sid_mask);

	MPASS(sc->sc_nsid == DART_STREAM_MAX);
	MPASS(sc->sc_sid_mask == DART_STREAM_MASK);	/* For now */

	/* Disable translations */
	for (sid = 0; sid < sc->sc_nsid; sid++) {
		DART_WRITE(sc, DART_TCR(sid), 0);
	}

	/* Remove page tables */
	for (sid = 0; sid < sc->sc_nsid; sid++) {
		for (idx = 0; idx < DART_L1_IDX_MAX; idx++) {
			DART_WRITE(sc, DART_TTBR(sid, idx), 0);
		}
	}

	apple_dart_flush_tlb(sc);

	if (bus_setup_intr(dev, sc->sc_res[1], INTR_TYPE_MISC, apple_dart_intr,
	    NULL, sc, &sc->sc_intr_cookie) != 0) {
		device_printf(dev, "Failed to setup interrupt handler\n");
		error = ENXIO;
		goto out;
	}

	error = apple_dart_attach_iommu(sc);
	if (error != 0) {
		device_printf(dev, "Failed to setup iommu contxt.\n");
		error = ENXIO;
		goto out;
	}

	OF_device_register_xref(OF_xref_from_node(phandle), dev);

out:
	if (error != 0)
		bus_release_resources(dev, dart_spec, sc->sc_res);

	return (error);
}

static int
apple_dart_find(device_t dev, device_t child)
{
	phandle_t iommu_node, ctrl_node;
	device_t ctrl;
	devclass_t pci_class;
	int error;

	/*
	 * XXX This whole thing is super ugly, and it really belongs as part of
	 * iommu_find(); ideally we'd figure out if we have an iommu-map to
	 * consult so that we can land precisely on the iommu device we need
	 * rather than having to search all IOMMUs.
	 */
	pci_class = devclass_find("pci");
	if (device_get_devclass(device_get_parent(child)) != pci_class)
		return (ENXIO);

	ctrl = pci_find_pcie_root_port(child);
	ctrl = device_get_parent(device_get_parent(ctrl));
	ctrl_node = ofw_bus_get_node(ctrl);
	if (ctrl_node <= 0)
		return (ENXIO);

	error = ofw_bus_iommu_map(ctrl_node, pci_get_rid(child), &iommu_node,
	    NULL);
	if (error != 0 || OF_device_from_xref(iommu_node) != dev)
		return (ENXIO);

	return (0);
}

static int
apple_dart_sid_acquire(struct apple_dart_domain *dom, int sid, bool bypass)
{
	struct apple_dart_softc *sc;
	vm_paddr_t pa;

	MPASS(!dom->ready);
	sc = dom->sc;
	DART_SID_LOCK(sc);
	MPASS((sc->sc_sid_allocated & (1 << sid)) == 0);

	if (sid == 0 || sid > DART_STREAM_MAX) {
		DART_SID_UNLOCK(sc);
		return (EINVAL);
	}

	/* Install page tables. */
	pa = vtophys(dom->dom_l1);
	for (int idx = 0; idx < dom->nl1; idx++) {
		DART_WRITE(sc, DART_TTBR(sid, idx),
		   (pa >> DART_TTBR_SHIFT) | DART_TTBR_VALID);
		pa += DART_PAGE_SIZE;
	}

	if (bypass)
		DART_WRITE(sc, DART_TCR(sid),
		    DART_TCR_BYPASS_DART | DART_TCR_BYPASS_DAPF);
	else
		DART_WRITE(sc, DART_TCR(sid),
		    DART_TCR_TXEN);

	sc->sc_sid_allocated |= (1 << sid);
	dom->sid = sid;
	dom->bypass = bypass;
	dom->ready = true;

	DART_SID_UNLOCK(sc);

	apple_dart_flush_tlb_sid(sc, sid);

	return (0);
}

static void
apple_dart_sid_release(struct apple_dart_domain *dom)
{
	struct apple_dart_softc *sc;
	int sid;

	MPASS(dom->ready);

	sc = dom->sc;
	sid = dom->sid;
	DART_SID_LOCK(sc);
	sc->sc_sid_allocated &= ~(1 << sid);

	DART_WRITE(sc, DART_TCR(sid), 0);
	for (int idx = 0; idx < DART_L1_IDX_MAX; idx++) {
		DART_WRITE(sc, DART_TTBR(sid, idx), 0);
	}

	DART_SID_UNLOCK(sc);

	apple_dart_flush_tlb_sid(sc, sid);
}

static int
apple_dart_map(device_t dev, struct iommu_domain *iodom,
    vm_offset_t va, vm_page_t *ma, vm_size_t size,
    vm_prot_t prot)
{
	struct apple_dart_domain *domain;
	struct apple_dart_softc *sc;
	volatile uint64_t *tte;
	vm_paddr_t pa;
	int i;

	sc = device_get_softc(dev);

	domain = (struct apple_dart_domain *)iodom;

#if 0
	dprintf("%s: %lx -> %lx, %ld, domain %d\n", __func__, va, pa, size,
	    domain->asid);
#endif

	size = DART_ROUND_PAGE(size);
	/*
	 * XXX PAGE_SIZE != DART_PAGE_SIZE?
	 */
	for (i = 0; size > 0; size -= DART_PAGE_SIZE) {
		device_printf(dev, "size == %jx\n", (uintmax_t)size);
		pa = DART_TRUNC_PAGE(VM_PAGE_TO_PHYS(ma[i++]));

		tte = apple_dart_lookup_tte(domain, va);
		device_printf(dev, "tte == %jx\n", (uintmax_t)(uintptr_t)tte);
		*tte = (pa >> sc->sc_shift) | DART_L2_PAGE;

#if 0
		error = pmap_smmu_enter(&domain->p, va, pa, prot, 0);
		if (error)
			return (error);
#endif
		apple_dart_flush_tlb_sid(sc, domain->sid);
		va += DART_PAGE_SIZE;
	}

	return (0);
}


static int
apple_dart_unmap(device_t dev, struct iommu_domain *iodom,
    vm_offset_t va, bus_size_t size)
{
	struct apple_dart_domain *domain;
	struct apple_dart_softc *sc;
	int err;
	int i;

	sc = device_get_softc(dev);

	domain = (struct apple_dart_domain *)iodom;

	err = 0;

#if 0
	dprintf("%s: %lx, %ld, domain %d\n", __func__, va, size, domain->asid);
#endif

	for (i = 0; i < size; i += PAGE_SIZE) {
#if 0
		if (pmap_smmu_remove(&domain->p, va) == 0) {
			/* pmap entry removed, invalidate TLB. */
			apple_dart_flush_tlb_sid(sc, domain->asid);
		} else {
			err = ENOENT;
			break;
		}
#endif
		va += PAGE_SIZE;
	}

	return (err);
}

static struct iommu_domain *
apple_dart_domain_alloc(device_t dev, struct iommu_unit *iommu)
{
	struct apple_dart_domain *domain;
	struct apple_dart_unit *unit;
	struct apple_dart_softc *sc;
	vm_paddr_t pa;

	device_printf(dev, "allocating new domain\n");
	sc = device_get_softc(dev);

	unit = (struct apple_dart_unit *)iommu;

	domain = malloc(sizeof(*domain), M_APLDART, M_WAITOK | M_ZERO);
	domain->ntte = howmany(DART_DVA_END, DART_PAGE_SIZE);
	domain->nl2 = howmany(domain->ntte,
	    DART_PAGE_SIZE / sizeof(**domain->dom_l2));
	domain->nl1 = howmany(domain->nl2,
	    DART_PAGE_SIZE / sizeof(*domain->dom_l1));

	MPASS(domain->nl1 > 0 && domain->nl1 <= DART_L1_IDX_MAX);

	domain->iodom.end = DART_DVA_END;
	domain->sc = sc;
	domain->dom_l1 = (uint64_t *)contigmalloc(domain->nl1 * DART_PAGE_SIZE,
	    M_APLDART, M_WAITOK, 0, DART_MAXADDR, DART_PAGE_SIZE, 0);
	domain->dom_l2 = mallocarray(domain->nl2, sizeof(*domain->dom_l2),
	    M_APLDART, M_WAITOK);
	for (int idx = 0; idx < domain->nl2; idx++) {
		domain->dom_l2[idx] = (uint64_t *)contigmalloc(DART_PAGE_SIZE,
		    M_APLDART, M_WAITOK | M_ZERO, 0, DART_MAXADDR,
		    DART_PAGE_SIZE, 0);
		pa = vtophys(domain->dom_l2[idx]);

		/* XXX WRONG */
		domain->dom_l1[idx] = (pa >> sc->sc_shift) | DART_L1_TABLE;
	}

	/*
	 * Note that the domain isn't usable until the ioctx is created, which
	 * assigns the domain a sid based on the iommu-map.
	 */

	IOMMU_LOCK(iommu);
	LIST_INSERT_HEAD(&unit->domain_list, domain, next);
	IOMMU_UNLOCK(iommu);

	return (&domain->iodom);
}

static void
apple_dart_domain_free(device_t dev, struct iommu_domain *iodom)
{
	struct apple_dart_domain *domain;
	struct apple_dart_softc *sc;

	sc = device_get_softc(dev);

	domain = (struct apple_dart_domain *)iodom;

	LIST_REMOVE(domain, next);

	for (int idx = 0; idx < domain->nl2; ++idx) {
		contigfree(domain->dom_l2[idx], DART_PAGE_SIZE, M_APLDART);
	}

	free(domain->dom_l2, M_APLDART);
	contigfree(domain->dom_l1, domain->nl1 * DART_PAGE_SIZE, M_APLDART);
	free(domain, M_APLDART);
}

#include <sys/stack.h>

static struct iommu_ctx *
apple_dart_ctx_alloc(device_t dev, struct iommu_domain *iodom, device_t child,
    bool disabled)
{
	struct apple_dart_domain *domain;
	struct apple_dart_softc *sc;
	struct apple_dart_ctx *ctx;
#if 0
	device_t iommudev;
#endif
	phandle_t node;
	uint16_t rid;
	u_int sid;
#if 0
	int err;
	struct stack st;
#endif

	sc = device_get_softc(dev);
	domain = (struct apple_dart_domain *)iodom;
	node = ofw_bus_get_node(device_get_parent(child));

	rid = pci_get_rid(child);

#if 0
	device_printf(dev, "allocate ctx\n");
	stack_save(&st);
	stack_print(&st);
	device_printf(dev, "\n");
#endif

	for (sid = 0; sid < sc->sc_nsid; sid++) {
		u_int mask = (1 << sid);

		if ((sc->sc_sid_mask & mask) == 0)
			continue;
		if ((sc->sc_sid_allocated & mask) == 0)
			break;
	}

	if (sid == sc->sc_nsid) {
		device_printf(dev, "exhausted sid space\n");
		return (NULL);
	}

#if 0
	err = ofw_bus_iommu_map(node, rid, &iommudev, &sid);
	if (err != 0)
		return (NULL);
#endif

	if (!apple_dart_sid_acquire(domain, sid, disabled)) {
		return (NULL);
	}

	ctx = malloc(sizeof(struct apple_dart_ctx), M_APLDART,
	    M_WAITOK | M_ZERO);
	ctx->dev = child;
	ctx->domain = domain;

	/*
	 * dart can only cope with 1:1 domain:ctx.  SIDs are allocated by FDT
	 * data, so this construction is a little awkward.
	 */
	IOMMU_DOMAIN_LOCK(iodom);
	MPASS(domain->ctx == NULL);
	domain->ctx = ctx;
	IOMMU_DOMAIN_UNLOCK(iodom);

	return (&ctx->ioctx);
}

static int
apple_dart_ctx_init(device_t dev, struct iommu_ctx *ioctx)
{

	/* part of ctx_alloc should probably move into here */
	return (0);
}

static void
apple_dart_ctx_free(device_t dev, struct iommu_ctx *ioctx)
{
	struct apple_dart_softc *sc;
	struct apple_dart_ctx *ctx;

	IOMMU_ASSERT_LOCKED(ioctx->domain->iommu);

	sc = device_get_softc(dev);
	ctx = (struct apple_dart_ctx *)ioctx;

	LIST_REMOVE(ctx, next);

	apple_dart_sid_release(ctx->domain);
	free(ctx, M_APLDART);
}

static struct apple_dart_ctx *
apple_dart_ctx_lookup_by_sid(device_t dev, u_int sid)
{
	struct apple_dart_softc *sc;
	struct apple_dart_domain *domain;
	struct apple_dart_unit *unit;

	sc = device_get_softc(dev);

	unit = &sc->sc_unit;

	LIST_FOREACH(domain, &unit->domain_list, next) {
		if (domain->sid == sid)
			return (domain->ctx);
	}

	return (NULL);
}

static struct iommu_ctx *
apple_dart_ctx_lookup(device_t dev, device_t child)
{
	struct iommu_unit *iommu;
	struct apple_dart_softc *sc;
	struct apple_dart_domain *domain;
	struct apple_dart_unit *unit;

	sc = device_get_softc(dev);

	unit = &sc->sc_unit;
	iommu = &unit->iommu;

	IOMMU_ASSERT_LOCKED(iommu);

	LIST_FOREACH(domain, &unit->domain_list, next) {
		IOMMU_DOMAIN_LOCK(&domain->iodom);

		if (domain->ctx != NULL && domain->ctx->dev == child) {
			IOMMU_DOMAIN_UNLOCK(&domain->iodom);
			return (&domain->ctx->ioctx);
		}
		IOMMU_DOMAIN_UNLOCK(&domain->iodom);
	}

	return (NULL);
}

static device_method_t apple_dart_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		apple_dart_probe),
	DEVMETHOD(device_attach,	apple_dart_attach),

	/* IOMMU interface */
	DEVMETHOD(iommu_find,		apple_dart_find),
	DEVMETHOD(iommu_map,		apple_dart_map),
	DEVMETHOD(iommu_unmap,		apple_dart_unmap),
	DEVMETHOD(iommu_domain_alloc,	apple_dart_domain_alloc),
	DEVMETHOD(iommu_domain_free,	apple_dart_domain_free),
	DEVMETHOD(iommu_ctx_alloc,	apple_dart_ctx_alloc),
	DEVMETHOD(iommu_ctx_init,	apple_dart_ctx_init),
	DEVMETHOD(iommu_ctx_free,	apple_dart_ctx_free),
	DEVMETHOD(iommu_ctx_lookup,	apple_dart_ctx_lookup),

	DEVMETHOD_END
};

static driver_t apple_dart_driver = {
	"dart",
	apple_dart_methods,
	sizeof(struct apple_dart_softc),
};

EARLY_DRIVER_MODULE(apple_dart, simplebus, apple_dart_driver,
    0, 0, BUS_PASS_INTERRUPT + BUS_PASS_ORDER_MIDDLE);
