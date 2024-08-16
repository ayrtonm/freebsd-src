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
#include <sys/stack.h>
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

#include "apple_dart_reg.h"

#include "bus_if.h"
#include "iommu_if.h"

MALLOC_DEFINE(M_APLDART, "apple_dart", "Apple DART (IOMMU)");

#define	DART_MAXADR		(1 << 48) - 1

#define	DART_LOCK(_sc)		mtx_lock(&(_sc)->sc_mtx)
#define DART_UNLOCK(_sc)	mtx_unlock(&(_sc)->sc_mtx)

#define	DART_SID_LOCK(_sc)	mtx_lock_spin(&(_sc)->sc_sid_mtx)
#define DART_SID_UNLOCK(_sc)	mtx_unlock_spin(&(_sc)->sc_sid_mtx)

#define DART_READ(sc, reg) \
	bus_space_read_4((sc)->sc_bst, (sc)->sc_bsh, (reg))
#define	DART_WRITE(sc, reg, val) \
	bus_space_write_4((sc)->sc_bst, (sc)->sc_bsh, (reg), (val))

struct dart_cfg {
	int64_t		nsid;
	int64_t		ias;
	uint32_t	nttbr;
	uint32_t	shift;
};

static const struct dart_cfg t6000_dart_cfg = {
	.nsid = DART_T6000_STREAMS,
	.ias = 32,
	.nttbr = 4,
	.shift = 4,
};

static const struct dart_cfg t8103_dart_cfg = {
	.nsid = DART_T6000_STREAMS,
	.ias = 32,
	.nttbr = 4,
	.shift = 0,
};

static const struct dart_cfg t8110_dart_cfg = {
	.nsid = -1, // get this from params4
	.ias = -1, // get this from params3
	.nttbr = 1,
	.shift = 4,
};

static const struct ofw_compat_data compat_data[] = {
	{ "apple,t6000-dart",	(uintptr_t)&t6000_dart_cfg },
	{ "apple,t8103-dart",	(uintptr_t)&t8103_dart_cfg },
	{ "apple,t8110-dart",	(uintptr_t)&t8110_dart_cfg },
	{ NULL,			0  },
};

enum {
	APPLE_DART_MEMRES = 0,
	APPLE_DART_IRQRES,
	APPLE_DART_NRES,
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
	struct mtx sc_sid_mtx;
	struct apple_dart_unit sc_unit;
	struct resource *sc_res[APPLE_DART_NRES];
	phandle_t sc_phandle;
	bus_space_tag_t sc_bst;
	bus_space_handle_t sc_bsh;
	bus_dma_tag_t sc_dmat;

	uint32_t	sc_ias;
	uint32_t	sc_nsid;
	uint32_t	sc_nttbr;
	uint32_t	sc_shift;
	bool		sc_is_t8110;

	bus_addr_t	sc_sid_enable_base;

	bus_addr_t	sc_tcr_base;
	uint32_t	sc_tcr_bypass;
	uint32_t	sc_tcr_translate_enable;

	bus_addr_t	sc_ttbr_base;
	uint32_t	sc_ttbr_valid;

	bus_addr_t	sc_tlb_op_base;
	uint32_t	sc_tlb_op_flush;
	uint32_t	sc_tlb_op_busy;

	bus_addr_t	sc_error_status;
	bus_addr_t	sc_error_addr_lo;
	bus_addr_t	sc_error_addr_hi;

	bool		sc_locked;
	bool		sc_translating;

	// will use to handle ndoes with apple,dma-range prop
	bus_addr_t	sc_dva_base;
	bus_addr_t	sc_dva_end;
	bus_addr_t	sc_dva_mask;

	void *sc_intr_cookie;
};

static void
apple_dart_flush_tlb_sid(struct apple_dart_softc *sc, u_int sid)
{
	uint32_t mask, op;
	dsb(sy);
	isb();

	DART_LOCK(sc);

	if (!sc->sc_is_t8110) {
		if (sid == -1) {
			mask = DART_ALL_STREAMS(sc);
		} else {
			mask = 1 << sid;
		}
		DART_WRITE(sc, DART_T6000_TLB_SIDMASK, mask);
	}

	op = sc->sc_tlb_op_flush;

	if (sc->sc_is_t8110) {
		if (sid == -1) {
			op = DART_T8110_TLB_OP_FLUSH_ALL;
		} else {
			op |= sid;
		}
	}

	DART_WRITE(sc, sc->sc_tlb_op_base, op);
	while ((DART_READ(sc, sc->sc_tlb_op_base) & sc->sc_tlb_op_busy) != 0) {
		__asm volatile ("yield" ::: "memory");
	}
	DART_UNLOCK(sc);
}

static int
apple_dart_intr(void *priv)
{
	struct apple_dart_softc * const sc = priv;
	uint64_t addr;
	uint32_t status;

	status = DART_READ(sc, sc->sc_error_status);
	addr = DART_READ(sc, sc->sc_error_addr_lo);
	addr |= (uint64_t)DART_READ(sc, sc->sc_error_addr_hi) << 32;
	DART_WRITE(sc, sc->sc_error_status, status);

	device_printf(sc->sc_dev, "error addr 0x%016lx status 0x%08x\n",
	    addr, status);

	// kevans originally had 1 (FILTER_STRAY). That kind of makes sense since
	// this device did not actually trigger the interrupt but that think it
	// disables further interrupts so maybe handled makes more sense
	return (FILTER_HANDLED);
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
	uint32_t params[4] = {0};
	uint32_t config, /*maj, min,*/ tcr, ttbr;
	u_int idx, sid;
	int error;
	bool bypass;

	if (bus_alloc_resources(dev, dart_spec, sc->sc_res) != 0) {
		device_printf(dev, "cannot allocate resources for device\n");
		return (ENXIO);
	}

	sc->sc_dev = dev;
	sc->sc_phandle = phandle;
	sc->sc_bst = rman_get_bustag(sc->sc_res[APPLE_DART_MEMRES]);
	sc->sc_bsh = rman_get_bushandle(sc->sc_res[APPLE_DART_MEMRES]);
	sc->sc_dmat = bus_get_dma_tag(dev);

	dcfg = (const struct dart_cfg *)ofw_bus_search_compatible(dev,
	    compat_data)->ocd_data;

	if (dcfg == &t8110_dart_cfg) {
		config = DART_READ(sc, DART_T8110_PROTECT);
		sc->sc_locked = config & DART_T8110_PROTECT_TTBR_TCR;
	} else {
		config = DART_READ(sc, DART_T6000_CONFIG);
		sc->sc_locked = config & DART_T6000_CONFIG_LOCK;
	}
	//if (sc->sc_locked) {
	//	//device_printf(dev, "locked, config: %x\n", config);
	//	bus_release_resources(dev, dart_spec, sc->sc_res);
	//	return (ENXIO);
	//}

	//maj = 0;
	//min = 0;
	params[0] = DART_READ(sc, DART_PARAMS1);
	params[1] = DART_READ(sc, DART_PARAMS2);
	if (dcfg == &t8110_dart_cfg) {
		sc->sc_is_t8110 = true;
		params[2] = DART_READ(sc, DART_PARAMS3);
		params[3] = DART_READ(sc, DART_PARAMS4);
		//maj = DART_T8110_PARAMS3_REV_MAJ(params[2]);
		//min = DART_T8110_PARAMS3_REV_MIN(params[2]);
		sc->sc_nsid = params[3] & DART_T8110_PARAMS4_NSID;
		sc->sc_ias = DART_T8110_PARAMS3_VA_WIDTH(params[2]);

		sc->sc_sid_enable_base = DART_T8110_SID_ENABLE_BASE;

		sc->sc_tcr_base = DART_T8110_TCR_BASE;
		sc->sc_tcr_bypass = DART_T8110_TCR_BYPASS_DAPF |
			DART_T8110_TCR_BYPASS_DART;
		sc->sc_tcr_translate_enable = DART_T8110_TCR_TRANSLATE;

		sc->sc_ttbr_base = DART_T8110_TTBR_BASE;
		sc->sc_ttbr_valid = DART_T8110_TTBR_VALID;

		sc->sc_tlb_op_base = DART_T8110_TLB_OP_BASE;
		sc->sc_tlb_op_flush = DART_T8110_TLB_OP_FLUSH;
		sc->sc_tlb_op_busy = DART_T8110_TLB_OP_BUSY;

		sc->sc_error_status = DART_T8110_ERROR_STATUS;
		sc->sc_error_addr_lo = DART_T8110_ERROR_ADDR_LO;
		sc->sc_error_addr_hi = DART_T8110_ERROR_ADDR_HI;
	} else {
		sc->sc_is_t8110 = false;
		sc->sc_nsid = dcfg->nsid;
		sc->sc_ias = dcfg->ias;

		sc->sc_sid_enable_base = DART_T6000_SID_ENABLE_BASE;

		sc->sc_tcr_base = DART_T6000_TCR_BASE;
		sc->sc_tcr_bypass = DART_T6000_TCR_BYPASS_DAPF |
			DART_T6000_TCR_BYPASS_DART;
		sc->sc_tcr_translate_enable = DART_T6000_TCR_TRANSLATE;

		sc->sc_ttbr_base = DART_T6000_TTBR_BASE;
		sc->sc_ttbr_valid = DART_T6000_TTBR_VALID;

		sc->sc_tlb_op_base = DART_T6000_TLB_OP_BASE;
		sc->sc_tlb_op_flush = DART_T6000_TLB_OP_FLUSH;
		sc->sc_tlb_op_busy = DART_T6000_TLB_OP_BUSY;

		sc->sc_error_status = DART_T6000_ERROR_STATUS;
		sc->sc_error_addr_lo = DART_T6000_ERROR_ADDR_LO;
		sc->sc_error_addr_hi = DART_T6000_ERROR_ADDR_HI;
	}
	sc->sc_shift = dcfg->shift;
	sc->sc_nttbr = dcfg->nttbr;

	//device_printf(dev, " found rev %d.%d with %d bits\n", maj, min, sc->sc_ias);
	if (sc->sc_ias > 36) {
		sc->sc_ias = 36;
	}
	bypass = params[1] & DART_PARAMS2_BYPASS_SUPPORT;
	if (!bypass) {
		return 0;
	}

	for (sid = 0; sid < sc->sc_nsid; sid++) {
		tcr = DART_READ(sc, DART_TCR(sc, sid));
		if ((tcr & sc->sc_tcr_translate_enable) == 0)
			continue;

		for (idx = 0; idx < sc->sc_nttbr; idx++) {
			ttbr = DART_READ(sc, DART_TTBR(sc, sid, idx));
			if (ttbr & sc->sc_ttbr_valid)
				sc->sc_translating = true;
		}
	}
	//bypass = params[1] & DART_PARAMS2_BYPASS_SUPPORT;

	//if (bypass && !sc->sc_locked && !sc->sc_translating) {
		for (sid = 0; sid < sc->sc_nsid; sid++) {
			DART_WRITE(sc, DART_TCR(sc, sid), sc->sc_tcr_bypass);
		}
		//device_printf(dev, "bypass\n");
		// return config failure when bypassing the IOMMU seems wrong...
		// especially given that this ties into the general iommu interface
		return (0);
	//}

	// not yet

	// openbsd checks apple,dma-range here, but I don't need that for mtp dart
	sc->sc_dva_base = DART_DVA_START;
	sc->sc_dva_end = DART_DVA_END;
	sc->sc_dva_mask = (1 << sc->sc_ias) - 1;

	if (!sc->sc_locked && sc->sc_translating) {
		for (sid = 0; sid < sc->sc_nsid; sid++) {
			DART_WRITE(sc, DART_TCR(sc, sid), 0);
		}

		for (sid = 0; sid < sc->sc_nsid; sid++) {
			for (idx = 0; idx < sc->sc_nttbr; idx++) {
				DART_WRITE(sc, DART_TTBR(sc, sid, idx), 0);
			}
		}
		apple_dart_flush_tlb_sid(sc, -1);
	}

	DART_WRITE(sc, sc->sc_error_status, DART_READ(sc, sc->sc_error_status));
	if (sc->sc_is_t8110) {
		DART_WRITE(sc, DART_T8110_ERROR_MASK, 0);
	}

	mtx_init(&sc->sc_mtx, device_get_nameunit(dev), "apple_dart", MTX_DEF);
	mtx_init(&sc->sc_sid_mtx, "asid alloc", NULL, MTX_SPIN);

	if (bus_setup_intr(dev, sc->sc_res[APPLE_DART_IRQRES], INTR_TYPE_MISC,
		apple_dart_intr, NULL, sc, &sc->sc_intr_cookie) != 0) {
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

	return (0);

out:
	bus_release_resources(dev, dart_spec, sc->sc_res);
	return (error);
}

static int
apple_dart_detach(device_t dev)
{
	panic("uh oh");
}

static int
apple_dart_find(device_t dev, device_t child)
{
	phandle_t node, iommu, self;
	int error;

	self = ofw_bus_get_node(dev);
	node = ofw_bus_get_node(child);

	error = OF_getencprop(node, "iommus", &iommu, sizeof(iommu));
	if (error != sizeof(iommu)) {
		return (ENXIO);
	}
	if (self == iommu) {
		return (0);
	}
	return (ENXIO);
}

#if 0
static struct iommut_unit *
apple_dart_domain_alloc(device_t dev, struct iommu_unit *iommu)
{
	struct apple_dart_domain *domain;
	struct apple_dart_softc *sc;
	int idx;

	sc = device_get_softc(dev);
	domain = malloc(sizeof(*domain), M_APLDART, M_WAITOK | M_ZERO);

	// original code counted the first page even though DART_DVA_START skips it
	// how many pages fit in the lower 4gb excluding the first and last pages
	domain->ntte = howmany(sc->sc_dva_end & sc->sc_dva_mask, DART_PAGE_SIZE) - 1;

	// how many (dom_l2's per page) are needed for the number of translation table entry
	domain->nl2 = howmany(domain->ntte,
		DART_PAGE_SIZE / sizeof(**domain->dom_l2));
	// how many (dom_l1's per page) are needed for the number of l2 entries
	domain->nl1 = howmany(domain->nl2,
		DART_PAGE_SIZE / sizeof(*domain->dom_l1));

	MPASS(domain->nl1 > 0 && domain->nl1 <= DART_L1_IDX_MAX);

	domain->iodom.end = DART_DVA_END;
	domain->sc = sc;
	domain->dom_l1 = (uint64_t *)contigmalloc(domain->nl1 * DART_PAGE_SIZE,
		M_APLDART, M_WAITOK, 0, DART_MAXADR, DART_PAGE_SIZE, 0);
	domain->dom_l2 = mallocarray(domain->nl2, sizeof(*domain->dom_l2),
		M_APLDART, M_WAITOK);
	for (idx = 0; idx < domain->nl2; idx++) {
		domain->dom_l2[idx] = (uint64_t *)contigmalloc(DART_PAGE_SIZE,
			M_APLDART, M_WAITOK | M_ZERO, 0, DART_MAXADDR,
			DART_PAGE_SIZE, 0);
		pa = vtophys(domain->dom_l2[idx]);

		/// set domain->dom_l1[...]
	}

	IOMMU_LOCK(iommu);
	LIST_INSERT_HEAD(&unit->domain_list, domain, next);
	IOMMU_UNLOCK(iommu);

	return (&domain->iodom);
}

static int
apple_dart_map(device_t dev, struct iommu_domain *iodom, vm_offset_t va,
	vm_page_ta *ma, bus_size_t size, vm_prot_t prot)
{
}

static int
apple_dart_unmap(device_t dev, struct iommu_domain *iodom, vm_offset_t va,
	bus_size_t size)
{
}
#endif

static device_method_t apple_dart_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		apple_dart_probe),
	DEVMETHOD(device_attach,	apple_dart_attach),
	DEVMETHOD(device_detach,	apple_dart_detach),

	/* IOMMU interface */
	//DEVMETHOD(iommu_find,		apple_dart_find),
	//DEVMETHOD(iommu_map,		apple_dart_map),
	//DEVMETHOD(iommu_unmap,	apple_dart_unmap),
	//DEVMETHOD(iommu_domain_alloc,	apple_dart_domain_alloc),
	//DEVMETHOD(iommu_domain_free,	apple_dart_domain_free),
	//DEVMETHOD(iommu_ctx_alloc,	apple_dart_ctx_alloc),
	//DEVMETHOD(iommu_ctx_init,	apple_dart_ctx_init),
	//DEVMETHOD(iommu_ctx_free,	apple_dart_ctx_free),
	//DEVMETHOD(iommu_ctx_lookup,	apple_dart_ctx_lookup),

	DEVMETHOD_END
};

static driver_t apple_dart_driver = {
	"dart",
	apple_dart_methods,
	sizeof(struct apple_dart_softc),
};

EARLY_DRIVER_MODULE(apple_dart, simplebus, apple_dart_driver,
    0, 0, BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LATE);
