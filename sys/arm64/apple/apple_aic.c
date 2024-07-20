/*-
 * Copyright (c) 2021 Andrew Turner
 * All rights reserved.
 * Copyright (c) 2022 Michael J. Karels <karels@freebsd.org>
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

#include "opt_platform.h"

#include <sys/cdefs.h>

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/proc.h>
#include <sys/rman.h>
#include <sys/smp.h>

#include <machine/bus.h>
#include <machine/machdep.h>
#include <machine/intr.h>
#ifdef SMP
#include <machine/smp.h>
#endif

#include <dev/fdt/fdt_intr.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dt-bindings/interrupt-controller/apple-aic.h>

#include "pic_if.h"

#define	AIC_INFO		0x0004
#define  AIC_INFO_NDIE(val)	(((val) >> 24) & 0xf)
#define	 AIC_INFO_NIRQS(val)	((val) & 0x0000ffff)

#define	AIC_WHOAMI		0x2000
#define	AIC_EVENT		0x2004
#define  AIC_EVENT_DIE(val)	(((val) >> 24) & 0xff)
#define  AIC_EVENT_TYPE(val)	(((val) >> 16) & 0xff)
#define  AIC_EVENT_TYPE_NONE	0
#define  AIC_EVENT_TYPE_IRQ	1
#define  AIC_EVENT_TYPE_IPI	4
#define  AIC_EVENT_IRQ(val)	((val) & 0xffff)
#define  AIC_EVENT_IPI_OTHER	1
#define  AIC_EVENT_IPI_SELF	2
#define	AIC_IPI_SEND		0x2008
#define	AIC_IPI_ACK		0x200c
#define AIC_IPI_MASK_SET	0x2024
#define	AIC_IPI_MASK_CLR	0x2028
#define	 AIC_IPI_OTHER		0x00000001
#define	 AIC_IPI_SELF		0x80000000
#define	AIC_TARGET_CPU(irq)	(0x3000 + (irq) * 4)
#define	AIC_SW_SET(irq)		(0x4000 + (((irq) >> 5) * 4))
#define	AIC_SW_CLEAR(irq)	(0x4080 + (((irq) >> 5) * 4))
#define	AIC_MASK_SET(irq)	(0x4100 + (((irq) >> 5) * 4))
#define	AIC_MASK_CLEAR(irq)	(0x4180 + (((irq) >> 5) * 4))
#define	 AIC_IRQ_MASK(irq)	(1u << ((irq) & 0x1f))

#define AIC_IPI_LOCAL_RR_EL1	s3_5_c15_c0_0
#define AIC_IPI_GLOBAL_RR_EL1	s3_5_c15_c0_1

#define AIC_TMR_CTL_GUEST_PHYS s3_5_c14_c2_1
#define AIC_TMR_CTL_GUEST_VIRT s3_5_c14_c3_1

#define AIC_IPI_SR_EL1		s3_5_c15_c1_1
#define  AIC_IPI_SR_EL1_PENDING	(1 << 0)

#define AIC_FIQ_VM_TIMER	s3_5_c15_c1_3
#define	AIC_FIQ_VM_TIMER_VEN	(1 << 0)
#define	AIC_FIQ_VM_TIMER_PEN	(1 << 1)
#define	AIC_FIQ_VM_TIMER_BITS	(AIC_FIQ_VM_TIMER_VEN | AIC_FIQ_VM_TIMER_PEN)

#define CNTV_CTL_ENABLE		(1 << 0)
#define CNTV_CTL_IMASK		(1 << 1)
#define CNTV_CTL_ISTATUS	(1 << 2)
#define	CNTV_CTL_BITS		\
    (CNTV_CTL_ENABLE | CNTV_CTL_IMASK | CNTV_CTL_ISTATUS)

#define	AIC2_CONFIG		0x0014
#define	AIC2_CONFIG_ENABLE		(1 << 0)
#define AIC2_SW_SET(die, irq)	(0x6000 + (die) * 0x4a00 + (((irq) >> 5) << 2))
#define AIC2_SW_CLEAR(die, irq)	(0x6200 + (die) * 0x4a00 + (((irq) >> 5) << 2))
#define AIC2_MASK_SET(die, irq)	(0x6400 + (die) * 0x4a00 + (((irq) >> 5) << 2))
#define AIC2_MASK_CLEAR(die, irq)	(0x6600 + (die) * 0x4a00 + (((irq) >> 5) << 2))
#define AIC2_EVENT	0xc000

#define	AIC_MAXCPUS		32
#define	AIC_MAXDIES		4

#define AIC_IRQ_CR_EL1		s3_4_c15_c10_4
#define AIC_IRQ_CR_EL1_DISABLE	(3 << 0)

#define debugf(fmt, ...) do { device_printf(dev, "%s: " fmt, __func__, ##__VA_ARGS__); } while (0)

static struct ofw_compat_data compat_data[] = {
	{ "apple,aic",				1 },
	{ "apple,aic2",				2 },
	{ NULL,					0 }
};

enum apple_aic_fiq_type {
	AIC_FIQ_TMR_HV_PHYS = AIC_TMR_HV_PHYS,
	AIC_FIQ_TMR_HV_VIRT = AIC_TMR_HV_VIRT,
	AIC_FIQ_TMR_GUEST_PHYS = AIC_TMR_GUEST_PHYS,
	AIC_FIQ_TMR_GUEST_VIRT = AIC_TMR_GUEST_VIRT,
	AIC_FIQ_CPU_PMU_ECORE = AIC_CPU_PMU_E,
	AIC_FIQ_CPU_PMU_PCORE = AIC_CPU_PMU_P,
	AIC_FIQ_NUM,
};

enum apple_aic_irq_type {
	AIC_TYPE_INVAL,
	AIC_TYPE_IRQ,
	AIC_TYPE_FIQ,
	AIC_TYPE_IPI,
};

struct apple_aic_irqsrc {
	struct intr_irqsrc	ai_isrc;
	uint32_t		ai_irq;
	uint32_t		ai_die;
	enum intr_polarity	ai_pol;
	enum intr_trigger	ai_trig;
	enum apple_aic_irq_type	ai_type;
};

#ifdef SMP
#define AIC_NIPIS		INTR_IPI_COUNT
#endif

struct apple_aic_softc {
	device_t		 sc_dev;
	struct resource		*sc_mem;
	struct resource		*sc_event;
	struct apple_aic_irqsrc	*sc_isrcs[AIC_MAXDIES];
	u_int			sc_version;
	u_int			sc_nirqs;
	u_int			sc_ndie;
#ifdef SMP
	struct apple_aic_irqsrc	sc_ipi_srcs[AIC_NIPIS];
	u_int			*sc_cpuids;	/* cpu index to AIC CPU ID */
	uint32_t		*sc_ipimasks;
#endif
};

// this is shadowed by another static in bind_intr but it only affects AICv1 so I don't care
static u_int aic_next_cpu;

// probably not necessary?
DPCPU_DEFINE_STATIC(uint32_t, aic_fiq_mask);

static device_probe_t apple_aic_probe;
static device_attach_t apple_aic_attach;

static pic_disable_intr_t apple_aic_disable_intr;
static pic_enable_intr_t apple_aic_enable_intr;
static pic_map_intr_t apple_aic_map_intr;
static pic_setup_intr_t apple_aic_setup_intr;
static pic_teardown_intr_t apple_aic_teardown_intr;
static pic_post_filter_t apple_aic_post_filter;
static pic_post_ithread_t apple_aic_post_ithread;
static pic_pre_ithread_t apple_aic_pre_ithread;
#ifdef SMP
static pic_bind_intr_t apple_aic_bind_intr;
static pic_init_secondary_t apple_aic_init_secondary;
static pic_ipi_send_t apple_aic_ipi_send;
static pic_ipi_setup_t apple_aic_ipi_setup;
#endif

static int apple_aic_irq(void *);
static int apple_aic_fiq(void *);

static void
apple_aic_mask_set(struct apple_aic_softc *sc, uint32_t die, uint32_t irq)
{
	if (sc->sc_version == 1) {
		bus_write_4(sc->sc_mem, AIC_MASK_SET(irq), AIC_IRQ_MASK(irq));
	} else {
		bus_write_4(sc->sc_mem, AIC2_MASK_SET(die, irq), AIC_IRQ_MASK(irq));
	}
}

static void
apple_aic_mask_clr(struct apple_aic_softc *sc, uint32_t die, uint32_t irq)
{
	if (sc->sc_version == 1) {
		bus_write_4(sc->sc_mem, AIC_MASK_CLEAR(irq), AIC_IRQ_MASK(irq));
	} else {
		bus_write_4(sc->sc_mem, AIC2_MASK_CLEAR(die, irq), AIC_IRQ_MASK(irq));
	}
}

static void
apple_aic_sw_set(struct apple_aic_softc *sc, uint32_t die, uint32_t irq)
{
	if (sc->sc_version == 1) {
		bus_write_4(sc->sc_mem, AIC_SW_SET(irq), AIC_IRQ_MASK(irq));
	} else {
		bus_write_4(sc->sc_mem, AIC2_SW_SET(die, irq), AIC_IRQ_MASK(irq));
	}
}

static void
apple_aic_sw_clr(struct apple_aic_softc *sc, uint32_t die, uint32_t irq)
{
	if (sc->sc_version == 1) {
		bus_write_4(sc->sc_mem, AIC_SW_CLEAR(irq), AIC_IRQ_MASK(irq));
	} else {
		bus_write_4(sc->sc_mem, AIC2_SW_CLEAR(die, irq), AIC_IRQ_MASK(irq));
	}
}

static void
apple_aic_fiq_unmask(struct apple_aic_softc *sc, uint32_t fiq)
{
	switch (fiq) {
	case AIC_FIQ_TMR_GUEST_PHYS:
		WRITE_SPECIALREG(AIC_FIQ_VM_TIMER, READ_SPECIALREG(AIC_FIQ_VM_TIMER) & ~AIC_FIQ_VM_TIMER_PEN);
		isb();
		break;
	case AIC_FIQ_TMR_GUEST_VIRT:
		WRITE_SPECIALREG(AIC_FIQ_VM_TIMER, READ_SPECIALREG(AIC_FIQ_VM_TIMER) & ~AIC_FIQ_VM_TIMER_VEN);
		isb();
	default:
		/* no mask bits for the hypervisor timers */
		break;
	}
	uint32_t mask = 1 << fiq;
	DPCPU_SET(aic_fiq_mask, DPCPU_GET(aic_fiq_mask) & ~mask);
}

static void
apple_aic_fiq_mask(struct apple_aic_softc *sc, uint32_t fiq)
{
	switch (fiq) {
	case AIC_FIQ_TMR_GUEST_PHYS:
		WRITE_SPECIALREG(AIC_FIQ_VM_TIMER, READ_SPECIALREG(AIC_FIQ_VM_TIMER) | AIC_FIQ_VM_TIMER_PEN);
		isb();
		break;
	case AIC_FIQ_TMR_GUEST_VIRT:
		WRITE_SPECIALREG(AIC_FIQ_VM_TIMER, READ_SPECIALREG(AIC_FIQ_VM_TIMER) | AIC_FIQ_VM_TIMER_VEN);
		isb();
	default:
		/* no mask bits for the hypervisor timers */
		break;
	}
	uint32_t mask = 1 << fiq;
	DPCPU_SET(aic_fiq_mask, DPCPU_GET(aic_fiq_mask) | mask);
}

static int
apple_aic_probe(device_t dev)
{

	u_int version;
	struct apple_aic_softc *sc;

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	version = ofw_bus_search_compatible(dev, compat_data)->ocd_data;
	if (version == 0)
		return (ENXIO);

	sc = device_get_softc(dev);
	sc->sc_version = version;

	device_set_desc(dev, "Apple Interrupt Controller");
	debugf("Found version %d\n", sc->sc_version);

	return (BUS_PROBE_DEFAULT);
}

static int
apple_aic_attach(device_t dev)
{
	struct apple_aic_softc *sc;
	struct intr_irqsrc *isrc;
	struct apple_aic_irqsrc *die_isrcs;
	const char *name;
	intptr_t xref;
	int error, rid;
	u_int i, cpu, j, info;
	uint32_t config;

	sc = device_get_softc(dev);
	sc->sc_dev = dev;

	debugf("Started AIC attach\n");
	rid = 0;
	sc->sc_mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->sc_mem == NULL) {
		device_printf(dev, "Unable to allocate memory for registers\n");
		return (ENXIO);
	}

	debugf("Allocated memory for registers\n");

	if (sc->sc_version == 2) {
		rid = 1;
		debugf("Allocating memory for event register\n");
		sc->sc_event = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
			RF_ACTIVE);
		if (sc->sc_event == NULL) {
			device_printf(dev, "Unable to allocate memory for event register\n");
			return (ENXIO);
		}
		debugf("Allocated memory for event register\n");
	}

	info = bus_read_4(sc->sc_mem, AIC_INFO);
	sc->sc_nirqs = AIC_INFO_NIRQS(info);
	sc->sc_ndie = AIC_INFO_NDIE(info) + 1;

	debugf("Found %d interrupts, %d die\n", sc->sc_nirqs, sc->sc_ndie);

#ifdef SMP
	sc->sc_ipimasks = malloc(sizeof(*sc->sc_ipimasks) * mp_maxid + 1,
	    M_DEVBUF, M_WAITOK | M_ZERO);

	if (sc->sc_version == 1) {
		sc->sc_cpuids = malloc(sizeof(*sc->sc_cpuids) * mp_maxid + 1,
			M_DEVBUF, M_WAITOK | M_ZERO);

		cpu = PCPU_GET(cpuid);
		sc->sc_cpuids[cpu] = bus_read_4(sc->sc_mem, AIC_WHOAMI);
		if (bootverbose)
			device_printf(dev, "BSP CPU %d: whoami %x\n", cpu,
			    sc->sc_cpuids[cpu]);
	}
#endif

	name = device_get_nameunit(dev);
	debugf("initializing irqsrc array\n");
	for (i = 0; i < sc->sc_ndie; i++) {
		sc->sc_isrcs[i] = mallocarray(sc->sc_nirqs,
		    sizeof(**sc->sc_isrcs), M_DEVBUF, M_WAITOK | M_ZERO);
		if (sc->sc_isrcs[i] == NULL) {
			device_printf(dev, "unable to allocate memory for irqsrc array\n");
			return (ENXIO);
		}

		die_isrcs = sc->sc_isrcs[i];
		debugf("initializing irqsrcs\n");
		for (j = 0; j < sc->sc_nirqs; j++) {
			isrc = &die_isrcs[j].ai_isrc;
			die_isrcs[j].ai_irq = j;
			die_isrcs[j].ai_die = i;
			die_isrcs[j].ai_pol = INTR_POLARITY_CONFORM;
			die_isrcs[j].ai_trig = INTR_TRIGGER_CONFORM;
			die_isrcs[j].ai_type = AIC_TYPE_INVAL;

			error = intr_isrc_register(isrc, dev, 0, "%s,d%us%u", name,
			    i, j);
			if (error != 0) {
				device_printf(dev, "Unable to register irq %u:%u\n",
				    i, j);
				return (error);
			}
		}
	}

	debugf("initialized %d irqsrcs on %d dies\n", sc->sc_nirqs, sc->sc_ndie);

	xref = OF_xref_from_node(ofw_bus_get_node(dev));
	if (intr_pic_register(dev, xref) == NULL) {
		device_printf(dev, "Unable to register interrupt handler\n");
		return (ENXIO);
	}

	OF_device_register_xref(xref, dev);

	if (intr_pic_claim_root(dev, xref, apple_aic_irq, sc) != 0) {
		device_printf(dev,
		    "Unable to set at root interrupt controller for IRQs\n");
		intr_pic_deregister(dev, xref);
		return (ENXIO);
	}

	if (intr_pic_claim_root_num(dev, xref, apple_aic_fiq, sc, INTR_ROOT_FIQ) != 0) {
		device_printf(dev,
		    "Unable to set at root interrupt controller for FIQs\n");
		intr_pic_deregister(dev, xref);
		return (ENXIO);
	}

#ifdef SMP
	error = intr_ipi_pic_register(dev, 0);
	if (error != 0) {
		device_printf(dev, "could not register as IPI source\n");
		return (ENXIO);
	}
#endif

	if (sc->sc_version == 2) {
		debugf("enabling AIC\n");
		config = bus_read_4(sc->sc_mem, AIC2_CONFIG);
		config |= AIC2_CONFIG_ENABLE;
		bus_write_4(sc->sc_mem, AIC2_CONFIG, config);
	}

	WRITE_SPECIALREG(AIC_IPI_SR_EL1, AIC_IPI_SR_EL1_PENDING);

	WRITE_SPECIALREG(cntp_ctl_el0, READ_SPECIALREG(cntp_ctl_el0) | CNTV_CTL_IMASK);
	WRITE_SPECIALREG(cntv_ctl_el0, READ_SPECIALREG(cntv_ctl_el0) | CNTV_CTL_IMASK);

	if (in_vhe()) {
		WRITE_SPECIALREG(AIC_FIQ_VM_TIMER, READ_SPECIALREG(AIC_FIQ_VM_TIMER) & ~AIC_FIQ_VM_TIMER_BITS);
	}
	isb();

	return (0);
}

static int
do_apple_aic_map_intr(device_t dev, struct intr_map_data *data, uint32_t *irqp,
	uint32_t *typep, enum intr_polarity *polp, enum intr_trigger *trigp,
	uint32_t *diep)
{
	u_int idx;
	uint32_t irq, type, die;
	struct intr_map_data_fdt *daf;
	struct apple_aic_softc *sc = device_get_softc(dev);

	if (data->type != INTR_MAP_DATA_FDT) {
		device_printf(dev,
			"found non-FDT intr_map_data of type %d\n", data->type);
		return (ENOTSUP);
	}

	daf = (struct intr_map_data_fdt *)data;

	/*
	 * The first cell is the interrupt type:
	 *   0 = IRQ
	 *   1 = FIQ
	 * The optional next cell is the die ID
	 * The next cell is the interrupt number
	 * The last cell is the flags
	 */
	if ((daf->ncells != 3) && (daf->ncells != 4)) {
		device_printf(dev,
			"devicetree contained unexpected #interrupt-cells %d\n", daf->ncells);
		return (EINVAL);
	}

	idx = 0;
	type = daf->cells[idx];
	if (typep) {
		switch (type) {
		case 0:
			*typep = AIC_TYPE_IRQ;
			break;
		case 1:
			*typep = AIC_TYPE_FIQ;
			break;
		default:
			device_printf(dev,
				"devicetree contained unexpected interrupt type %d\n", daf->cells[0]);
			return (EINVAL);
		}
	}

	if (daf->ncells == 4) {
		idx += 1;
		if (diep != NULL) {
			die = daf->cells[idx];
			if (die >= AIC_MAXDIES) {
				device_printf(dev, "devicetree referenced non-existent die %d\n", die);
				return (EINVAL);
			}
			*diep = die;
		}
	} else {
		if (diep != NULL)
			*diep = 0;
	}

	idx += 1;
	irq = daf->cells[idx];
	if (irq >= sc->sc_nirqs) {
		device_printf(dev, "devicetree referenced non-existent IRQ %d\n", irq);
		return (EINVAL);
	}
	if (irqp != NULL) {
		*irqp = irq;
	}

	idx += 1;
	if (polp != NULL) {
		if ((daf->cells[idx] & FDT_INTR_LEVEL_HIGH) != 0) {
			*polp = INTR_POLARITY_HIGH;
		} else {
			*polp = INTR_POLARITY_LOW;
		}
	}
	if (trigp != NULL) {
		if ((daf->cells[idx] & FDT_INTR_EDGE_MASK) != 0) {
			*trigp = INTR_TRIGGER_EDGE;
		} else {
			*trigp = INTR_TRIGGER_LEVEL;
		}
	}

	return (0);
}

static int
apple_aic_map_intr(device_t dev, struct intr_map_data *data,
    struct intr_irqsrc **isrcp)
{
	int error = 0;
	uint32_t irq, die;
	struct apple_aic_softc *sc = device_get_softc(dev);

	if (isrcp == NULL) {
		return (EINVAL);
	}

	error = do_apple_aic_map_intr(dev, data, &irq, NULL, NULL, NULL, &die);
	if (error == 0) {
		*isrcp = &sc->sc_isrcs[die][irq].ai_isrc;
	}
	return error;
}

static int
apple_aic_setup_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{
	struct  apple_aic_softc *sc;
	struct  apple_aic_irqsrc *ai;
	uint32_t irq, die;
	enum apple_aic_irq_type type;
	enum intr_trigger trig;
	enum intr_polarity pol;
	int error;

	sc = device_get_softc(dev);
	ai = (struct apple_aic_irqsrc *)isrc;

	if (data == NULL)
		return (ENOTSUP);

	error = do_apple_aic_map_intr(dev, data, &irq, &type, &pol, &trig,
		&die);
	if (error != 0)
		return error;

	if (isrc->isrc_handlers != 0) {
		return 0;
	}

	if (pol == INTR_POLARITY_CONFORM)
		pol = INTR_POLARITY_LOW;

	if (trig == INTR_TRIGGER_CONFORM)
		trig = INTR_TRIGGER_EDGE;

	ai->ai_pol = pol;
	ai->ai_trig = trig;
	ai->ai_type = type;

	switch (type) {
	case AIC_TYPE_IRQ:
		if (sc->sc_version == 1) {
			aic_next_cpu = intr_irq_next_cpu(aic_next_cpu, &all_cpus);
			bus_write_4(sc->sc_mem, AIC_TARGET_CPU(irq),
				1 << sc->sc_cpuids[aic_next_cpu]);
		}
		break;
	case AIC_TYPE_FIQ:
		isrc->isrc_flags |= INTR_ISRCF_PPI;
		break;
	case AIC_TYPE_IPI:
		break;
	default:
		return (EINVAL);
	}
	return 0;
}

static int
apple_aic_teardown_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{
	panic("%s\n", __func__);
}

static void
apple_aic_enable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	struct apple_aic_irqsrc *ai;
	struct apple_aic_softc *sc;
	uint32_t irq, die;

	ai = (struct apple_aic_irqsrc *)isrc;
	irq = ai->ai_irq;
	die = ai->ai_die;
	sc = device_get_softc(dev);
	switch(ai->ai_type) {
	case AIC_TYPE_IRQ:
		if (irq < AIC_FIQ_NUM) {
			panic("%s: tried to treat FIQ %d as an irq\n", __func__, irq);
		}
		apple_aic_mask_clr(sc, die, irq);
		break;
	case AIC_TYPE_FIQ:
		if (irq >= AIC_FIQ_NUM) {
			panic("%s: tried to enable non-existent FIQ %d\n", __func__, irq);
		}
		apple_aic_fiq_unmask(sc, irq);
		break;
	case AIC_TYPE_IPI:
		break;
	default:
		panic("%s: %x\n", __func__, ai->ai_type);
	}
}

static void
apple_aic_disable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	struct apple_aic_irqsrc *ai;
	struct apple_aic_softc *sc;
	uint32_t irq, die;

	ai = (struct apple_aic_irqsrc *)isrc;
	irq = ai->ai_irq;
	die = ai->ai_die;
	sc = device_get_softc(dev);
	switch(ai->ai_type) {
	case AIC_TYPE_IRQ:
		if (irq < AIC_FIQ_NUM) {
			panic("%s: tried to treat FIQ %d as an irq\n", __func__, irq);
		}
		apple_aic_mask_set(sc, die, irq);
		break;
	case AIC_TYPE_FIQ:
		if (irq >= AIC_FIQ_NUM) {
			panic("%s: tried to enable non-existent FIQ %d\n", __func__, irq);
		}
		apple_aic_fiq_mask(sc, irq);
		break;
	case AIC_TYPE_IPI:
		break;
	default:
		panic("%s: %x\n", __func__, ai->ai_type);
	}
}

static void
apple_aic_post_filter(device_t dev, struct intr_irqsrc *isrc)
{
	struct apple_aic_softc *sc;
	struct apple_aic_irqsrc *ai;
	uint32_t irq, die;

	sc = device_get_softc(dev);
	ai = (struct apple_aic_irqsrc *)isrc;
	irq = ai->ai_irq;
	die = ai->ai_die;
	switch(ai->ai_type) {
	case AIC_TYPE_IRQ:
		apple_aic_sw_clr(sc, die, irq);
		apple_aic_mask_clr(sc, die, irq);
		break;
	case AIC_TYPE_FIQ:
		apple_aic_fiq_mask(sc, irq);
		break;
	default:
		panic("%s: %x\n", __func__, ai->ai_type);
	}
}

static void
apple_aic_pre_ithread(device_t dev, struct intr_irqsrc *isrc)
{
	struct apple_aic_softc *sc;
	struct apple_aic_irqsrc *ai;
	uint32_t irq, die;

	ai = (struct apple_aic_irqsrc *)isrc;
	sc = device_get_softc(dev);
	irq = ai->ai_irq;
	die = ai->ai_die;

	if (ai->ai_type != AIC_TYPE_IRQ) {
		panic("%s: registered an ithread for non-IRQ interrupt %d\n", __func__, irq);
	}

	apple_aic_sw_clr(sc, die, irq);
	// reading the event register automatically sets the irq mask
	// apple_aic_disable_intr(dev, isrc);
}

static void
apple_aic_post_ithread(device_t dev, struct intr_irqsrc *isrc)
{
	//struct apple_aic_softc *sc;
	struct apple_aic_irqsrc *ai;
	uint32_t irq;//, die;

	ai = (struct apple_aic_irqsrc *)isrc;
	//sc = device_get_softc(dev);
	irq = ai->ai_irq;
	//die = ai->ai_die;
	if (ai->ai_type != AIC_TYPE_IRQ) {
		panic("%s: registered an ithread for non-IRQ interrupt %d\n", __func__, irq);
	}

	// apple_aic_mask_clr(sc, die, irq);
	apple_aic_enable_intr(dev, isrc);
}

static void
apple_aic_ipi_received(struct apple_aic_softc *sc, struct trapframe *tf)
{
#ifdef SMP
	uint32_t mask;
	uint32_t ipi;
	int cpu;

	cpu = PCPU_GET(cpuid);
	rmb();
	mask = atomic_readandclear_32(&sc->sc_ipimasks[cpu]);

	while (mask != 0) {
		ipi = ffs(mask) - 1;
		mask &= ~(1 << ipi);

		intr_ipi_dispatch(ipi);
	}
#endif
}

static int
apple_aic_irq(void *arg)
{
	struct apple_aic_softc *sc;
	uint32_t die, event, irq, type;
	struct apple_aic_irqsrc	*aisrc;
	struct trapframe *tf;

	sc = arg;
	tf = curthread->td_intr_frame;

	if (sc->sc_version == 1) {
		event = bus_read_4(sc->sc_mem, AIC_EVENT);
	} else {
		event = bus_read_4(sc->sc_event, 0);
	}
	type = AIC_EVENT_TYPE(event);

	/* If we get an IPI here, we really goofed. */
	MPASS(type != AIC_EVENT_TYPE_IPI);

	if (type != AIC_EVENT_TYPE_IRQ) {
		if (type != AIC_EVENT_TYPE_NONE)
			device_printf(sc->sc_dev, "unexpected event type %d\n",
			    type);
		return (FILTER_STRAY);
	}

	die = AIC_EVENT_DIE(event);
	irq = AIC_EVENT_IRQ(event);

	if (die >= sc->sc_ndie)
		panic("%s: unexpected die %d", __func__, die);
	if (irq >= sc->sc_nirqs)
		panic("%s: unexpected irq %d", __func__, irq);

	aisrc = &sc->sc_isrcs[die][irq];
	if (intr_isrc_dispatch(&aisrc->ai_isrc, tf) != 0) {
		device_printf(sc->sc_dev, "Stray irq %u:%u disabled\n",
		    die, irq);
		return (FILTER_STRAY);
	}

	return (FILTER_HANDLED);
}

static int
apple_aic_fiq(void *arg)
{
	struct apple_aic_softc *sc;
	struct apple_aic_irqsrc *isrcs;
	struct trapframe *tf;

	sc = arg;
	tf = curthread->td_intr_frame;

#ifdef SMP
	/* Handle IPIs. */
	if ((READ_SPECIALREG(AIC_IPI_SR_EL1) & AIC_IPI_SR_EL1_PENDING) != 0) {
		WRITE_SPECIALREG(AIC_IPI_SR_EL1, AIC_IPI_SR_EL1_PENDING);
		isb();
		apple_aic_ipi_received(sc, tf);
	}
#endif

	isrcs = sc->sc_isrcs[0];

	uint64_t reg = READ_SPECIALREG(cntp_ctl_el0);
	if ((reg & CNTV_CTL_BITS) == (CNTV_CTL_ENABLE | CNTV_CTL_ISTATUS)) {
		intr_isrc_dispatch(&isrcs[AIC_FIQ_TMR_HV_PHYS].ai_isrc, tf);
	}


	reg = READ_SPECIALREG(cntv_ctl_el0);
	if ((reg & CNTV_CTL_BITS) == (CNTV_CTL_ENABLE | CNTV_CTL_ISTATUS)) {
		intr_isrc_dispatch(&isrcs[AIC_FIQ_TMR_HV_VIRT].ai_isrc, tf);
	}

	if (in_vhe()) {
		reg = READ_SPECIALREG(AIC_FIQ_VM_TIMER);

		if ((reg & AIC_FIQ_VM_TIMER_PEN) != 0) {
			reg = READ_SPECIALREG(AIC_TMR_CTL_GUEST_PHYS);
			if ((reg & CNTV_CTL_BITS) == (CNTV_CTL_ENABLE | CNTV_CTL_ISTATUS)) {
				intr_isrc_dispatch(&isrcs[AIC_FIQ_TMR_GUEST_PHYS].ai_isrc, tf);
			}
		}

		if ((reg & AIC_FIQ_VM_TIMER_VEN) != 0) {
			reg = READ_SPECIALREG(AIC_TMR_CTL_GUEST_VIRT);
			if ((reg & CNTV_CTL_BITS) == (CNTV_CTL_ENABLE | CNTV_CTL_ISTATUS)) {
				intr_isrc_dispatch(&isrcs[AIC_FIQ_TMR_GUEST_VIRT].ai_isrc, tf);
			}
		}
	}

	return (FILTER_HANDLED);
}

#ifdef SMP
static int
apple_aic_bind_intr(device_t dev, struct intr_irqsrc *isrc)
{
	struct apple_aic_softc *sc = device_get_softc(dev);
	static int aic_next_cpu;
	uint32_t targets = 0;
	uint32_t irq;
	u_int cpu;

	MPASS(((struct apple_aic_irqsrc *)isrc)->ai_type == AIC_TYPE_IRQ);
	if (sc->sc_version != 1) {
		return (ENOTSUP);
	}

	irq = ((struct apple_aic_irqsrc *)isrc)->ai_irq;
	if (CPU_EMPTY(&isrc->isrc_cpu)) {
		aic_next_cpu = intr_irq_next_cpu(aic_next_cpu, &all_cpus);
		CPU_SETOF(aic_next_cpu, &isrc->isrc_cpu);
		if (sc->sc_version == 1) {
			bus_write_4(sc->sc_mem, AIC_TARGET_CPU(irq),
			    sc->sc_cpuids[aic_next_cpu] << 1);
		}
	} else {
		CPU_FOREACH_ISSET(cpu, &isrc->isrc_cpu) {
			targets |= sc->sc_cpuids[cpu] << 1;
		}
		if (sc->sc_version == 1) {
			bus_write_4(sc->sc_mem, AIC_TARGET_CPU(irq), targets);
		}
	}
	return (0);
}

static void
apple_aic_ipi_send(device_t dev, struct intr_irqsrc *isrc, cpuset_t cpus,
    u_int ipi)
{
	struct apple_aic_softc *sc;
	uint64_t aff, localgrp, sendmask;
	u_int cpu;

	sc = device_get_softc(dev);
	sendmask = 0;
	localgrp = CPU_AFF1(CPU_AFFINITY(PCPU_GET(cpuid)));

	KASSERT(isrc == &sc->sc_ipi_srcs[ipi].ai_isrc,
	    ("%s: bad ISRC %p argument", __func__, isrc));
	for (cpu = 0; cpu <= mp_maxid; cpu++) {
		if (CPU_ISSET(cpu, &cpus)) {
			aff = CPU_AFFINITY(cpu);
			sendmask = CPU_AFF0(aff);
			atomic_set_32(&sc->sc_ipimasks[cpu], 1 << ipi);
			wmb();

			if (CPU_AFF1(aff) == localgrp) {
				WRITE_SPECIALREG(AIC_IPI_LOCAL_RR_EL1,
				    sendmask);
			} else {
				sendmask |= CPU_AFF1(aff) << 16;
				WRITE_SPECIALREG(AIC_IPI_GLOBAL_RR_EL1,
				    sendmask);
			}

			isb();
		}
	}
}

static int
apple_aic_ipi_setup(device_t dev, u_int ipi, struct intr_irqsrc **isrcp)
{
	struct apple_aic_softc *sc = device_get_softc(dev);
	struct apple_aic_irqsrc *ai;

	KASSERT(ipi < AIC_NIPIS, ("%s: ipi %u too high", __func__, ipi));

	ai = &sc->sc_ipi_srcs[ipi];
	ai->ai_type = AIC_TYPE_IPI;

	*isrcp = &ai->ai_isrc;
	return (0);
}

static void
apple_aic_init_secondary(device_t dev)
{
	struct apple_aic_softc *sc = device_get_softc(dev);
	u_int cpu = PCPU_GET(cpuid);
	/*
	 * AIC2 doesn't provide a way to target external interrupt to a
	 * particular core so disable IRQ delivery to secondary CPUs
	 */
	device_printf(dev, "Disabling IRQ delivery to CPU %d\n", cpu);
	WRITE_SPECIALREG(AIC_IRQ_CR_EL1, AIC_IRQ_CR_EL1_DISABLE);

	if (sc->sc_version == 2) {
		return;
	}

	sc->sc_cpuids[cpu] = bus_read_4(sc->sc_mem, AIC_WHOAMI);
	if (bootverbose)
		device_printf(dev, "CPU %d: whoami %x\n", cpu,
		    sc->sc_cpuids[cpu]);

	bus_write_4(sc->sc_mem, AIC_IPI_MASK_SET, AIC_IPI_SELF | AIC_IPI_OTHER);
}
#endif

static device_method_t apple_aic_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		apple_aic_probe),
	DEVMETHOD(device_attach,	apple_aic_attach),

	/* Interrupt controller interface */
	DEVMETHOD(pic_disable_intr,	apple_aic_disable_intr),
	DEVMETHOD(pic_enable_intr,	apple_aic_enable_intr),
	DEVMETHOD(pic_map_intr,		apple_aic_map_intr),
	DEVMETHOD(pic_setup_intr,	apple_aic_setup_intr),
	DEVMETHOD(pic_teardown_intr,	apple_aic_teardown_intr),
	DEVMETHOD(pic_post_filter,	apple_aic_post_filter),
	DEVMETHOD(pic_post_ithread,	apple_aic_post_ithread),
	DEVMETHOD(pic_pre_ithread,	apple_aic_pre_ithread),
#ifdef SMP
	DEVMETHOD(pic_bind_intr,	apple_aic_bind_intr),
	DEVMETHOD(pic_init_secondary,	apple_aic_init_secondary),
	DEVMETHOD(pic_ipi_send,		apple_aic_ipi_send),
	DEVMETHOD(pic_ipi_setup,	apple_aic_ipi_setup),
#endif

	/* End */
	DEVMETHOD_END
};

static DEFINE_CLASS_0(aic, apple_aic_driver, apple_aic_methods,
    sizeof(struct apple_aic_softc));

EARLY_DRIVER_MODULE(aic, simplebus, apple_aic_driver, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_MIDDLE);
