/*-
 * SPDX-License-Identifier: ISC
 *
 * Copyright (c) 2021 Mark Kettenis <kettenis@openbsd.org>
 * Copyright (c) 2021 Jared McNeill <jmcneill@invisible.ca>
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

#ifndef _APPLE_DART_REG_H_
#define	_APPLE_DART_REG_H_

#define __BIT(n)	(1 << (n))

/*
 * DART registers
 */
#define	DART_PARAMS1		0x0000
#define	DART_PARAMS2		0x0004
#define	DART_PARAMS3		0x0008
#define	DART_PARAMS4		0x000c

#define	DART_PARAMS2_BYPASS_SUPPORT		__BIT(0)
#define	DART_T8110_PARAMS3_VA_WIDTH(x)	(((x) >> 16) & 0x3f)
#define	DART_T8110_PARAMS3_REV_MAJ(x)	(((x) >> 8) & 0xff)
#define	DART_T8110_PARAMS3_REV_MIN(x)	((x) & 0xff)
#define	DART_T8110_PARAMS4_NSID			(0x1ff)

/*
 * T6000 and T8103 DART registers
 */
 // not on t8110
#define	DART_T6000_CONFIG		0x0060
#define	DART_T6000_CONFIG_LOCK	__BIT(15)

#define	DART_T6000_SID_ENABLE_BASE	0x00fc

#define	DART_T6000_TCR_BASE			0x0100
#define	DART_T6000_TCR_BYPASS_DAPF	__BIT(12)
#define	DART_T6000_TCR_BYPASS_DART	__BIT(8)
#define	DART_T6000_TCR_TRANSLATE	__BIT(7)

#define	DART_T6000_TTBR_BASE		0x0200
#define	DART_T6000_TTBR_VALID		__BIT(31)

#define	DART_T6000_TLB_OP_BASE		0x0020
#define	DART_T6000_TLB_OP_FLUSH	__BIT(20)
#define	DART_T6000_TLB_OP_BUSY		__BIT(2)

// not on t8110
#define	DART_T6000_TLB_SIDMASK		0x0034

#define	DART_T6000_ERROR_STATUS		0x0040
#define	DART_T6000_ERROR_ADDR_LO	0x0050
#define	DART_T6000_ERROR_ADDR_HI	0x0054


/*
 * T8110 DART registers
 */
#define	DART_T8110_PROTECT				0x0200
#define	DART_T8110_PROTECT_TTBR_TCR		__BIT(0)

#define	DART_T8110_SID_ENABLE_BASE	0x0c00

#define	DART_T8110_TCR_BASE			0x1000
#define	DART_T8110_TCR_BYPASS_DAPF	__BIT(2)
#define	DART_T8110_TCR_BYPASS_DART	__BIT(1)
#define	DART_T8110_TCR_TRANSLATE	__BIT(0)

#define	DART_T8110_TTBR_BASE		0x1400
#define	DART_T8110_TTBR_VALID		__BIT(0)

#define	DART_T8110_TLB_OP_BASE		0x0080
// only for a single sid
#define DART_T8110_TLB_OP_FLUSH		__BIT(8)
#define DART_T8110_TLB_OP_FLUSH_ALL	(0)
#define	DART_T8110_TLB_OP_BUSY		__BIT(31)

#define	DART_ALL_STREAMS(sc)	((1 << (sc)->sc_nsid) - 1)

#define	DART_T8110_ERROR_STATUS		0x0100
#define	DART_T8110_ERROR_ADDR_LO	0x0170
#define	DART_T8110_ERROR_ADDR_HI	0x0174
// only on t8110
#define	DART_T8110_ERROR_MASK		0x0104

/*
 * Miscellaneous macros
 */
#define	DART_PAGE_SIZE		(16 * 1024)
#define	DART_T6000_STREAMS	16

/*
 * Skip the first page to help catching bugs where a device is
 * doing DMA to/from address zero because we didn't properly
 * set up the DMA transfer.  Skip the last page to avoid using
 * the address reserved for MSIs.
 */
#define	DART_DVA_START		DART_PAGE_SIZE
#define	DART_DVA_END		(0xffffffff - DART_PAGE_SIZE)

// where does this come from?
#define	DART_L1_IDX_MAX		4

#define	DART_TCR(sc, sid)		((sc)->sc_tcr_base + (sid) * 0x4)

#define	DART_TTBR(sc, sid, idx) \
	((sc)->sc_ttbr_base + (4 * (sc)->sc_nttbr * (sid)) + (4 * (idx)))

#endif /* _APPLE_DART_REG_H_ */
