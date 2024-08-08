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

#define __BIT(n)	(1 << (n))

/*
 * DART registers
 */
#define	DART_PARAMS2		0x0004
#define  DART_PARAMS2_BYPASS_SUPPORT	__BIT(0)
#define	DART_TLB_OP		0x0020
#define	 DART_TLB_OP_FLUSH	__BIT(20)
#define	 DART_TLB_OP_BUSY	__BIT(2)
#define	DART_TLB_OP_SIDMASK	0x0034
#define	DART_ERR_STATUS		0x0040
#define	DART_ERR_ADDRL		0x0050
#define	DART_ERR_ADDRH		0x0054
#define	DART_CONFIG		0x0060
#define  DART_CONFIG_LOCK	__BIT(15)
#define	DART_TCR(sid)		(0x0100 + (sid) * 0x4)
#define	 DART_TCR_TXEN		__BIT(7)
#define	 DART_TCR_BYPASS_DART	__BIT(8)
#define	 DART_TCR_BYPASS_DAPF	__BIT(12)
#define	DART_TTBR(sid, idx)	(0x0200 + (sid) * 0x10 + (idx) * 0x4)
#define	 DART_TTBR_VALID	__BIT(31)
#define	 DART_TTBR_SHIFT	12

#define	DART_PAGE_SIZE		16384
#define	DART_PAGE_MASK		(DART_PAGE_SIZE - 1)

/*
 * SID allocation scheme is lightweight and just uses a 32-bit int.  This will
 * need to be revisited if we have to support more streams at some point.
 */
#define	DART_STREAM_MAX		16
#define	DART_STREAM_MASK	((1 << DART_STREAM_MAX) - 1)

#define	DART_L1_IDX_MAX		4

#define	DART_L1_TABLE		0xb
#define	DART_L2_INVAL		0x0
#define	DART_L2_PAGE		0x3

#define	DART_ROUND_PAGE(pa)	(((pa) + DART_PAGE_MASK) & ~DART_PAGE_MASK)
#define	DART_TRUNC_PAGE(pa)	((pa) & ~DART_PAGE_MASK)

/*
 * Skip the first page to help catching bugs where a device is
 * doing DMA to/from address zero because we didn't properly
 * set up the DMA transfer.  Skip the last page to avoid using
 * the address reserved for MSIs.
 */
#define	DART_DVA_START		DART_PAGE_SIZE
#define	DART_DVA_END		(0xffffffff - DART_PAGE_SIZE)
