/*	$OpenBSD: aplmbox.h,v 1.1 2021/12/18 13:33:52 kettenis Exp $	*/
/*
 * Copyright (c) 2021 Mark Kettenis <kettenis@openbsd.org>
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

struct apple_mbox_msg {
	uint64_t data0;
	uint32_t data1;
};

typedef int (*apple_mbox_rx)(void *, struct apple_mbox_msg);

struct apple_mbox {
	device_t		dev;
};

int apple_mbox_get(device_t, device_t *);

void apple_mbox_set_rx(struct apple_mbox *, apple_mbox_rx, void *);

int apple_mbox_write(struct apple_mbox *, const struct apple_mbox_msg *);
