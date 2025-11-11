/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2025 Ayrton Muñoz
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

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>

#include <machine/bus.h>

#include <dev/sound/pcm/sound.h>
#include "feeder_if.h"

#include <dev/virtio/sound/virtio_snd.h>
#include <dev/virtio/virtio.h>
#include <dev/virtio/virtqueue.h>

static int vtsnd_modevent(module_t mod, int type, void *unused);

extern driver_t vtsnd_driver;

VIRTIO_DRIVER_MODULE(virtio_snd, vtsnd_driver, vtsnd_modevent, NULL);
MODULE_VERSION(virtio_snd, 1);
MODULE_DEPEND(virtio_snd, virtio, 1, 1, 1);

VIRTIO_SIMPLE_PNPINFO(virtio_snd, VIRTIO_ID_SOUND, "VirtIO Sound Adapter");

static int
vtsnd_modevent(module_t mod, int type, void *unused)
{
	int error;

	error = 0;

	switch (type) {
	case MOD_LOAD:
	case MOD_QUIESCE:
	case MOD_UNLOAD:
	case MOD_SHUTDOWN:
		break;
	default:
		error = EOPNOTSUPP;
		break;
	}

	return (error);
}

int
vtsnd_device_probe(device_t dev)
{
	/*
	 * This macro references the variable created by VIRTIO_SIMPLE_PNPINFO.
	 * That macro produces a static so it can't be accessed outside the
	 * translation unit. I'd rather not untangle and reimplement that C
	 * macro in rust so I'm just invoking it in C then calling
	 * vtsnd_device_probe from rust.
	 */
	return (VIRTIO_SIMPLE_PROBE(dev, virtio_snd));
}

//static void *
//vtsnd_chan_init(kobj_t obj, void *devinfo, struct snd_dbuf *b,
//    struct pcm_channel *c, int dir)
//{
//	int res = vtsnd_chan_init_rust(devinfo, b, c, dir);
//	KASSERT(res == 0, ("vtsnd_chan_init_rust failed"));
//	return devinfo;
//}
//
//static uint32_t
//vtsnd_chan_setspeed(kobj_t obj, void *data, uint32_t speed) {
//    return vtsnd_chan_setspeed_rust(obj, data, speed);
//}
//
//static struct pcmchan_caps *
//vtsnd_chan_getcaps(kobj_t obj, void *devinfo) {
//    struct pcmchan_caps *x = NULL;
//    int res = vtsnd_chan_getcaps_rust(devinfo, &x);
//    KASSERT(res == 0, ("vtsnd_chan_getcaps_rust failed"));
//    return x;
//}
//
//static struct pcmchan_matrix *
//vtsnd_chan_getmatrix(kobj_t obj, void *devinfo, uint32_t format) {
//    return (feeder_matrix_format_map(format));
//}
//
//static kobj_method_t vtsnd_chan_methods[] = {
//	KOBJMETHOD(channel_init, vtsnd_chan_init),
//	KOBJMETHOD(channel_setformat, vtsnd_chan_setformat),
//	KOBJMETHOD(channel_setspeed, vtsnd_chan_setspeed),
//	KOBJMETHOD(channel_getcaps, vtsnd_chan_getcaps),
//	KOBJMETHOD(channel_getmatrix, vtsnd_chan_getmatrix),
//	KOBJMETHOD_END
//};
//// KOBJMETHOD(channel_free, vtsnd_chan_free),
//// KOBJMETHOD(channel_setblocksize, vtsnd_chan_setblocksize),
//// KOBJMETHOD(channel_setfragments, vtsnd_chan_setfragments),
//// KOBJMETHOD(channel_trigger, vtsnd_chan_trigger),
//// KOBJMETHOD(channel_getptr, vtsnd_chan_getptr),
//CHANNEL_DECLARE(vtsnd_chan);
//
//struct kobj_class *
//vtsnd_get_chan_class(void)
//{
//	return &vtsnd_chan_class;
//}
//
//static uint32_t vtsnd_playfmt[] = {
//    SND_FORMAT(AFMT_U8, 2 /* channels */, 0),
//    0
//};
//
//uint32_t *vtsnd_get_fmt(void) {
//    return &vtsnd_playfmt[0];
//}
