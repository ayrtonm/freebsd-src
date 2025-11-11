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
#include "mixer_if.h"

#include <dev/virtio/sound/virtio_snd.h>
#include <dev/virtio/virtio.h>
#include <dev/virtio/virtqueue.h>

static int vtsnd_modevent(module_t mod, int type, void *unused);

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

extern driver_t vtsnd_driver;

VIRTIO_DRIVER_MODULE(virtio_snd, vtsnd_driver, vtsnd_modevent, NULL);
MODULE_VERSION(virtio_snd, 1);
MODULE_DEPEND(virtio_snd, virtio, 1, 1, 1);
MODULE_DEPEND(virtio_snd, sound, SOUND_MINVER, SOUND_PREFVER, SOUND_MAXVER);

VIRTIO_SIMPLE_PNPINFO(virtio_snd, VIRTIO_ID_SOUND, "VirtIO Sound Adapter");

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

static int
vtsnd_mixer_init(struct snd_mixer *m)
{
	struct snddev_info *sc;

	sc = mix_getdevinfo(m);
	if (sc == NULL)
		return (-1);

	pcm_setflags(sc->dev, pcm_getflags(sc->dev) | SD_F_SOFTPCMVOL);
	mix_setdevs(m, SOUND_MASK_PCM | SOUND_MASK_VOLUME | SOUND_MASK_RECLEV);
	mix_setrecdevs(m, SOUND_MASK_RECLEV);

	return (0);
}

static int
vtsnd_mixer_set(struct snd_mixer *m, unsigned dev, unsigned left, unsigned right)
{
	return (0);
}

static uint32_t
vtsnd_mixer_setrecsrc(struct snd_mixer *m, uint32_t src)
{
	return (src == SOUND_MASK_RECLEV ? src : 0);
}

static kobj_method_t vtsnd_mixer_methods[] = {
	KOBJMETHOD(mixer_init, vtsnd_mixer_init),
	KOBJMETHOD(mixer_set, vtsnd_mixer_set),
	KOBJMETHOD(mixer_setrecsrc, vtsnd_mixer_setrecsrc),
	KOBJMETHOD_END
};

MIXER_DECLARE(vtsnd_mixer);

kobj_class_t
vtsnd_get_mixer_class(void)
{
	return (&vtsnd_mixer_class);
}
