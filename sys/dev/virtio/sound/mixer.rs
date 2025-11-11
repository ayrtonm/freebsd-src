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

use crate::VtSoundSoftc;
use kpi::bindings::{
    SD_F_SOFTPCMVOL, SOUND_MASK_PCM, SOUND_MASK_RECLEV, SOUND_MASK_VOLUME, snd_mixer,
};
use kpi::kobj::KobjLayout;
use kpi::prelude::*;
use kpi::{define_class, method_table};
use sound::prelude::*;
use sound::{MIXER_KOBJ_SIZE, MixerIf};

impl MixerIf for VtSoundMixerClass {
    type DevInfo = VtSoundSoftc;

    fn mixer_init(sc: &VtSoundSoftc, m: *mut snd_mixer) -> Result<()> {
        pcm_setflags(sc.dev, pcm_getflags(sc.dev) | SD_F_SOFTPCMVOL as u32);
        let flags = SOUND_MASK_PCM | SOUND_MASK_VOLUME | SOUND_MASK_RECLEV;
        unsafe { bindings::mix_setdevs(m, flags as u32) };
        unsafe { bindings::mix_setrecdevs(m, SOUND_MASK_RECLEV as u32) };

        Ok(())
    }

    fn mixer_uninit(sc: &VtSoundSoftc, m: *mut snd_mixer) -> Result<()> {
        Ok(())
    }

    fn mixer_set(
        sc: &VtSoundSoftc,
        m: *mut snd_mixer,
        dev: u32,
        left: u32,
        right: u32,
    ) -> Result<()> {
        Ok(())
    }

    fn mixer_setrecsrc(sc: &VtSoundSoftc, m: *mut snd_mixer, src: u32) -> u32 {
        if src == SOUND_MASK_RECLEV as u32 {
            src
        } else {
            0
        }
    }
}

impl KobjLayout for VtSoundMixerClass {
    type Layout = [u8; MIXER_KOBJ_SIZE];
}

define_class!(
    vtsnd_mixer,
    c"vtsnd_mixer",
    VtSoundMixerClass,
    vtsnd_mixer_methods
);

method_table! {
    vtsnd_mixer, VtSoundMixerClass,
    vtsnd_mixer_methods = {
        mixer_init vtsnd_mixer_init,
        mixer_uninit vtsnd_mixer_uninit,
        mixer_set vtsnd_mixer_set,
        mixer_setrecsrc vtsnd_mixer_setrecsrc,
    };
    with interfaces from { sound };
}
