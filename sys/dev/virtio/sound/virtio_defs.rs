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

#![allow(dead_code)]

use kpi::ErrCode;
use kpi::bindings::{AFMT_S16_LE, AFMT_S24_LE, AFMT_S32_LE};
use kpi::collections::Pod;
use kpi::prelude::*;
use sound::prelude::*;
use sound::{Hz, SndFormat};
use virtio::define_config_space;

// Definitions taken from the virtio 1.3 specifications section 5.14 Sound Device

// VIRTIO_SND_S_*
#[repr(u32)]
#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum VtSoundStatus {
    NoErr = 0x8000,
    BadMsg,
    NotSupp,
    IoErr,
}

// VIRTIO_SND_R_PCM_*
#[repr(u32)]
#[derive(Debug, PartialEq, Eq, Copy, Clone)]
pub enum VtSoundPcmReq {
    Info = 0x0100,
    SetParams,
    Prepare,
    Release,
    Start,
    Stop,
}

type VtSoundHdr = u32;

#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct VtSoundQueryInfo {
    pub hdr: VtSoundHdr,
    pub start_id: u32,
    pub count: u32,
    pub size: u32,
}

#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct VtSoundPcmInfo {
    pub hdr: VtSoundHdr,
    pub features: u32,
    pub formats: u64,
    pub rates: u64,
    // This should be one of the values in VtSoundDirection, but cannot be defined as the enum
    // itself to allow implementing Pod for this struct
    pub direction: u8,
    pub channels_min: u8,
    pub channels_max: u8,
    pub _padding: [u8; 5],
}

#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct VtSoundPcmHdr {
    pub hdr: VtSoundHdr,
    pub stream_id: u32,
}

#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct VtSoundPcmSetParams {
    pub hdr: VtSoundPcmHdr,
    pub buffer_bytes: u32,
    pub period_bytes: u32,
    pub features: u32,
    pub channels: u8,
    pub format: u8,
    pub rate: u8,
    pub _padding: u8,
}

#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct VtSoundPcmXfer {
    pub stream_id: u32,
}

#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct VtSoundPcmStatus {
    pub status: u32,
    pub latency_bytes: u32,
}

// virtqueue_enqueue uses sglist which requires that the appended buffers implement Pod so implement
// it for the virtio sound device messages
// SAFETY: The following are all structs consisting of other types that implement Pod
unsafe impl Pod for VtSoundQueryInfo {}
unsafe impl Pod for VtSoundPcmInfo {}
unsafe impl Pod for VtSoundPcmHdr {}
unsafe impl Pod for VtSoundPcmSetParams {}
unsafe impl Pod for VtSoundPcmXfer {}
unsafe impl Pod for VtSoundPcmStatus {}

// VIRTIO_SND_D_*
#[repr(u8)]
#[derive(Debug)]
pub enum VtSoundDirection {
    Output = 0,
    Input,
}

// VIRTIO_SND_PCM_RATE_*
#[repr(u64)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum VtSoundPcmRate {
    Rate5512 = 0,
    Rate8000,
    Rate11025,
    Rate16000,
    Rate22050,
    Rate32000,
    Rate44100,
    Rate48000,
    Rate64000,
    Rate88200,
    Rate96000,
    Rate176400,
    Rate192000,
    Rate384000,
}

const VT_RATES: [(Hz, VtSoundPcmRate); 14] = [
    (5512, VtSoundPcmRate::Rate5512),
    (8000, VtSoundPcmRate::Rate8000),
    (11025, VtSoundPcmRate::Rate11025),
    (16000, VtSoundPcmRate::Rate16000),
    (22050, VtSoundPcmRate::Rate22050),
    (32000, VtSoundPcmRate::Rate32000),
    (44100, VtSoundPcmRate::Rate44100),
    (48000, VtSoundPcmRate::Rate48000),
    (64000, VtSoundPcmRate::Rate64000),
    (88200, VtSoundPcmRate::Rate88200),
    (96000, VtSoundPcmRate::Rate96000),
    (176400, VtSoundPcmRate::Rate176400),
    (192000, VtSoundPcmRate::Rate192000),
    (384000, VtSoundPcmRate::Rate384000),
];

impl VtSoundPcmRate {
    pub fn into_hz(self) -> Hz {
        for (rate, vt_rate) in VT_RATES {
            if self == vt_rate {
                return rate;
            }
        }
        unreachable!("VT_RATES is missing a VtSoundPcmRate variant")
    }

    pub fn closest_to(target_rate: Hz) -> Self {
        let diffs = VT_RATES.map(|(rate, vt_rate)| {
            let target_rate = target_rate as i32;
            let rate = rate as i32;
            // Take the absolute value of the diff
            let diff = (target_rate - rate).abs();
            (vt_rate, diff)
        });

        let (closest_vt_rate, _smallest_diff) = *diffs.iter().min_by_key(|(_, diff)| diff).unwrap();
        closest_vt_rate
    }
}

impl TryFrom<u32> for VtSoundPcmRate {
    type Error = ErrCode;

    fn try_from(x: u32) -> Result<Self> {
        for (_rate, vt_rate) in VT_RATES {
            if vt_rate as u32 == x {
                return Ok(vt_rate);
            }
        }
        return Err(EINVAL);
    }
}

#[allow(nonstandard_style)]
// VIRTIO_SND_PCM_FMT_*
#[repr(u64)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum VtSoundPcmFormat {
    IMA_ADPCM = 0, /*  4 /  4 bits */
    MU_LAW,        /*  8 /  8 bits */
    A_LAW,         /*  8 /  8 bits */
    S8,            /*  8 /  8 bits */
    U8,            /*  8 /  8 bits */
    S16,           /* 16 / 16 bits */
    U16,           /* 16 / 16 bits */
    S18_3,         /* 18 / 24 bits */
    U18_3,         /* 18 / 24 bits */
    S20_3,         /* 20 / 24 bits */
    U20_3,         /* 20 / 24 bits */
    S24_3,         /* 24 / 24 bits */
    U24_3,         /* 24 / 24 bits */
    S20,           /* 20 / 32 bits */
    U20,           /* 20 / 32 bits */
    S24,           /* 24 / 32 bits */
    U24,           /* 24 / 32 bits */
    S32,           /* 32 / 32 bits */
    U32,           /* 32 / 32 bits */
    FLOAT,         /* 32 / 32 bits */
    FLOAT64,       /* 64 / 64 bits */
    /* digital formats (width / physical width) */
    DSD_U8,          /*  8 /  8 bits */
    DSD_U16,         /* 16 / 16 bits */
    DSD_U32,         /* 32 / 32 bits */
    IEC958_SUBFRAME, /* 32 / 32 bits */
}

const SUPPORTED_FORMATS: [(i32, VtSoundPcmFormat); 3] = [
    (AFMT_S32_LE, VtSoundPcmFormat::S32),
    (AFMT_S24_LE, VtSoundPcmFormat::S24),
    (AFMT_S16_LE, VtSoundPcmFormat::S16),
];

impl VtSoundPcmFormat {
    pub fn new(format: SndFormat) -> Result<Self> {
        for (f, vtf) in SUPPORTED_FORMATS {
            if format == snd_format(f, 2, 0) {
                return Ok(vtf);
            }
        }
        return Err(EINVAL);
    }

    pub fn supported_formats() -> [(SndFormat, Self); 3] {
        SUPPORTED_FORMATS.map(|(f, vtf)| (snd_format(f, 2, 0), vtf))
    }
}

// virtio sound config space
define_config_space! {
    struct VtSoundConfig {
        jacks: u32,
        streams: u32,
        chmaps: u32,
        controls: u32,
    }
}
