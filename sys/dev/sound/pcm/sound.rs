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

/* Bindings for PCM sound. */

#![no_std]

use core::ffi::{CStr, c_int, c_void};
use core::mem::forget;
use kpi::bindings::{kobj_class_t, pcm_channel, snd_dbuf, snddev_info, pcmchan_caps, pcmchan_matrix};
use kpi::ffi::{Ptr, RefCounted, SubClass};
use kpi::prelude::*;
use kpi::objects::KobjClass;
use kpi::vec::Vec;
use kpi::define_interface;

#[repr(u32)]
pub enum PcmDir {
    Play = bindings::PCMDIR_PLAY,
    Rec = bindings::PCMDIR_REC,
}

impl AsRustType<PcmDir> for c_int {
    fn as_rust_type(self) -> PcmDir {
        match self as u32 {
            bindings::PCMDIR_PLAY => PcmDir::Play,
            bindings::PCMDIR_REC => PcmDir::Rec,
            _ => panic!("invalid pcm channel direction"),
        }
    }
}

pub trait PcmChannel {
    type DevInfo;
    fn get_devinfo(&self) -> *const Self::DevInfo;
}

type PcmSoftc<T> = SubClass<snddev_info, T>;

fn softc_is_init<T>(sc: &PcmSoftc<T>) -> Result<device_t> {
    // Should be initialized by pcm_init
    let dev = unsafe { base!(sc->dev) };
    if dev.is_null() {
        return Err(EDOOFUS);
    }
    Ok(dev)
}
pub fn pcm_init<T>(dev: device_t, sc: &RefCounted<PcmSoftc<T>>) -> Result<()> {
    if softc_is_init(&sc).is_ok() {
        return Err(EDOOFUS);
    }
    // Check that the softc arg belongs to the device_t arg
    let sc_for_dev = unsafe { bindings::device_get_softc(dev) };
    let sc_arg: *mut snddev_info = base!(&sc);
    if sc_for_dev.cast::<snddev_info>() != sc_arg {
        return Err(EDOOFUS);
    }
    unsafe { bindings::pcm_init(dev, sc_for_dev) };
    Ok(())
}

pub fn pcm_addchan<T: PcmChannel, U: ChannelIf>(sc: Ptr<PcmSoftc<T>>, dir: PcmDir, chan_class: &'static U) -> Result<()> {
    let dev = softc_is_init(&sc)?;
    let devinfo = sc.get_devinfo();
    forget(sc);
    let res = unsafe {
        bindings::pcm_addchan(
            dev,
            dir as u32 as i32,
            chan_class.get_class(),
            devinfo.cast::<c_void>().cast_mut(),
        )
    };
    if res != 0 {
        return Err(ErrCode::from(res));
    }
    Ok(())
}

pub fn pcm_register<T>(sc: &RefCounted<PcmSoftc<T>>, name: &'static CStr) -> Result<()> {
    let dev = softc_is_init(&sc)?;
    let res = unsafe { bindings::pcm_register(dev, name.as_ptr().cast_mut()) };
    if res != 0 {
        return Err(ErrCode::from(res));
    }
    Ok(())
}

pub fn sndbuf_setup(b: *mut snd_dbuf, buf: Vec<u8>) -> Result<()> {
    let res = unsafe {
        bindings::sndbuf_setup(
            b,
            buf.as_ptr().cast_mut().cast::<c_void>(),
            buf.capacity().try_into().unwrap(),
        )
    };
    forget(buf);
    if res != 0 {
        return Err(ErrCode::from(res));
    }
    Ok(())
}

#[macro_export]
macro_rules! channel_init {
    ($driver_ty:ident $impl_fn_name:ident) => {
        pub unsafe extern "C" fn $impl_fn_name(
            _obj: *mut kobj,
            devinfo: *mut void,
            b: *mut snd_dbuf,
            c: *mut pcm_channel,
            dir: int,
        ) -> *mut void {
            const _TYPES_MATCH: kpi::bindings::channel_init_t = Some($impl_fn_name);
            let c_devinfo = devinfo;
            let devinfo = devinfo.as_rust_type();
            let b = b.as_rust_type();
            let c = c.as_rust_type();
            let dir = dir.as_rust_type();

            <$driver_ty as $crate::ChannelIf>::channel_init(devinfo, b, c, dir);

            return c_devinfo;
        }
    };
}

define_interface! {
    in ChannelIf
    fn channel_setformat(obj: kobj_t, devinfo: *mut void, format: u32) -> int;
    fn channel_setspeed(obj: kobj_t, devinfo: *mut void, speed: u32) -> u32;
    fn channel_setblocksize(obj: kobj_t, devinfo: *mut void, blocksize: u32) -> u32;
    fn channel_setfragments(obj: kobj_t, devinfo: *mut void, blocksize: u32, blockcount: u32) -> int;
    fn channel_trigger(obj: kobj_t, devinfo: *mut void, go: int) -> int;
    fn channel_getptr(obj: kobj_t, devinfo: *mut void) -> u32;
    fn channel_getcaps(obj: kobj_t, devinfo: *mut void) -> *mut pcmchan_caps;
    fn channel_getmatrix(obj: kobj_t, devinfo: *mut void, format: u32) -> *mut pcmchan_matrix;
}

pub trait ChannelIf: KobjClass {
    type DevInfo;
    fn channel_init(devinfo: &Self::DevInfo, b: *mut snd_dbuf, c: *mut pcm_channel, dir: PcmDir);
    fn channel_setformat(devinfo: &Self::DevInfo, format: u32) -> Result<()> { todo!("") }
    fn channel_setspeed(devinfo: &Self::DevInfo, speed: u32) -> Result<()> { todo!("") }
    fn channel_getcaps(devinfo: &Self::DevInfo) -> Result<&pcmchan_caps> { todo!("") }
    fn channel_getmatrix(devinfo: &Self::DevInfo, format: u32) -> Result<&pcmchan_matrix> { todo!("") }
    fn channel_setblocksize(devinfo: &Self::DevInfo, blocksize: u32) -> Result<()> { todo!("") }
    fn channel_setfragments(devinfo: &Self::DevInfo, blocksize: u32, blockcount: u32) -> Result<()> { todo!("") }
    fn channel_trigger(devinfo: &Self::DevInfo, go: c_int) -> Result<()> { todo!("") }
    fn channel_getptr(devinfo: &Self::DevInfo) -> u32 { todo!("") }
}
