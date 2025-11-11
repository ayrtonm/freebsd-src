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
use core::mem::transmute;
use kpi::bindings::{
    SOUND_MASK_RECLEV, bus_addr_t, driver_intr_t, kobj_class_t, kobj_t, pcm_channel, pcmchan_caps,
    pcmchan_matrix, snd_dbuf, snd_mixer, snddev_info,
};
use kpi::bus::Irq;
use kpi::ffi::SubClass;
use kpi::objects::KobjClass;
use kpi::prelude::*;
use kpi::sync::arc::{Arc, ArcRef};
use kpi::vec::Vec;
use kpi::{define_interface, gen_newtype};

#[derive(Debug, Default, PartialEq, Eq)]
pub struct PcmDir(u32);

unsafe impl Sync for PcmDir {}
unsafe impl Send for PcmDir {}

gen_newtype! {
    PcmDir,
    PCMDIR_PLAY,
    PCMDIR_REC,
}

impl AsRustType<PcmDir> for c_int {
    fn as_rust_type(self) -> PcmDir {
        PcmDir(self as u32)
    }
}

#[derive(Debug)]
pub struct PcmChannelCaps {
    inner: pcmchan_caps,
    fmtlist: Vec<SndFormat>,
}

impl PcmChannelCaps {
    pub fn new(minspeed: u32, maxspeed: u32, fmtlist: Vec<SndFormat>) -> Self {
        assert!(fmtlist[fmtlist.len() - 1] == 0);
        Self {
            inner: pcmchan_caps {
                minspeed,
                maxspeed,
                fmtlist: fmtlist.as_ptr().cast::<u32>().cast_mut(),
                caps: 0,
            },
            fmtlist,
        }
    }

    pub fn as_ptr(&self) -> *mut pcmchan_caps {
        (&raw const self.inner).cast_mut()
    }

    pub fn get_minspeed(&self) -> u32 {
        self.inner.minspeed
    }

    pub fn get_maxspeed(&self) -> u32 {
        self.inner.maxspeed
    }

    pub fn get_fmtlist(&self) -> &[SndFormat] {
        self.fmtlist.as_slice()
    }
}

unsafe impl Send for PcmChannelCaps {}
unsafe impl Sync for PcmChannelCaps {}

pub type SndFormat = u32;

pub fn snd_format(f: i32, c: u32, e: u32) -> SndFormat {
    unsafe { bindings::snd_format(f as u32, c, e) }
}

pub fn pcm_getflags(dev: device_t) -> u32 {
    unsafe { bindings::pcm_getflags(dev) }
}

pub fn pcm_setflags(dev: device_t, flags: u32) {
    unsafe { bindings::pcm_setflags(dev, flags) }
}

pub type PcmSoftc<T> = SubClass<snddev_info, T>;

fn get_dev_for_pcm_softc<T>(sc: &PcmSoftc<T>) -> Result<device_t> {
    // Should be initialized by pcm_init
    let dev = unsafe { base!(sc->dev) };
    if dev.is_null() {
        return Err(EDOOFUS);
    }
    Ok(dev)
}

pub fn pcm_init<T>(dev: device_t, sc: &PcmSoftc<T>) -> Result<()> {
    if get_dev_for_pcm_softc(&sc).is_ok() {
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

pub fn pcm_addchan<C: ChannelIf>(
    dev: device_t,
    dir: PcmDir,
    cls: &'static C,
    devinfo: C::DevInfo,
) -> Result<()> {
    let cls = cls.get_class();
    let devinfo_arg = devinfo.as_c_type();
    //let devinfo_arg = Arc::into_raw(devinfo);
    //let devinfo = devinfo.leak_ref() as *const C::DevInfo;
    let res =
        unsafe { bindings::pcm_addchan(dev, dir.0 as i32, cls, devinfo_arg.cast::<c_void>()) };
    if res != 0 {
        return Err(ErrCode::from(res));
    }
    Ok(())
}

pub fn pcm_getbuffersize(dev: device_t, minsz: u32, dflt: u32, maxsz: u32) -> usize {
    let res = unsafe { bindings::pcm_getbuffersize(dev, minsz, dflt, maxsz) };
    res as usize
}

pub fn sndbuf_setup(b: *mut snd_dbuf, mut buffer: Vec<u8>) -> Result<SndDbuf> {
    buffer.expand_to_capacity();
    let res = unsafe {
        bindings::sndbuf_setup(
            b,
            buffer.as_ptr().cast_mut().cast::<c_void>(),
            buffer.capacity().try_into().unwrap(),
        )
    };
    if res != 0 {
        return Err(ErrCode::from(res));
    }
    Ok(SndDbuf {
        b,
        buffer: Mutable::new(buffer),
    })
}

use kpi::sync::Mutable;

#[derive(Debug)]
pub struct SndDbuf {
    b: *mut snd_dbuf,
    pub buffer: Mutable<Vec<u8>>,
}

impl SndDbuf {
    pub fn block_size(&self) -> u32 {
        unsafe { (*self.b).blksz }
    }

    pub fn buffer_size(&self) -> u32 {
        unsafe { (*self.b).bufsize }
    }
}

unsafe impl Sync for SndDbuf {}
unsafe impl Send for SndDbuf {}

pub fn pcm_register<T>(dev: device_t, sc: &PcmSoftc<T>, name: &'static CStr) -> Result<()> {
    let sc_dev = get_dev_for_pcm_softc(&sc)?;
    //if sc_dev != dev {
    //    return Err(EDOOFUS);
    //}
    let res = unsafe { bindings::pcm_register(dev, name.as_ptr().cast_mut()) };
    if res != 0 {
        return Err(ErrCode::from(res));
    }
    Ok(())
}

pub type SndHandlerFn<T> = extern "C" fn(ArcRef<T>);

pub fn snd_setup_intr<T>(
    dev: device_t,
    irq: &Irq,
    flags: u32,
    handler: SndHandlerFn<T>,
    arg: Arc<T>,
) -> Result<()> {
    let handler = unsafe { transmute::<Option<SndHandlerFn<T>>, driver_intr_t>(Some(handler)) };
    let arg_ptr = Arc::into_raw(arg);
    // FIXME: cookie could move
    let cookiep = irq.cookie.get();
    let res = unsafe {
        bindings::snd_setup_intr(
            dev,
            irq.res,
            flags as i32,
            handler,
            arg_ptr.cast::<c_void>(),
            cookiep,
        )
    };
    if res != 0 {
        return Err(ErrCode::from(res));
    };
    Ok(())
}

//pub fn mixer_init<M: MixerIf>(
//    dev: device_t,
//    cls: &'static M,
//    devinfo: Arc<M::DevInfo>,
//) -> Result<()> {
//    let cls = cls.get_class();
//    let devinfo_arg = Arc::into_raw(devinfo);
//    let res = unsafe { bindings::mixer_init(dev, cls, devinfo_arg.cast::<c_void>()) };
//    if res != 0 {
//        return Err(ErrCode::from(res));
//    }
//    Ok(())
//}

//pub fn mix_getdevinfo<'a, M: MixerIf>(m: *mut snd_mixer) -> ArcRef<'a, M::DevInfo> {
//    let res = unsafe { bindings::mix_getdevinfo(m) };
//    unsafe { ArcRef::from_raw(res.cast()) }
//}

pub type Hz = u32;

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

            let devinfo = devinfo.as_rust_type();
            let dir = dir.as_rust_type();

            let ch = match <$driver_ty as $crate::ChannelIf>::channel_init(devinfo, b, c, dir) {
                Ok(ch_arc) => kpi::sync::arc::Arc::into_raw(ch_arc),
                Err(_) => core::ptr::null_mut(),
            };

            return ch.cast::<core::ffi::c_void>();
        }
    };
}

define_interface! {
    in ChannelIf
    fn channel_setformat(obj: kobj_t, ch: *mut void, format: u32) -> int;
    fn channel_setspeed(obj: kobj_t, ch: *mut void, speed: u32) -> u32, infallible;
    fn channel_setblocksize(obj: kobj_t, ch: *mut void, blocksize: u32) -> u32, infallible;
//    fn channel_setfragments(obj: kobj_t, devinfo: *mut void, blocksize: u32, blockcount: u32) -> int;
    fn channel_trigger(obj: kobj_t, ch: *mut void, go: int) -> int;
    fn channel_getptr(obj: kobj_t, ch: *mut void) -> u32, infallible;
    fn channel_getcaps(obj: kobj_t, ch: *mut void) -> *mut pcmchan_caps, infallible;
//    fn channel_getmatrix(obj: kobj_t, devinfo: *mut void, format: u32) -> *mut pcmchan_matrix, infallible;
}

pub trait ChannelIf: KobjClass {
    type DevInfoPtr;
    type DevInfo: AsCType<*mut Self::DevInfoPtr>;
    type Channel: 'static + Sync;
    fn channel_init(
        devinfo: Self::DevInfo,
        b: *mut snd_dbuf,
        c: *mut pcm_channel,
        dir: PcmDir,
    ) -> Result<Arc<Self::Channel>>;
    fn channel_setformat(_: kobj_t, ch: ArcRef<Self::Channel>, format: u32) -> Result<()>;
    fn channel_setspeed(_: kobj_t, ch: ArcRef<Self::Channel>, speed: Hz) -> Hz;
    fn channel_setblocksize(_: kobj_t, ch: ArcRef<Self::Channel>, blocksize: u32) -> u32;
    //    fn channel_setfragments(
    //        _: kobj_t,
    //        devinfo: &Self::DevInfo,
    //        blocksize: u32,
    //        blockcount: u32,
    //    ) -> Result<()> {
    //        todo!("")
    //    }
    fn channel_trigger(_: kobj_t, ch: ArcRef<Self::Channel>, go: c_int) -> Result<()>;
    //        todo!("")
    //    }
    fn channel_getptr(_: kobj_t, ch: ArcRef<Self::Channel>) -> u32;
    fn channel_getcaps(_: kobj_t, ch: ArcRef<Self::Channel>) -> *mut pcmchan_caps;
    //    fn channel_getmatrix(_: kobj_t, devinfo: &Self::DevInfo, format: u32) -> *mut pcmchan_matrix {
    //        todo!("")
    //    }
}

//define_interface! {
//    in MixerIf
//    fn mixer_init(m: *mut snd_mixer) -> int;
//    fn mixer_set(m: *mut snd_mixer, dev: u32, left: u32, right: u32) -> int;
//    fn mixer_setrecsrc(m: *mut snd_mixer, src: u32) -> u32, infallible;
//}
//
//pub trait MixerIf: KobjClass {
//    type DevInfo;
//
//    fn mixer_init(m: *mut snd_mixer) -> Result<()>;
//    fn mixer_set(m: *mut snd_mixer, dev: u32, left: u32, right: u32) -> Result<()> {
//        Ok(())
//    }
//    fn mixer_setrecsrc(m: *mut snd_mixer, src: u32) -> u32 {
//        if src == SOUND_MASK_RECLEV as u32 {
//            src
//        } else {
//            0
//        }
//    }
//}
