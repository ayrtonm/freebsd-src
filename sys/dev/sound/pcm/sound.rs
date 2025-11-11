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
use core::marker::PhantomData;
use core::ops::Range;
use kpi::bindings::{
    device_t, kobj, kobj_t, oss_mixer_enuminfo, pcm_channel, pcmchan_caps, snd_dbuf, snd_mixer,
    snddev_info,
};
use kpi::boxed::Box;
use kpi::collections::Appendable;
use kpi::ffi::{DevRef, SubClass};
use kpi::kobj::{AsRustType, KobjClass, KobjLayout};
use kpi::prelude::*;
use kpi::sync::Mutable;
use kpi::vec::Vec;
use kpi::{ErrCode, base, define_interface, gen_newtype};

pub type Hz = u32;
pub type PcmSoftc<T> = SubClass<snddev_info, T>;

#[derive(Debug, Default, PartialEq, Eq)]
pub struct PcmTrig(c_int);

impl AsRustType<PcmTrig> for c_int {
    fn as_rust_type(self) -> PcmTrig {
        PcmTrig(self)
    }
}

#[derive(Debug, Default, PartialEq, Eq)]
pub struct PcmDir(u32);

unsafe impl Sync for PcmDir {}
unsafe impl Send for PcmDir {}

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
    pub fn new(minspeed: u32, maxspeed: u32, fmtlist: Vec<SndFormat>) -> Result<Self> {
        if fmtlist[fmtlist.len() - 1] != 0 {
            return Err(EINVAL);
        }
        Ok(Self {
            inner: pcmchan_caps {
                minspeed,
                maxspeed,
                fmtlist: fmtlist.as_ptr().cast::<u32>().cast_mut(),
                caps: 0,
            },
            fmtlist,
        })
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

#[derive(Debug)]
pub struct PcmChannel<T>(*mut pcm_channel, PhantomData<*const T>);

impl<T> PcmChannel<T> {
    pub fn get_softc(&self) -> DevRef<T> {
        let chan_ptr = self.0;
        let dev = unsafe { (*chan_ptr).dev };
        let void_ptr = unsafe { bindings::device_get_softc(dev) };
        let sc = void_ptr.cast::<T>();
        unsafe { DevRef::from_raw(sc) }
    }
}

impl<T> AsRustType<PcmChannel<T>> for *mut pcm_channel {
    fn as_rust_type(self) -> PcmChannel<T> {
        PcmChannel::<T>(self, PhantomData)
    }
}

unsafe impl<T> Sync for PcmChannel<T> {}
unsafe impl<T> Send for PcmChannel<T> {}

pub type SndFormat = u32;

#[derive(Debug)]
pub struct SndDbuf {
    dbuf: *mut snd_dbuf,
    raw_buf: Vec<u8>,
    taken: Mutable<Option<u32>>,
}

impl Drop for SndDbuf {
    fn drop(&mut self) {
        // If there exists a SndDbufSlice we can't drop raw_buf. Leaking memory is an option by
        // calling forget so make the default panic
        assert!(self.taken.get_mut().is_some());
    }
}

impl AsRustType<SndDbuf> for *mut snd_dbuf {
    fn as_rust_type(self) -> SndDbuf {
        SndDbuf {
            dbuf: self,
            raw_buf: Vec::new(),
            taken: Mutable::new(None),
        }
    }
}

/// A proxy for a slice of the raw_buf in SndDbuf
///
/// SndDbuf ensures that it will not access or drop the slice in raw_buf while there exists a
/// SndDbufSlice.
pub struct SndDbufSlice {
    offset: u32,
    ptr: *mut u8,
    len: usize,
}

// SAFETY: SndDbuf ensures it won't access or drop the memory represented by SndDbufSlice for its
// lifetime so it has unique access to it.
unsafe impl Appendable for SndDbufSlice {
    fn get_vaddr_range(&mut self) -> (*mut c_void, usize) {
        let ptr = self.ptr.cast::<c_void>();
        let len = self.len;
        (ptr, len)
    }
}

impl SndDbuf {
    /// Gives unique access to a slice of the raw_buf in SndDbuf
    ///
    /// The slice starts at offset and has the length of the block size. This is the only way to
    /// create a SndDbufSlice. Only one slice may be taken at a time and the caller must use
    /// SndDbuf::replace to put the slice back before grabbing another one.
    pub fn take(&self, offset: u32) -> Result<SndDbufSlice> {
        let mut taken = self.taken.get_mut();
        if taken.is_some() {
            return Err(EDOOFUS);
        }
        *taken = Some(offset);
        let ptr = unsafe { self.raw_buf.as_ptr().byte_add(offset as usize).cast_mut() };
        let len = self.block_size() as usize;
        Ok(SndDbufSlice { offset, ptr, len })
    }

    pub fn replace(&self, slice: SndDbufSlice) -> Result<()> {
        let mut taken = self.taken.get_mut();
        match *taken {
            Some(taken) => {
                if slice.offset != taken {
                    return Err(EDOOFUS);
                }
            }
            None => {
                return Err(EDOOFUS);
            }
        }
        *taken = None;
        Ok(())
    }

    pub fn block_size(&self) -> u32 {
        unsafe { (*self.dbuf).blksz }
    }

    pub fn buffer_size(&self) -> u32 {
        unsafe { (*self.dbuf).bufsize }
    }
}

unsafe impl Sync for SndDbuf {}
unsafe impl Send for SndDbuf {}
unsafe impl Sync for SndDbufSlice {}
unsafe impl Send for SndDbufSlice {}

#[allow(unused_variables)]
pub trait ChannelIf: KobjClass + KobjLayout<Layout = kobj> {
    type Softc: 'static + Sync;
    type DevInfo;
    type Channel: Sync;

    //fn channel_get_softc(chan: &PcmChannel) -> &Self::Softc {
    //    let chan_ptr = chan.0;
    //    let dev = unsafe { (*chan_ptr).dev };
    //    let void_ptr = unsafe { bindings::device_get_softc(dev) };
    //    unsafe { void_ptr.cast::<Self::Softc>().as_ref().unwrap() }
    //}

    fn channel_init(
        _: kobj_t,
        devinfo: &mut Self::DevInfo,
        buf: SndDbuf,
        c: PcmChannel<Self::Softc>,
        dir: PcmDir,
    ) -> Box<Self::Channel> {
        unimplemented!()
    }
    fn channel_free(_: kobj_t, ch: Box<Self::Channel>) -> Result<()> {
        unimplemented!()
    }
    fn channel_setformat(_: kobj_t, ch: DevRef<Self::Channel>, format: u32) -> Result<()> {
        unimplemented!()
    }
    fn channel_setspeed(_: kobj_t, ch: DevRef<Self::Channel>, speed: Hz) -> Hz {
        unimplemented!()
    }
    fn channel_setblocksize(_: kobj_t, ch: DevRef<Self::Channel>, blocksize: u32) -> u32 {
        unimplemented!()
    }
    fn channel_trigger(_: kobj_t, ch: DevRef<Self::Channel>, go: PcmTrig) -> Result<()> {
        unimplemented!()
    }
    fn channel_getptr(_: kobj_t, ch: DevRef<Self::Channel>) -> u32 {
        unimplemented!()
    }
    fn channel_getcaps(_: kobj_t, ch: DevRef<Self::Channel>) -> *mut pcmchan_caps {
        unimplemented!()
    }
}

// Copy whatever the fuck MIXER_DECLARE is doing...
pub const MIXER_KOBJ_SIZE: usize = 512 + size_of::<kobj>() + size_of::<oss_mixer_enuminfo>();

pub trait MixerIf: KobjClass + KobjLayout<Layout = [u8; MIXER_KOBJ_SIZE]> {
    type DevInfo: Sync;

    fn mixer_reinit(m: *mut snd_mixer) -> Result<()> {
        panic!("not supported")
    }
    fn mixer_init(devinfo: &Self::DevInfo, m: *mut snd_mixer) -> Result<()> {
        unimplemented!()
    }
    fn mixer_set(
        devinfo: &Self::DevInfo,
        m: *mut snd_mixer,
        dev: u32,
        left: u32,
        right: u32,
    ) -> Result<()> {
        unimplemented!()
    }
    fn mixer_setrecsrc(devinfo: &Self::DevInfo, m: *mut snd_mixer, src: u32) -> u32 {
        unimplemented!()
    }
    fn mixer_uninit(devinfo: &Self::DevInfo, m: *mut snd_mixer) -> Result<()> {
        unimplemented!()
    }
}

define_interface! {
    in ChannelIf
    fn channel_init(obj: kobj_t, devinfo: *mut void, b: *mut snd_dbuf, c: *mut pcm_channel, dir: int) -> *mut void,
        with desc channel_init_desc
        and typedef channel_init_t,
        is infallible;
    fn channel_setformat(obj: kobj_t, ch: *mut void, format: u32) -> int,
        with desc channel_setformat_desc
        and typedef channel_setformat_t;
    fn channel_setspeed(obj: kobj_t, ch: *mut void, speed: u32) -> u32,
        with desc channel_setspeed_desc
        and typedef channel_setspeed_t,
        is infallible;
    fn channel_setblocksize(obj: kobj_t, ch: *mut void, blocksize: u32) -> u32,
        with desc channel_setblocksize_desc
        and typedef channel_setblocksize_t,
        is infallible;
    fn channel_trigger(obj: kobj_t, ch: *mut void, go: int) -> int,
        with desc channel_trigger_desc
        and typedef channel_trigger_t;
    fn channel_getptr(obj: kobj_t, ch: *mut void) -> u32,
        with desc channel_getptr_desc
        and typedef channel_getptr_t,
        is infallible;
    fn channel_getcaps(obj: kobj_t, ch: *mut void) -> *mut pcmchan_caps,
        with desc channel_getcaps_desc
        and typedef channel_getcaps_t,
        is infallible;
    fn channel_free(obj: kobj_t, ch: *mut void) -> int,
        with desc channel_free_desc
        and typedef channel_free_t;
}

define_interface! {
    in MixerIf
    fn mixer_reinit(m: *mut snd_mixer) -> c_int,
        with desc mixer_reinit_desc
        and typedef mixer_reinit_t;
    fn mixer_init(m: *mut snd_mixer) -> c_int,
        with desc mixer_init_desc
        and typedef mixer_init_t,
        with init glue {
            let void_ptr = unsafe { bindings::mix_getdevinfo(m) };
            let devinfo = void_ptr.as_rust_type();
        },
        with prefix args { devinfo };
    fn mixer_uninit(m: *mut snd_mixer) -> c_int,
        with desc mixer_uninit_desc
        and typedef mixer_uninit_t,
        with init glue {
            let void_ptr = unsafe { bindings::mix_getdevinfo(m) };
            let devinfo = void_ptr.as_rust_type();
        },
        with prefix args { devinfo };
    fn mixer_set(m: *mut snd_mixer, dev: u_int, left: u_int, right: u_int) -> c_int,
        with desc mixer_set_desc
        and typedef mixer_set_t,
        with init glue {
            let void_ptr = unsafe { bindings::mix_getdevinfo(m) };
            let devinfo = void_ptr.as_rust_type();
        },
        with prefix args { devinfo };
    fn mixer_setrecsrc(m: *mut snd_mixer, src: u32) -> u32,
        with desc mixer_setrecsrc_desc
        and typedef mixer_setrecsrc_t,
        with init glue {
            let void_ptr = unsafe { bindings::mix_getdevinfo(m) };
            let devinfo = void_ptr.as_rust_type();
        },
        with prefix args { devinfo },
        is infallible;
}

pub mod prelude {
    use super::*;

    gen_newtype! {
        PcmDir,
        PCMDIR_PLAY,
        PCMDIR_REC,
    }

    gen_newtype! {
        PcmTrig,
        PCMTRIG_START,
        PCMTRIG_STOP,
        PCMTRIG_ABORT,
        PCMTRIG_EMLDMAWR,
        PCMTRIG_EMLDMARD,
    }

    pub fn pcm_getflags(dev: device_t) -> u32 {
        unsafe { bindings::pcm_getflags(dev) }
    }

    pub fn pcm_setflags(dev: device_t, flags: u32) {
        unsafe { bindings::pcm_setflags(dev, flags) }
    }

    pub fn pcm_init<T>(dev: device_t, sc: &PcmSoftc<T>) -> Result<()> {
        let base_sc = base!(&sc);
        unsafe { bindings::pcm_init(dev, base_sc.cast::<c_void>()) };
        Ok(())
    }

    pub fn pcm_addchan<C: ChannelIf>(
        dev: device_t,
        dir: PcmDir,
        chan_class: &'static C,
        devinfo: &mut C::DevInfo,
    ) -> Result<()> {
        let dir = dir.0 as i32;
        let class = C::get_class();
        let arg = devinfo as *mut C::DevInfo;
        let res = unsafe { bindings::pcm_addchan(dev, dir, class, arg.cast::<c_void>()) };
        if res != 0 {
            return Err(ErrCode::from(res));
        }
        Ok(())
    }

    pub fn pcm_register(dev: device_t, name: &'static CStr) -> Result<()> {
        let res = unsafe { bindings::pcm_register(dev, name.as_ptr().cast_mut()) };
        if res != 0 {
            return Err(ErrCode::from(res));
        }
        Ok(())
    }

    pub fn pcm_getbuffersize(dev: device_t, size: Range<u32>, default: u32) -> usize {
        let min_sz = size.start;
        let max_sz = size.end;
        let res = unsafe { bindings::pcm_getbuffersize(dev, min_sz, default, max_sz) };
        res as usize
    }

    pub fn snd_format(f: i32, c: u32, e: u32) -> SndFormat {
        unsafe { bindings::snd_format(f as u32, c, e) }
    }

    pub fn sndbuf_resize(buf: &SndDbuf, block_count: usize, block_size: u32) -> Result<()> {
        let res = unsafe {
            bindings::sndbuf_resize(
                buf.dbuf,
                block_count.try_into().unwrap(),
                block_size.try_into().unwrap(),
            )
        };
        if res != 0 {
            return Err(ErrCode::from(res));
        }
        Ok(())
    }

    pub fn sndbuf_setup(buf: &mut SndDbuf, mut raw_buf: Vec<u8>) -> Result<()> {
        raw_buf.fill_to_capacity(0u8);
        let res = unsafe {
            bindings::sndbuf_setup(
                buf.dbuf,
                raw_buf.as_ptr().cast_mut().cast::<c_void>(),
                raw_buf.capacity().try_into().unwrap(),
            )
        };
        if res != 0 {
            return Err(ErrCode::from(res));
        }
        buf.raw_buf = raw_buf;
        Ok(())
    }

    pub fn chn_intr<T>(chan: &PcmChannel<T>) {
        unsafe { bindings::chn_intr(chan.0) }
    }

    // devinfo passed to mixer_init must have the same type that the mixer will use to access it
    pub fn mixer_init<M: MixerIf>(
        dev: device_t,
        mixer_class: &'static M,
        devinfo: &M::DevInfo,
    ) -> Result<()> {
        let arg = devinfo as *const M::DevInfo;
        let class = M::get_class();
        let res = unsafe { bindings::mixer_init(dev, class, arg.cast::<c_void>().cast_mut()) };
        if res != 0 {
            return Err(ErrCode::from(res));
        }
        Ok(())
    }

    // This calls channel_free for each channel and mixer_uninit, the former triggers calls to the
    // rust channel_free impl which drops each channel's Arc and the latter triggers calls to the rust mixer_uninit
    // release the refcount in Mixer
    pub fn pcm_unregister(dev: device_t) -> Result<()> {
        let res = unsafe { bindings::pcm_unregister(dev) };
        if res != 0 {
            return Err(ErrCode::from(res));
        }
        Ok(())
    }
}
