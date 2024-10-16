/*	$OpenBSD: rtkit.c,v 1.6 2022/09/03 19:04:28 kettenis Exp $	*/
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

#![no_std]
#![feature(allocator_api)]

extern crate alloc;

use alloc::boxed::Box;
use core::ffi::{c_int, c_void};
use core::mem::MaybeUninit;
use core::ptr::{addr_of_mut, null_mut};
use kpi::device::Device;
use kpi::taskq::Task;
use kpi::prelude::*;
use kpi::{bindings, enum_c_macros, RefMut, get_field, AsRustType, PointsTo};

use apple_mbox::apple_mbox_driver;

#[repr(C)]
struct RTKitCtx {
    msg: bindings::apple_mbox_msg,
    state: *mut bindings::rtkit_state,
}

struct RTKitTask {
    task: Task,
    ctx: bindings::rtkit_task,//RTKitCtx,
}

mod Endpoint {
    pub const MGMT: u32 = 0;
    pub const CRASHLOG: u32 = 1;
    pub const SYSLOG: u32 = 2;
    pub const DEBUG: u32 = 3;
    pub const IOREPORT: u32 = 4;
    pub const OSLOG: u32 = 8;
    pub const TRACEKIT: u32 = 10;
}

extern "C" fn rx_task(mut ctx: RefMut<RTKitTask>, pending: c_int) {
    /*
    let state = ctx.state;
    let msg = &raw mut ctx.msg;
    let endpoint = ctx.msg.data1;
    let res = match endpoint {
        Endpoint::MGMT => {
            = unsafe { bindings::rtkit_handle_mgmt(state, msg) };
        },
        Endpoint::CRASHLOG => {
        },
        Endpoint::SYSLOG => {
        },
        Endpoint::IOREPORT => {
        },
        Endpoint::OSLOG => {
        },
        Endpoint::TRACEKIT => {
        },
        _ => {
        },
    };
    if res != 0 {
    }
    unsafe {
        bindings::rtkit_rx_task(get_field!(ctx, ctx).as_ptr(), pending);
    }
    */
}

extern "C" fn rx_callback(cookie: *mut c_void, msg: bindings::apple_mbox_msg) -> c_int {
    /*
    let mut rktask = RefMut::new_in_heap(RTKitTask {
        task: Task::new(), ctx: /*RTKitCtx*/bindings::rtkit_task { msg, state: cookie.cast() }
    }, NOWAIT);
    get_field!(rktask, task).init_and_enqueue(rx_task, rktask, unsafe { bindings::taskqueue_thread }).unwrap();
    */
    0
}

#[repr(C)]
#[derive(Debug)]
pub struct RTKit {
    cstruct: bindings::rtkit_state,
}

impl RTKit {
    pub fn new(dev: Device, noalloc: bool) -> Box<Self> {
        let mut init: bindings::rtkit_state = unsafe { MaybeUninit::zeroed().assume_init() };
        init.dev = dev.as_ptr();
        init.iop_pwrstate = bindings::RTKIT_MGMT_PWR_STATE_SLEEP as u16;
        init.ap_pwrstate = bindings::RTKIT_MGMT_PWR_STATE_QUIESCED as u16;
        init.verbose = false;
        init.noalloc = noalloc;
        Box::new(Self { cstruct: init })
    }

    pub fn boot(&mut self) -> Result<()> {
        let dev = self.cstruct.dev.as_rust_type();
        let mbox = apple_mbox::Driver::get(dev)?;
        let state = addr_of_mut!(self.cstruct);
        unsafe {
            (*state).mbox.dev = mbox.as_ptr();
        }
        apple_mbox_driver
            .set_rx(mbox, rx_callback, state.cast())
            .unwrap();
        let res = self.set_iop_pwrstate(bindings::RTKIT_MGMT_PWR_STATE_ON as u16);
        if res != 0 {
            Err(ErrCode::from(res))
        } else {
            Ok(())
        }
    }

    pub fn set_iop_pwrstate(&mut self, pwrstate: u16) -> c_int {
        unsafe { bindings::rtkit_set_iop_pwrstate(addr_of_mut!(self.cstruct), pwrstate) }
    }
    pub fn set_ap_pwrstate(&mut self, pwrstate: u16) -> c_int {
        unsafe { bindings::rtkit_set_ap_pwrstate(addr_of_mut!(self.cstruct), pwrstate) }
    }
}

#[no_mangle]
extern "C" fn rtkit_init(dev: bindings::device_t, noalloc: bool) -> *mut bindings::rtkit_state {
    let dev = dev.as_rust_type();
    Box::into_raw(RTKit::new(dev, noalloc)).cast()
}

#[no_mangle]
unsafe extern "C" fn rtkit_boot(state: *mut bindings::rtkit_state) -> c_int {
    let rtkit = unsafe { state.cast::<RTKit>().as_mut().unwrap() };
    rtkit.boot().unwrap();
    0
}
