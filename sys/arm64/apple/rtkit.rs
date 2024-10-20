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

use core::ffi::{c_int, c_void};
use core::mem::MaybeUninit;
use core::ptr::{addr_of_mut, null_mut};
use kpi::allocator::NOWAIT;
use kpi::device::Device;
use kpi::prelude::*;
use kpi::taskq;
use kpi::taskq::Task;
use kpi::{bindings, enum_c_macros, get_field};

use apple_mbox::{apple_mbox_driver, AppleMboxMsg};

// I don't want a real enum here since other values may also be endpoints but CamelCase is not
// standard style
#[allow(nonstandard_style)]
mod Endpoint {
    pub const MGMT: u32 = 0;
    pub const CRASHLOG: u32 = 1;
    pub const SYSLOG: u32 = 2;
    pub const DEBUG: u32 = 3;
    pub const IOREPORT: u32 = 4;
    pub const OSLOG: u32 = 8;
    pub const TRACEKIT: u32 = 10;
}

#[repr(u16)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum PwrState {
    Sleep = 0x0001,
    Quiesced = 0x0010,
    On = 0x0020,
}

#[repr(C)]
#[derive(Debug)]
pub struct RTKit {
    client: Device,
    mbox: Device,

    iop_pwrstate: PwrState,
    ap_pwrstate: PwrState,
    verbose: bool,
    noalloc: bool,
    /*
    epmap: u64,
    callbacks: [Callback; 32],
    */
}

// RTKitTask is a subclass of `struct task`
type RTKitTask = SubClass<Task, RTKitTaskFields>;

#[derive(Debug)]
struct RTKitTaskFields {
    rtkit: Ptr<RTKit>,
    msg: AppleMboxMsg,
}

impl RTKit {
    pub fn new(client: Device, noalloc: bool) -> Result<Box<Self>> {
        let mbox = apple_mbox_driver.get(client)?;
        let iop_pwrstate = PwrState::Sleep;
        let ap_pwrstate = PwrState::Quiesced;
        Ok(Box::new_in(
            Self {
                client,
                mbox,

                iop_pwrstate,
                ap_pwrstate,

                verbose: false,
                // TODO: only used in smc, may be unnecessary
                noalloc,
            },
            WAITOK,
        ))
    }

    pub fn boot(&mut self) -> Result<()> {
        let rtkit = unsafe { Ptr::new(self as *mut Self) };
        apple_mbox_driver.set_rx(self.mbox, rx_callback, rtkit)?;
        let res = self.set_iop_pwrstate(PwrState::On);
        if res != 0 {
            Err(ErrCode::from(res))
        } else {
            Ok(())
        }
    }

    pub fn set_iop_pwrstate(&mut self, pwrstate: PwrState) -> c_int {
        0
    }

    pub fn set_ap_pwrstate(&mut self, pwrstate: PwrState) -> c_int {
        0
    }
}

fn rx_callback(rtkit: Ptr<RTKit>, msg: AppleMboxMsg) -> Result<()> {
    let ctx = RTKitTaskFields { rtkit, msg };
    // Put the task and its context in a `Box` (i.e. on the heap). The `struct task` is
    // zero-initialized while the other context fields are initialized with the arguments
    let mut task = Box::new_in(RTKitTask::new(ctx), NOWAIT);
    // Set the task's own address as the context and enqueue on taskqueue_thread with the
    // callback rx_task. `enqueue_with_self` consumes `task` so it can't be used again in this
    // function and internally `enqueue_with_self` calls `Box::into_raw` to avoid freeing `task`
    // itself. When the callback is invoked it's passed the boxed task as an argument so it
    // either explicitly passes ownership of the Box to something else or implicitly drops it at
    // the end of the function (i.e. frees the allocated task and context)
    task.enqueue_with_self(rx_task, taskq::thread())?;
    Ok(())
}

mod mgmt {
    pub const HELLO: u64 = 1;
    pub const HELLO_ACK: u64 = 2;
    pub const STARTEP: u64 = 5;
    pub const IOP_PWR_STATE: u64 = 6;
    pub const IOP_PWR_STATE_ACK: u64 = 7;
    pub const EPMAP: u64 = 8;
    pub const AP_PWR_STATE: u64 = 11;
    pub fn msg_type(x: u64) -> u64 {
        (x >> 52) & 0xff
    }
    pub fn hello_minver(x: u64) -> u64 {
        x & 0xffff
    }
    pub fn hello_maxver(x: u64) -> u64 {
        (x >> 16) & 0xffff
    }
}

/* versions we support */
const MINVER: u64 = 11;
const MAXVER: u64 = 12;

fn handle_mgmt(ctx: &RTKitTask) -> Result<()> {
    let data0 = ctx.msg.data0;
    let ty = mgmt::msg_type(data0);
    match ty {
        mgmt::HELLO => {
            let minver = mgmt::hello_minver(data0);
            let maxver = mgmt::hello_maxver(data0);
            if minver > MAXVER {
                return Err(EINVAL);
            }
            if maxver < MINVER {
                return Err(EINVAL);
            }
            let mut ver = maxver;
            if MAXVER < ver {
                ver = MAXVER;
            }
            apple_mbox_driver.write(get_field!(ctx.rtkit, mbox).flatten(), todo!(""));
        },
        mgmt::HELLO_ACK => {
        },
        mgmt::STARTEP => {
        },
        mgmt::IOP_PWR_STATE => {
        },
        mgmt::IOP_PWR_STATE_ACK => {
        },
        mgmt::EPMAP => {
        },
        mgmt::AP_PWR_STATE => {
        },
        _ => {
        },
    }
    Ok(())
}

extern "C" fn rx_task(ctx: Box<RTKitTask>, pending: c_int) {
    let endpoint = ctx.msg.data1;
    let res = match endpoint {
        Endpoint::MGMT => handle_mgmt(&ctx),
    //    Endpoint::CRASHLOG => {}
    //    Endpoint::SYSLOG => {}
    //    Endpoint::DEBUG => {}
    //    Endpoint::IOREPORT => {}
    //    Endpoint::OSLOG => {}
    //    Endpoint::TRACEKIT => {}
        _ => {
            todo!("")
        }
    };
    if let Err(err) = res {
        // We have one extra level of indirection (i.e. `device_t *` or equivalently
        // `struct _device **` so we need to flatten the `Ptr<Ptr<_device>>` into `Ptr<_device>`.
        let client_dev: Ptr<Device> = get_field!(ctx.rtkit, client);
        dprintln!(client_dev.flatten(),
            "Error ({err:?}) while handling RTKit message from endpoint {endpoint:?}"
        );
    }
}

#[no_mangle]
extern "C" fn rtkit_init(dev: Device, noalloc: bool) -> *mut bindings::rtkit_state {
    Box::into_raw(RTKit::new(dev, noalloc).unwrap()).cast()
}

#[no_mangle]
unsafe extern "C" fn rtkit_boot(state: *mut bindings::rtkit_state) -> c_int {
    let rtkit = unsafe { state.cast::<RTKit>().as_mut().unwrap() };
    rtkit.boot().unwrap();
    0
}
