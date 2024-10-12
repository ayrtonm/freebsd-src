#![no_std]
#![feature(allocator_api)]

extern crate alloc;

use alloc::boxed::Box;
use core::ffi::{c_int, c_void};
use core::mem::MaybeUninit;
use core::ptr::{addr_of_mut, null_mut};
use kpi::device::Device;
use kpi::prelude::*;
use kpi::{bindings, AsRustType, PointsTo};

use apple_mbox::apple_mbox_driver;

extern "C" fn rtkit_rx_callback(cookie: *mut c_void, msg: bindings::apple_mbox_msg) -> c_int {
    let state = cookie.cast::<bindings::rtkit_state>();
    let mut task = Box::new_in(
        bindings::rtkit_task {
            task: bindings::task {
                ta_link: unsafe { MaybeUninit::zeroed().assume_init() },
                ta_pending: 0,
                ta_priority: 0,
                ta_flags: 0,
                ta_func: Some(bindings::rtkit_rx_task),
                ta_context: null_mut(),
            },
            msg,
            state,
        },
        NOWAIT,
    );
    let tq_ptr = addr_of_mut!(task.task);
    let ctx_ptr = addr_of_mut!(task.task.ta_context);
    let task_ptr = Box::into_raw(task);
    unsafe {
        *ctx_ptr = task_ptr.cast();
        bindings::taskqueue_enqueue(bindings::taskqueue_thread, tq_ptr);
    }
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
            .driver
            .set_rx(mbox, rtkit_rx_callback, state.cast())
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
