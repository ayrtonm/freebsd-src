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

use core::ffi::c_void;
use core::sync::atomic::{AtomicU64, Ordering};
use kpi::misc::Sleepable;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Endpoint {
    Mgmt,
    CrashLog,
    SysLog,
    Debug,
    IOReport,
    OsLog,
    TraceKit,
    Other(u32),
}

const RTKIT_EP_MGMT: u32 = 0;
const RTKIT_EP_CRASHLOG: u32 = 1;
const RTKIT_EP_SYSLOG: u32 = 2;
const RTKIT_EP_DEBUG: u32 = 3;
const RTKIT_EP_IOREPORT: u32 = 4;
const RTKIT_EP_OSLOG: u32 = 8;
const RTKIT_EP_TRACEKIT: u32 = 10;

impl Endpoint {
    pub fn new(ep: u32) -> Self {
        match ep {
            RTKIT_EP_MGMT => Endpoint::Mgmt,
            RTKIT_EP_CRASHLOG => Endpoint::CrashLog,
            RTKIT_EP_SYSLOG => Endpoint::SysLog,
            RTKIT_EP_DEBUG => Endpoint::Debug,
            RTKIT_EP_IOREPORT => Endpoint::IOReport,
            RTKIT_EP_OSLOG => Endpoint::OsLog,
            RTKIT_EP_TRACEKIT => Endpoint::TraceKit,
            another_ep => Endpoint::Other(another_ep),
        }
    }
}

impl Into<u32> for Endpoint {
    fn into(self) -> u32 {
        match self {
            Endpoint::Mgmt => RTKIT_EP_MGMT,
            Endpoint::CrashLog => RTKIT_EP_CRASHLOG,
            Endpoint::SysLog => RTKIT_EP_SYSLOG,
            Endpoint::Debug => RTKIT_EP_DEBUG,
            Endpoint::IOReport => RTKIT_EP_IOREPORT,
            Endpoint::OsLog => RTKIT_EP_OSLOG,
            Endpoint::TraceKit => RTKIT_EP_TRACEKIT,
            Endpoint::Other(another_ep) => another_ep,
        }
    }
}

#[derive(Debug)]
pub struct EpMap(AtomicU64);

impl Sleepable for EpMap {
    fn as_ptr(&self) -> *mut c_void {
        <AtomicU64 as Sleepable>::as_ptr(&self.0)
    }
}

impl EpMap {
    pub fn new() -> Self {
        Self(AtomicU64::new(0))
    }

    pub fn contains(&self, ep: Endpoint) -> bool {
        let ep: u32 = ep.into();
        let ep_bitset = self.0.load(Ordering::Relaxed);
        ep_bitset & u64::from(1u32 << ep) != 0
    }

    pub fn insert(&self, new_bitset: u64) -> u64 {
        let prev = self.0.fetch_or(new_bitset, Ordering::Relaxed);
        prev | new_bitset
    }
}
