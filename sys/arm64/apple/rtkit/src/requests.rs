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

use kpi::prelude::*;

use crate::endpoints::Endpoint;
use apple_mbox::AppleMboxMsg;
use core::ffi::{c_int, c_void};
use core::mem::transmute;
use core::ptr::null_mut;
use core::sync::atomic::{AtomicU16, AtomicU64, Ordering};
use kpi::bindings;
use kpi::bindings::{
    bus_addr_t, bus_dma_segment_t, bus_dma_tag_t, bus_dmamap_t, bus_size_t, device_t,
};
use kpi::prelude::*;
use kpi::sync::{Mutable, RefMut};

#[derive(Debug, Copy, Clone)]
pub enum MgmtRxMsg {
    Hello {
        minver: u16,
        maxver: u16,
    },
    IopPwrStateAck(u16),
    ApPwrStateAck(u16),
    EpMap {
        base: u8, // only 3 bits
        bitmap: u32,
        last: bool,
    },
    Unknown(u8),
}

impl MgmtRxMsg {
    pub fn new(data0: u64) -> Self {
        match mgmt_msg_type(data0) {
            MGMT_HELLO => {
                let minver = mgmt_hello_minver(data0);
                let maxver = mgmt_hello_maxver(data0);
                Self::Hello { minver, maxver }
            }
            MGMT_IOP_PWR_STATE_ACK => Self::IopPwrStateAck(mgmt_pwr_state(data0)),
            // next one is not a typo
            MGMT_AP_PWR_STATE => Self::ApPwrStateAck(mgmt_pwr_state(data0)),
            MGMT_EP_MAP => {
                let base = mgmt_ep_map_base(data0);
                let bitmap = mgmt_ep_map_bitmap(data0);
                let last = mgmt_ep_map_last(data0);
                Self::EpMap { base, bitmap, last }
            }
            x => Self::Unknown(x),
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub enum MgmtTxMsg {
    HelloAck { minver: u16, maxver: u16 },
    IopPwrState { pwr_state: u16 },
    ApPwrState { pwr_state: u16 },
    StartEp { ep: Endpoint },
    EpMap { base: u8, last: bool },
}

impl Into<AppleMboxMsg> for MgmtTxMsg {
    fn into(self) -> AppleMboxMsg {
        let data0 = match self {
            Self::HelloAck { minver, maxver } => mgmt_hello_ack(minver, maxver),
            Self::IopPwrState { pwr_state } => mgmt_iop_pwr_state(pwr_state),
            Self::ApPwrState { pwr_state } => mgmt_ap_pwr_state(pwr_state),
            Self::EpMap { base, last } => {
                let res = (u64::from(base) << EP_MAP_BASE_SHIFT)
                    | ((MGMT_EP_MAP as u64) << MSG_TYPE_SHIFT);
                if last {
                    res | (1 << EP_MAP_LAST_SHIFT)
                } else {
                    res | EP_MAP_MORE
                }
            }
            Self::StartEp { ep } => {
                let ep: u32 = ep.into();
                let mut res = u64::from(ep) << START_EP_SHIFT;
                res |= START_EP_START;
                res |= u64::from(MGMT_START_EP) << MSG_TYPE_SHIFT;
                res
            }
        };
        AppleMboxMsg {
            data0,
            data1: Endpoint::Mgmt.into(),
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub enum EpTxMsg {
    BufferReq { ep: Endpoint, data: u64 },
}

impl Into<AppleMboxMsg> for EpTxMsg {
    fn into(self) -> AppleMboxMsg {
        let (data0, ep) = match self {
            Self::BufferReq { ep, data } => {
                let shift = if ep == Endpoint::OsLog {
                    OSLOG_TYPE_SHIFT
                } else {
                    MSG_TYPE_SHIFT
                };
                let res = (u64::from(BUFFER_REQUEST) << shift) | data;
                (res, ep)
            }
        };
        AppleMboxMsg {
            data0,
            data1: ep.into(),
        }
    }
}

pub const BUFFER_REQUEST: u8 = 1;
pub const BUFFER_SIZE_SHIFT: u64 = 44;

pub fn buffer_addr(x: u64) -> u64 {
    let mask = (1 << BUFFER_SIZE_SHIFT) - 1;
    x & mask
}
pub fn buffer_size(x: u64) -> u64 {
    ((x >> BUFFER_SIZE_SHIFT) as u8) as u64
}

const MGMT_HELLO: u8 = 1;
const MGMT_HELLO_ACK: u8 = 2;
const MGMT_START_EP: u8 = 5;
const MGMT_IOP_PWR_STATE: u8 = 6;
const MGMT_IOP_PWR_STATE_ACK: u8 = 7;
const MGMT_EP_MAP: u8 = 8;
const MGMT_AP_PWR_STATE: u8 = 11;

const MAXVER_SHIFT: u64 = 16;
const EP_MAP_BASE_SHIFT: u64 = 32;
const EP_MAP_LAST_SHIFT: u64 = 51;
const EP_MAP_MORE: u64 = 1 << 0;
const START_EP_SHIFT: u64 = 32;
const START_EP_START: u64 = 1 << 1;

const MSG_TYPE_SHIFT: u64 = 52;
const OSLOG_TYPE_SHIFT: u64 = 56;

pub fn mgmt_msg_type(x: u64) -> u8 {
    (x >> MSG_TYPE_SHIFT) as u8
}

fn mgmt_hello_minver(x: u64) -> u16 {
    x as u16
}

fn mgmt_hello_maxver(x: u64) -> u16 {
    (x >> MAXVER_SHIFT) as u16
}

fn mgmt_hello_ack(minver: u16, maxver: u16) -> u64 {
    let mut res = minver as u64;
    res |= (maxver as u64) << MAXVER_SHIFT;
    res |= (MGMT_HELLO_ACK as u64) << MSG_TYPE_SHIFT;
    res
}

fn mgmt_pwr_state(x: u64) -> u16 {
    x as u16
}

fn mgmt_iop_pwr_state(pwr_state: u16) -> u64 {
    let mut res = pwr_state as u64;
    res |= (MGMT_IOP_PWR_STATE as u64) << MSG_TYPE_SHIFT;
    res
}

fn mgmt_ap_pwr_state(pwr_state: u16) -> u64 {
    let mut res = pwr_state as u64;
    res |= (MGMT_AP_PWR_STATE as u64) << MSG_TYPE_SHIFT;
    res
}

fn mgmt_ep_map_base(x: u64) -> u8 {
    (x >> EP_MAP_BASE_SHIFT) as u8
}

fn mgmt_ep_map_bitmap(x: u64) -> u32 {
    x as u32
}

fn mgmt_ep_map_last(x: u64) -> bool {
    x & (1 << EP_MAP_LAST_SHIFT) != 0
}
