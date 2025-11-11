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

use crate::RTKit;
use kpi::prelude::*;

use crate::bindings::{
    bus_addr_t, bus_dma_segment_t, bus_dma_tag_t, bus_dmamap_t, bus_size_t, device_t,
};
use crate::{MgmtTxMsg, bindings};
use core::ffi::{c_int, c_void};
use core::mem::transmute;
use core::ptr::null_mut;
use core::sync::atomic::{AtomicU16, AtomicU64, Ordering};
use kpi::prelude::*;
use kpi::sync::{Mutable, RefMut};

#[repr(u16)]
#[derive(Debug)]
pub enum PwrState {
    Off = 0x0000,
    Sleep = 0x0001,
    Idle = 0x0201,
    Quiesced = 0x0010,
    On = 0x0020,
    Init = 0x0220,
}

impl Into<u16> for PwrState {
    fn into(self) -> u16 {
        self as u16
    }
}

impl RTKit {
    pub fn set_iop(&self, pwr_state: PwrState) -> Result<()> {
        dbg!(self, "called set_iop with {pwr_state:x?}");
        let pwr_state = pwr_state.into();
        assert!(unsafe { bindings::cold } == 0);
        // If already in the correct power state do nothing
        if self.iop.load(Ordering::Relaxed) == pwr_state {
            dbg!(self, "IOP already in state {pwr_state:x?}");
            return Ok(());
        }

        // Try setting the power state
        let msg = MgmtTxMsg::IopPwrState { pwr_state };
        dbg!(self, "sending IOP pwr state request {msg:x?}");
        self.send(msg)?;

        // If the power state hasn't changed then sleep
        if self.iop.load(Ordering::Relaxed) != (pwr_state & 0xff) {
            dbg!(self, "sleep since IOP pwr state didn't change");
            let _ = tsleep(&self.iop, bindings::PWAIT, c"ioppwr", 5 * hz()).inspect_err(|e| {
                device_println!(
                    self.client,
                    "timed out waiting for IOP power state change {e:?}"
                );
            });
            Ok(())
        } else {
            device_println!(self.client, "IOP power state changed to {pwr_state:x?}");
            Ok(())
        }
    }

    pub fn set_ap(&self, pwr_state: PwrState) -> Result<()> {
        let pwr_state = pwr_state.into();
        if self.ap.load(Ordering::Relaxed) == pwr_state {
            return Ok(());
        }

        let msg = MgmtTxMsg::ApPwrState { pwr_state };
        self.send(msg)?;

        if self.ap.load(Ordering::Relaxed) != pwr_state {
            tsleep(&self.ap, bindings::PWAIT, c"appwr", hz())
        } else {
            Ok(())
        }
    }
}
