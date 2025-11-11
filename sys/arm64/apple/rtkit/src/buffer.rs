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

use crate::requests::{BUFFER_SIZE_SHIFT, EpTxMsg, buffer_addr, buffer_size};
use crate::{Endpoint, RTKit};
use core::ffi::{c_int, c_void};
use core::mem::transmute;
use core::ptr::null_mut;
use core::sync::atomic::{AtomicU16, AtomicU64, Ordering};
use kpi::bindings;
use kpi::bindings::{
    bus_addr_t, bus_dma_segment_t, bus_dma_tag_t, bus_dmamap_t, bus_size_t, device_t,
};
use kpi::prelude::*;
use kpi::bus::dma::BusDmaTag;
use kpi::sync::{Mutable, RefMut};

#[derive(Debug, Default)]
pub struct RTKitBuffer {
    pub addr: bus_addr_t,
    size: bus_size_t,
    kva: *mut c_void,
    tag: BusDmaTag,
    map: bus_dmamap_t,
    state: Option<&'static RTKit>,
}

pub fn handle_buffer_req(
    rtkit: &'static RTKit,
    ep: Endpoint,
    data0: u64,
    get_buffer: fn(&RTKit) -> RefMut<RTKitBuffer>,
) -> Result<()> {
    let size = buffer_size(data0);
    device_println!(
        rtkit.client,
        "RTKit endpoint {ep:?} requested {} byte buffer",
        size << bindings::PAGE_SHIFT_4K
    );
    rtkit_alloc(rtkit, size << bindings::PAGE_SHIFT_4K, get_buffer)?;

    let data = (size << BUFFER_SIZE_SHIFT) | buffer_addr(get_buffer(&rtkit).addr);
    rtkit.send(EpTxMsg::BufferReq { ep, data })
}

extern "C" fn rtkit_dmamap_cb(
    buffer_as_void_ptr: *mut c_void,
    segs: *mut bus_dma_segment_t,
    nsegs: i32,
    error: i32,
) {
    let buffer_ptr = buffer_as_void_ptr.cast::<RTKitBuffer>();
    let buffer = unsafe { buffer_ptr.as_mut().unwrap() };
    let dev = buffer.state.as_ref().unwrap().client;
    device_println!(
        dev,
        "dma map callback reported {nsegs} segments and error {error}"
    );
    buffer.addr = unsafe { (*segs).ds_addr };
    buffer.size = unsafe { (*segs).ds_len };
    buffer.state = None;
    // TODO: call buffer.state.map_fn
    //rtkit.
}

fn rtkit_alloc(
    rtkit: &'static RTKit,
    req_size: bus_size_t,
    get_buffer: fn(&RTKit) -> RefMut<RTKitBuffer>,
) -> Result<()> {
    let dev = rtkit.client;
    let mut buffer = get_buffer(&rtkit);
    let parent_tag = bus_get_dma_tag(dev);
    buffer.tag = bus_dma_tag_create(parent_tag)
        .alignment(PAGE_SIZE) /* 16K alignment */
        .max_size(req_size)
        .max_seg_size(req_size)
        .flags(Some(BUS_DMA_COHERENT))
        .build()
        .inspect_err(|e| {
            device_println!(dev, "bus_dma_tag_create failed {e}");
        })?;
    buffer.map = bus_dmamem_alloc(buffer.tag.0, &mut buffer.kva, BUS_DMA_WAITOK | BUS_DMA_ZERO)
        .inspect_err(|e| {
            device_println!(dev, "bus_dmamem_alloc failed {e}");
        })?;

    // these should be initialized by the callback in the next function
    buffer.size = req_size;
    buffer.addr = 0;
    buffer.state = Some(rtkit.clone());
    //let buffer_cref: FatPtr<RTKitBuffer> = rtkit.project(|rtk| get_buffer(rtk));

    //let rc = bus_dmamap_load(
    //    buffer.tag,
    //    buffer.map,
    //    buffer.kva,
    //    req_size,
    //    Some(rtkit_dmamap_cb),
    //    // TODO: don't leak the rtkit buffer
    //    //null_mut(), //buffer,
    //    //unsafe { buffer_cref.leak_ref() }.0.cast::<c_void>(),
    //    (buffer.deref() as *const RTKitBuffer)
    //        .cast::<c_void>()
    //        .cast_mut(),
    //    None,
    //);
    //use core::ops::Deref;
    //if (rc != Ok(())) && (rc != Err(EINPROGRESS)) {
    //    device_println!(dev, "bus_dmamap_load failed {rc:?}");
    //    return rc;
    //}

    Ok(())
}
