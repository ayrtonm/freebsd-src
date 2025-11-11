/*-
 * Copyright (c) 2024 Ayrton Muñoz
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

#![no_std]

use core::ffi::CStr;
use core::sync::atomic::{AtomicU8, AtomicU64, Ordering};
use kpi::bindings::{SB_FLAG_NO_RANGES, bus_size_t, bus_space_handle_t, device_t, simplebus_softc};
use kpi::bus::Register;
use kpi::device::{BusProbe, DeviceIf};
use kpi::driver;
use kpi::ffi::SubClass;
use kpi::intr::ConfigHook;
use kpi::prelude::*;
use kpi::sync::Mutable;
use kpi::sync::arc::{Arc, ArcRef, UninitArc};

use apple_mbox::AppleMboxMsg;
use rtkit::{Endpoint, RTKit};

#[repr(u16)]
#[derive(Copy, Clone, Debug)]
enum SmcCmd {
    ReadKey = 0x10,
    WriteKey = 0x11,
    MsgInit = 0x17,
    Notif = 0x18,
}

fn smc_cmd(data: u64) -> u16 {
    (data & 0xff) as u16
}

// I don't care about the null-terminator here but want to avoid &str because the UTF-8 requirement
// means the conversion to bytes isn't always a 1 for 1 with the characters as written.
fn smc_key(key: &CStr) -> u32 {
    let key_bytes = key.to_bytes();
    let s0 = key_bytes[0] as u32;
    let s1 = key_bytes[1] as u32;
    let s2 = key_bytes[2] as u32;
    let s3 = key_bytes[3] as u32;
    (s0 << 24) | (s1 << 16) | (s2 << 8) | s3
}

const SMC_EP: Endpoint = 0x20;

pub type AppleSmcSoftc = SubClass<simplebus_softc, AppleSmcSoftcFields>;

#[repr(C)]
#[derive(Debug)]
pub struct AppleSmcSoftcFields {
    dev: device_t,
    gpiobus: Mutable<Option<device_t>>,
    sram: Mutable<Register>,
    rtkit: Arc<RTKit>,
    config_hook: ConfigHook,
    msgid: AtomicU8,
    data: AtomicU64,
}

impl DeviceIf for AppleSmcDriver {
    type Softc = AppleSmcSoftc;

    fn device_probe(dev: device_t) -> Result<BusProbe> {
        if !ofw_bus_status_okay(dev) {
            return Err(ENXIO);
        }
        if !ofw_bus_is_compatible(dev, c"apple,smc") {
            return Err(ENXIO);
        }
        device_set_desc(dev, c"Apple SMC");
        return Ok(BUS_PROBE_DEFAULT);
    }

    fn device_attach(uninit_sc: UninitArc<AppleSmcSoftc>, dev: device_t) -> Result<()> {
        let mut simplebus_sc = simplebus_softc::default();
        simplebus_sc.dev = dev;
        simplebus_sc.flags |= SB_FLAG_NO_RANGES;

        let mut rtkit =
            RTKit::new(dev).inspect_err(|e| device_println!(dev, "failed to create RTKit {e}"))?;

        rtkit.set_verbose();
        let rtkit = Arc::new(rtkit, M_DEVBUF, M_WAITOK);

        let node = ofw_bus_get_node(dev);
        let rid = ofw_bus_find_string_index(node, c"reg-names", c"sram")?;
        let sram = Register::new(bus_alloc_resource_any(
            dev,
            SYS_RES_MEMORY,
            rid,
            RF_ACTIVE | RF_UNMAPPED,
        )?)?;

        let smc_sc = AppleSmcSoftcFields {
            dev,
            gpiobus: Mutable::new(None),
            sram: Mutable::new(sram),
            rtkit,
            config_hook: ConfigHook::new(),
            msgid: AtomicU8::new(0),
            data: AtomicU64::new(0),
        };
        let mut sc = uninit_sc.init(SubClass::new_with_base(simplebus_sc, smc_sc));

        let sc = sc.into_arc();
        sc.config_hook.init(start_config_hook, sc.clone());

        let res = unsafe { bindings::simplebus_attach(dev) };
        if res != 0 {
            device_println!(dev, "simplebus_attach failed {res}");
            return Err(ENXIO);
        };
        config_intrhook_establish(&sc.config_hook).map_err(|e| {
            device_println!(dev, "failed to establish hook {e}");
            return ENXIO;
        })?;

        Ok(())
    }

    fn device_detach(sc: Arc<AppleSmcSoftc>, dev: device_t) -> Result<()> {
        todo!("")
    }
}

impl AppleSmcDriver {
    fn apple_smc_pin_set(dev: device_t, pin: u32, val: u32) -> Result<()> {
        todo!("")
    }
}

extern "C" fn start_config_hook(sc: Arc<AppleSmcSoftc>) {
    let dev = sc.dev;
    callback(sc)
        .inspect_err(|e| {
            device_println!(dev, "config hook failed {e}");
        })
        .unwrap();

    fn callback(sc: Arc<AppleSmcSoftc>) -> Result<()> {
        let dev = sc.dev;
        // Set the mailbox rx callback
        RTKit::set_rx(sc.rtkit.clone())?;

        // Set IOP power state
        sc.rtkit.boot()?;

        sc.rtkit
            .start_endpoint(SMC_EP, smc_rtkit_callback, sc.clone())?;

        send_cmd(&sc, SmcCmd::MsgInit, 0, 0);
        tsleep(&sc.data, Some(PWAIT), c"apple,smc", 5 * hz())?;
        let data = sc.data.load(Ordering::Relaxed);

        *sc.sram.get_mut() = Register::new(bus_alloc_resource_any(
            dev,
            SYS_RES_MEMORY,
            1,
            RF_ACTIVE | RF_UNMAPPED,
        )?)?;
        let mut handle = bus_space_handle_t::default();
        unsafe {
            bindings::rman_set_bustag(sc.sram.get_mut().as_ptr(), &raw mut bindings::memmap_bus)
        };

        /*
        let res = unsafe {
            bindings::rust_bindings_bus_space_map(
                &raw mut bindings::memmap_bus,
                data,
                0x4000,
                bindings::BUS_SPACE_MAP_NONPOSTED,
                &mut handle,
            )
        };
        assert!(res == 0);
        unsafe { bindings::rman_set_bushandle(sc.sram.get_mut().as_ptr(), handle) };
        */

        let mut node = OF_child(ofw_bus_get_node(dev));
        while let Some(child) = node {
            if !OF_hasprop(child, c"gpio-controller") {
                node = OF_peer(child);
                continue;
            }
            let xref = OF_getencprop_as_xref(child, c"phandle")?;
            OF_device_register_xref(xref, dev);
            *sc.gpiobus.get_mut() = Some(gpiobus_add_bus(dev)?);
            node = OF_peer(child);
        }

        device_println!(dev, "enabling notifications");
        let ntap = 1;
        write_key(&sc, smc_key(c"NTAP"), &[ntap]).unwrap();
        device_println!(dev, "enabled notifications");

        config_intrhook_disestablish(&sc.config_hook);
        unsafe { bindings::bus_attach_children(dev) };
        Ok(())
    }
}

fn smc_rtkit_callback(sc: ArcRef<AppleSmcSoftc>, data: u64) -> Result<()> {
    device_println!(sc.dev, "apple SMC callback got data {data:?}");
    if smc_cmd(data) == SmcCmd::Notif as u16 {
        handle_notification(sc, data);
        return Ok(());
    }
    sc.data.store(data, Ordering::Relaxed);
    wakeup(&sc.data);
    Ok(())
}

fn handle_notification(sc: ArcRef<AppleSmcSoftc>, data: u64) {}

fn write_key(sc: &AppleSmcSoftc, key: u32, data: &[u8]) -> Result<()> {
    let len = data.len() as bus_size_t;
    let sram = sc.sram.get_mut().as_ptr();
    unsafe { bindings::rust_bindings_bus_write_region_1(sram, 0, data.as_ptr().cast_mut(), len) }
    unsafe { bindings::rust_bindings_bus_barrier(sram, 0, len, bindings::BUS_SPACE_BARRIER_WRITE) };
    send_cmd(sc, SmcCmd::WriteKey, key, len as u16)
}

fn send_cmd(sc: &AppleSmcSoftc, cmd: SmcCmd, key: u32, len: u16) -> Result<()> {
    let msgid = sc.msgid.fetch_add(1, Ordering::Relaxed);
    sc.rtkit.send(SmcTxMsg::new(cmd, key, len, msgid))
}

pub struct SmcTxMsg(u64);

impl SmcTxMsg {
    pub fn new(cmd: SmcCmd, key: u32, len: u16, msgid: u8) -> Self {
        let mut data = cmd as u16 as u64;
        data |= (len as u64) << 16;
        data |= (key as u64) << 32;
        data |= u64::from(msgid & 0xf) << 12;
        Self(data)
    }
}

impl Into<AppleMboxMsg> for SmcTxMsg {
    fn into(self) -> AppleMboxMsg {
        AppleMboxMsg {
            data0: self.0,
            data1: SMC_EP.into(),
        }
    }
}

driver!(apple_smc_driver, c"apple_smc", AppleSmcDriver,
    apple_smc_methods = {
        device_probe apple_smc_probe,
        device_attach apple_smc_attach,
        device_detach apple_smc_detach,
    },
    inherit from simplebus_driver,
    //EXPORTS {
    //    apple_smc_pin_set(dev: device_t, pin: u32, val: u32) -> int;
    //}
);
