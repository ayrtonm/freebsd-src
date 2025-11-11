/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
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
use kpi::driver::Driver;
use kpi::ffi::{Ref, SubClass, UninitRef};
use kpi::intr::ConfigHook;
use kpi::ofw::XRef;
use kpi::prelude::*;
use kpi::sync::Checked;
use kpi::{base, driver, proj};
use rtkit::{RTKitDriver, rtkit_boot, rtkit_init, PwrState, rtkit_set_ap};
use simplebus::{SimpleBusDriver, SimpleBusSoftc};

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

fn smc_key(key: &[u8]) -> u32 {
    let s0 = key[0] as u32;
    let s1 = key[1] as u32;
    let s2 = key[2] as u32;
    let s3 = key[3] as u32;
    (s0 << 24) | (s1 << 16) | (s2 << 8) | s3
}

const SMC_EP: Endpoint = 0x20;
const SMC_GPIO_CMD_OUTPUT: u32 = 1 << 24;

pub type AppleSmcSoftc = SimpleBusSoftc<AppleSmcSoftcFields>;

#[derive(Debug)]
pub struct AppleSmcSoftcFields {
    dev: device_t,
    gpiobus: Checked<Option<device_t>>,
    sram: Checked<Register>,
    rtk: RTKit<AppleSmcSoftc>,
    config_hook: ConfigHook,
    msgid: AtomicU8,
    data: AtomicU64,
}

impl RTKitDriver for AppleSmcDriver {
    type CallbackArg = AppleSmcSoftc;

    fn get_rtkit(sc: Ref<Self::Softc>) -> Ref<RTKit<Self::CallbackArg>> {
        proj!(&sc->rtk)
    }
}

impl SimpleBusDriver for AppleSmcDriver {}

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

    fn device_attach(uninit_sc: UninitRef<AppleSmcSoftc>, dev: device_t) -> Result<()> {
        let mut rtk = Self::new_rtkit(dev)
            .inspect_err(|e| device_println!(dev, "failed to create RTKit {e}"))?;

        rtk.no_alloc = true;
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
            gpiobus: Checked::new(None),
            sram: Checked::new(sram),
            rtk,
            config_hook: config_intrhook_init!(dev, start_config_hook),
            msgid: AtomicU8::new(0),
            data: AtomicU64::new(0),
        };
        let mut sc = uninit_sc.init(SubClass::new(smc_sc)).into_ref();

        let res = Self::simplebus_attach(dev, |simplebus_sc| {
            simplebus_sc.dev = dev;
            simplebus_sc.flags |= SB_FLAG_NO_RANGES;
        })
        .map_err(|e| {
            device_println!(dev, "simplebus_attach failed {e}");
            ENXIO
        })?;
        config_intrhook_establish(&sc.config_hook).map_err(|e| {
            device_println!(dev, "failed to establish hook {e}");
            ENXIO
        })?;

        Ok(())
    }
}

extern "C" fn start_config_hook(sc: Ref<AppleSmcSoftc>) {
    let dev = sc.dev;
    AppleSmcDriver::try_start_config_hook(sc)
        .inspect_err(|e| {
            device_println!(dev, "config hook failed {e}");
        })
        .unwrap();
}

impl AppleSmcDriver {
    fn try_start_config_hook(sc: Ref<AppleSmcSoftc>) -> Result<()> {
        let dev = sc.dev;
        rtkit_init(proj!(&sc->rtk))?;

        // Set IOP power state
        device_println!(dev, "setting IOP power state");
        rtkit_boot(proj!(&sc->rtk))?;

        device_println!(dev, "starting SMC endpoint");
        rtkit_set_ap(proj!(&sc->rtk), PwrState::On)?;
        sc.rtk
            .start_endpoint(SMC_EP, AppleSmcDriver::rtkit_callback)?;
        device_println!(dev, "sending SMC command");

        AppleSmcDriver::send_cmd(sc, SmcCmd::MsgInit, 0, 0)?;
        device_println!(sc.dev, "waiting 5s for RTKit callback to respond");
        if sc.data.load(Ordering::Relaxed) == 0 {
            let _ = tsleep(&sc.data, Some(PWAIT), c"apple,smc", 5 * hz());
            if sc.data.load(Ordering::Relaxed) == 0 {
                return Err(EWOULDBLOCK);
            }
        }
        let data = sc.data.load(Ordering::Relaxed);
        device_println!(sc.dev, "finished waiting 5s for RTKit callback to respond {data:x?}");

        let mut handle = bus_space_handle_t::default();
        unsafe {
            bindings::rman_set_bustag(sc.sram.get_mut().as_ptr(), &raw mut bindings::memmap_bus)
        };

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

        let mut node = OF_child(ofw_bus_get_node(dev));
        while let Some(child) = node {
            if !OF_hasprop(child, c"gpio-controller") {
                node = OF_peer(child);
                continue;
            }
            let xref = unsafe { OF_getencprop_unchecked::<XRef>(child, c"phandle")? };
            OF_device_register_xref(xref, dev);
            *sc.gpiobus.get_mut() = Some(gpiobus_add_bus(dev)?);
            node = OF_peer(child);
        }

        device_println!(dev, "enabling notifications");
        let ntap = 1;
        AppleSmcDriver::write_key(sc, smc_key(b"NTAP"), &[ntap]).unwrap();
        device_println!(dev, "enabled notifications");

        config_intrhook_disestablish(&sc.config_hook);
        unsafe { bindings::bus_attach_children(dev) };
        Ok(())
    }

    fn rtkit_callback(sc: Ref<AppleSmcSoftc>, data: u64) -> Result<()> {
        device_println!(sc.dev, "apple SMC callback got data {data:x?}");
        if smc_cmd(data) == SmcCmd::Notif as u16 {
            AppleSmcDriver::handle_notification(sc, data);
            return Ok(());
        }
        sc.data.store(data, Ordering::Relaxed);
        wakeup(&sc.data);
        Ok(())
    }

    fn handle_notification(sc: Ref<AppleSmcSoftc>, data: u64) {}

    fn write_key(sc: Ref<AppleSmcSoftc>, key: u32, data: &[u8]) -> Result<()> {
        let len = data.len() as bus_size_t;
        let sram = sc.sram.get_mut().as_ptr();
        unsafe {
            bindings::rust_bindings_bus_write_region_1(sram, 0, data.as_ptr().cast_mut(), len)
        }
        unsafe {
            bindings::rust_bindings_bus_barrier(sram, 0, len, bindings::BUS_SPACE_BARRIER_WRITE)
        };
        AppleSmcDriver::send_cmd(sc, SmcCmd::WriteKey, key, len as u16)
    }

    fn send_cmd(sc: Ref<AppleSmcSoftc>, cmd: SmcCmd, key: u32, len: u16) -> Result<()> {
        let msgid = sc.msgid.fetch_add(1, Ordering::Relaxed);
        sc.rtk.send(SmcTxMsg::new(cmd, key, len, msgid))
    }

    pub fn pin_set(dev: device_t, pin: u32, val: u32) -> Result<()> {
        assert_eq!(device_get_driver(dev), <Self as Driver>::DRIVER);
        let digits = b"0123456789abcdef";
        let mut key = smc_key(b"gP\0\0");
        let pin = pin as usize;
        key |= u32::from(digits[(pin >> 0) & 0xF]) << 0;
        key |= u32::from(digits[(pin >> 4) & 0xF]) << 8;

        //assume active low
        let val_bit = if val != 0 { 0 } else { 1 };

        let data = SMC_GPIO_CMD_OUTPUT | val_bit;
        let sc = device_get_softc!(dev);
        AppleSmcDriver::write_key(sc, key, &data.to_ne_bytes())
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn apple_smc_get_bus(dev: device_t) -> device_t {
        let sc = device_get_softc!(dev);
        sc.gpiobus.get_mut().unwrap()
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn apple_smc_pin_set(dev: device_t, pin: u32, value: u32) -> core::ffi::c_int {
        use kpi::kobj::AsCType;
        match AppleSmcDriver::pin_set(dev, pin, value) {
            Ok(()) => 0,
            Err(e) => e.as_c_type(),
        }
    }
}

pub struct SmcTxMsg(u64);

impl SmcTxMsg {
    pub fn new(cmd: SmcCmd, key: u32, len: u16, msgid: u8) -> Self {
        let mut data = cmd as u16 as u64;
        data |= u64::from(len) << 16;
        data |= u64::from(key) << 32;
        data |= u64::from(msgid & 0xf) << 12;
        Self(data)
    }
}

impl Into<AppleMboxMsg> for SmcTxMsg {
    fn into(self) -> AppleMboxMsg {
        AppleMboxMsg {
            data0: self.0,
            data1: SMC_EP,
        }
    }
}
macro_rules! gpio_get_bus {
    (get_desc) => { gpio_get_bus_desc };
}

macro_rules! gpio_pin_set {
    (get_desc) => { gpio_pin_set_desc };
}

driver!(apple_smc_driver, c"apple_smc", AppleSmcDriver,
    apple_smc_methods = {
        device_probe apple_smc_probe,
        device_attach apple_smc_attach,

        gpio_get_bus apple_smc_get_bus defined in C,
        gpio_pin_set apple_smc_pin_set defined in C,
    },
    inherit from simplebus_driver,
);
