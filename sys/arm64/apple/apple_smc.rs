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
#![feature(macro_metavar_expr_concat)]

use core::ffi::c_void;
use core::sync::atomic::{AtomicU64, Ordering};
use kpi::bindings::{intr_config_hook, simplebus_softc, SB_FLAG_NO_RANGES};
use kpi::bus::Register;
use kpi::cell::OwnedRef;
use kpi::cell::{Checked, SubClass};
use kpi::device::{BusProbe, Device, DeviceIf};
use kpi::driver;
use kpi::intr::ConfigHook;
use rtkit::RTKit;

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

pub type AppleSmcSoftcSub = SubClass<simplebus_softc, AppleSmcSoftc>;

#[derive(Debug)]
pub struct AppleSmcSoftc {
    dev: Device,
    //gpiobus: Device,
    //sram: Checked<Register>,
    rtkit: RTKit,
    config_hook: ConfigHook<AppleSmcSoftcSub>,
    //msgid: u8,
    data: AtomicU64,
}

impl DeviceIf for AppleSmcDriver {
    type Softc = AppleSmcSoftcSub;

    fn device_probe(dev: Device) -> Result<BusProbe> {
        if !ofw_bus_status_okay(dev) {
            return Err(ENXIO);
        }
        if !ofw_bus_is_compatible(dev, c"apple,smc") {
            return Err(ENXIO);
        }
        device_set_desc(dev, c"Apple SMC");
        return Ok(BUS_PROBE_DEFAULT);
    }

    fn device_attach(dev: Device) -> Result<()> {
        let mut simplebus_sc = simplebus_softc::default();
        simplebus_sc.flags |= SB_FLAG_NO_RANGES;

        let rtkit =
            RTKit::new(dev).inspect_err(|e| device_println!(dev, "failed to create RTKit {e}"))?;
        let smc_sc = AppleSmcSoftc {
            dev,
            rtkit,
            config_hook: ConfigHook::new(),
            data: AtomicU64::new(0),
        };
        let mut sc = device_init_softc!(dev, SubClass::new_with_base(simplebus_sc, smc_sc));

        //let rid = ofw_bus_find_string_index(node, c"reg-names", c"sram")?;
        //sc.sram = bus_alloc_resource_any(dev, SYS_RES_MEMORY, rid, RF_ACTIVE)?;

        sc.config_hook.func = Self::start_config_hook;
        sc.config_hook.arg = sc.erase_lifetime();

        let error = unsafe { bindings::simplebus_attach(dev.as_ptr()) };
        if error != 0 {
            device_println!(dev, "simplebus_attach failed {error}");
            return Err(ENXIO);
        }
        config_intrhook_establish(project!(&mut sc.config_hook)).map_err(|e| {
            device_println!(dev, "failed to establish hook {e}");
            return ENOMEM;
        })?;

        Ok(())
    }
}

impl AppleSmcDriver {
    fn start_config_hook(sc: OwnedRef<AppleSmcSoftcSub>) -> Result<()> {
        let dev = sc.dev;
        let node = ofw_bus_get_node(dev);
        let rtkit = project!(&sc.rtkit);
        RTKit::boot(rtkit).inspect_err(|e| {
            device_println!(dev, "failed to boot RTKit {e}");
        })?;
        rtkit.start_endpoint(0x20, Self::rtkit_callback, &sc)?;

        Self::send_cmd(&sc, SmcCmd::MsgInit, 0, 0);
        device_println!(dev, "waiting up to 5s for command response");
        tsleep(&sc.data, bindings::PWAIT, c"apple,smc", unsafe {
            5 * bindings::hz
        });
        device_println!(dev, "got command response");
        Ok(())
    }

    extern "C" fn rtkit_callback(sc: OwnedRef<AppleSmcSoftcSub>, data: u64) -> Result<()> {
        if smc_cmd(data) == SmcCmd::Notif as u16 {
            Self::handle_notification(sc, data);
            return Ok(());
        }
        sc.data.store(data, Ordering::Relaxed);
        wakeup(&sc.data);
        //let mut data_ref = sc.data.get_mut();
        //*data_ref = Sleepable::new(data);
        //wakeup(&data_ref);
        Ok(())
    }

    fn handle_notification(sc: OwnedRef<AppleSmcSoftcSub>, data: u64) {}

    fn send_cmd(sc: &OwnedRef<AppleSmcSoftcSub>, cmd: SmcCmd, key: u32, len: u16) {}

    fn apple_smc_pin_set(dev: Device, pin: u32, val: u32) -> Result<()> {
        Ok(())
    }
}

driver!(apple_smc_driver, c"apple_smc", AppleSmcDriver, apple_smc_methods,
    INTERFACES {
        device_probe apple_smc_probe,
        device_attach apple_smc_attach,
    },
    EXPORTS {
        apple_smc_pin_set(dev: device_t, pin: u32, val: u32) -> int;
    }
);
