#![no_std]

use core::ffi::{CStr, c_char, c_void};
use kpi::bindings::{
    FWRITE, GID_WHEEL, IOCNUM_RUN, IOCNUM_SUSPEND, MAKEDEV_CHECKNAME, MAKEDEV_WAITOK, UID_ROOT,
    VM_MAX_NAMELEN, macro_VMMCTL_VM_CREATE, macro_VMMCTL_VM_DESTROY, vmmctl_vm_create,
    vmmctl_vm_destroy, vm_run, vm_suspend
};
use kpi::vec::Vec;
use kpi::boxed::{Box, LinkedList};
use kpi::cdev::{CDevSw, IoctlCmd};
use kpi::collections::Pod;
use kpi::ffi::{CString, NullablePtr, Ptr, Softc};
use kpi::module::Module;
use kpi::prelude::*;
use kpi::proc::{CurThread, Thread, Ucred};
use kpi::sync::Checked;
use kpi::sync::sx::SxLock;
use kpi::{define_cdev, define_module, define_pinned_struct};

struct VmmrMod;

static VMMR: NullablePtr<Softc<VmmrCtlSoftc>> = NullablePtr::new();

impl Module for VmmrMod {
    fn on_load(_: *mut c_void) -> Result<()> {
        println!("vmmr: loaded the module");
        let ctl = vmmrctl_create()?;
        VMMR.swap(ctl);
        Ok(())
    }

    fn on_unload(_: *mut c_void) -> Result<()> {
        println!("vmmr: unloaded the module");
        let ctl = VMMR.take().ok_or(EINVAL)?;
        destroy_dev(ctl)?;
        Ok(())
    }
}

define_pinned_struct! {
    #[derive(Default)]
    pub struct VmmrCtlSoftc {
        vms: SxLock<LinkedList<Ptr<Softc<VmmrDevSoftc>>>>,
    }
}

define_pinned_struct! {
    pub struct VmmrDevSoftc {
        name: [c_char; 230],//CString,
        cred: Ptr<Ucred>,
        vm: Box<Vm>,
    }
}

struct Vcpu {
}

#[derive(Default)]
struct Vm {
    vcpus: Vec<Vcpu>,
}

fn md_vm_create() -> Box<Vm> {
    let mut vcpus = Vec::with_capacity(4, M_WAITOK);
    while vcpus.len() < vcpus.capacity() {
        vcpus.push(Vcpu {});
    }
    let vm = Vm { vcpus };
    Box::new(vm, M_WAITOK)
}

fn vmmrctl_create() -> Result<Ptr<Softc<VmmrCtlSoftc>>> {
    let ctl = Softc::new::<M_DEVBUF>(VmmrCtlSoftc::default(), M_WAITOK);
    sx_init(ctl.project().vms, c"vmmrctl");
    let mut args = make_dev_args_init(ctl, &vmmrctl_cdevsw);
    args.name = c"vmmrctl";
    args.flags = MAKEDEV_CHECKNAME | MAKEDEV_WAITOK;
    args.uid = UID_ROOT;
    args.gid = GID_WHEEL;
    args.mode = 0o600;
    make_dev_s(args)
}

fn vmmrdev_create(ctl: &VmmrCtlSoftc, name: &[c_char; 230], cred: &Ucred) -> Result<()> {
    let mut vms = sx_xlock(&ctl.vms);
    for vm in vms.iter() {
        if vm.name == *name && cr_cansee(cred, &vm.cred) {
            return Err(EEXIST);
        }
    }

    let cred = crhold(cred);
    let vm = md_vm_create();
    let sc = Softc::new::<M_DEVBUF>(VmmrDevSoftc { name: *name, cred, vm }, M_WAITOK);

    let mut args = make_dev_args_init(sc, &vmmrdev_cdevsw);
    args.name = c"vmmr/vm0";
    args.flags = MAKEDEV_CHECKNAME | MAKEDEV_WAITOK;
    args.uid = UID_ROOT;
    args.gid = GID_WHEEL;
    //args.mda_cr = sc->ucred;
    args.mode = 0o600;

    let sc = make_dev_s(args)?;
    vms.push_back(sc, M_WAITOK);
    sx_xunlock(vms);

    Ok(())
}

fn vmmrdev_destroy(ctl: &VmmrCtlSoftc, name: &[c_char; 230], cred: &Ucred) -> Result<()> {
    let mut vms = sx_xlock(&ctl.vms);
    let mut found = None;
    for vm in vms.iter() {
        if vm.name == *name && cr_cansee(cred, &vm.cred) {
            found = Some(vm);
            break;
        }
    }
    sx_xunlock(vms);
    // vm_suspend
    //if let Some(vm) = found {
    //    destroy_dev(vm);
    //}
    Ok(())
}

impl CDevSw for VmmrDev {
    type Softc = VmmrDevSoftc;
    fn d_open(sc: &Softc<VmmrDevSoftc>, fflag: i32, devtype: i32, td: &Thread) -> Result<()> {
        Ok(())
    }
    fn d_ioctl(
        sc: &Softc<VmmrDevSoftc>,
        cmd: IoctlCmd,
        data: Option<&mut [u8]>,
        fflag: i32,
        td: &CurThread,
    ) -> Result<()> {
        if cmd.group() != b'v' {
            return Err(EINVAL);
        }
        let Some(data) = data else {
            return Err(EINVAL);
        };
        if u32::from(cmd.number()) == IOCNUM_RUN {
            let run: &vm_run = Pod::from_bytes(data)?;
            println!("VM_RUN");
        }
        if u32::from(cmd.number()) == IOCNUM_SUSPEND {
            let suspend: &vm_suspend = Pod::from_bytes(data)?;
            println!("VM_SUSPEND");
        }
        Ok(())
    }
}

impl CDevSw for VmmrCtl {
    type Softc = VmmrCtlSoftc;

    fn d_open(ctl: &Softc<VmmrCtlSoftc>, fflag: i32, devtype: i32, td: &Thread) -> Result<()> {
        if fflag & FWRITE != 0 {
            return Err(EINVAL);
        }
        Ok(())
    }
    fn d_close(ctl: &Softc<VmmrCtlSoftc>, fflag: i32, devtype: i32, td: &Thread) -> Result<()> {
        Ok(())
    }
    fn d_ioctl(
        ctl: &Softc<VmmrCtlSoftc>,
        cmd: IoctlCmd,
        data: Option<&mut [u8]>,
        fflag: i32,
        td: &CurThread,
    ) -> Result<()> {
        let Some(data) = data else {
            return Err(EINVAL);
        };
        if cmd.as_raw() == macro_VMMCTL_VM_CREATE {
            let vmc: &mut vmmctl_vm_create = Pod::from_bytes_mut(data)?;
            vmc.name[VM_MAX_NAMELEN as usize] = 0;
            //let name = CStr::from_bytes_until_nul(&vmc.name);
            for n in vmc.reserved {
                if n != 0 {
                    return Err(EINVAL);
                }
            }
            vmmrdev_create(ctl, &vmc.name, td.ucred())?;
            return Ok(());
        }
        if cmd.as_raw() == macro_VMMCTL_VM_DESTROY {
            let vmd: &mut vmmctl_vm_destroy = Pod::from_bytes_mut(data)?;
            vmd.name[VM_MAX_NAMELEN as usize] = 0;
            //let name = CStr::from_bytes_until_nul(&vmd.name);
            for n in vmd.reserved {
                if n != 0 {
                    return Err(EINVAL);
                }
            }
            return Ok(());
        }
        Err(ENOTTY)
    }
}

define_cdev! {
    static vmmrdev_cdevsw: VmmrDev = {
        d_name: c"vmmrdev",
        d_open: vmmrdev_open,
    }
}

define_cdev! {
    static vmmrctl_cdevsw: VmmrCtl = {
        d_name: c"vmmrctl",
        d_open: vmmrctl_open,
        d_close: vmmrctl_close,
        d_ioctl: vmmrctl_ioctl,
    }
}

define_module!(VmmrMod, vmmr_modevent);
