#![no_std]

use kpi::boxed::LinkedList;
use core::ffi::{c_int, c_char, c_short, c_void};
use kpi::bindings::{AF_VSOCK, PR_CONNREQUIRED, SOCK_STREAM, sa_family_t};
use kpi::sync::arc::Arc;
use kpi::{define_protosw, define_sockaddr};
use kpi::misc::Thread;
use kpi::net::{ProtoSw, SockAddr, Socket};
use kpi::sync::OnceInit;
use kpi::sync::mtx::Mutex;
use kpi::prelude::*;
use core::pin::Pin;

define_sockaddr! {
    struct VsockAddr {
        port: u32,
        cid: u32,
    }
}

static SOCKETS: Mutex<SocketList> = Mutex::new(SocketList {
    bound: LinkedList::new(),
    connected: LinkedList::new(),
});

struct SocketList {
    bound: LinkedList<VsockAddr>,
    connected: LinkedList<VsockAddr>,
}

#[unsafe(no_mangle)]
extern "C" fn vsock_init(_arg: *mut c_void) {
    mtx_init(Pin::static_ref(&SOCKETS), c"vsock_trans_mtx", None, None);
}

pub struct VsockPcb {
    local_addr: OnceInit<VsockAddr>,
    remote_addr: OnceInit<VsockAddr>,
}

const VSOCK_SOCK_PROTO_TRANS: i16 = 1;

impl ProtoSw for VsockSw {
    type Pcb = VsockPcb;

    /*
     * Called in two cases:
     * 1) When user calls socket();
     * 2) When we accept new incoming conneciton and call sonewconn().
     */
    fn pr_attach(so: Socket<VsockPcb>, proto: i32, td: Thread) -> Result<()> {
        if so.so_type() != SOCK_STREAM {
            return Err(ESOCKTNOSUPPORT);
        }
        if proto != i32::from(VSOCK_SOCK_PROTO_TRANS) {
            return Err(EPROTONOSUPPORT);
        }
        if so.so_pcb().is_some() {
            return Err(EISCONN);
        }
        let pcb: Arc<_, M_DEVBUF> = Arc::try_new(VsockPcb {
            local_addr: OnceInit::uninit(),
            remote_addr: OnceInit::uninit(),
        }, M_WAITOK)?;
        so.so_pcb_set(pcb)?;
        Ok(())
    }

    fn pr_bind(so: Socket<VsockPcb>, addr: Option<&SockAddr>, td: Thread) -> Result<()> {
        let pcb = so.so_pcb().ok_or(EINVAL)?;
        let sa = addr.ok_or(EINVAL)?;
        let vsa = VsockAddr::from_ref(sa);
        if i32::from(sa.sa_family) != AF_VSOCK {
            return Err(EAFNOSUPPORT);
        }
        //if sa.sa_len != size_of::<X>() {
        //    return Err(EINVAL);
        //}
        let mut sockets = mtx_lock(&SOCKETS);

        for bound_sa in sockets.bound.iter() {
            if bound_sa.port == vsa.port && bound_sa.cid == vsa.cid {
                mtx_unlock(sockets);
                return Err(EADDRINUSE);
            }
        }

        for connected_sa in sockets.connected.iter() {
            if connected_sa.port == vsa.port && connected_sa.cid == vsa.cid {
                mtx_unlock(sockets);
                return Err(EADDRINUSE);
            }
        }

        pcb.local_addr.init(vsa);
        sockets.bound.push_back(vsa, M_WAITOK);

        mtx_unlock(sockets);

        Ok(())
    }

    fn pr_listen(so: Socket<VsockPcb>, backlog: i32, td: Thread) -> Result<()> {
        let mut sockets = mtx_lock(&SOCKETS);

        /* Check if the address is already bound and it was by us. */
        //for bound_sa in &sockets.bound {
        //}
        let pcb = so.so_pcb().ok_or(EINVAL)?;
        if !pcb.local_addr.is_init() {
            return Err(EADDRNOTAVAIL);
        }
        let sock = SOCK_LOCK(&so);
        solisten_proto_check(&sock)?;
        solisten_proto(&sock, backlog);
        SOCK_UNLOCK(sock);

        mtx_unlock(sockets);

        Ok(())
    }

    fn pr_accept(so: Socket<VsockPcb>, addr: Option<&SockAddr>) -> Result<()> {
        let pcb = so.so_pcb().ok_or(EINVAL)?;
        let sa = addr.ok_or(EINVAL)?;
        let vsa = VsockAddr::from_ref(sa);
        pcb.remote_addr.init(vsa);
        Ok(())
    }

    fn pr_connect(so: Socket<VsockPcb>, addr: Option<&SockAddr>, td: Thread) -> Result<()> {
        todo!("")
    }
}

define_protosw! {
    static vsock_protosw: VsockSw = {
        pr_type: SOCK_STREAM as i16,
        pr_protocol: VSOCK_SOCK_PROTO_TRANS,
        pr_flags: PR_CONNREQUIRED as i16,

        pr_attach: vsock_attach,
        pr_bind: vsock_bind,
        pr_listen: vsock_listen,
        pr_accept: vsock_accept,
        pr_connect: vsock_connect,
        //pr_peeraddr: vsock_peeraddr,
        //pr_sockaddr: vsock_sockaddr,
        //pr_soreceive: vsock_soreceive,
        //pr_sosend: vsock_sosend,
        //pr_disconnect: vsock_disconnect,
        //pr_close: vsock_close,
        //pr_detach: vsock_detach,
        //pr_shutdown: vsock_shutdown,
        //pr_abort: vsock_abort,
    }
}
