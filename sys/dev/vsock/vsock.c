#include <sys/param.h>
#include <sys/bus.h>
#include <sys/domain.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/protosw.h>
#include <sys/socket.h>
#include <sys/sysctl.h>
#include <sys/systm.h>

extern struct protosw vsock_protosw;

SYSCTL_NODE(_net, OID_AUTO, vsock, CTLFLAG_RD, 0, "VirtIO socket");

//int vsock_dbg_level;
//SYSCTL_INT(_net_vsock, OID_AUTO, vsock_dbg_level, CTLFLAG_RWTUN, &vsock_dbg_level),
//	0, "virtio socket debug level: 0 = none, 1 = info, 2 = error, 3 = trace");

static int
vsock_dom_probe(void)
{
//	if (vm_guest != VM_GUEST_KVM)
//		return (ENXIO);
	return (0);
}

static struct domain vsock_domain = {
	.dom_family = AF_VSOCK,
	.dom_name = "vsock",
	.dom_probe = vsock_dom_probe,
	.dom_nprotosw = 1,
	.dom_protosw = { &vsock_protosw },
};

DOMAIN_SET(vsock_);

void vsock_init(void *arg);

SYSINIT(vsock_init, SI_SUB_PROTO_DOMAIN, SI_ORDER_THIRD,
    vsock_init, NULL);

MODULE_VERSION(vsock, 1);
