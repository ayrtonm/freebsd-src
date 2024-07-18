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

#include <sys/param.h>
#include <sys/systm.h>
//#include <sys/device.h>
#include <sys/malloc.h>
#include <sys/taskqueue.h>

#include <machine/_inttypes.h>
#include <machine/bus.h>
//#include <machine/fdt.h>

//#include <dev/ofw/openfirm.h>
//#include <dev/ofw/fdt.h>

#include <arm64/apple/apple_mboxvar.h>
#include <arm64/apple/rtkit.h>

#define RTKIT_EP_MGMT			0
#define RTKIT_EP_CRASHLOG		1
#define RTKIT_EP_SYSLOG			2
#define RTKIT_EP_DEBUG			3
#define RTKIT_EP_IOREPORT		4
#define RTKIT_EP_OSLOG		8
#define RTKIT_EP_TRACEKIT	10

#define RTKIT_MGMT_TYPE(x)		(((x) >> 52) & 0xff)
#define RTKIT_MGMT_TYPE_SHIFT		52

#define RTKIT_MGMT_PWR_STATE(x)		(((x) >> 0) & 0xffff)

#define RTKIT_MGMT_HELLO		1
#define RTKIT_MGMT_HELLO_ACK		2
#define RTKIT_MGMT_STARTEP		5
#define RTKIT_MGMT_IOP_PWR_STATE	6
#define RTKIT_MGMT_IOP_PWR_STATE_ACK	7
#define RTKIT_MGMT_EPMAP		8
#define RTKIT_MGMT_AP_PWR_STATE		11

#define RTKIT_MGMT_HELLO_MINVER(x)	(((x) >> 0) & 0xffff)
#define RTKIT_MGMT_HELLO_MINVER_SHIFT	0
#define RTKIT_MGMT_HELLO_MAXVER(x)	(((x) >> 16) & 0xffff)
#define RTKIT_MGMT_HELLO_MAXVER_SHIFT	16

#define RTKIT_MGMT_STARTEP_EP_SHIFT	32
#define RTKIT_MGMT_STARTEP_START	(1ULL << 1)

#define RTKIT_MGMT_EPMAP_LAST		(1ULL << 51)
#define RTKIT_MGMT_EPMAP_BASE(x)	(((x) >> 32) & 0x7)
#define RTKIT_MGMT_EPMAP_BASE_SHIFT	32
#define RTKIT_MGMT_EPMAP_BITMAP(x)	(((x) >> 0) & 0xffffffff)
#define RTKIT_MGMT_EPMAP_MORE		(1ULL << 0)

#define RTKIT_BUFFER_REQUEST		1
#define RTKIT_BUFFER_ADDR(x)		(((x) >> 0) & 0xfffffffffff)
#define RTKIT_BUFFER_SIZE(x)		(((x) >> 44) & 0xff)
#define RTKIT_BUFFER_SIZE_SHIFT		44

#define RTKIT_SYSLOG_LOG		5
#define RTKIT_SYSLOG_INIT		8

#define RTKIT_IOREPORT_UNKNOWN1		8
#define RTKIT_IOREPORT_UNKNOWN2		12

#define RTKIT_OSLOG_TYPE(x)		(((x) >> 56) & 0xff)
#define RTKIT_OSLOG_TYPE_SHIFT		(56 - RTKIT_MGMT_TYPE_SHIFT)
#define RTKIT_OSLOG_BUFFER_REQUEST	1
#define RTKIT_OSLOG_BUFFER_ADDR(x)	(((x) >> 0) & 0xfffffffff)
#define RTKIT_OSLOG_BUFFER_SIZE(x)	(((x) >> 36) & 0xfffff)

/* Versions we support. */
#define RTKIT_MINVER			11
#define RTKIT_MAXVER			12

#define LOCAL_TRACE 1
#include <sys/ltrace.h>

struct rtkit_state {
	mbox_t			mc;
	struct rtkit	*rk;
	char			*crashlog;
	bus_addr_t		crashlog_addr;
	bus_size_t		crashlog_size;
	struct task		crashlog_task;

	struct task ioreport_task;
	struct task oslog_task;
	struct task syslog_task;

	uint16_t		iop_pwrstate;
	uint16_t		ap_pwrstate;
	uint64_t		epmap;
	void			(*callback[32])(void *, uint64_t);
	void			*arg[32];

	//bus_dma_tag_t	dmat;
};

static int
rtkit_recv(mbox_t mc, struct apple_mbox_msg *msg)
{
	return mbox_read(mc, msg, sizeof(*msg));
}

static int
rtkit_send(mbox_t mc, uint32_t endpoint,
    uint64_t type, uint64_t data)
{
	struct apple_mbox_msg msg;
	int rc;

	ltracef("sending msg of type %" PRIx64 " to endpoint %" PRIx32, type, endpoint);
	msg.data0 = (type << RTKIT_MGMT_TYPE_SHIFT) | data;
	msg.data1 = endpoint;
	rc = mbox_write(mc, &msg, sizeof(msg));

	if (rc)
		ltracef("mbox_write failed (%d)", rc);

	return rc;
}

static void
rtkit_dma_cb(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	ltracef("DMA callback found error %d", error);
	return;
}

static bus_addr_t
rtkit_alloc(struct rtkit_state *state, bus_size_t size, caddr_t *kvap)
{
	struct rtkit *rk = state->rk;
	bus_dmamap_t map;
	void *vaddr;
	int rc;

	ltracef("requesting DMA buffer size: %" PRIx64 ", tag max size: %" PRIx64, size, rk->rk_dma_maxsize);

	// I probably should just create the dma tag here tbh
	if (size > rk->rk_dma_maxsize)
		panic("unable to allocate enough memory for DMA\n");

	// may allocate more memory than requested since it uses the DMA tag maxsize
	rc = bus_dmamem_alloc(rk->rk_dmat, &vaddr, BUS_DMA_WAITOK | BUS_DMA_ZERO, &map);
	if (rc) {
		ltracef("dmamem_alloc failed %d", rc);
		goto err_mem_alloc;
	}

	// loads less memory than allocated since it uses the DMA request size
	rc = bus_dmamap_load(rk->rk_dmat, map, vaddr, size, rtkit_dma_cb, NULL /* cb arg */, 0);
	ltracef("bus_dmamap_load returned %d (should be 0 or %d)", rc, EINPROGRESS);
	if ((rc != 0) || (rc != EINPROGRESS)) {
		ltracef("dmamap_load failed %d", rc);
		goto err_map_load;
	}

	if (rk->rk_map) {
		rc = rk->rk_map(rk->rk_cookie, (bus_addr_t)vaddr, size);
		if (rc) {
			ltracef("rkit_alloc callback failed %d", rc);
			goto err_client_map;
		}
	}

	return (bus_addr_t)vaddr;

err_client_map:
	// necessary in error path?
	//bus_dmamap_sync(rk->rk_dmat, map, BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);
	bus_dmamap_unload(rk->rk_dmat, map);
err_map_load:
	bus_dmamem_free(rk->rk_dmat, vaddr, map);
err_mem_alloc:
	return (bus_addr_t)-1;
}

static int
rtkit_start(struct rtkit_state *state, uint32_t endpoint)
{
	mbox_t mc = state->mc;
	uint64_t reply;

	reply = ((uint64_t)endpoint << RTKIT_MGMT_STARTEP_EP_SHIFT);
	reply |= RTKIT_MGMT_STARTEP_START;
	return rtkit_send(mc, RTKIT_EP_MGMT, RTKIT_MGMT_STARTEP, reply);
}

static int
rtkit_handle_mgmt(struct rtkit_state *state, struct apple_mbox_msg *msg)
{
	mbox_t mc = state->mc;
	uint64_t minver, maxver, ver;
	uint64_t base, bitmap, reply;
	uint32_t endpoint;
	int error;

	switch (RTKIT_MGMT_TYPE(msg->data0)) {
	case RTKIT_MGMT_HELLO:
		minver = RTKIT_MGMT_HELLO_MINVER(msg->data0);
		maxver = RTKIT_MGMT_HELLO_MAXVER(msg->data0);
		if (minver > RTKIT_MAXVER) {
			printf("%s: unsupported minimum firmware version %lu\n",
			    __func__, minver);
			return EINVAL;
		}
		if (maxver < RTKIT_MINVER) {
			printf("%s: unsupported maximum firmware version %lu\n",
			    __func__, maxver);
			return EINVAL;
		}
		ver = min(RTKIT_MAXVER, maxver);
		error = rtkit_send(mc, RTKIT_EP_MGMT, RTKIT_MGMT_HELLO_ACK,
		    (ver << RTKIT_MGMT_HELLO_MINVER_SHIFT) |
		    (ver << RTKIT_MGMT_HELLO_MAXVER_SHIFT));
		if (error)
			return error;
		break;
	case RTKIT_MGMT_IOP_PWR_STATE_ACK:
		state->iop_pwrstate = RTKIT_MGMT_PWR_STATE(msg->data0);
		wakeup(&state->iop_pwrstate);
		break;
	case RTKIT_MGMT_AP_PWR_STATE:
		state->ap_pwrstate = RTKIT_MGMT_PWR_STATE(msg->data0);
		wakeup(&state->ap_pwrstate);
		break;
	case RTKIT_MGMT_EPMAP:
		base = RTKIT_MGMT_EPMAP_BASE(msg->data0);
		bitmap = RTKIT_MGMT_EPMAP_BITMAP(msg->data0);
		state->epmap |= (bitmap << (base * 32));
		reply = (base << RTKIT_MGMT_EPMAP_BASE_SHIFT);
		if (msg->data0 & RTKIT_MGMT_EPMAP_LAST)
			reply |= RTKIT_MGMT_EPMAP_LAST;
		else
			reply |= RTKIT_MGMT_EPMAP_MORE;
		error = rtkit_send(state->mc, RTKIT_EP_MGMT,
		    RTKIT_MGMT_EPMAP, reply);
		if (error)
			return error;
		if (msg->data0 & RTKIT_MGMT_EPMAP_LAST) {
			for (endpoint = 1; endpoint < 32; endpoint++) {
				if ((state->epmap & (1ULL << endpoint)) == 0)
					continue;

				switch (endpoint) {
				case RTKIT_EP_MGMT:
					/* nothing to do here */
					break;
				case RTKIT_EP_CRASHLOG:
				case RTKIT_EP_SYSLOG:
				case RTKIT_EP_DEBUG:
				case RTKIT_EP_IOREPORT:
				case RTKIT_EP_OSLOG:
				case RTKIT_EP_TRACEKIT:
					printf("%s: starting rtkit endpoint %d\n", __func__, endpoint);
					error = rtkit_start(state, endpoint);
					if (error)
						return error;
					break;
				default:
					printf("%s: skipping endpoint %d\n",
					    __func__, endpoint);
					break;
				}
			}
		}
		break;
	default:
		printf("%s: unhandled management event 0x%016lu\n",
		    __func__, msg->data0);
		return EIO;
	}

	return 0;
}

static void
rtkit_handle_crashlog_buffer(void *context, int pending)
{
	struct rtkit_state *state = context;
	struct rtkit *rk = state->rk;
	bus_addr_t addr = state->crashlog_addr;
	bus_size_t size = state->crashlog_size;

#if 0
	if (addr) {
		paddr_t pa = addr;
		vaddr_t va;

		// nvme ans doesn't use rk_logmap only pci/drm/apldcp.c does
		if (rk && rk->rk_logmap) {
		}
		state->crashlog = km_alloc(size * PAGE_SIZE, &kv_any, &kp_none, &kd_waitok);
		va = (vaddr_t)state->crashlog;
	}
#endif

	if (rk) {
		addr = rtkit_alloc(state, size << PAGE_SHIFT,
			&state->crashlog);
		if (addr == (bus_addr_t)-1)
			return;
	}

	rtkit_send(state->mc, RTKIT_EP_CRASHLOG, RTKIT_BUFFER_REQUEST,
		(size << RTKIT_BUFFER_SIZE_SHIFT) | addr);
}

static int
rtkit_handle_crashlog(struct rtkit_state *state, struct apple_mbox_msg *msg)
{
	//mbox_t mc = state->mc;
	//struct rtkit *rk = state->rk;
	bus_addr_t addr;
	bus_size_t size;
	//int error;

	switch (RTKIT_MGMT_TYPE(msg->data0)) {
	case RTKIT_BUFFER_REQUEST:
		addr = RTKIT_BUFFER_ADDR(msg->data0);
		size = RTKIT_BUFFER_SIZE(msg->data0);

		if (state->crashlog) {
			char *buf;
			ltracef("RTKIT crashed");

			buf = malloc(size * PAGE_SIZE, M_TEMP, M_NOWAIT);
			if (buf) {
				memcpy(buf, state->crashlog, size * PAGE_SIZE);
				//rtkit_crashlog_dump(buf, size * PAGE_SIZE);
			}
			break;
		}

		state->crashlog_addr = addr;
		state->crashlog_size = size;
		if (cold)
			rtkit_handle_crashlog_buffer(state, 0);
		else
			taskqueue_enqueue(taskqueue_thread, &state->crashlog_task);
		break;
	default:
		printf("unhandled crashlog event 0x%016lx", msg->data0);
		//return EIO;
		break;
	}

	return 0;
}

static int
rtkit_handle_syslog(struct rtkit_state *state, struct apple_mbox_msg *msg)
{
#if 1
printf("rtkit_handle_crashlog\n");
#else
	mbox_t mc = state->mc;
	struct rtkit *rk = state->rk;
	bus_addr_t addr;
	bus_size_t size;
	int error;

	switch (RTKIT_MGMT_TYPE(msg->data0)) {
	case RTKIT_BUFFER_REQUEST:
		addr = RTKIT_BUFFER_ADDR(msg->data0);
		size = RTKIT_BUFFER_SIZE(msg->data0);
		if (addr)
			break;

		if (rk) {
			addr = rtkit_alloc(rk, size << PAGE_SHIFT);
			if (addr == (bus_addr_t)-1)
				return ENOMEM;
			if (rk->rk_map) {
				error = rk->rk_map(rk->rk_cookie, addr,
				    size << PAGE_SHIFT);
				if (error)
					return error;
			}
		}

		error = rtkit_send(mc, RTKIT_EP_SYSLOG, RTKIT_BUFFER_REQUEST,
		    (size << RTKIT_BUFFER_SIZE_SHIFT) | addr);
		if (error)
			return error;
		break;
	case RTKIT_SYSLOG_INIT:
		break;
	case RTKIT_SYSLOG_LOG:
		error = rtkit_send(mc, RTKIT_EP_SYSLOG,
		    RTKIT_MGMT_TYPE(msg->data0), msg->data0);
		if (error)
			return error;
		break;
	default:
		printf("%s: unhandled syslog event 0x%016llx\n",
		    __func__, msg->data0);
		return EIO;
	}
#endif

	return 0;
}

static int
rtkit_handle_ioreport(struct rtkit_state *state, struct apple_mbox_msg *msg)
{
#if 1
printf("rtkit_handle_crashlog\n");
#else
	mbox_t mc = state->mc;
	struct rtkit *rk = state->rk;
	bus_addr_t addr;
	bus_size_t size;
	int error;

	switch (RTKIT_MGMT_TYPE(msg->data0)) {
	case RTKIT_BUFFER_REQUEST:
		addr = RTKIT_BUFFER_ADDR(msg->data0);
		size = RTKIT_BUFFER_SIZE(msg->data0);
		if (addr)
			break;

		if (rk) {
			addr = rtkit_alloc(rk, size << PAGE_SHIFT);
			if (addr == (bus_addr_t)-1)
				return ENOMEM;
			if (rk->rk_map) {
				error = rk->rk_map(rk->rk_cookie, addr,
				    size << PAGE_SHIFT);
				if (error)
					return error;
			}
		}

		error = rtkit_send(mc, RTKIT_EP_IOREPORT, RTKIT_BUFFER_REQUEST,
		    (size << RTKIT_BUFFER_SIZE_SHIFT) | addr);
		if (error)
			return error;
		break;
	case RTKIT_IOREPORT_UNKNOWN1:
	case RTKIT_IOREPORT_UNKNOWN2:
		/* These unknown events have to be acked to make progress. */
		error = rtkit_send(mc, RTKIT_EP_IOREPORT,
		    RTKIT_MGMT_TYPE(msg->data0), msg->data0);
		if (error)
			return error;
		break;
	default:
		printf("%s: unhandled ioreport event 0x%016llx\n",
		    __func__, msg->data0);
		return EIO;
	}
#endif

	return 0;
}

static int
rtkit_handle_oslog(struct rtkit_state *state, struct apple_mbox_msg *msg)
{
printf("rtkit_handle_oslog\n");
#if 0
	bus_addr_t addr;
	bus_size_t size;

	switch (RTKIT_OSLOG_TYPE(msg->data0)) {
	case RTKIT_OSLOG_BUFFER_REQUEST:
		addr = RTKIT_OSLOG_BUFFER_ADDR(msg->data0) << PAGE_SHIFT;
		size = RTKIT_OSLOG_BUFFER_SIZE(msg->data0);
		if (addr)
			break;

		state->oslog_addr = addr;
		state->oslog_size = size;
		if (cold)
			rtkit_handle_oslog_buffer(state);
		else
			task_add(systq, &state->oslog_task);
		break;
	case RTKIT_OSLOG_UNKNOWN1:
	case RTKIT_OSLOG_UNKNOWN2:
	case RTKIT_OSLOG_UNKNOWN3:
		break;
	default:
		printf("%s: unhandled oslog event 0x%lx\n", __func__, msg->data0);
		break;
	}

#endif	
	return 0;
}

static int
rtkit_poll(struct rtkit_state *state)
{
	mbox_t mc = state->mc;
	struct apple_mbox_msg msg;
	void (*callback)(void *, uint64_t);
	void *arg;
	uint32_t endpoint;
	int error;

	error = rtkit_recv(mc, &msg);
	if (error) {
		// this is noisy
		//ltracef("rtkit_recv failed (%d)", error);
		return error;
	}

	endpoint = msg.data1;
	ltracef("read message from endpoint %" PRIx32, endpoint);

	switch (endpoint) {
	case RTKIT_EP_MGMT:
		error = rtkit_handle_mgmt(state, &msg);
		if (error)
			return error;
		break;
	case RTKIT_EP_CRASHLOG:
		error = rtkit_handle_crashlog(state, &msg);
		if (error)
			return error;
		break;
	case RTKIT_EP_SYSLOG:
		error = rtkit_handle_syslog(state, &msg);
		if (error)
			return error;
		break;
	case RTKIT_EP_IOREPORT:
		error = rtkit_handle_ioreport(state, &msg);
		if (error)
			return error;
		break;
	case RTKIT_EP_OSLOG:
		error = rtkit_handle_oslog(state, &msg);
		if (error)
			return error;
		break;
	//case RTKIT_EP_TRACEKIT:
	//	break;
	default:
		if (endpoint >= 32 && endpoint < 64 && 
		    state->callback[endpoint - 32]) {
			callback = state->callback[endpoint - 32];
			arg = state->arg[endpoint - 32];
			callback(arg, msg.data0);
			break;
		}

		printf("%s: unhandled endpoint %d\n", __func__, msg.data1);
		return EIO;
	}

	return 0;
}

static void
rtkit_rx_callback(void *cookie, int channel)
{
	ltracef("rtkit callback invoked");
	rtkit_poll(cookie /*, channel is hardcoded to -1 in apple_mbox.c */);
}

struct rtkit_state *
rtkit_init(int node, const char *name, struct rtkit *rk, mbox_t mc)
{
	struct rtkit_state *state;
	//struct mbox_client client;
	int ret;

	state = malloc(sizeof(*state), M_DEVBUF, M_WAITOK | M_ZERO);
	//client.mc_rx_callback = rtkit_rx_callback;
	//client.mc_rx_arg = state;
	state->mc = mc;
	state->rk = rk;
	/* XXX setup_channel; what should third parameter be? */
	if ((ret = mbox_setup_channel(mc, &rtkit_rx_callback, state)) != 0) {
		printf("mbox_setuep_channel ret %d\n", ret);
		free(state, M_DEVBUF);
		return NULL;
	}

	state->iop_pwrstate = RTKIT_MGMT_PWR_STATE_SLEEP;
	state->ap_pwrstate = RTKIT_MGMT_PWR_STATE_QUIESCED;

	TASK_INIT(&state->crashlog_task, 0, rtkit_handle_crashlog_buffer, state);
	//TASK_INIT(&state->syslog_task, 0, rtkit_handle_syslog_buffer, state);
	//TASK_INIT(&state->ioreport_task, 0, rtkit_handle_ioreport_buffer, state);
	//TASK_INIT(&state->oslog_task, 0, rtkit_handle_oslog_buffer, state);

	return state;
}

int
rtkit_boot(struct rtkit_state *state)
{
	/* Wake up! */
	ltracef("setting IOP power state ON");
	return rtkit_set_iop_pwrstate(state, RTKIT_MGMT_PWR_STATE_ON);
}

int
rtkit_set_ap_pwrstate(struct rtkit_state *state, uint16_t pwrstate)
{
	mbox_t mc = state->mc;
	int error, timo;

	if (state->ap_pwrstate == pwrstate)
		return 0;

	error = rtkit_send(mc, RTKIT_EP_MGMT, RTKIT_MGMT_AP_PWR_STATE,
	    pwrstate);
	if (error)
		return error;

	if (cold) {
		for (timo = 0; timo < 100000; timo++) {
			error = rtkit_poll(state);
			if (error == EWOULDBLOCK) {
				DELAY(10);
				continue;
			}
			if (error)
				return error;

			if (state->ap_pwrstate == pwrstate)
				return 0;
		}
	}

	while (state->ap_pwrstate != pwrstate) {
		//error = tsleep_sbt(&state->ap_pwrstate, PWAIT, "appwr",
		//	nstosbt(1));
		if (error)
			return error;
	}
	return 0;
}

int
rtkit_set_iop_pwrstate(struct rtkit_state *state, uint16_t pwrstate)
{
	mbox_t mc = state->mc;
	int error, timo;

	ltracef("changing power state: %d -> %d", state->iop_pwrstate, pwrstate);
	/* nothing to do in this case */
	if (state->iop_pwrstate == (pwrstate & 0xff))
		return 0;

	error = rtkit_send(mc, RTKIT_EP_MGMT, RTKIT_MGMT_IOP_PWR_STATE,
		pwrstate);

	ltracef("rtkit_send returned %d", error);

	if (error)
		return error;

	if (cold) {
		ltracef("setting IOP on cold boot");
		for (timo = 0; timo < 100000; timo++) {
			error = rtkit_poll(state);
			if (error == EWOULDBLOCK) {
				DELAY(10);
				continue;
			}
			if (error)
				return error;

			if (state->iop_pwrstate == (pwrstate & 0xff))
				return 0;
		}
	}
	ltracef("finished cold polling thing");

	while (state->iop_pwrstate != (pwrstate & 0xff)) {
		ltracef("got here");
		//error = tsleep_sbt(&state->iop_pwrstate, PWAIT, "ioppwr",
		//	nstosbt(1));
		if (error)
			return error;
	}
	return 0;
}

int
rtkit_start_endpoint(struct rtkit_state *state, uint32_t endpoint,
    void (*callback)(void *, uint64_t), void *arg)
{
	if (endpoint < 32 || endpoint >= 64)
		return EINVAL;

	if ((state->epmap & (1ULL << endpoint)) == 0)
		return EINVAL;

	state->callback[endpoint - 32] = callback;
	state->arg[endpoint - 32] = arg;
	return rtkit_start(state, endpoint);
}

int
rtkit_send_endpoint(struct rtkit_state *state, uint32_t endpoint,
    uint64_t data)
{
	return rtkit_send(state->mc, endpoint, 0, data);
}
