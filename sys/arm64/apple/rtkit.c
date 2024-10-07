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
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/taskqueue.h>

#include <machine/_inttypes.h>
#include <machine/bus.h>

#include <dev/ofw/openfirm.h>

#include <arm64/apple/apple_mbox.h>
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
#define RTKIT_BUFFER_ADDR(x)		((x) & ((1ULL << 44) - 1))
#define RTKIT_BUFFER_SIZE(x)		(((x) >> 44) & 0xff)
#define RTKIT_BUFFER_SIZE_SHIFT		44

#define RTKIT_SYSLOG_LOG		5
#define RTKIT_SYSLOG_LOG_ENTRY(x)	((x) & 0xff)
#define RTKIT_SYSLOG_INIT		8
#define RTKIT_SYSLOG_INIT_ENTRIES(x)	((x) & 0xff)
#define RTKIT_SYSLOG_INIT_MSG_SIZE(x)	(((x) >> 24) & 0xff)

#define RTKIT_IOREPORT_UNKNOWN1		8
#define RTKIT_IOREPORT_UNKNOWN2		12

#define RTKIT_OSLOG_TYPE(x)		(((x) >> 56) & 0xff)
#define RTKIT_OSLOG_TYPE_SHIFT		56

#define RTKIT_OSLOG_BUFFER_REQUEST	1
#define RTKIT_OSLOG_BUFFER_ADDR(x)	((x) & ((1ULL << 36) - 1))
#define RTKIT_OSLOG_BUFFER_SIZE(x)	(((x) >> 36) & 0xfffff)
#define RTKIT_OSLOG_BUFFER_SIZE_SHIFT	36
#define	RTKIT_OSLOG_UNKNOWN1		3
#define	RTKIT_OSLOG_UNKNOWN2		4
#define	RTKIT_OSLOG_UNKNOWN3		5

/* Versions we support. */
#define RTKIT_MINVER			11
#define RTKIT_MAXVER			12

static bool rtkit_verbose = true;

#define	rtkit_printf(state, fmt, ...) \
    do { if (rtkit_verbose && state->verbose) { \
	device_printf(state->dev, fmt, ##__VA_ARGS__); } } while (0)

struct rtkit_buffer {
	bus_addr_t			addr;
	bus_size_t			size;
	void				*kva;

	bus_dma_tag_t		tag;
	bus_dmamap_t		map;

	// TODO: this is a pretty ugly workaround for making handle_buffer_req
	// endpoint oblivious. I should probably find a better way to dedup code
	struct rtkit_state	*state;
};

struct rtkit_task {
	struct task				task;
	struct apple_mbox_msg	msg;
	struct rtkit_state		*state;
};

struct rtkit_state {
	device_t			dev;
	struct apple_mbox	mbox;

	uint16_t			iop_pwrstate;
	uint16_t			ap_pwrstate;

	uint64_t			epmap;
	void (*callbacks[32])(void *, uint64_t);
	void				*args[32];

	struct rtkit_buffer crashlog_buffer;
	struct rtkit_buffer syslog_buffer;
	struct rtkit_buffer ioreport_buffer;
	struct rtkit_buffer oslog_buffer;

	rtkit_map			map_fn;
	void				*map_arg;

	uint8_t				syslog_entries;
	uint8_t				syslog_msg_size;
	char				*syslog_msg;

	bool				verbose;
	bool				noalloc;
};

static bool
rtkit_endpoint_is_valid(uint32_t endpoint)
{
	switch (endpoint) {
	case RTKIT_EP_MGMT:
	case RTKIT_EP_CRASHLOG:
	case RTKIT_EP_SYSLOG:
	case RTKIT_EP_DEBUG:
	case RTKIT_EP_IOREPORT:
	case RTKIT_EP_OSLOG:
	case RTKIT_EP_TRACEKIT:
		return true;
	}
	return (endpoint >= 32) && (endpoint < 64);
}

static int
rtkit_send(struct apple_mbox *mbox, uint32_t endpoint, uint8_t type,
	uint64_t data)
{
	struct apple_mbox_msg msg;

	MPASS(rtkit_endpoint_is_valid(endpoint));

	u_int shift = RTKIT_MGMT_TYPE_SHIFT;
	if (endpoint == RTKIT_EP_OSLOG) {
		shift = RTKIT_OSLOG_TYPE_SHIFT;
	}
	// Make sure message data does not overlap with message type
	//MPASS((data >> shift) == 0);

	msg.data0 = ((uint64_t)type) << shift;
	msg.data0 |= data;
	msg.data1 = endpoint;

	return apple_mbox_write(mbox->dev, &msg);
}

int
rtkit_set_ap_pwrstate(struct rtkit_state *state, uint16_t pwrstate)
{
	int error;
	device_t dev = state->dev;
	struct apple_mbox *mbox = &state->mbox;

	MPASS(!cold);

	if (state->ap_pwrstate == (pwrstate & 0xff)) {
		device_printf(dev, "RTKit AP already in requested power state %d\n",
			pwrstate);
		return 0;
	}

	error = rtkit_send(mbox, RTKIT_EP_MGMT, RTKIT_MGMT_AP_PWR_STATE,
		pwrstate);

	if (error) {
		device_printf(dev,
			"error (%d) sending AP_PWR_STATE message to MGMT endpoint\n",
			error);
		return error;
	}

	if (state->ap_pwrstate != (pwrstate & 0xff)) {
		error = tsleep(&state->ap_pwrstate, PWAIT, "appwr", hz);
		if (error != 0) {
			device_printf(dev,
				"timed out (%d) waiting for AP power state change\n", error);
		}
	}
	return error;
}

static int
rtkit_set_iop_pwrstate(struct rtkit_state *state, uint16_t pwrstate)
{
	int error;
	device_t dev = state->dev;
	struct apple_mbox *mbox = &state->mbox;

	MPASS(!cold);

	if (state->iop_pwrstate == (pwrstate & 0xff)) {
		device_printf(dev, "RTKit IOP already in requested power state %d\n",
			pwrstate);
		return 0;
	}

	error = rtkit_send(mbox, RTKIT_EP_MGMT, RTKIT_MGMT_IOP_PWR_STATE,
		pwrstate);

	if (error) {
		device_printf(dev,
			"error (%d) sending IOP_PWR_STATE message to MGMT endpoint\n",
			error);
		return error;
	}

	rtkit_printf(state, "waiting for IOP power state change\n");
	if (state->iop_pwrstate != (pwrstate & 0xff)) {
		error = tsleep(&state->iop_pwrstate, PWAIT, "ioppwr", hz);
		if (error != 0) {
			device_printf(dev,
				"timed out (%d) waiting for IOP power state change\n", error);
		}
	}
	rtkit_printf(state, "IOP power state changed to %d\n", pwrstate);
	return error;
}

static void
rtkit_dmamap_cb(void *arg, bus_dma_segment_t *segs, int nsegs, int error)
{
	struct rtkit_buffer *buffer = arg;
	struct rtkit_state *state = buffer->state;
	device_t dev = state->dev;
	int rc;

	rtkit_printf(state, "dma map callback reported %d segments\n", nsegs);
	MPASS(error == 0);
	MPASS(nsegs == 1);

	buffer->addr = segs->ds_addr;
	buffer->size = segs->ds_len;

	if (state->map_fn) {
		// might need to sleep to ensure callback in previous function was
		// called. For now just assert it to avoid using buffer->addr
		// uninitialized
		MPASS(buffer->addr != 0);
		rc = state->map_fn(state->map_arg, buffer->addr, buffer->size);
		if (rc != 0) {
			device_printf(dev, "dma map callback failed %d\n", rc);
		}
	}
}

static int
rtkit_alloc(struct rtkit_state *state, bus_size_t req_size,
	struct rtkit_buffer *buffer)
{
	if (state->noalloc) {
		return 0;
	}
	device_t dev = state->dev;
	int rc;

	rc = bus_dma_tag_create(bus_get_dma_tag(dev),
		PAGE_SIZE, /* 16K alignment */
		0, /* bounds */
		BUS_SPACE_MAXADDR, BUS_SPACE_MAXADDR, /* low/high addr */
		NULL, NULL, /* filter depreacated and not supported anyway */ 
		req_size, /* maxsize */
		1, /* nsegments */
		req_size, /* maxsegsize */
		BUS_DMA_COHERENT,
		NULL, NULL, /* lockfunc */
		&buffer->tag);
	if (rc != 0) {
		device_printf(dev, "bus_dma_tag_create failed %d\n", rc);
		return rc;
	}

	rc = bus_dmamem_alloc(buffer->tag, &buffer->kva,
		BUS_DMA_WAITOK | BUS_DMA_ZERO, &buffer->map);
	if (rc != 0) {
		device_printf(dev, "bus_dmamem_alloc failed %d\n", rc);
		return rc;
	}

	// these should be initialized by the callback in the next function
	buffer->size = req_size;
	buffer->addr = 0;
	buffer->state = state;

	rc = bus_dmamap_load(buffer->tag, buffer->map, buffer->kva, req_size,
		rtkit_dmamap_cb, buffer, 0);
	if ((rc != 0) && (rc != EINPROGRESS)) {
		device_printf(dev, "bus_dmamap_load failed %d\n", rc);
		return rc;
	}

	return 0;
}

static int
rtkit_start(struct rtkit_state *state, uint32_t endpoint)
{
	struct apple_mbox *mbox = &state->mbox;
	uint64_t reply;

	reply = ((uint64_t)endpoint << RTKIT_MGMT_STARTEP_EP_SHIFT);
	reply |= RTKIT_MGMT_STARTEP_START;
	return rtkit_send(mbox, RTKIT_EP_MGMT, RTKIT_MGMT_STARTEP, reply);
}

static int
rtkit_handle_mgmt(struct rtkit_state *state, struct apple_mbox_msg *msg)
{
	device_t dev = state->dev;
	struct apple_mbox *mbox = &state->mbox;
	uint64_t minver, maxver, ver;
	uint64_t base, bitmap, reply;
	uint32_t endpoint;
	int error;

	switch (RTKIT_MGMT_TYPE(msg->data0)) {
	case RTKIT_MGMT_HELLO:
		minver = RTKIT_MGMT_HELLO_MINVER(msg->data0);
		maxver = RTKIT_MGMT_HELLO_MAXVER(msg->data0);
		if (minver > RTKIT_MAXVER) {
			device_printf(dev,
				"%s: unsupported minimum firmware version %lu\n", __func__,
				minver);
			return EINVAL;
		}
		if (maxver < RTKIT_MINVER) {
			device_printf(dev,
				"%s: unsupported maximum firmware version %lu\n", __func__,
				maxver);
			return EINVAL;
		}
		ver = min(RTKIT_MAXVER, maxver);
		error = rtkit_send(mbox, RTKIT_EP_MGMT, RTKIT_MGMT_HELLO_ACK,
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
		error = rtkit_send(mbox, RTKIT_EP_MGMT,
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
					rtkit_printf(state, "starting rtkit endpoint %d\n", endpoint);
					error = rtkit_start(state, endpoint);
					if (error) {
						device_printf(dev, "failed to start rtkit endpoint %d\n", endpoint);
						return error;
					}
					break;
				default:
					device_printf(dev, "%s: skipping endpoint %d\n",
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

static int
rtkit_handle_buffer_req(struct rtkit_state *state, uint32_t endpoint,
	struct apple_mbox_msg *msg, struct rtkit_buffer *buffer)
{
	int error;
	struct apple_mbox *mbox = &state->mbox;
	//bus_addr_t addr = RTKIT_BUFFER_ADDR(msg->data0);
	bus_size_t size = RTKIT_BUFFER_SIZE(msg->data0);

	rtkit_printf(state, "RTKit endpoint %d requested %ld byte buffer\n",
	    endpoint, size << PAGE_SHIFT_4K);

	error = rtkit_alloc(state, size << PAGE_SHIFT_4K, buffer);
	if (error != 0) {
		return error;
	}

	uint64_t data = size << RTKIT_BUFFER_SIZE_SHIFT;
	data |= RTKIT_BUFFER_ADDR(buffer->addr);
	return rtkit_send(mbox, endpoint, RTKIT_BUFFER_REQUEST, data);
}

static int
rtkit_handle_crashlog(struct rtkit_state *state, struct apple_mbox_msg *msg)
{
	int rc;
	device_t dev = state->dev;
	uint8_t type = RTKIT_MGMT_TYPE(msg->data0);
	if (type != RTKIT_BUFFER_REQUEST) {
		device_printf(dev,
			"unexpected RTKit msg of type %d from crashlog endpoint", type);
		return EINVAL;
	}

	if (state->crashlog_buffer.addr) {
		panic("RTKIT crashed");
	}

	rc = rtkit_handle_buffer_req(state, RTKIT_EP_CRASHLOG, msg,
		&state->crashlog_buffer);

	return rc;
}

static void
rtkit_handle_syslog_log(struct rtkit_state *state, struct apple_mbox_msg *msg)
{
	char ctxt[24];
	u_int entry, pos;
	size_t syslog_msg_size;
	char *syslog_msg;
	device_t dev = state->dev;

	if (state->syslog_msg == NULL) {
		device_printf(dev, "no buffer since syslog called log before init\n");
		return;
	}
	entry = RTKIT_SYSLOG_LOG_ENTRY(msg->data0);
	if (entry > state->syslog_entries) {
		device_printf(state->dev, "invalid log entry %d (max %d)\n", entry, state->syslog_entries);
		return;
	}
	syslog_msg_size = state->syslog_msg_size + 32;
	syslog_msg = (char *)state->syslog_buffer.kva + (entry * syslog_msg_size + 8);
	memcpy(ctxt, syslog_msg, sizeof(ctxt));
	ctxt[sizeof(ctxt) - 1] = 0;

	syslog_msg += sizeof(ctxt);
	memcpy(state->syslog_msg, syslog_msg, state->syslog_msg_size);
	state->syslog_msg[state->syslog_msg_size - 1] = 0;

	pos = strlen(state->syslog_msg) - 1;
	while (pos >= 0) {
		if (state->syslog_msg[pos] != ' ' &&
		    state->syslog_msg[pos] != '\n' &&
		    state->syslog_msg[pos] != '\r') {
			break;
		}
		state->syslog_msg[pos--] = 0;
	}

	device_printf(dev, "RTKit syslog %d: %s:%s\n", entry, ctxt, state->syslog_msg);
}

static int
rtkit_handle_syslog(struct rtkit_state *state, struct apple_mbox_msg *msg)
{
	int rc = 0;
	uint8_t type = RTKIT_MGMT_TYPE(msg->data0);
	switch (type) {
	case RTKIT_BUFFER_REQUEST:
		rc = rtkit_handle_buffer_req(state, RTKIT_EP_SYSLOG, msg,
			&state->syslog_buffer);
		break;
	case RTKIT_SYSLOG_INIT:
		state->syslog_entries = RTKIT_SYSLOG_INIT_ENTRIES(msg->data0);
		state->syslog_msg_size = RTKIT_SYSLOG_INIT_MSG_SIZE(msg->data0);
		state->syslog_msg = malloc(state->syslog_msg_size, M_DEVBUF, M_NOWAIT);
		break;
	case RTKIT_SYSLOG_LOG:
		rtkit_handle_syslog_log(state, msg);
		rc = rtkit_send(&state->mbox, RTKIT_EP_SYSLOG, RTKIT_MGMT_TYPE(msg->data0), msg->data0);
		if (rc) {
			device_printf(state->dev, "failed to ack syslog log\n");
			return (rc);
		}
		break;
	default:
		panic("unknown syslog message of type %d\n", type);
		break;
	}

	return rc;
}

static int
rtkit_handle_ioreport(struct rtkit_state *state, struct apple_mbox_msg *msg)
{
	int rc;
	struct apple_mbox *mbox = &state->mbox;
	uint8_t type = RTKIT_MGMT_TYPE(msg->data0);
	switch (type) {
	case RTKIT_BUFFER_REQUEST:
		rc = rtkit_handle_buffer_req(state, RTKIT_EP_IOREPORT, msg,
			&state->ioreport_buffer);
		break;
	case RTKIT_IOREPORT_UNKNOWN1:
		/* fallthrough */
	case RTKIT_IOREPORT_UNKNOWN2:
		/* must acknowledge these unknown ioreport messages to avoid a hang */
		rc = rtkit_send(mbox, RTKIT_EP_IOREPORT, 0, msg->data0);
		rtkit_printf(state,
		    "Ack'ed unknown message of type %d from RTKit ioreport endpoint\n",
		    type);
		break;
	default:
		panic("unknown ioreport message of type %d", type);
		break;
	}
	return rc;
}

static int
rtkit_handle_oslog(struct rtkit_state *state, struct apple_mbox_msg *msg)
{
	int rc = 0;
	uint8_t type = RTKIT_OSLOG_TYPE(msg->data0);
	struct apple_mbox *mbox = &state->mbox;
	//bus_addr_t addr;
	bus_size_t size;

	switch (type) {
	case RTKIT_OSLOG_BUFFER_REQUEST:
		//addr = RTKIT_OSLOG_BUFFER_ADDR(msg->data0) << PAGE_SHIFT_4K;
		size = RTKIT_OSLOG_BUFFER_SIZE(msg->data0);

		rc = rtkit_alloc(state, size, &state->oslog_buffer);

		if (rc == 0) {
			uint64_t data = size << RTKIT_OSLOG_BUFFER_SIZE_SHIFT;
			data |= RTKIT_OSLOG_BUFFER_ADDR(state->oslog_buffer.addr >> PAGE_SHIFT_4K);
			rc = rtkit_send(mbox, RTKIT_EP_OSLOG, RTKIT_OSLOG_BUFFER_REQUEST, data);
		}
		break;
	case RTKIT_OSLOG_UNKNOWN1:
		/* fallthrough */
	case RTKIT_OSLOG_UNKNOWN2:
		/* fallthrough */
	case RTKIT_OSLOG_UNKNOWN3:
		rc = rtkit_send(mbox, RTKIT_EP_OSLOG, 0, msg->data0);
		rtkit_printf(state,
		    "Ack'ed unknown message of type %d from RTKit oslog endpoint\n",
		    type);
		break;
	default:
		panic("unknown oslog message of type %d", type);
		break;
	}
	return rc;
}

// called in kthread
static void
rtkit_rx_task(void *context, int pending)
{
	struct rtkit_task *task = (struct rtkit_task *)context;
	struct rtkit_state *state = task->state;
	struct apple_mbox_msg *msg = &task->msg;
	uint32_t endpoint = msg->data1;
	void (*callback)(void *, uint64_t);
	void *arg;
	bool valid_endpoint;
	int error = 0;

	switch (endpoint) {
	case RTKIT_EP_MGMT:
		error = rtkit_handle_mgmt(state, msg);
		break;
	case RTKIT_EP_CRASHLOG:
		error = rtkit_handle_crashlog(state, msg);
		break;
	case RTKIT_EP_SYSLOG:
		error = rtkit_handle_syslog(state, msg);
		break;
	case RTKIT_EP_IOREPORT:
		error = rtkit_handle_ioreport(state, msg);
		break;
	case RTKIT_EP_OSLOG:
		error = rtkit_handle_oslog(state, msg);
		break;
	case RTKIT_EP_TRACEKIT:
		panic("rtkit_handle_tracekit unimplemented\n");
	default:
 		valid_endpoint = rtkit_endpoint_is_valid(endpoint);
		if (!valid_endpoint) {
			error = EIO;
			break;
		}
		if (state->callbacks[endpoint - 32]) {
			callback = state->callbacks[endpoint - 32];
			arg = state->args[endpoint - 32];
			callback(arg, msg->data0);
		}
	}

	if (error != 0) {
		device_printf(state->dev,
			"Error (%d) while handling RTKit message from endpoint %d\n",
			error, endpoint);
	}

	free(task, M_DEVBUF);
}

// called in ithread can't sleep here
static int
rtkit_rx_callback(void *cookie, struct apple_mbox_msg msg)
{
	struct rtkit_state *state = (struct rtkit_state *)cookie;

	struct rtkit_task *task = malloc(sizeof(*task), M_DEVBUF,
		M_NOWAIT | M_USE_RESERVE);

	if (task == NULL)
		return ENOMEM;

	task->msg = msg;
	task->state = state;

	TASK_INIT(&task->task, 0, rtkit_rx_task, task);
	taskqueue_enqueue(taskqueue_thread, &task->task);

	return 0;
}

int
rtkit_init(device_t dev, struct rtkit_state **statep, bool noalloc)
{
	struct rtkit_state *state;

	if (statep == NULL) {
		return EINVAL;
	}

	state = malloc(sizeof(*state), M_DEVBUF, M_WAITOK | M_ZERO);
	if (state == NULL) {
		return ENOMEM;
	}

	state->dev = dev;
	state->iop_pwrstate = RTKIT_MGMT_PWR_STATE_SLEEP;
	state->ap_pwrstate = RTKIT_MGMT_PWR_STATE_QUIESCED;
	state->verbose = false;
	state->noalloc = noalloc;

	*statep = state;

	return 0;
}

int
rtkit_boot(struct rtkit_state *state)
{
	state->mbox.dev = apple_mbox_get(state->dev);
	if (state->mbox.dev == NULL) {
		free(state, M_DEVBUF);
		return (-1);
	}

	apple_mbox_set_rx(state->mbox.dev, rtkit_rx_callback, state);

	/* Wake up! */
	rtkit_printf(state, "setting RTKit IOP power state ON\n");
	return rtkit_set_iop_pwrstate(state, RTKIT_MGMT_PWR_STATE_ON);
}

void
rtkit_set_map_callback(struct rtkit_state *state, rtkit_map map_fn, void *cookie)
{
	state->map_fn = map_fn;
	state->map_arg = cookie;
}

int
rtkit_start_endpoint(struct rtkit_state *state, uint32_t endpoint,
    void (*callback)(void *, uint64_t), void *arg)
{
	if (endpoint < 32 || endpoint >= 64) {
		device_printf(state->dev, "invalid endpoint %d\n", endpoint);
		return EINVAL;
	}

	if ((state->epmap & (1ULL << endpoint)) == 0) {
		device_printf(state->dev, "WARNING: endpoint not set in map %d (0x%lx)\n", endpoint, state->epmap);
		return EINVAL;
	}

	state->callbacks[endpoint - 32] = callback;
	state->args[endpoint - 32] = arg;

	return rtkit_start(state, endpoint);
}

int
rtkit_send_endpoint(struct rtkit_state *state, uint32_t endpoint, uint64_t data)
{
	return rtkit_send(&state->mbox, endpoint, 0, data);
}
