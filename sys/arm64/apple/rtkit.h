/* public domain */

#ifndef _RTKIT_H_
#define	_RTKIT_H_

#define RTKIT_MGMT_PWR_STATE_SLEEP	0x0001
#define RTKIT_MGMT_PWR_STATE_QUIESCED	0x0010
#define RTKIT_MGMT_PWR_STATE_ON		0x0020

struct rtkit_state;

typedef int (*rtkit_map)(void *, bus_addr_t, bus_size_t);

struct rtkit_state *rtkit_init(device_t, bool);
int rtkit_boot(struct rtkit_state *);

void rtkit_set_map_callback(struct rtkit_state *, rtkit_map, void *);
int	apple_sart_map(uint32_t, bus_addr_t, bus_size_t);

int rtkit_set_ap_pwrstate(struct rtkit_state *, uint16_t);

int apple_rtkit_boot(device_t client, phandle_t);

int rtkit_start_endpoint(struct rtkit_state *, uint32_t,
    void (*)(void *, uint64_t), void *);

int rtkit_send_endpoint(struct rtkit_state *, uint32_t, uint64_t);

struct rtkit_task;

int rtkit_set_iop_pwrstate(struct rtkit_state *state, uint16_t pwrstate);
int rtkit_rx_callback(void *cookie, struct apple_mbox_msg msg);
void rtkit_rx_task(struct rtkit_task *context, int pending);

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
	//struct task				task;
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
int rtkit_handle_mgmt(struct rtkit_state *state, struct apple_mbox_msg *msg);
#endif /* _RTKIT_H_ */
