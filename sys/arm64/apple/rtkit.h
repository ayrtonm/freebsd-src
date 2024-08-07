/* public domain */

#define RTKIT_MGMT_PWR_STATE_SLEEP	0x0001
#define RTKIT_MGMT_PWR_STATE_QUIESCED	0x0010
#define RTKIT_MGMT_PWR_STATE_ON		0x0020

struct rtkit_state;

typedef int (*rtkit_map)(void *, bus_addr_t, bus_size_t);

int rtkit_init(device_t, struct rtkit_state **);
int rtkit_boot(struct rtkit_state *);

void rtkit_set_map_callback(struct rtkit_state *, rtkit_map, void *);
int	apple_sart_map(uint32_t, bus_addr_t, bus_size_t);

int rtkit_set_ap_pwrstate(struct rtkit_state *state, uint16_t pwrstate);



//struct rtkit {
//	void *rk_cookie;
//	bus_dma_tag_t rk_dmat;
//	bus_size_t rk_dma_maxsize;
//	int (*rk_map)(void *, bus_addr_t, bus_size_t);
//};
//
////struct rtkit_state *rtkit_init(int, const char *, struct rtkit *, mbox_t);
//int	rtkit_boot(struct rtkit_state *);
//int	rtkit_start_endpoint(struct rtkit_state *, uint32_t,
//	    void (*)(void *, uint64_t), void *);
//int	rtkit_send_endpoint(struct rtkit_state *, uint32_t, uint64_t);
//
