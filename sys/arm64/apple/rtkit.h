struct rtkit_state;
typedef int (*rtkit_map)(void *, bus_addr_t, bus_size_t);

struct rtkit_state *rtkit_init(device_t, bool);
int rtkit_boot(struct rtkit_state *);
void rtkit_set_map_callback(struct rtkit_state *, rtkit_map, void *);
int	apple_sart_map(uint32_t, bus_addr_t, bus_size_t);

