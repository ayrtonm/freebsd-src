#ifdef LOCAL_TRACE
#define ltracef(fmt, ...) printf("%s:" fmt "\n", __func__, ##__VA_ARGS__)
#else
#define ltracef(...)
#endif
