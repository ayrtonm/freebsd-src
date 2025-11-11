#include <sys/param.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>

#if 1
extern driver_t apple_aic_driver;
#endif
extern driver_t apple_mbox_driver;
extern driver_t apple_rtkit_driver;
extern driver_t nvme_ans_driver;
extern driver_t apple_smc_driver;

// Defined in apple_aic.c
#if 1
EARLY_DRIVER_MODULE(aic, simplebus, apple_aic_driver, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_MIDDLE);
#endif

EARLY_DRIVER_MODULE(apple_mbox, simplebus, apple_mbox_driver, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LATE);

EARLY_DRIVER_MODULE(apple_rtkit, simplebus, apple_rtkit_driver, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LAST);

DRIVER_MODULE(nvme, simplebus, nvme_ans_driver, 0, 0);

DRIVER_MODULE(apple_smc, simplebus, apple_smc_driver, 0, 0);
