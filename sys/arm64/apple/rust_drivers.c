#include <sys/param.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>

extern driver_t apple_aic_driver;
extern driver_t apple_mbox_driver;
extern driver_t apple_rtkit_driver;
extern driver_t nvme_ans_driver;

EARLY_DRIVER_MODULE(aic, simplebus, apple_aic_driver, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_MIDDLE);

EARLY_DRIVER_MODULE(apple_mbox, simplebus, apple_mbox_driver, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LATE);

EARLY_DRIVER_MODULE(apple_rtkit, simplebus, apple_rtkit_driver, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LAST);

DRIVER_MODULE(nvme, simplebus, nvme_ans_driver, NULL, NULL);
