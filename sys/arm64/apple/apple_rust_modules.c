#include <sys/param.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>

extern driver_t apple_aic_driver;

extern driver_t apple_mbox_driver;

extern driver_t nvme_ans_driver;

extern driver_t apple_smc_driver;

extern driver_t apple_rtkit_driver;
extern driver_t dockchannel_driver;
extern driver_t dockchannel_hid_driver;

// Defined in apple_aic.c
EARLY_DRIVER_MODULE(aic, simplebus, apple_aic_driver, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_MIDDLE);

EARLY_DRIVER_MODULE(apple_mbox, simplebus, apple_mbox_driver, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LATE);

EARLY_DRIVER_MODULE(apple_rtkit, simplebus, apple_rtkit_driver, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LAST);

//DRIVER_MODULE(nvme, simplebus, nvme_ans_driver, 0, 0);

DRIVER_MODULE(apple_smc, simplebus, apple_smc_driver, 0, 0);

DRIVER_MODULE(dockchannel, simplebus, dockchannel_driver, 0, 0);

DRIVER_MODULE(dockchannel_hid, dockchannel, dockchannel_hid_driver, 0, 0);
MODULE_DEPEND(dockchannel_hid, hid, 1, 1, 1);
MODULE_DEPEND(dockchannel_hid, hidbus, 1, 1, 1);
