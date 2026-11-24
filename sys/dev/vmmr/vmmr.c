#include <sys/param.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/module.h>

int vmmr_modevent(module_t mod, int type, void *data);

DEV_MODULE(vmmr, vmmr_modevent, NULL);
