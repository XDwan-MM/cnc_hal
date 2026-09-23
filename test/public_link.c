#include "hal_c_api.h"
#include <string.h>

int main(void) {
    HalCConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.abi_major = HAL_C_ABI_MAJOR;
    cfg.abi_minor = HAL_C_ABI_MINOR;
    cfg.struct_size = sizeof(cfg);
    cfg.cycle_us = 1000;
    cfg.start_timeout_ms = 120000;
    cfg.cycle_timeout_ms = 1000;
    HalContext* c = 0;
    if (hal_context_create(&cfg, &c, 0, 0) || !c) return 1;
    const int failed = hal_axis_count(c) != 0 || hal_context_request_stop(c) != 0 || hal_context_stop(c) != 0;
    hal_context_destroy(c);
    return failed;
}
