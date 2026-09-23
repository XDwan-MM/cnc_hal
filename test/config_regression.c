#include "hal_c_api.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static HalCConfig good(void) {
    HalCConfig c = {0};
    c.abi_major = HAL_C_ABI_MAJOR;
    c.abi_minor = HAL_C_ABI_MINOR;
    c.struct_size = sizeof(c);
    c.cycle_us = 1000;
    c.start_timeout_ms = 120000;
    c.cycle_timeout_ms = 5000;
    c.axis_count = 1;
    c.axes[0].estop_action = HAL_ESTOP_DISABLE_OPERATION;
    c.axes[0].work_mode = HAL_WORK_POSITION;
    c.axes[0].encoder_type = HAL_ENC_INCREMENTAL_Z;
    c.axes[0].feedback_pulses_per_rev = 10000;
    c.axes[0].command_units_per_count = .001;
    c.axes[0].feedback_units_per_count = .002;
    return c;
}

int main(void) {
    HalCConfig c = good();
    char err[128];
    assert(hal_config_validate(&c, err, sizeof(err)) == HAL_OK && !err[0]);
    assert(hal_config_validate(NULL, err, 1) == HAL_ERROR_ARGUMENT && !err[0]);
    c.abi_minor = HAL_C_ABI_MINOR - 1;
    assert(hal_config_validate(&c, NULL, 0) == HAL_ERROR_ABI);
    c = good(); c.struct_size--;
    assert(hal_config_validate(&c, NULL, 0) == HAL_ERROR_ABI);
    c = good(); c.axes[0].estop_action = 0;
    assert(hal_config_validate(&c, err, sizeof(err)) == HAL_ERROR_CONFIG && err[0]);
    c = good(); c.axes[0].command_units_per_count = NAN;
    assert(hal_config_validate(&c, NULL, 0) == HAL_ERROR_CONFIG);
    c = good(); c.axis_count = 31;
    assert(hal_config_validate(&c, NULL, 0) == HAL_ERROR_CONFIG);
    c = good(); c.spindle_count = 1;
    c.spindles[0].axis = c.axes[0]; c.spindles[0].max_speed = 1000;
    c.spindles[0].axis.logical_axis = 1;
    assert(hal_config_validate(&c, NULL, 0) == HAL_ERROR_CONFIG);
    c.spindles[0].axis.axis_index = 1;
    assert(hal_config_validate(&c, NULL, 0) == HAL_OK);
    c.spindles[0].axis.logical_axis = 0;
    assert(hal_config_validate(&c, NULL, 0) == HAL_ERROR_CONFIG);
    c = good(); c.io_count = 1; c.ios[0].slave_pos = 1;
    c.panel_count = 1; c.panels[0].slave_pos = 1;
    assert(hal_config_validate(&c, NULL, 0) == HAL_ERROR_CONFIG);
    c.panels[0].slave_pos = 2;
    assert(hal_config_validate(&c, NULL, 0) == HAL_OK);
    c.panels[0].y_start = -1;
    assert(hal_config_validate(&c, NULL, 0) == HAL_ERROR_CONFIG);
    c = good(); c.cycle_timeout_ms = 0;
    assert(hal_config_validate(&c, NULL, 0) == HAL_ERROR_CONFIG);
    memset(err, 'x', sizeof(err)); hal_error_text(HAL_ERROR_STOPPED, err, sizeof(err));
    assert(strcmp(err, "已请求停止") == 0);
    puts("Config regression passed");
}
