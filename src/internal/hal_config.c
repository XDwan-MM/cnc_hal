#include "hal_config_api.h"
#include <math.h>
#include <stdio.h>

static int32_t fail(char* err, uint32_t len, int32_t code, const char* text) {
    if (err && len) snprintf(err, len, "%s", text);
    return code;
}

static const HalCAxisCfg* axis_at(const HalCConfig* c, int i) {
    return i < c->axis_count ? &c->axes[i] : &c->spindles[i - c->axis_count].axis;
}

int32_t hal_config_validate(const HalCConfig* c, char* err, uint32_t len) {
    if (err && len) err[0] = '\0';
    if (!c) return fail(err, len, HAL_ERROR_ARGUMENT, "配置为空");
    if (c->abi_major != HAL_C_ABI_MAJOR || c->abi_minor != HAL_C_ABI_MINOR ||
        c->struct_size != sizeof(*c) || c->reserved)
        return fail(err, len, HAL_ERROR_ABI, "ABI 版本、结构大小或保留字段不匹配");
    if (c->topology_fingerprint)
        return fail(err, len, HAL_ERROR_CONFIG, "拓扑指纹算法尚未启用，请使用 0"); // TODO 指纹算法未定暂时都填0
    if (c->cycle_us < 250 || !c->start_timeout_ms || !c->cycle_timeout_ms ||
        (c->dc_enable != 0 && c->dc_enable != 1))
        return fail(err, len, HAL_ERROR_CONFIG, "主站周期、超时或 DC 配置无效");
    if (c->axis_count < 0 || c->axis_count > (int)HAL_C_MAX_DEV ||
        c->spindle_count < 0 || c->spindle_count > (int)HAL_C_MAX_SPINDLE ||
        c->io_count < 0 || c->io_count > (int)HAL_C_MAX_DEV ||
        c->panel_count < 0 || c->panel_count > (int)HAL_C_MAX_DEV ||
        c->axis_count + c->spindle_count + c->io_count + c->panel_count > (int)HAL_C_MAX_DEV)
        return fail(err, len, HAL_ERROR_CONFIG, "设备数量超出槽位容量");
    const int count = c->axis_count + c->spindle_count;
    for (int i = 0; i < count; ++i) {
        const HalCAxisCfg* a = axis_at(c, i);
        if (a->slave_pos < 0 || a->slave_pos >= (int)HAL_C_MAX_DEV || a->axis_index < 0 ||
            a->logical_axis < -1 || a->logical_axis >= 32 ||
            (i >= c->axis_count && a->logical_axis < 0) ||
            (a->estop_action != HAL_ESTOP_DISABLE_OPERATION && a->estop_action != HAL_ESTOP_DISABLE_VOLTAGE) ||
            (a->work_mode != HAL_WORK_POSITION && a->work_mode != HAL_WORK_VELOCITY) ||
            (i < c->axis_count && a->work_mode != HAL_WORK_POSITION) ||
            (a->encoder_type != HAL_ENC_INCREMENTAL_Z && a->encoder_type != HAL_ENC_ABSOLUTE) ||
            a->feedback_pulses_per_rev <= 0 ||
            (a->feedback_wrap != HAL_WRAP_LINEAR && a->feedback_wrap != HAL_WRAP_MODULAR) ||
            !isfinite(a->command_units_per_count) || a->command_units_per_count <= 0 ||
            !isfinite(a->feedback_units_per_count) || a->feedback_units_per_count <= 0 ||
            !isfinite(a->enc_off))
            return fail(err, len, HAL_ERROR_CONFIG, "轴绑定、急停动作或换算参数无效");
        for (int j = 0; j < i; ++j) {
            const HalCAxisCfg* b = axis_at(c, j);
            if ((a->slave_pos == b->slave_pos && a->axis_index == b->axis_index) ||
                (a->logical_axis >= 0 && a->logical_axis == b->logical_axis))
                return fail(err, len, HAL_ERROR_CONFIG, "轴或主轴的物理绑定、逻辑轴号重复");
        }
    }
    for (int i = 0; i < c->spindle_count; ++i) {
        const HalCSpindleCfg* s = &c->spindles[i];
        if (!isfinite(s->max_speed) || s->max_speed <= 0 || !isfinite(s->accel) || s->accel < 0 ||
            !isfinite(s->speed_window) || s->speed_window < 0)
            return fail(err, len, HAL_ERROR_CONFIG, "主轴转速参数无效");
    }
    for (int i = 0; i < c->io_count + c->panel_count; ++i) {
        const int panel = i >= c->io_count;
        const int index = panel ? i - c->io_count : i;
        const int slave = panel ? c->panels[index].slave_pos : c->ios[index].slave_pos;
        const int x = panel ? c->panels[index].x_start : c->ios[index].x_start;
        const int y = panel ? c->panels[index].y_start : c->ios[index].y_start;
        if (slave < 0 || slave >= (int)HAL_C_MAX_DEV || x < 0 || y < 0)
            return fail(err, len, HAL_ERROR_CONFIG, "IO 或面板地址无效");
        for (int j = 0; j < count; ++j)
            if (axis_at(c, j)->slave_pos == slave)
                return fail(err, len, HAL_ERROR_CONFIG, "同一从站不能同时绑定伺服和 IO");
        for (int j = 0; j < i; ++j) {
            const int prev = j < c->io_count ? c->ios[j].slave_pos : c->panels[j - c->io_count].slave_pos;
            if (prev == slave) return fail(err, len, HAL_ERROR_CONFIG, "IO 或面板从站重复");
        }
    }
    return HAL_OK;
}

void hal_error_text(int32_t code, char* out, uint32_t len) {
    const char* text;
    switch (code) {
    case HAL_OK: text = "成功"; break;
    case HAL_ERROR_ARGUMENT: text = "参数无效"; break;
    case HAL_ERROR_ABI: text = "ABI 不匹配"; break;
    case HAL_ERROR_CONFIG: text = "配置无效"; break;
    case HAL_ERROR_NOT_RUNNING: text = "尚未就绪"; break;
    case HAL_ERROR_STOPPED: text = "已请求停止"; break;
    case HAL_ERROR_BUS: text = "总线错误"; break;
    case HAL_ERROR_BUSY: text = "主站已被占用"; break;
    case HAL_ERROR_STATE: text = "调用顺序错误"; break;
    case HAL_ERROR_MEMORY: text = "内存分配失败"; break;
    case HAL_ERROR_TIMEOUT: text = "启动总预算已耗尽"; break;
    default: text = "未知错误"; break;
    }
    (void)fail(out, len, code, text);
}
