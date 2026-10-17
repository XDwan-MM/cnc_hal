#include "hal_config_api.h"
#include <math.h>
#include <stdio.h>

/* X/Y 是 PLC 的字节地址域，上界 65535（64 KiB）；调用方按 hal_io_image_size() 的返回值分配。 */
#define HAL_MAX_IO_BYTE_ADDR 65535

static int32_t fail(char* err, uint32_t len, int32_t code, const char* text) {
    if (err && len) snprintf(err, len, "%s", text);
    return code;
}

static const HalCAxisCfg* axis_at(const HalCConfig* c, int i) {
    return i < c->axis_count ? &c->axes[i] : &c->spindles[i - c->axis_count].axis;
}

int32_t hal_config_validate(const HalCConfig* c, char* err, uint32_t len) {
    if (err && len) err[0] = '\0';   /* 先清空：不论成败，err 都不留上一次的旧文本 */
    if (!c) return fail(err, len, HAL_ERROR_ARGUMENT, "配置为空");
    /* 版本闸门：abi_major/abi_minor/struct_size 都要对，struct_size 精确相等；reserved 必须为 0。 */
    if (c->abi_major != HAL_C_ABI_MAJOR || c->abi_minor != HAL_C_ABI_MINOR ||
        c->struct_size != sizeof(*c) || c->reserved)
        return fail(err, len, HAL_ERROR_ABI, "ABI 版本、结构大小或保留字段不匹配");
    /* 当前必须为 0（指纹算法未定） */
    if (c->topology_fingerprint)
        return fail(err, len, HAL_ERROR_CONFIG, "拓扑指纹算法尚未启用，请使用 0");
    /* 主站周期与三个超时。cycle_us 下限 250、上限 UINT32_MAX/1000（驱动层按周期算纳秒 uint32，
     * 不能溢出）；三个超时都不能为 0；dc_enable 只收 0/1，运行期不可变。 */
    if (c->cycle_us < 250 || c->cycle_us > UINT32_MAX / 1000u ||
        !c->start_timeout_ms || !c->cycle_timeout_ms ||
        !c->fault_reset_timeout_ms ||
        (c->dc_enable != 0 && c->dc_enable != 1))
        return fail(err, len, HAL_ERROR_CONFIG, "主站周期、超时或 DC 配置无效");
    /* 各数组容量。最后一条是总量限制：四类设备数量之和也受槽位预算约束。 */
    if (c->axis_count < 0 || c->axis_count > (int)HAL_C_MAX_DEV ||
        c->spindle_count < 0 || c->spindle_count > (int)HAL_C_MAX_SPINDLE ||
        c->io_count < 0 || c->io_count > (int)HAL_C_MAX_DEV ||
        c->panel_count < 0 || c->panel_count > (int)HAL_C_MAX_DEV ||
        c->axis_count + c->spindle_count + c->io_count + c->panel_count > (int)HAL_C_MAX_DEV)
        return fail(err, len, HAL_ERROR_CONFIG, "设备数量超出槽位容量");
    /* 进给轴与主轴合成一张循环查（主轴紧接在进给轴之后，除 work_mode 外约束相同）。 */
    const int count = c->axis_count + c->spindle_count;
    for (int i = 0; i < count; ++i) {
        const HalCAxisCfg* a = axis_at(c, i);
        /* 下面一条 if 串起四类检查，任何一条不过都报同一句话（错误码相同，细节对照配置）。 */
        if (a->slave_pos < 0 || a->slave_pos >= (int)HAL_C_MAX_DEV || a->axis_index < 0 ||
            a->logical_axis < -1 || a->logical_axis >= 32 ||
            (i >= c->axis_count && a->logical_axis < 0) ||
            (a->estop_action != HAL_ESTOP_DISABLE_OPERATION && a->estop_action != HAL_ESTOP_DISABLE_VOLTAGE) ||
            (a->work_mode != HAL_WORK_POSITION && a->work_mode != HAL_WORK_VELOCITY) ||
            (i < c->axis_count && a->work_mode != HAL_WORK_POSITION) ||   /* 进给轴强制位置模式；主轴可配速度 */
            (a->encoder_type != HAL_ENC_INCREMENTAL_Z && a->encoder_type != HAL_ENC_ABSOLUTE) ||
            a->feedback_pulses_per_rev <= 0 ||
            (a->speed_unit != HAL_SPEED_RPM && a->speed_unit != HAL_SPEED_PULSES_PER_S) ||
            (a->speed_unit == HAL_SPEED_PULSES_PER_S && a->command_pulses_per_rev <= 0) ||
            (a->feedback_wrap != HAL_WRAP_LINEAR && a->feedback_wrap != HAL_WRAP_MODULAR) ||
            !isfinite(a->command_units_per_count) || a->command_units_per_count <= 0 ||
            !isfinite(a->feedback_units_per_count) || a->feedback_units_per_count <= 0 ||
            !isfinite(a->enc_off))
            return fail(err, len, HAL_ERROR_CONFIG, "轴绑定、急停动作、速度单位或换算参数无效");
        /* 物理绑定与逻辑轴号都不能与前面任一轴重复；逻辑轴号 -1 不占轴槽，不参与去重。 */
        for (int j = 0; j < i; ++j) {
            const HalCAxisCfg* b = axis_at(c, j);
            if ((a->slave_pos == b->slave_pos && a->axis_index == b->axis_index) ||
                (a->logical_axis >= 0 && a->logical_axis == b->logical_axis))
                return fail(err, len, HAL_ERROR_CONFIG, "轴或主轴的物理绑定、逻辑轴号重复");
        }
    }
    /* 主轴只校验转速上限；加减速曲线与「到速」判定不归 HAL，没有对应字段。 */
    for (int i = 0; i < c->spindle_count; ++i) {
        const HalCSpindleCfg* s = &c->spindles[i];
        if (!isfinite(s->max_speed) || s->max_speed <= 0)
            return fail(err, len, HAL_ERROR_CONFIG, "主轴转速参数无效");
    }
    /* IO 与面板共用 PLC 字节地址域，一起查地址范围与从站占用冲突。 */
    for (int i = 0; i < c->io_count + c->panel_count; ++i) {
        const int panel = i >= c->io_count;
        const int index = panel ? i - c->io_count : i;
        const int slave = panel ? c->panels[index].slave_pos : c->ios[index].slave_pos;
        const int x = panel ? c->panels[index].x_start : c->ios[index].x_start;
        const int y = panel ? c->panels[index].y_start : c->ios[index].y_start;
        if (slave < 0 || slave >= (int)HAL_C_MAX_DEV ||
            x < 0 || x > HAL_MAX_IO_BYTE_ADDR || y < 0 || y > HAL_MAX_IO_BYTE_ADDR)
            return fail(err, len, HAL_ERROR_CONFIG, "IO 或面板地址无效");
        /* 伺服与 IO 的 PDO 结构完全不同，同一从站不能两用。 */
        for (int j = 0; j < count; ++j)
            if (axis_at(c, j)->slave_pos == slave)
                return fail(err, len, HAL_ERROR_CONFIG, "同一从站不能同时绑定伺服和 IO");
        /* IO 与面板之间也不能重复占同一个从站。 */
        for (int j = 0; j < i; ++j) {
            const int prev = j < c->io_count ? c->ios[j].slave_pos : c->panels[j - c->io_count].slave_pos;
            if (prev == slave) return fail(err, len, HAL_ERROR_CONFIG, "IO 或面板从站重复");
        }
    }
    /* 静态检查到此为止；真实轴数、PDO 位长、X/Y 段重叠都要等 start 认领设备才知道。 */
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
    case HAL_ERROR_UNSUPPORTED: text = "设备不支持该功能"; break;
    case HAL_ERROR_NOT_ENABLED: text = "轴尚未使能"; break;
    case HAL_ERROR_MODE_MISMATCH: text = "运行模式未到位"; break;
    default: text = "未知错误"; break;
    }
    (void)fail(out, len, code, text);
}
