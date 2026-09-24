#include "hal_hardware_test_config.h"
#include <string.h>

int hal_hardware_test_config(HalCConfig* config, HalHardwareTestSettings* settings) {
    memset(config, 0, sizeof(*config));
    memset(settings, 0, sizeof(*settings));

    config->abi_major = HAL_C_ABI_MAJOR;
    config->abi_minor = HAL_C_ABI_MINOR;
    config->struct_size = sizeof(*config);
    config->reserved = 0;
    config->topology_fingerprint = 0;

    /* 已知台架拓扑：从站 2 为主轴，从站 3～5 为三根进给轴。 */
    config->axis_count = 3;
    for (int i = 0; i < 3; ++i) {
        config->axes[i].slave_pos = i + 3;
        config->axes[i].axis_index = 0;
        config->axes[i].logical_axis = i + 1;
        config->axes[i].work_mode = HAL_WORK_POSITION;
        config->axes[i].feedback_wrap = HAL_WRAP_LINEAR;
        /* 暂以“计数”为用户单位，仅便于编译配置；运动被 scaling_confirmed 拦住。 */
        config->axes[i].command_units_per_count = 1.0;
        config->axes[i].feedback_units_per_count = 1.0;
        config->axes[i].feedback_pulses_per_rev = 1;
    }
    config->spindle_count = 1;
    config->spindles[0].axis.slave_pos = 2;
    config->spindles[0].axis.axis_index = 0;
    config->spindles[0].axis.logical_axis = 0;
    config->spindles[0].axis.work_mode = HAL_WORK_VELOCITY;
    config->spindles[0].axis.feedback_wrap = HAL_WRAP_LINEAR;
    /* 占位“计数单位”；主轴转速/角度用例受 confirmed 标志阻止。 */
    config->spindles[0].axis.command_units_per_count = 1.0;
    config->spindles[0].axis.feedback_units_per_count = 1.0;
    config->spindles[0].axis.feedback_pulses_per_rev = 1;
    config->panel_count = 1;
    config->panels[0].slave_pos = 0;
    config->panels[0].x_start = 0;
    config->panels[0].y_start = 0;
    config->io_count = 1;
    config->ios[0].slave_pos = 1;
    config->ios[0].x_start = 256;
    config->ios[0].y_start = 256;
    settings->observe_cycles = 10;
    settings->x_len = 512;
    settings->y_len = 512;

    /*
     * 已填从站顺序，下面仍需确认主站参数、伺服型号、急停动作、编码器类型、
     * PDO 长度和 IO 点位，然后把末尾 return -1 改成 return 0。
     * 面板与 IO 的 0/256 地址段预留了单模块每方向最大 256 字节；
     * 真实位映射需按实际 PDO 与接线表核对。
     */
     config->cycle_us = 1000;            // >= 250
     config->start_timeout_ms = 120000;
     config->cycle_timeout_ms = 5000;
     config->dc_enable = 1;
     
     for (int i = 0; i < 3; ++i) {
          config->axes[i].estop_action = HAL_ESTOP_DISABLE_VOLTAGE; // 现场逐轴确定
          config->axes[i].encoder_type = HAL_ENC_ABSOLUTE; // 按实物确定
          config->axes[i].feedback_pulses_per_rev = 100000;
          config->axes[i].command_units_per_count = 0.16;
          config->axes[i].feedback_units_per_count = 0.16;
          config->axes[i].command_invert = 0;
          config->axes[i].feedback_invert = 0;
          config->axes[i].enc_off = 150;
     }
     config->spindles[0].axis.estop_action = HAL_ESTOP_DISABLE_VOLTAGE; // 同样必须显式选择
     config->spindles[0].axis.encoder_type = HAL_ENC_ABSOLUTE;
     config->spindles[0].axis.feedback_pulses_per_rev = 100000;
     config->spindles[0].axis.command_units_per_count = 0.16; // deg/count
     config->spindles[0].axis.feedback_units_per_count = 0.16; // deg/count
     config->spindles[0].max_speed = 8000;    // rpm
     config->spindles[0].accel = 100;
     config->spindles[0].speed_window = config->spindles[0].max_speed * 0.15; // rpm
     
     settings->observe_cycles = 10000;    // 建议先从有限的小次数开始
     settings->settle_cycles = 10000;     // 运动完成的最大等待周期
     for (int i = 0; i < 3; ++i) {
          settings->feed[i].scaling_confirmed = 1; // 两侧当量、方向已实测
          settings->feed[i].target = -3200;          // 获批的绝对目标坐标
          settings->feed[i].max_step = 3000;        // 单次最大允许位移
          settings->feed[i].tolerance = 1;
          settings->feed[i].calibrated_position = 0;
          settings->feed[i].offset_test_delta = 1;
     }
     settings->spindle.scaling_confirmed = 1; // deg/count 及方向已核实
     settings->spindle.speed_pdo_units_confirmed = 1; // 速度 PDO 确认为 counts/s
     settings->spindle.rpm = 1000;              // 批准的低速正/反转目标
     settings->spindle.speed_tolerance = 20;  // rpm
     settings->spindle.angle_target = 360;     // deg，本用例把静止当前位置设为 0° 后的目标
     settings->spindle.max_angle_step = 360;   // deg
     settings->spindle.angle_tolerance = 5;  // deg
     
     settings->x_len = 512; // 覆盖全部 X 映像的字节数
     settings->y_len = 512; // 覆盖全部 Y 映像的字节数，<= 4096
     settings->y_safe[256] = 1; // 整块 Y 的已审核安全输出值
     memcpy(settings->y_test, settings->y_safe, settings->y_len);
     settings->y_test[0] |= (1u << 1); // 仅修改获批输出点
     settings->io_images_confirmed = 1; // 两幅完整 Y 映像已经逐位核对
     /*
     * 主轴虽然也是伺服，仍必须在 spindles[] 中配置才能测试 CSV/CSP。
     */
    return 0;
}
