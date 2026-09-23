#pragma once
#include "hal_c_api.h"

/* 现场参数由用户在 hal_hardware_test_config.c 填写；不从 HAL 猜设备行程。 */
#define HAL_TEST_IMAGE_MAX 4096u

typedef struct {
    int scaling_confirmed;       /* 非 0：该轴命令/反馈当量及方向已核实；否则拒绝运动、设坐标和偏置测试。 */
    double target;               /* axis 用例的绝对目标位置，单位与该轴 HAL 配置的用户单位一致。 */
    double max_step;             /* 本次允许的最大位移：|target - 当前反馈位置|，同上单位；不是速度限制。 */
    double tolerance;            /* 目标到位和 offset 差值判定的允许误差，同上单位。 */
    double calibrated_position; /* calibrate 用例把轴当前位置定义成此坐标，同上单位；需独立基准确认。 */
    double offset_test_delta;    /* offset 用例第二次启动时给 enc_off 增加的量，同上单位；必须非零。 */
} HalFeedHardwareCase;

typedef struct {
    int scaling_confirmed;          /* 主轴角度的命令/反馈当量及方向已核实，未确认则禁止角度运动。 */
    int speed_pdo_units_confirmed;  /* 已核对驱动速度 PDO 的单位与 HAL 的 counts/s 假设一致。 */
    double rpm;                     /* 获批的低速试转目标，单位 rpm，必须小于配置的 max_speed。 */
    double speed_tolerance;         /* 转速到位及停转判定的允许误差，单位 rpm。 */
    double angle_target;            /* CSP 用例的累计绝对目标角度，单位 deg。 */
    double max_angle_step;          /* 本次允许的最大角位移，单位 deg；不是速度限制。 */
    double angle_tolerance;         /* 角度到位允许误差，单位 deg。 */
} HalSpindleHardwareCase;

typedef struct {
    unsigned observe_cycles;      /* observe 用例执行的 wait/begin/commit 周期数。 */
    unsigned settle_cycles;       /* 等待使能、到位、停转或急停生效的最大周期数；超出判失败。 */
    HalFeedHardwareCase feed[3];   /* feed[0..2] = 从站 3..5，逻辑轴 1..3。 */
    HalSpindleHardwareCase spindle; /* 从站 2、逻辑轴 0 的主轴试验参数。 */
    uint32_t x_len;
    uint32_t y_len;
    uint8_t y_safe[HAL_TEST_IMAGE_MAX];
    uint8_t y_test[HAL_TEST_IMAGE_MAX];
    int io_images_confirmed;
} HalHardwareTestSettings;

/* 返回 0 表示所有现场配置已填写并审核；非 0 时程序拒绝启动主站。 */
int hal_hardware_test_config(HalCConfig* config, HalHardwareTestSettings* settings);
