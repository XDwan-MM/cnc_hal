#pragma once

#include <cstdint>

namespace hal {

/**
 * @brief HAL 统一错误码
 *
 * 编码规则（32 位）：高 16 位 = 模块/设备类型段，低 16 位 = 段内序号
 *   0x0000 ~ 公共段（跨设备通用）
 *   0x2000 ~ 轴（Axis）
 *   0x3000 ~ 主轴（Spindle）
 *   0x4000 ~ 数字量 IO（DIO）
 */
enum class HalErrorCode : uint32_t {
    // ==================== 公共（0x00xx） ====================
    InvalidStateTransition = 0x0001,  // 状态机非法迁移（DeviceBase::setState）
    NullPointer            = 0x0002,  // 出参指针为空
    NotRunning             = 0x0003,  // 设备未处于 Running（或 IO 未启动），操作被拒
    NotEnabledState        = 0x0004,  // enable(true) 时不在 Enabled 状态
    CannotStart            = 0x0005,  // start() 前置状态不满足（需 Standby）
    CannotStop             = 0x0006,  // stop() 前置状态不满足（需 Enabled/Running）
    BadConfig              = 0x0007,  // 配置参数非法
    DeviceBusy             = 0x0008,  // 设备忙（如回零进行中，不接受运动指令）
    InvalidArgument        = 0x0009,
    AbiMismatch            = 0x000A,
    StopRequested          = 0x000B,

    // ==================== 总线周期（0x01xx） ====================
    BusSyncFailed          = 0x0101,  // 等待主站同步中断失败或超时
    BusReceiveFailed       = 0x0102,  // 接收本周期 PDO 失败
    BusSendFailed          = 0x0103,  // 发送本周期 PDO 失败

    // ==================== 轴（0x20xx） ====================
    AxisSoftLimit          = 0x2003,  // 保留旧错误码编号；当前 HAL 不产生此码。

    // ==================== 主轴（0x30xx） ====================
    SpindleSpeedOutOfRange = 0x3001,  // 指令转速非法（负值）或超上限
    SpindleBadDirection    = 0x3002,  // 方向参数非法（仅允许 -1/0/1）
};

} // namespace hal
