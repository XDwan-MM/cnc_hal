# pragma once
# include "HalError.h"   // 统一错误码
# include <cstdint>
# include <cstring>
# include <string>

namespace hal{

    /**
     * 逻辑轴号上界（开区间）。
     *
     * **逻辑轴号就是对外暴露的 HalAxisId**（见 include/c_api/hal_types.h），
     * HAL 内部按它建 O(1) 索引表，所以必须有上界——否则一个手填的大轴号
     * 就会撑爆表。
     *
     * 放在这里（而不是 GmMaster 里）是因为**配置校验也要用它**，而配置库
     * 是零硬件依赖的，不能 include 驱动头。
     */
    constexpr int kMaxLogicalAxis = 32;

    
    // 设备生命周期状态
    //
    // 语义边界（见 doc/架构设计_v2.md §14）：只表达「HAL 认为这台设备现在能不能
    // 收指令」，不表达任何机床工艺语义——「正在运动／已回零／被屏蔽／被互锁」
    // 属于上层，HAL 不持有也不推断。因此本状态由驱动事实单向推进（poll 到
    // DS402 OperationEnabled 即 Enabled），上层只读，不通过调 enable() 去推它。
    enum class DeviceState : uint8_t {
        Uninitialized = 0,   // 构造后、init 前
        Ready,               // 已装配、可受理使能（伺服：DS402 尚未到位）
        Enabled,             // 已使能、可收指令（伺服：DS402 OperationEnabled）
        Fault,               // 驱动器报警／急停
        Shutdown             // 已销毁
    };

    // 内部维护结构体
    struct HalStatus
    {
        bool ok = true;                             // 操作成功标志
        DeviceState state = DeviceState::Uninitialized;  // 设备生命周期状态
        uint32_t code = 0;                          // 数值错误码
        char text[64]= {0};                         // 人眼可读的辅助信息（固定长度）

        // ==================== 静态工厂方法 ====================

        static HalStatus success() {
            HalStatus s;
            s.ok = true;
            s.code = 0;
            s.text[0] = '\0';
            return s;
        }

        static HalStatus error(uint32_t code, const char* msg = nullptr) {
            HalStatus s;
            s.ok = false;
            s.code = code;
            if (msg) {
                strncpy(s.text, msg, sizeof(s.text) - 1);
                s.text[sizeof(s.text) - 1] = '\0';
            } else {
                s.text[0] = '\0';
            }
            return s;
        }

        /// @brief 错误工厂（HalErrorCode 枚举重载，避免调用处强转）
        static HalStatus error(HalErrorCode code, const char* msg = nullptr) {
            return error(static_cast<uint32_t>(code), msg);
        }
    };
}// namespace Hal