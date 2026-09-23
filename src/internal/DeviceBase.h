#pragma once

#include "IDevice.h"

#include <atomic>
#include <cstdint>
#include <mutex>

namespace hal {

/// 设备 id 缓冲区长度（含结尾 NUL）。id 形如 "axis.3" / "spindle.0"。
constexpr int kMaxDeviceIdLen = 32;

/**
 * @brief 状态迁移是否合法。
 *
 *   Uninitialized ──> Ready ──> Enabled
 *                       ^          │
 *                       └──────────┘        去使能
 *   任意状态 ──> Fault ──> Ready            人工复位
 *   任意状态 ──> Shutdown                   不可逆
 *
 * 同态迁移（from == to）视为合法 —— 状态推进是幂等的。
 *
 * 提成自由函数（而不是塞在 setState 的 switch 里）是为了让「不许非法跳转」
 * 不需要构造任何设备就能穷举测试：25 种组合一张表跑完。
 */
bool isLegalTransition(DeviceState from, DeviceState to) noexcept;

/**
 * @brief 所有设备的通用实现：身份、状态、最后一次操作状态。
 *
 * 子类只实现四个生命周期方法、cycleBegin() 和自己的能力接口。
 * 状态推进有两个入口，别用错：
 *
 *   - `transitionStateRt()` —— **周期内**用。CAS、无锁、不写 lastStatus。
 *   - `setState()`           —— **生命周期阶段**用。带合法性校验、写 lastStatus（有互斥量）。
 *
 * 没有任何地方用 setState() 去「命令设备进入某状态」——状态由驱动事实决定。
 */
class DeviceBase : public virtual IDevice {
public:
    DeviceBase(const char* id, const char* type, const char* driver, int channel) noexcept;
    ~DeviceBase() override = default;

    // ==================== 身份（非 RT） ====================
    const char* id() const final { return m_id; }
    const char* type() const final { return m_type; }
    const char* driver() const final { return m_driver; }
    int channel() const final { return m_channel; }

    // ==================== 状态 ====================
    DeviceState state() const final { return m_state.load(std::memory_order_acquire); }

    /// @warning 带互斥量，**非 RT**。周期线程只读 state()。
    HalStatus lastStatus() const final;

    /// 带合法性校验的状态设置。非法迁移返回 InvalidStateTransition。
    HalStatus setState(DeviceState newState) final;

protected:
    /**
     * @brief 周期内的精确状态收敛：CAS、无锁、不写 lastStatus。
     * @return 当前状态等于 expected、迁移合法、且切换成功时为 true。
     * @note  只给周期推进点用。生命周期阶段走 setState()。
     */
    bool transitionStateRt(DeviceState expected, DeviceState desired) noexcept;

    /// 记录最后一次操作状态（供日志/错误处理，非 RT）。
    /// 落库前会把 state 覆盖成当前设备状态，保证读到的状态属实。
    void updateLastStatus(const HalStatus& st);

private:
    char        m_id[kMaxDeviceIdLen];
    const char* m_type;
    const char* m_driver;
    const int   m_channel;

    std::atomic<DeviceState> m_state;
    mutable std::mutex       m_statusMutex;
    HalStatus                m_lastStatus;
};

} // namespace hal
