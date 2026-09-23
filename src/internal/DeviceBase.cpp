#include "DeviceBase.h"

#include <cstdio>

namespace hal {

bool isLegalTransition(DeviceState from, DeviceState to) noexcept {
    if (from == to) return true;   // 幂等

    switch (from) {
    case DeviceState::Uninitialized:
        return to == DeviceState::Ready ||
               to == DeviceState::Fault ||
               to == DeviceState::Shutdown;

    case DeviceState::Ready:
        return to == DeviceState::Enabled ||
               to == DeviceState::Fault ||
               to == DeviceState::Shutdown;

    case DeviceState::Enabled:
        return to == DeviceState::Ready ||      // 去使能
               to == DeviceState::Fault ||
               to == DeviceState::Shutdown;

    case DeviceState::Fault:
        return to == DeviceState::Ready ||      // 人工复位
               to == DeviceState::Shutdown;

    case DeviceState::Shutdown:
        return false;                            // 不可逆
    }
    return false;
}

DeviceBase::DeviceBase(const char* id, const char* type,
                       const char* driver, int channel) noexcept
    : m_type(type ? type : ""),
      m_driver(driver ? driver : ""),
      m_channel(channel),
      m_state(DeviceState::Uninitialized) {
    if (id) std::snprintf(m_id, sizeof(m_id), "%s", id);
    else    m_id[0] = '\0';
    m_lastStatus = HalStatus::success();
}

HalStatus DeviceBase::lastStatus() const {
    std::lock_guard<std::mutex> lk(m_statusMutex);
    return m_lastStatus;
}

HalStatus DeviceBase::setState(DeviceState newState) {
    const DeviceState cur = m_state.load(std::memory_order_acquire);

    if (!isLegalTransition(cur, newState)) {
        const HalStatus st = HalStatus::error(HalErrorCode::InvalidStateTransition,
                                              "状态迁移非法");
        updateLastStatus(st);
        return st;
    }

    m_state.store(newState, std::memory_order_release);

    HalStatus ok = HalStatus::success();
    ok.state = newState;
    updateLastStatus(ok);
    return ok;
}

bool DeviceBase::transitionStateRt(DeviceState expected, DeviceState desired) noexcept {
    if (!isLegalTransition(expected, desired)) return false;

    DeviceState cur = expected;
    return m_state.compare_exchange_strong(cur, desired,
                                           std::memory_order_acq_rel,
                                           std::memory_order_acquire);
}

void DeviceBase::updateLastStatus(const HalStatus& st) {
    std::lock_guard<std::mutex> lk(m_statusMutex);
    m_lastStatus = st;
    /* 落库前用当前状态覆盖，保证读到的 state 属实——
     * 调用方传进来的 state 是它构造时的猜测，不一定对。 */
    m_lastStatus.state = m_state.load(std::memory_order_acquire);
}

} // namespace hal
