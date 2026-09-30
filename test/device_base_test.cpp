/* DeviceBase 与 isLegalTransition 的离线验证。
 * 纯内存对象，不需要任何硬件。 */
#include "internal/DeviceBase.h"
#include <cstdio>

using namespace hal;

static int fails = 0;
static void chk(bool c, const char* what) {
    std::printf("  %-52s %s\n", what, c ? "OK" : "<-- FAIL");
    if (!c) fails++;
}

/* 测试用最小设备：只把生命周期映射到状态，不做别的事。 */
class TestDevice : public DeviceBase {
public:
    TestDevice() : DeviceBase("test.0", "test", "none", 7) {}

    HalStatus init() override     { return setState(DeviceState::Ready); }
    HalStatus start() override    { return setState(DeviceState::Enabled); }
    HalStatus stop() override     { return setState(DeviceState::Ready); }
    void      shutdown() override { setState(DeviceState::Shutdown); }

    HalStatus cycleBegin() noexcept override {
        /* 模拟驱动事实推进：Ready → Enabled（DS402 到位） */
        transitionStateRt(DeviceState::Ready, DeviceState::Enabled);
        return HalStatus::success();
    }

    /* 暴露 protected 的 CAS 给测试 */
    bool rtGoto(DeviceState from, DeviceState to) noexcept { return transitionStateRt(from, to); }
};

static const char* sname(DeviceState s) {
    switch (s) {
    case DeviceState::Uninitialized: return "Uninitialized";
    case DeviceState::Ready:         return "Ready";
    case DeviceState::Enabled:       return "Enabled";
    case DeviceState::Fault:         return "Fault";
    case DeviceState::Shutdown:      return "Shutdown";
    }
    return "?";
}

int main() {
    /* ---- 1. isLegalTransition 全表 25 种 ---- */
    const DeviceState all[] = {
        DeviceState::Uninitialized, DeviceState::Ready, DeviceState::Enabled,
        DeviceState::Fault, DeviceState::Shutdown
    };
    /* 期望表：legal[from][to] */
    const bool legal[5][5] = {
        /* Un  Rd  En  Fa  Sh */
        {  1,  1,  0,  1,  1 },   /* Uninitialized */
        {  0,  1,  1,  1,  1 },   /* Ready         */
        {  0,  1,  1,  1,  1 },   /* Enabled       */
        {  0,  1,  0,  1,  1 },   /* Fault         */
        {  0,  0,  0,  0,  1 },   /* Shutdown      */
    };

    std::printf("isLegalTransition 全表（5×5 = 25 种）：\n");
    bool tableOk = true;
    for (int i = 0; i < 5; ++i)
        for (int j = 0; j < 5; ++j) {
            const bool got = isLegalTransition(all[i], all[j]);
            if (got != (legal[i][j] != 0)) {
                std::printf("    %s -> %s: 期望 %d 实际 %d  <-- 不符\n",
                            sname(all[i]), sname(all[j]), legal[i][j], (int)got);
                tableOk = false;
            }
        }
    chk(tableOk, "25 种组合与期望表逐条一致");

    std::printf("\n几条语义要点：\n");
    chk(isLegalTransition(DeviceState::Ready, DeviceState::Ready),          "同态迁移合法（幂等）");
    chk(!isLegalTransition(DeviceState::Enabled, DeviceState::Uninitialized),"Enabled 不能回 Uninitialized");
    chk(!isLegalTransition(DeviceState::Fault, DeviceState::Enabled),       "Fault 必须先回 Ready（人工复位）");
    chk(!isLegalTransition(DeviceState::Shutdown, DeviceState::Ready),      "Shutdown 不可逆");

    /* ---- 2. DeviceBase 身份 ---- */
    std::printf("\n身份：\n");
    TestDevice d;
    chk(std::string(d.id()) == "test.0", "id");
    chk(std::string(d.type()) == "test", "type");
    chk(std::string(d.driver()) == "none", "driver");
    chk(d.channel() == 7, "channel");
    chk(d.state() == DeviceState::Uninitialized, "构造后是 Uninitialized");

    /* ---- 3. 生命周期与校验 ---- */
    std::printf("\n生命周期：\n");
    chk(d.init().ok && d.state() == DeviceState::Ready, "init() → Ready");
    chk(d.start().ok && d.state() == DeviceState::Enabled, "start() → Enabled");

    {   /* 非法迁移必须被拒 */
        const HalStatus st = d.setState(DeviceState::Uninitialized);
        chk(!st.ok && st.code == (uint32_t)HalErrorCode::InvalidStateTransition,
            "Enabled → Uninitialized 被拒（InvalidStateTransition）");
        chk(d.state() == DeviceState::Enabled, "被拒后状态没变");
    }

    /* ---- 4. 周期推进（CAS）---- */
    std::printf("\n周期推进 transitionStateRt（CAS）：\n");
    chk(d.stop().ok && d.state() == DeviceState::Ready, "stop() → Ready");
    chk(d.cycleBegin().ok && d.state() == DeviceState::Enabled, "cycleBegin() 推进 Ready → Enabled");

    chk(!d.rtGoto(DeviceState::Ready, DeviceState::Enabled),
        "expected 不匹配时 CAS 失败（当前已是 Enabled）");
    chk(d.rtGoto(DeviceState::Enabled, DeviceState::Fault), "Enabled → Fault 成功");
    /* 这一条要的是「expected 对得上、但迁移本身非法」——上面那条测的是不匹配，
     * 别混为一谈。当前在 Fault，而 Fault → Enabled 非法。 */
    chk(!d.rtGoto(DeviceState::Fault, DeviceState::Enabled),
        "expected 匹配但迁移非法（Fault → Enabled）被拒");
    chk(d.state() == DeviceState::Fault, "被拒后仍在 Fault");

    /* ---- 5. lastStatus 的 state 必须属实 ---- */
    std::printf("\nlastStatus 带出的 state：\n");
    d.setState(DeviceState::Ready);
    {
        const HalStatus ls = d.lastStatus();
        chk(ls.ok && ls.state == DeviceState::Ready,
            "lastStatus().state 与设备真实状态一致");
    }

    std::printf("\n%s（%d 处失败）\n", fails ? "有问题" : "全部通过", fails);
    return fails ? 1 : 0;
}
