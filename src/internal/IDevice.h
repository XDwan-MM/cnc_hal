#pragma once

#include "HalTypes.h"

namespace hal {

/**
 * @brief 硬件设备的通用接口 —— 所有设备类型（伺服轴 / 主轴 / DIO /
 *        将来的手轮、模拟量模块…）共用的一套契约。
 *
 * 这一层只表达「任何硬件都必然具备」的东西：身份、生命周期、状态、
 * 周期推进点。设备特有的能力（轴的目标位置、主轴的目标转速、IO 的
 * 字节图）一律留给派生接口，不往上堆。
 *
 * ## 三条调用上下文规则
 *
 * 1. **身份查询与生命周期只在非实时上下文调用**（init/start/stop/
 *    shutdown）：启动期一次，或停机期一次。
 *
 * 2. **cycleBegin() 是唯一的周期推进点**：每设备每周期恰好调用一次，
 *    由 hal_rt_begin_cycle() 遍历设备表驱动。所有读接口都是纯快照——
 *    读一次和读十次结果相同，且不影响任何推进。
 *
 * 3. **state() 是周期内唯一可安全调用的本接口方法**（无锁读）。
 *    lastStatus() 带互斥量，**禁止在周期线程调用**。
 *
 * 第 2 条是本设计相对早期实现的核心修正。早期实现把 DS402 推进藏在
 * getter 里，结果是「不读就不推进」「读两次推进两次」、行为依赖调用
 * 顺序。这条不变量及其注释是防止那个 bug 复现的屏障。
 */
class IDevice {
public:
    virtual ~IDevice() = default;

    // ==================== 身份（非 RT） ====================

    /// 设备标识，形如 "axis.3" / "spindle.0"。指向对象内部存储，
    /// 生命周期同设备对象，调用方不要保存后再跨销毁使用。
    virtual const char* id() const = 0;

    /// 设备类型："servo" / "spindle" / "dio" / …
    virtual const char* type() const = 0;

    /// 驱动名："stub" / "sim" / "ethercat" / …
    virtual const char* driver() const = 0;

    /// 通道号（多通道路由用）
    virtual int channel() const = 0;

    // ==================== 生命周期（非 RT） ====================

    /// 冷启动配置。仅在 start() 之前调用一次。
    virtual HalStatus init() = 0;

    /// 启动设备。由生命周期编排器在装配完成后调用。
    virtual HalStatus start() = 0;

    /// 停止设备。幂等。
    virtual HalStatus stop() = 0;

    /// 强制销毁。无返回值；调用后对象不可再用。
    virtual void shutdown() = 0;

    // ==================== 周期推进点（RT） ====================

    /// 每周期恰好调用一次：推进设备状态机至多一步，并刷新反馈缓存。
    ///
    /// 契约：无锁、无分配、不阻塞、不抛异常。
    /// 无周期语义的设备（如 DIO）给空实现即可。
    ///
    /// @note noexcept 是硬约束，不是修饰。实时线程里抛异常 = terminate，
    ///       这正是想要的失败方式——宁可当场死，不要带着坏状态跑完一个
    ///       周期再去污染机床。
    virtual HalStatus cycleBegin() noexcept = 0;

    // ==================== 状态 ====================

    /// 当前状态（RT 安全：无锁 acquire 读）
    virtual DeviceState state() const = 0;

    /// 最后一次操作的完整状态（含错误码与文本），供日志/报警线程使用。
    ///
    /// @warning 带互斥量，**非 RT**。周期线程只读 state()。
    virtual HalStatus lastStatus() const = 0;

    /// 设置设备状态。
    ///
    /// @warning 这不是「命令设备进入某状态」的手段。状态由驱动事实单向
    ///          推进（poll 到 DS402 OperationEnabled 即 Enabled），上层
    ///          只读，不通过调 enable() 去推它。本方法供设备内部状态机
    ///          与生命周期编排使用；公共 C ABI 不暴露它，实时端够不着。
    virtual HalStatus setState(DeviceState newState) = 0;
};

/**
 * @brief 状态迁移合法性。
 *
 *   Uninitialized ──> Ready ──> Enabled
 *                       ^          │
 *                       └──────────┘        去使能
 *   任意状态 ──> Fault ──> Ready            人工复位
 *   任意状态 ──> Shutdown                   不可逆
 *
 * 同态迁移（from == to）视为合法——状态推进是幂等的。
 *
 * 提成自由函数（而不是塞在 setState 的 switch 里）是为了让「不许非法
 * 跳转」不需要构造任何设备就能穷举测试：25 种组合一张表跑完。
 */
bool isLegalTransition(DeviceState from, DeviceState to) noexcept;

} // namespace hal
