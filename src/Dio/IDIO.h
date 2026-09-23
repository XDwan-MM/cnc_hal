#pragma once

#include "internal/ConfigModel.h"
#include "internal/IDevice.h"

#include <cstdint>

namespace hal {

/**
 * @brief 数字量 IO 接口 —— 覆盖普通 IO 模块与机床操作面板。
 *
 * 两者物理上都是总线上的数字量从站，HAL 对它们的处理完全一致：把输入
 * 搬进字节图、把输出字节图搬出去。**HAL 不知道某一"位"是急停按钮还是
 * 冷却泵继电器**——那是机床语义，归上层（见 docs/decisions.md 的边界
 * 判据：删掉整个实时端业务层，这一位还有意义吗？没有 → 上层的）。
 *
 * ## 字节图契约
 *
 * 模块的全部输入/输出按 **Entry 声明顺序、LSB 在前** 位打包进一段连续
 * 字节图：**bit i 就是第 i 个点位**。
 *
 *   - Entry 位宽 1~32 全支持：1 位离散量 = 1 点，16 位 = 2 字节，
 *     12 位 = 1.5 字节（跨字节，不要求对齐）。
 *   - 字节序固定小端（低字节 → 低 X 地址）。字交换的厂商差异在**设备
 *     字典层**处理，不在这里开洞。
 *   - 打包是纯函数，实现见 `src/internal/EntryBits.h`，可独立测试。
 *
 * 本接口只暴露**模块局部**的字节图。X/Y 是**字节**地址，到 PLC 域的
 * 段映射由配置模块在启动期建好（`HalIoMap`），不在热路径里算。
 *
 * ## IO 不参与使能
 *
 * 本接口**没有 `enable()`**。使能序列是运动安全语义——它存在的理由是
 * 「上电时电机不能自己转」。IO 没有这个语义，`start()` 之后立即可读可
 * 写。这是设备矩阵里唯一的例外，不是遗漏。
 */
class IDIO : public virtual IDevice {
public:
    // ==================== 冷启动（非 RT） ====================

    /// 加载已解析的槽配置。仅在 start() 之前调用一次。
    virtual HalStatus configure(const DioCfgDev& cfg) = 0;

    // ==================== 规模（非 RT） ====================

    /// 模块输入字节数。由 Entry 布局决定，装配时确定，之后不变。
    virtual int inputBytes() const = 0;

    /// 模块输出字节数
    virtual int outputBytes() const = 0;

    // ==================== 热路径 ====================

    /// 读模块输入字节图。buf 容量须 >= inputBytes()。
    ///
    /// 纯快照：本周期 `begin_cycle` 时总线已经收完，这里只是把已到的
    /// 映像拷出来。读一次和读十次结果相同，且不影响任何推进——所以是
    /// `const`。
    virtual HalStatus readInputs(uint8_t* buf, int len) const = 0;

    /// 写输出字节图。values 与 mask 均为 len 字节，**mask 位 = 1 表示
    /// 该位生效**，为 0 的位保持原值不变。
    ///
    /// 之所以以掩码为原语、而不是提供"整段覆写"：总线是按**整 Entry**
    /// 下发的，而一个 Entry 可能同时装着属于不同逻辑设备的点位。写半个
    /// Entry 时驱动以影子寄存器为当前值做读改写，否则会把邻居的位冲掉。
    /// 需要整段覆写就传全 0xFF 的 mask。
    virtual HalStatus writeOutputs(const uint8_t* values,
                                   const uint8_t* mask,
                                   int len) = 0;

    /// 读输出**影子**字节图——即 HAL 认为自己已经下发的值，不是回读结
    /// 果。供 HMI 与日志使用，非热路径。
    virtual HalStatus readOutputs(uint8_t* buf, int len) const = 0;

    // ==================== 周期推进点 ====================

    /// IO 无周期语义：输入在总线 Receive 时就已到位，输出在 Send 时统一
    /// 下发，中间不需要逐拍推进状态机。这里给空实现而不是让每个驱动各
    /// 写一遍。
    ///
    /// 之所以仍然存在，是因为 `IDevice` 要求「每设备每周期恰好调用一次
    /// cycleBegin()」，好让 hal_rt_begin_cycle() 用**一个循环**遍历设备
    /// 表。有周期语义的设备重写它，没有的用这个默认实现。
    HalStatus cycleBegin() noexcept override { return HalStatus::success(); }
};

} // namespace hal
