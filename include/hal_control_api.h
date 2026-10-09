#pragma once
#include "hal_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 非周期面 —— 生命周期、设备查询、映像尺寸、总线健康。
 *
 * 与周期面（hal_rt_api.h）的分工：那边由单个 RT 线程按三拍串行调用；这里都是非周期的，
 * 启动前后、运行中都能调。但**不保证线程安全**，分三种情况：
 *
 *   - hal_context_request_stop() 例外，明确允许与周期调用并发（它只写原子通知）。
 *   - 查询类（identity / capability / io_image_size / bus_health / slave_*）读的是周期
 *     线程正在写的快照，**并发调用时须由调用方自己同步**。
 *   - 生命周期类（create / start / stop / destroy）进程内串行；stop / destroy 之前
 *     **必须确认所有周期调用已经退出**——本模块不做这个等待，也不做检测。
 *
 * 公共返回码（每个函数都可能返回，下面不再逐个重复）：
 *   HAL_ERROR_ARGUMENT    context / 出参为空，轴号、从站号或 function 不存在或越界
 *   HAL_ERROR_NOT_RUNNING 未 start，或该查询要的信息得等 start 认领设备之后才有
 *   HAL_ERROR_BUSY        已有启动中的 context 占着主站（主站是进程全局单例）
 *   HAL_ERROR_TIMEOUT     start 的总预算耗尽（配置里的 start_timeout_ms）
 *   HAL_ERROR_BUS         主站启动/关闭失败，或底层健康查询失败
 *
 * 两个「返回数量」的函数不返回错误码，而是**负数形式的错误码**——因为非负值本身有含义
 * （0 是合法数量）：hal_axis_count()、hal_slave_count()。例如 c 为空返回
 * -HAL_ERROR_ARGUMENT，未启动返回 -HAL_ERROR_NOT_RUNNING。
*/

/**
 * @brief 校验静态配置、分配并初始化 context
 *
 * **不访问硬件，不启动主站。** 只做配置校验与内存分配，所以失败时没有任何资源残留，
 * 改完配置可以直接重来。配置按值整体拷进 context，调用方之后可以释放自己那份。
 *
 * 出参 `*out` 在成功之前一律被置空，失败时不留半成品。
 *
 * @param config  配置，按值拷贝；abi_major / abi_minor / struct_size / reserved 必须正确
 * @param out     带出 context 句柄，成功前被置空
 * @param err     错误文本缓冲，可为空
 * @param err_len err 的长度；非空且 > 0 时始终以 NUL 结尾
 * @return int32_t 0 = 成功；HAL_ERROR_ABI = ABI 版本/结构大小/保留字段不符；
 *                 HAL_ERROR_CONFIG = 静态校验不过；HAL_ERROR_MEMORY = 分配失败；其余见文件头
 */
int32_t hal_context_create(const HalCConfig* config, HalContext** out, char* err, uint32_t err_len);

/**
 * @brief 启动主站：起主站 → 扫从站 → 按设备字典与配置认领设备/PDO → 算 X/Y 映像尺寸
 *
 * **唯一会碰主站的地方。** 任何一步失败都整体回滚（关主站、清运行状态），不留半启动的
 * context，所以失败后可以直接改了配置重试。
 *
 * 主站是**进程全局单例**：同一时刻只允许一个启动中的 context，重复 start 返回
 * HAL_ERROR_BUSY。这一步才知道真实轴数、PDO 位宽、以及 X/Y 地址段是否重叠——这三样
 * create 的静态校验看不到，所以这里会再查一遍，不通过同样拒绝启动。
 *
 * 整个 start 的耗时受配置里的 `start_timeout_ms` 约束，超了返回 HAL_ERROR_TIMEOUT。
 *
 * @param c       已 create 的 context
 * @param err     错误文本缓冲，可为空
 * @param err_len err 的长度
 * @return int32_t 0 = 成功；HAL_ERROR_BUSY = 主站已被占用；
 *                 HAL_ERROR_CONFIG = 设备字典加载失败、轴/IO 找不到匹配设备、X/Y 段重叠，或位长越界；
 *                 HAL_ERROR_UNSUPPORTED = 认到了设备但必需 PDO 位长不符合 CiA402（或主轴缺速度 PDO）；
 *                 HAL_ERROR_TIMEOUT = 启动总预算耗尽；HAL_ERROR_BUS = 主站启动失败；其余见文件头
 */
int32_t hal_context_start(HalContext* c, char* err, uint32_t err_len);

/**
 * @brief 请求停止：只写通知，**不释放任何资源**
 *
 * 可与周期调用**并发**，这是本文件唯一明确允许并发的一类调用。
 *
 * 同时置两份通知：HAL 自己那份让后续接口返回 HAL_ERROR_STOPPED；驱动层那份让阻塞中的
 * hal_rt_wait_cycle() 提前退出。只置前者的话，PDO 线程若正卡在等主站同步里，得白等一个
 * `cycle_timeout_ms` 才醒。
 *
 * 收到之后仍须走 stop / destroy 才算收尾——本函数不关主站、不释放内存。
 *
 * @param c 已 create 的 context
 * @return int32_t 0 = 成功；HAL_ERROR_ARGUMENT = c 为空
 */
int32_t hal_context_request_stop(HalContext* c);

/**
 * @brief 关主站并清运行状态；重复调用无害
 *
 * **调用方必须先确认所有周期调用已经退出**，本函数既不等待也不检测。若 context 不是当前
 * 占用主站的那个（例如从没 start 过、已被别的 context 接管），只清状态、不关主站。
 *
 * @param c 已 create 的 context
 * @return int32_t 0 = 成功；HAL_ERROR_BUS = 关主站失败；HAL_ERROR_ARGUMENT = c 为空
 */
int32_t hal_context_stop(HalContext* c);

/**
 * @brief stop + free，一步收尾
 *
 * @param c 可为 NULL；非空时先 stop 再释放
 */
void hal_context_destroy(HalContext* c);

/**
 * @brief 逻辑轴号 → HalAxisId
 *
 * 当前实现是「校验存在后原样返回」（两者暂时相等）。留这个口子是为了让
 * 「逻辑轴号 ≠ 装配下标」将来有个落点：真加映射表时不必改调用方。
 *
 * 失败时 `*out` 被置成 HAL_INVALID_ID。
 *
 * @param c            已 create 的 context
 * @param logical_axis 逻辑轴号，取值 [0, 32)
 * @param out          带出 HalAxisId
 * @return int32_t 0 = 成功；HAL_ERROR_ARGUMENT = 越界，或该轴未配置
 */
int32_t hal_axis_resolve(const HalContext* c, int32_t logical_axis, HalAxisId* out);

/**
 * @brief 可寻址的逻辑轴数量
 *
 * 只数 `logical_axis >= 0` 的轴，**主轴也算在内**，装配下标为负的轴不算。
 * 读的是配置，不需要 start。
 *
 * @param c 已 create 的 context
 * @return int32_t >= 0 轴数；负数 = 错误码取负（-HAL_ERROR_ARGUMENT = c 为空）
 */
int32_t hal_axis_count(const HalContext* c);

/**
 * @brief 读一根轴的身份（厂商 / 产品 / 修订 / 序列号 / 设备名 / 同型号序号）
 *
 * 身份是 **start 认领设备时才填的**，所以未启动返回 HAL_ERROR_NOT_RUNNING。
 *
 * @param c   已 start 的 context
 * @param id  逻辑轴号
 * @param out 带出身份
 * @return int32_t 0 = 成功；HAL_ERROR_NOT_RUNNING = 还没 start；
 *                 HAL_ERROR_ARGUMENT = 轴号不存在，或 out 为空
 */
int32_t hal_device_identity(const HalContext* c, HalAxisId id, HalCIdentity* out);

/**
 * @brief 查询一根已配置轴在某个功能上的能力
 *
 * **只读 start 时的绑定结果，不访问总线。** 拿不到的能力不算错误：返回 0，但把 `state`
 * 置成 HAL_CAP_UNSUPPORTED，并在 `reason` 里写明缺什么（例如"缺少实际速度或目标速度
 * PDO"）。所以**判断能不能用要看 state，不能只看返回值**。
 *
 * @param c        已 start 的 context
 * @param id       逻辑轴号
 * @param function HAL_FUNC_POSITION / HAL_FUNC_SPEED / HAL_FUNC_ERROR_CODE
 *                 （HAL_FUNC_ALARM_CONTROL 归 hal_slave_capability，这里会被拒）
 * @param out      带出能力
 * @return int32_t 0 = 成功（**state 可能仍是 UNSUPPORTED**）；
 *                 HAL_ERROR_NOT_RUNNING = 还没 start；HAL_ERROR_ARGUMENT = 轴号或 function 非法
 */
int32_t hal_axis_capability(const HalContext* c, HalAxisId id, uint32_t function,
                            HalCCapability* out);

/**
 * @brief 从站数量
 *
 * 数量是 start 扫出来的，未启动返回负的 NOT_RUNNING。
 *
 * @param c 已 start 的 context
 * @return int32_t >= 0 从站数；负数 = 错误码取负
 */
int32_t hal_slave_count(const HalContext* c);

/**
 * @brief 输入/输出映像的实际字节数
 *
 * 这是 **start 时按各设备实际装配到的 Entry 位长算出来的**，调用方不用自己推：
 * x_size = 各 X 段（起始地址 + 段长）的最大值，y_size 同理。
 *
 * 为什么需要它：调用方拿到的是"一整块映像"，但要往自己的寄存器区拷贝时得知道拷多少。
 * 以前只能照抄一个硬编码的数字，设备一改就静默错位。有了它，段长完全由设备决定，
 * 调用方与 HAL 不会再有第二套理解。
 *
 * 未 start 时返回 HAL_ERROR_NOT_RUNNING，出参不改。与周期线程并发调用时须由调用方同步。
 *
 * @param c      已 start 的 context
 * @param x_size 带出输入映像字节数
 * @param y_size 带出输出映像字节数
 * @return int32_t 0 = 成功；HAL_ERROR_NOT_RUNNING = 还没 start；HAL_ERROR_ARGUMENT = 空指针
 */
int32_t hal_io_image_size(const HalContext* c, uint32_t* x_size, uint32_t* y_size);

/**
 * @brief 取最近一次收帧的总线健康快照
 *
 * **只记录不判断**——什么算异常、要不要报警由上层决定。字段是逐个从驱动层结构体搬过来的，
 * 不是 memcpy：两边字段类型与顺序都不保证一致，memcpy 会静默错位。
 *
 * 与周期线程并发调用时须由调用方同步（与本文件其余非周期查询同一条约定）。
 *
 * @param c   已 start 的 context
 * @param out 带出健康快照
 * @return int32_t 0 = 成功；HAL_ERROR_NOT_RUNNING = 还没 start；
 *                 HAL_ERROR_BUS = 底层查询失败；HAL_ERROR_ARGUMENT = c 或 out 为空
 */
int32_t hal_bus_health(const HalContext* c, HalCBusHealth* out);

/**
 * @brief 读一个从站的身份信息
 *
 * @param c         已 start 的 context
 * @param slave_pos 从站位置号，取值 [0, hal_slave_count())
 * @param out       带出信息
 * @return int32_t 0 = 成功；HAL_ERROR_NOT_RUNNING = 还没 start；
 *                 HAL_ERROR_ARGUMENT = 位置号为负、越界，或 out 为空
 */
int32_t hal_slave_info(const HalContext* c, int32_t slave_pos, HalCSlaveInfo* out);

/**
 * @brief 查询一个从站在某个功能上的能力
 *
 * 目前只接受 `HAL_FUNC_ALARM_CONTROL`，而且**恒为 HAL_CAP_NOT_CONFIGURED**
 * （reason = "设备未定义报警控制功能"）。这个口子是留给厂商专有报警控制用的，
 * 现在的设备表里还没有设备声明它。同样：**判断看 state，不看返回值**。
 *
 * @param c         已 start 的 context
 * @param slave_pos 从站位置号，取值 [0, hal_slave_count())
 * @param function  目前只接受 HAL_FUNC_ALARM_CONTROL
 * @param out       带出能力
 * @return int32_t 0 = 成功；HAL_ERROR_NOT_RUNNING = 还没 start；
 *                 HAL_ERROR_ARGUMENT = 位置号越界、function 不是 ALARM_CONTROL，或 out 为空
 */
int32_t hal_slave_capability(const HalContext* c, int32_t slave_pos, uint32_t function,
                             HalCCapability* out);

#ifdef __cplusplus
}
#endif
