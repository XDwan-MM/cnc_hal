#pragma once
#include "hal_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 非周期面：生命周期、设备查询、映像尺寸、总线健康。
 * 与周期面（hal_rt_api.h）分工：那边单 RT 线程按三拍调用；这里启动前后与运行中都可调。
 * 线程：hal_context_request_stop() 可与周期调用并发；SDO 只允许非实时线程调用，
 * 并且 stop/destroy 前须等待 SDO 返回。查询类快照并发须调用方同步；
 * 生命周期类进程内串行，stop/destroy 前还须确认所有周期调用已退出。
 * 公共返回码（下面不再逐个重复）：
 *   HAL_ERROR_ARGUMENT    参数/轴号/从站号/function 非法
 *   HAL_ERROR_NOT_RUNNING 未 start，或信息需 start 认领设备后才有
 *   HAL_ERROR_BUSY        主站已被启动中的 context 占用（进程全局单例）
 *   HAL_ERROR_TIMEOUT     start 总预算耗尽（start_timeout_ms）
 *   HAL_ERROR_BUS         主站启停失败，或底层健康查询失败
 * 两个计数函数（hal_axis_count / hal_slave_count）返回**负数形式的错误码**：0 是合法数量。
 * 参数约定：id = 逻辑轴号；c 为空一律 HAL_ERROR_ARGUMENT。 */

/**
 * @brief 校验静态配置、分配并初始化 context；不访问硬件、不启动主站
 * 配置按值整体拷入 context（调用方可释放自己那份）；出参 *out 成功前一律置空。
 * @param config  配置；abi_major / abi_minor / struct_size / reserved 必须正确
 * @param out     带出 context 句柄
 * @param err     错误文本缓冲，可为空
 * @param err_len err 长度；非空且 > 0 时始终以 NUL 结尾
 * @return 0 = 成功；HAL_ERROR_ABI = ABI 版本/结构大小/保留字段不符；
 *         HAL_ERROR_CONFIG = 静态校验不过；HAL_ERROR_MEMORY = 分配失败；其余见文件头
 */
int32_t hal_context_create(const HalCConfig* config, HalContext** out, char* err, uint32_t err_len);

/**
 * @brief 启动主站：起主站 -> 扫从站 -> 按设备字典认领设备/PDO -> 算 X/Y 映像尺寸
 * 任何一步失败整体回滚（不留半启动 context）。主站是进程全局单例，已有启动中的 context 时
 * 返回 HAL_ERROR_BUSY。真实轴数、PDO 位宽、X/Y 段重叠在此复查（create 的静态校验看不到）。
 * 耗时受 start_timeout_ms 约束。
 * @param err / err_len 错误文本缓冲，同 hal_context_create
 * @return 0 = 成功；HAL_ERROR_BUSY = 主站被占用；HAL_ERROR_TIMEOUT = 预算耗尽；
 *         HAL_ERROR_CONFIG = 字典加载失败、轴/IO 无匹配设备、X/Y 段重叠或位长越界；
 *         HAL_ERROR_UNSUPPORTED = 必需 PDO 位长不符 CiA402（或主轴缺速度 PDO）；
 *         HAL_ERROR_BUS = 主站启动失败；其余见文件头
 */
int32_t hal_context_start(HalContext* c, char* err, uint32_t err_len);

/**
 * @brief 请求停止：只写通知，不释放资源；本文件唯一允许与周期调用并发的接口
 * 同时置两份通知：HAL 侧使后续接口返回 HAL_ERROR_STOPPED，驱动层使阻塞中的
 * hal_rt_wait_cycle() 提前退出。收尾仍须 stop / destroy。
 * @return 0 = 成功；HAL_ERROR_ARGUMENT = c 为空
 */
int32_t hal_context_request_stop(HalContext* c);

/**
 * @brief 关主站并清运行状态；重复调用无害
 * 调用方必须先确认所有周期调用已退出（本函数不等待、不检测）。非当前占用主站的 context
 * 只清状态、不关主站。
 * @return 0 = 成功；HAL_ERROR_BUS = 关主站失败；HAL_ERROR_ARGUMENT = c 为空
 */
int32_t hal_context_stop(HalContext* c);

/* 非实时线程的 SDO 邮箱访问；须已 start，且调用方在 stop/destroy 前等待本调用结束。
 * 同一时刻只受理一笔请求，PDO 周期线程不得调用。SDK 单次数据最多 4 字节。
 * timeout_ms 是总等待时间；返回 BUSY/STOPPED/TIMEOUT/SDO/BUS/ARGUMENT。
 * 读失败时 data 保持不变。 */
int32_t hal_sdo_read(HalContext* c, int32_t slave_pos, uint16_t index,
                     uint8_t subindex, void* data, uint32_t size, uint32_t timeout_ms);
int32_t hal_sdo_write(HalContext* c, int32_t slave_pos, uint16_t index,
                      uint8_t subindex, const void* data, uint32_t size, uint32_t timeout_ms);

/**
 * @brief stop + free，一步收尾；c 可为 NULL
 */
void hal_context_destroy(HalContext* c);

/**
 * @brief 逻辑轴号 -> HalAxisId（当前是校验存在后原样返回）
 * 失败时 *out = HAL_INVALID_ID。
 * @param logical_axis 逻辑轴号，取值 [0, 32)
 * @param out          带出 HalAxisId
 * @return 0 = 成功；HAL_ERROR_ARGUMENT = 越界或该轴未配置
 */
int32_t hal_axis_resolve(const HalContext* c, int32_t logical_axis, HalAxisId* out);

/**
 * @brief 可寻址逻辑轴数量（含主轴；装配下标为负的不算）；只读配置，不需 start
 * @return >= 0 轴数；负数 = 错误码取负（-HAL_ERROR_ARGUMENT = c 为空）
 */
int32_t hal_axis_count(const HalContext* c);

/**
 * @brief 读轴身份（厂商/产品/修订/序列号/设备名/同型号序号）；在 start 认领设备时才填
 * @param out 带出身份
 * @return 0 = 成功；HAL_ERROR_NOT_RUNNING = 还没 start；
 *         HAL_ERROR_ARGUMENT = 轴号不存在或 out 为空
 */
int32_t hal_device_identity(const HalContext* c, HalAxisId id, HalCIdentity* out);

/**
 * @brief 查询已配置轴在某功能上的能力；只读 start 的绑定结果，不访问总线
 * 能力不可用不算错误：返回 0 且 state = HAL_CAP_UNSUPPORTED，reason 写明缺什么
 * （判断能不能用要看 state，不能只看返回值）。
 * @param function HAL_FUNC_POSITION / HAL_FUNC_SPEED / HAL_FUNC_ERROR_CODE
 *                 （HAL_FUNC_ALARM_CONTROL 归 hal_slave_capability，这里拒绝）
 * @param out      带出能力
 * @return 0 = 成功（state 可能仍是 UNSUPPORTED）；HAL_ERROR_NOT_RUNNING = 还没 start；
 *         HAL_ERROR_ARGUMENT = 轴号或 function 非法
 */
int32_t hal_axis_capability(const HalContext* c, HalAxisId id, uint32_t function,
                            HalCCapability* out);

/**
 * @brief 从站数量（start 扫出）；未启动返回负的 NOT_RUNNING
 * @return >= 0 从站数；负数 = 错误码取负
 */
int32_t hal_slave_count(const HalContext* c);

/**
 * @brief 输入/输出映像实际字节数；start 时按各设备实际 Entry 位长算出
 * x_size = 各 X 段（起始地址 + 段长）最大值，y_size 同理；调用方按它决定拷贝长度。
 * 未 start 返回 HAL_ERROR_NOT_RUNNING 且出参不改；并发调用须调用方同步。
 * @param x_size 带出输入映像字节数
 * @param y_size 带出输出映像字节数
 * @return 0 = 成功；HAL_ERROR_NOT_RUNNING = 还没 start；HAL_ERROR_ARGUMENT = 空指针
 */
int32_t hal_io_image_size(const HalContext* c, uint32_t* x_size, uint32_t* y_size);

/**
 * @brief 取最近一次收帧的总线健康快照；只记录不判断（异常口径由上层定）
 * 字段逐个从驱动层结构体搬入，不是 memcpy（两边类型与顺序不保证一致）。
 * @param out 带出健康快照
 * @return 0 = 成功；HAL_ERROR_NOT_RUNNING = 还没 start；HAL_ERROR_BUS = 底层查询失败；
 *         HAL_ERROR_ARGUMENT = c 或 out 为空
 */
int32_t hal_bus_health(const HalContext* c, HalCBusHealth* out);

/**
 * @brief 读一个从站的身份信息
 * @param slave_pos 从站位置号，取值 [0, hal_slave_count())
 * @param out       带出信息
 * @return 0 = 成功；HAL_ERROR_NOT_RUNNING = 还没 start；
 *         HAL_ERROR_ARGUMENT = 位置号为负/越界，或 out 为空
 */
int32_t hal_slave_info(const HalContext* c, int32_t slave_pos, HalCSlaveInfo* out);

/**
 * @brief 查询从站在某功能上的能力；目前只接受 HAL_FUNC_ALARM_CONTROL，恒为 HAL_CAP_NOT_CONFIGURED
 * 判断看 state，不看返回值。
 * @param slave_pos 从站位置号，取值 [0, hal_slave_count())
 * @param function  目前只接受 HAL_FUNC_ALARM_CONTROL
 * @param out       带出能力
 * @return 0 = 成功；HAL_ERROR_NOT_RUNNING = 还没 start；
 *         HAL_ERROR_ARGUMENT = 位置号越界、function 非 ALARM_CONTROL，或 out 为空
 */
int32_t hal_slave_capability(const HalContext* c, int32_t slave_pos, uint32_t function,
                             HalCCapability* out);

#ifdef __cplusplus
}
#endif
