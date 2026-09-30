#pragma once
#include "hal_types.h"

#ifdef __cplusplus
extern "C" {
#endif

int32_t hal_context_create(const HalCConfig*, HalContext**, char* err, uint32_t err_len);
int32_t hal_context_start(HalContext*, char* err, uint32_t err_len);
/* 可与周期调用并发；只通知，资源保留至 stop。 */
int32_t hal_context_request_stop(HalContext*);
/* stop/destroy 前调用方必须等待全部周期调用退出；所有 context 的生命周期调用串行。 */
int32_t hal_context_stop(HalContext*);
void hal_context_destroy(HalContext*);
int32_t hal_axis_resolve(const HalContext*, int32_t logical_axis, HalAxisId* out);
/* 返回非负数量，参数错误返回负数。 */
int32_t hal_axis_count(const HalContext*);
int32_t hal_device_identity(const HalContext*, HalAxisId, HalCIdentity*);
/* 查询一根已配置轴上的功能能力；只读启动时的绑定结果，不访问总线。 */
int32_t hal_axis_capability(const HalContext*, HalAxisId, uint32_t function, HalCCapability*);
int32_t hal_slave_count(const HalContext*);
/* 输入/输出映像的实际字节数。
 *
 * 这是 **start 时按各设备实际装配到的 Entry 位长算出来的**，调用方不用自己推：
 * x_size = 各 X 段（起始地址 + 段长）的最大值，y_size 同理。
 *
 * 为什么需要它：调用方拿到的是"一整块映像"，但要往自己的寄存器区拷贝时得知道
 * 拷多少。以前只能照抄一个硬编码的数字，设备一改就静默错位。有了它，
 * 段长完全由设备决定，调用方与 HAL 不会再有第二套理解。
 *
 * 未 start 时为 0（此时还没枚举设备）。与周期线程并发调用时须由调用方同步。 */
int32_t hal_io_image_size(const HalContext*, uint32_t* x_size, uint32_t* y_size);
/* 取最近一次收帧的总线健康快照。只记录不判断，报警策略归上层。
 * 与周期线程并发调用时须由调用方同步（与本文档其余非周期查询同一条约定）。 */
int32_t hal_bus_health(const HalContext*, HalCBusHealth* out);
int32_t hal_slave_info(const HalContext*, int32_t slave_pos, HalCSlaveInfo*);
int32_t hal_slave_capability(const HalContext*, int32_t slave_pos, uint32_t function, HalCCapability*);

#ifdef __cplusplus
}
#endif
