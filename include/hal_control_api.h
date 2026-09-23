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

#ifdef __cplusplus
}
#endif
