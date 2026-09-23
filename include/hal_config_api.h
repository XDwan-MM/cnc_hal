#pragma once
#include "hal_types.h"

#define HAL_OK                0
#define HAL_ERROR_ARGUMENT    0x0009
#define HAL_ERROR_ABI         0x000A
#define HAL_ERROR_CONFIG      0x0007
#define HAL_ERROR_NOT_RUNNING 0x0003
#define HAL_ERROR_STOPPED     0x000B
#define HAL_ERROR_BUS         0x0101
#define HAL_ERROR_BUSY        0x0008
#define HAL_ERROR_STATE       0x0001
#define HAL_ERROR_MEMORY      0x000C
#define HAL_ERROR_TIMEOUT     0x0104

#ifdef __cplusplus
extern "C" {
#endif

/* 只校验静态配置；实际轴数、PDO 长度与 X/Y 段重叠在 start 时校验。
 * err 可为空；非空且 err_len > 0 时始终以 NUL 结尾。 */
int32_t hal_config_validate(const HalCConfig* config, char* err, uint32_t err_len);
void hal_error_text(int32_t code, char* out, uint32_t out_len);

#ifdef __cplusplus
}
#endif
