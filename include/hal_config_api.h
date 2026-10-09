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
#define HAL_ERROR_UNSUPPORTED 0x0105
/* 轴的实时状态没到位。以前这些情况一律回 NOT_RUNNING，上层分不出"没使能"
 * 和"模式还没切过来"——而这两件事的处理方式完全不同（一个要等使能，一个要等模式）。 */
#define HAL_ERROR_NOT_ENABLED   0x0106
#define HAL_ERROR_MODE_MISMATCH 0x0107

/* hal_rt_axis_readiness 的位。用宏不用 enum，与文件内其它取值同一条规矩。 */
#define HAL_READY_PHASE        0x01u  /* 不在 begin 与 commit 之间 */
#define HAL_READY_NOT_ENABLED  0x02u  /* 尚未到 OperationEnabled */
#define HAL_READY_NOT_READY    0x04u  /* 状态机或采样未到位 */
/* 驱动器实际模式（0x6061）还没跟上 HAL 要求的模式——切换还没生效。 */
#define HAL_READY_MODE_ACTUAL  0x10u
/* 0x08 原来是 MODE_DESIRED（"目标模式与配置的工作模式不一致"）。那条判据把
 * "正常切模式"当成了"没就绪"：主轴配置是速度模式，M29/攻丝切到位置模式后它永远为真，
 * 而此刻 write_pos 其实是受理的 —— 与受理条件矛盾，已删。
 * 位号**留空不复用**，免得旧代码读到的 0x08 被当成别的意思。 */

/* 故障复位进展（hal_rt_axis_fault_reset_state 的出参）。
 * 用宏不用 enum：C 的 enum 底层宽度由实现定义，不适合跨 ABI（与 hal_types.h 同一条规矩）。 */
#define HAL_RESET_NONE     0   /* 无复位请求，或被显式停止取消；不表示复位成功 */
#define HAL_RESET_PENDING  1   /* 已请求，正在生成低/高脉冲并等待离开 Fault */
#define HAL_RESET_DONE     2   /* 已完成：状态字已离开 Fault。轴保持未使能 */
#define HAL_RESET_TIMEOUT  3   /* 超时仍未离开 Fault。轴保持未使能 */

/* 主轴定向进展（hal_rt_spindle_orient_state 的出参）。
 * 定向期间由 HAL 每拍下发厂商控制字，普通 DS402 推进暂停。 */
#define HAL_ORIENT_NONE     0   /* 未请求；普通速度/位置控制照常 */
#define HAL_ORIENT_RUNNING  1   /* 已请求，等状态字的 done 位 */
#define HAL_ORIENT_DONE     2   /* done 位置起。保持控制字直到 release */

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
