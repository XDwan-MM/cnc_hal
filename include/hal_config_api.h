#pragma once
#include "hal_types.h"

/* 非零即失败。0x01xx 为运行期/总线类，其余为调用与配置类。 */
#define HAL_OK                0       /* 成功 */
#define HAL_ERROR_ARGUMENT    0x0009  /* 参数无效：context 为空、轴号不存在、入参越界或非有限值 */
#define HAL_ERROR_ABI         0x000A  /* ABI 不匹配：abi_major/minor、struct_size 或 reserved 与头文件不符 */
#define HAL_ERROR_CONFIG      0x0007  /* 配置无效：静态校验不过，或设备/PDO 绑定失败 */
#define HAL_ERROR_NOT_RUNNING 0x0003  /* 尚未就绪：没 start、没 begin，或该操作要求的运行状态还没达成 */
#define HAL_ERROR_STOPPED     0x000B  /* 已 request_stop，或阻塞等待被主动停止打断。请求之后仍须走 stop/destroy */
#define HAL_ERROR_BUS         0x0101  /* 总线或主站失败。运行期首个错误码被闭锁，此后周期调用持续返回它，直到 stop/start */
#define HAL_ERROR_BUSY        0x0008  /* 主站已被占用：本进程已有活着的 context（一个主站不能开两个） */
#define HAL_ERROR_STATE       0x0001  /* 调用顺序错误，phase 不对：write_* 要在 begin 与 commit 之间；wait 要求上一拍已 commit */
#define HAL_ERROR_MEMORY      0x000C  /* 内存分配失败（context 或内部缓冲） */
#define HAL_ERROR_TIMEOUT     0x0104  /* 启动总预算耗尽：等主站/状态机就位超时 */
#define HAL_ERROR_UNSUPPORTED 0x0105  /* 设备不支持该功能：PDO 没绑或位长不合规、定向没配。不静默当成功 */
#define HAL_ERROR_NOT_ENABLED   0x0106  /* 轴尚未使能，或状态机还没到位。写指令被拒，等使能/等状态机 */
#define HAL_ERROR_MODE_MISMATCH 0x0107  /* 运行模式未到位：实际或期望模式不是该指令要求的 CSP（写位置）或 CSV（写转速） */
#define HAL_ERROR_SDO         0x0108  /* SDO 应答失败或数据长度不符 */
#define HAL_READY_PHASE        0x01u  /* 不在 begin 与 commit 之间（调用时机错） */
#define HAL_READY_NOT_ENABLED  0x02u  /* 尚未到 OperationEnabled */
#define HAL_READY_NOT_READY    0x04u  /* 状态机或采样未到位 */
#define HAL_READY_MODE_ACTUAL  0x10u  /* 驱动器实际模式（0x6061）还没跟上 HAL 要求的模式 */

/* 以下为复位状态 */
#define HAL_RESET_NONE     0   /* 无复位请求，或被显式停止取消；不表示复位成功 */
#define HAL_RESET_PENDING  1   /* 已请求，正在生成低/高脉冲并等待离开 Fault */
#define HAL_RESET_DONE     2   /* 已完成：状态字已离开 Fault。轴保持未使能 */
#define HAL_RESET_TIMEOUT  3   /* 超时仍未离开 Fault。轴保持未使能 */

/* 以下为定向状态 */
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
