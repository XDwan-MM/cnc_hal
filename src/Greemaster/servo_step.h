#pragma once

#include <stdint.h>
#include "export.h"
#include "common/Ds402.h"   /* Ds402Request / Ds402ModeSwitch */

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 朝指定请求推进一拍 DS402 状态机，并保证运行模式。
 *  @param slot     槽号（不是从站号）
 *  @param req      往哪推：使能 / 去使能 / 断电 / 清错 / 不动
 *  @param op_mode  期望运行模式（CSP / CSV），仅 REQ_ENABLE 用
 *  @param sw       可选，带出这一拍的状态字（0x6041）
 *  @note  请求每拍传进来，本层不持有；同一个要一直传，直到状态字显示已达成为止
 *         （典型使能约 3~4 拍）。模式不对会自动切——切前先预置目标位置防飞车，
 *         而且模式不对就不上使能。
 *  @return 0 成功；< 0 = 参数、句柄或任一 PDO 读写失败 */
MASTER_API int Master_ServoStep(int slot, Ds402Request req,
                                uint16_t op_mode, uint16_t* sw);

/** @brief 设切模式策略。
 *  @param slot  槽号
 *  @param how   DISABLE_FIRST（默认，退回 SwitchedOn 再改 0x6060）/ IN_PLACE
 *               （使能中直接改，能否生效取决于驱动器，上机确认后再用）
 *  @return 0 成功；< 0 = 槽号越界 */
MASTER_API int Master_ServoSetModeSwitch(int slot, Ds402ModeSwitch how);

/** @brief 取切模式策略。
 *  @param slot  槽号
 *  @param out   带出策略
 *  @return 0 成功；< 0 = 槽号越界或 out 为空 */
MASTER_API int Master_ServoGetModeSwitch(int slot, Ds402ModeSwitch* out);

/* 生命周期阶段清除每槽策略，不与周期调用并发。 */
void Master_ServoReset(void);

#ifdef __cplusplus
}
#endif
