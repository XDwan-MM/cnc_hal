#pragma once

#include <stdint.h>
#include "export.h"
#include "common/devdict.h"   /* DevDictRole */

#ifdef __cplusplus
extern "C" {
#endif

/* Entry 读写包装 —— HAL 取设备实际值 / 下指令用，不必碰裸 GM API。
 * 句柄装配时绑好；没绑的角色返回失败，不读未初始化数据。
 * 进出都是硬件原始值，单位换算归上层。slot 是槽号，不是从站号。 */

/** @brief 读伺服的一个语义角色（如实际位置 0x6064）。值右对齐。
 *  @param slot  槽号
 *  @param role  语义角色（见 DevDictRole）
 *  @param out   带出值
 *  @return 0 成功；< 0 = 越界 / 角色没绑 / 方向不对（只读 Tx 侧）/ 读失败 */
MASTER_API int Master_ServoRead(int slot, DevDictRole role, uint32_t* out);

/** @brief 写伺服的一个语义角色（如目标位置 0x607A）。值先进主站缓冲，
 *         到 Master_CommitCycle() 才真正下发。
 *  @param slot   槽号
 *  @param role   语义角色
 *  @param value  要写的值
 *  @return 0 成功；< 0 = 越界 / 角色没绑 / 方向不对（只写 Rx 侧）/ 写失败 */
MASTER_API int Master_ServoWrite(int slot, DevDictRole role, uint32_t value);

/** @brief 读词设备（面板 / IO 模块）的第 index 个输入 Entry。位长 1~32，值右对齐。
 *  @param slot        槽号
 *  @param index       Entry 序号（0 起）
 *  @param value       带出值
 *  @param bit_length  可选，带出该 Entry 的位长——HAL 打包字节图要用，
 *                     12 位和 16 位必须分得开
 *  @return 0 成功；< 0 = 越界 / 不是词设备 / 序号超范围 / 没绑 / 读失败 */
MASTER_API int Master_IoReadEntry(int slot, int index, uint32_t* value, int* bit_length);

/** @brief 写词设备的第 index 个输出 Entry。值按该 Entry 的实际位长掩码后下发，
 *         多出来的高位不会冲掉邻居。
 *  @param slot   槽号
 *  @param index  Entry 序号（0 起）
 *  @param value  要写的值
 *  @note  只改其中几位的话，调用方自己做读改写——总线按整 Entry 下发。
 *  @return 0 成功；< 0 = 越界 / 不是词设备 / 序号超范围 / 没绑 / 写失败 */
MASTER_API int Master_IoWriteEntry(int slot, int index, uint32_t value);

#ifdef __cplusplus
}
#endif
