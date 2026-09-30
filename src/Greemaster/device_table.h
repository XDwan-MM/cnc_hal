#pragma once

#include "device.h"
#include "export.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 按【槽】索引的设备表 —— 本层对 HAL 的取数口。
 *
 * 按槽不按从站：一台从站可含多根轴（格力 axis6 占 6 槽），HAL 关心的是轴。
 * 装配后建表一次，之后只读。带 tag 和身份——这两样正是 g_device_data 那个
 * 无 tag union 缺的：光看字节分不出是伺服、面板还是 IO 模块。 */

#define DEV_SLOT_NAME_LEN 64

typedef struct {
    DEVICE_TYPE type;        /* UNKNOWN_TYPE = 该槽未装配 */
    int         slave_pos;   /* 挂在哪个从站上（多轴时不从槽号反推） */
    int         axis_index;  /* 从站内 0 起；非伺服为 -1 */

    uint32_t vendor_id;      /* 以下四项来自 EEPROM */
    uint32_t product_code;
    uint32_t revision;
    uint32_t serial;
    char     name[DEV_SLOT_NAME_LEN];   /* 设备字典里的名称；未命中为空串 */

    /* Entry 句柄快照，按 type 取 entries.slave / .control / .io。装配时填好，之后不变。 */
    device_data_t entries;
} DeviceSlot;

/** @brief 建表。装配完成后调一次（device_match 之后、握手之前）。
 *  @param slave_num  从站数
 *  @return 实际槽数（<= MAX_DEVICE_NUM）；负数表示容量或参数错误 */
int DeviceTable_Build(int slave_num);
void DeviceTable_Reset(void);
int DeviceTable_IsServo(int slot);

/** @brief 取表（只读）。
 *  @param out  带出表首地址；未建表时为 NULL
 *  @return 槽数 */
MASTER_API int DeviceTable_Get(const DeviceSlot** out);

#ifdef __cplusplus
}
#endif
