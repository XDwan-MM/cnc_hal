#pragma once

#include "device.h"
#include <stdio.h>

/* 启动期诊断快照：身份来自 EEPROM，PDO 链表是下发前构造的方案。
 * 输出不宣称 PDO 已经启用；调用方负责打开/关闭文件。 */
int TopologySnapshot_Write(FILE* out, int slave_count, const Slave_info* list);
