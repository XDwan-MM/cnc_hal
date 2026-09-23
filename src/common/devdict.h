#pragma once

/**
 * @brief 设备字典 JSON 读取器（自包含单文件，无 cJSON 依赖）
 *
 * 从 EEPROM 读到 厂商号/产品号/版本号 后查字典，得到设备类型 / 名称 / 对象映射——
 * 替代写死的类型匹配表，新设备加条目不改代码。
 *
 * 格式（v1，完整样例见 `Greemaster/devices.json`）：
 *   { "version": 1, "devices": [ { "vendor_id": …, "product_code": …,
 *       "revision": …, "type": …, "profile": …, "name": …, "objects": {…} } ] }
 *
 * - revision 可为数字或 "*"（通配）；缺省视为 "*"
 * - type 为 servo / gree6 / gree4 / panel / io / io_expansion
 * - profile 描述对象映射策略，三选一：
 *     "ds402"  用内置标准 DS402 对象号，不得写 objects
 *     "custom" 用本条的 objects 段（语义角色 → 实际对象号），必须写
 *     "none"   本字典不提供对象映射（面板/IO 按位置建句柄，非标多轴在代码里）
 * - "index" 支持 "0x2000" 字符串（推荐）或十进制数字
 * - objects 的键必须是 DevDictRole 的角色名；写错、重复即 Load 失败
 * - bits/dir 缺省按角色确定；显式给值必须一致。custom servo 必须包含 mode_display
 * - version 必须为 1；整数不接受负数、溢出、小数或指数。失败清空旧字典
 * - **index 不是 PDO offset** —— 它只是对象号，实际句柄仍要从 PDO 映射里查。
 *
 * 实现：内置最小 JSON 解析器（支持 // 注释），无堆常驻，启动时 Load 一次，单线程。
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ==================== 语义角色（与 MhServoEntries 一一对应） ====================
typedef enum {
    DEV_DICT_ROLE_STATUS_WORD = 0,
    DEV_DICT_ROLE_CONTROL_WORD,
    DEV_DICT_ROLE_TARGET_POS,
    DEV_DICT_ROLE_ACTUAL_POS,
    DEV_DICT_ROLE_TARGET_SPEED,
    DEV_DICT_ROLE_ACTUAL_SPEED,
    DEV_DICT_ROLE_ACTUAL_TORQUE,
    DEV_DICT_ROLE_TARGET_TORQUE,
    DEV_DICT_ROLE_MODE_DISPLAY,
    DEV_DICT_ROLE_ERROR_CODE,
    DEV_DICT_ROLE_OP_MODE,
    DEV_DICT_ROLE_COUNT
} DevDictRole;

/** @brief 角色名 → 枚举；未识别返回 DEV_DICT_ROLE_COUNT */
DevDictRole DevDict_RoleFromString(const char* role);

// ==================== 查询结果 ====================
#define DEV_DICT_MAX_OBJECTS 11
#define DEV_DICT_NAME_LEN 64
#define DEV_DICT_TYPE_LEN 16
#define DEV_DICT_PROFILE_LEN 16

typedef struct {
    DevDictRole role;
    uint16_t index;          // 对象字典索引（如 0x2000）
    uint8_t  sub;            // 子索引
    uint8_t  bits;           // 位长（8/16/32）
    int      is_tx;          // 1=Tx（从站→主站） 0=Rx
} DevDictObject;

typedef struct {
    char type[DEV_DICT_TYPE_LEN];        // "servo"/"panel"/"io"（调用方映射到 MhDeviceType）
    char name[DEV_DICT_NAME_LEN];        // 设备名称
    char profile[DEV_DICT_PROFILE_LEN];  // "ds402"/"custom"
    DevDictObject objects[DEV_DICT_MAX_OBJECTS];
    int object_count;
} DevDictEntry;

// ==================== API ====================

/**
 * @brief 加载并解析字典文件（启动时一次；重复调用会先释放上次结果）
 * @param err    可选，失败时带回原因（含条目序号/字段名，便于排查手编文件）
 * @return 0 成功；-1 失败（文件不存在/JSON 语法错/字段校验错/超容量）
 */
int  DevDict_Load(const char* path, char* err, int errLen);

/**
 * @brief 查询设备（匹配规则：先 (vendor,product,revision) 精确，再 revision="*" 通配）
 * @return 1=命中（out 已填充） 0=未命中（out 不变）
 */
int  DevDict_Lookup(uint32_t vendor, uint32_t product, uint32_t revision,
                    DevDictEntry* out);

/**
 * @brief 在已命中的设备条目里按语义角色查对象。
 * @return 1=命中（out 已填充） 0=该角色未配置或参数为空。
 * @note 封装侧用它把 JSON 的 index/sub 与 ENI/PDO 解析结果比对；它不返回
 *       GM Entry 句柄，也不替代对实际 PDO 布局的解析。
 */
int  DevDict_FindObject(const DevDictEntry* entry, DevDictRole role,
                        DevDictObject* out);

/** @brief 已加载的条目数（诊断用；未加载为 0） */
int  DevDict_EntryCount(void);

/** @brief 释放内部数据（退出时调用；不调用也无泄漏危害——静态存储） */
void DevDict_Unload(void);

#ifdef __cplusplus
}
#endif
