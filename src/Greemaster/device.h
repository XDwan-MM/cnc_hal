#ifndef _DEVICE_H
#define _DEVICE_H
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "/opt/GreeMaster/include/libGREEMASTER/GreeMasterAPI.h"
#include "/opt/GreeMaster/include/libGREEMASTER/gm_errcode.h"

// ===============错误检查宏=================
#define CHECK_RC(rc,info,label) \
    do { \
        if ((rc) != 0) { \
            printf("%s,rc = %d\n", info,rc); \
            goto label; \
        } \
    } while(0)
#define CHECK_FUNC(func) do { \
    int ret = (func); \
    if (ret < 0) { \
        fprintf(stderr, "Error in function: %s, rc = %d\n", #func, ret); \
    } \
} while (0)

#define SYNC0_CYCLE 1000000   
#define SYNC0_SHIFT 500000                  
#define SYNC1_CYCLE 0                
#define SYNC1_SHIFT 0 

typedef struct{
   int slave_pos;
   int slave_total;
   uint32_t ID;
   uint32_t CODE;
   uint32_t Revision;
   uint32_t Serial;
   int type;
}DEVICE_BASIC_INFO;

// 宏定义
#define EEPROM_FIXED_WORDS      64  // 【关键修正】64 Words = 128 Bytes
#define WORD_TO_BYTES(w)        ((w) * 2)
// 类型标识
#define TYPE_END                0xFFFF
#define TYPE_TX_PDO             0x0032
#define TYPE_RX_PDO             0x0033
#define TYPE_SyncM              0x0029
#define DC_MODE                 0x003C
#define MAX_PDO_COUNT           10   // 假设最多有10个有效PDOentry
#define MAX_ENTRY_PER_PDO       20   // 假设每个 PDO 最多 20 个条目

// 紧凑结构体 (禁止填充)
typedef struct {
    unsigned short int type;      // 数据类型 (Word 1)
    unsigned short int length;    // 数据内容长度 (Word 2)，注意：此值通常代表**字节数**
} ECAT_EEPROM_CLASS_HEADER;

// 1. PDO 头部结构
// 布局：PDO_ENTRY(2B), EntryCount(1B), SM(1B), DCRef(1B), NameIdx(1B), Flags(2B)
// 总大小：8 Bytes
// 2. PDO 头部结构 (紧跟在分类头之后，8 Bytes)
typedef struct {
    unsigned short int pdoEntry;      // PDO Entry 值 (如 0x1B00)
    unsigned char entryCount;    // 条目数目 (如 12)
    unsigned char smIndex;       // 同步管理器 SM (如 3)
    unsigned char dcRef;         // DC 参考
    unsigned char nameIdx;       // Name IDx
    unsigned short int flags;         // Flags
} PDO_BLOCK_HEADER;

// 3. 条目 (Entry) 结构 (每个 8 Bytes)
typedef struct {
    unsigned short int index;         // 对象字典值
    unsigned char subIndex;      // 子对象字典值
    unsigned char nameIdx;       // NameIdx
    unsigned char dataType;      // 数据类型
    unsigned char bitLen;        // 数据长度/位长
    unsigned short int flags;         // Flags
} PDO_ENTRY_ITEM;

// 4. 结果容器
typedef struct {
    PDO_BLOCK_HEADER headerInfo; // 存储有效 PDO 的头部信息
    PDO_ENTRY_ITEM entries[MAX_ENTRY_PER_PDO];  // 存储有效条目
    int entryCount;                // 实际条目数
} VALID_PDO_INFO;

typedef struct {
    VALID_PDO_INFO validPdos[MAX_PDO_COUNT];
    int count; //有效pdo_entry数
} PDO_RESULT;

/* 同步管理器块总结构体 (包含 SM0 - SM3) */
#pragma pack(push, 1)  // 将对齐方式压栈并设置为 1 字节（紧凑排列）

typedef struct {
    unsigned short int wPhysAddr;      // 2 Bytes
    unsigned short int wDataLen;       // 2 Bytes
    unsigned short int wConfigReg;     // 2 Bytes
    unsigned char bActive;        // 1 Byte
    unsigned char bDirection;     // 1 Byte
} ECAT_SYNC_MANAGER_CFG;

typedef struct {
    ECAT_SYNC_MANAGER_CFG sm0;
    ECAT_SYNC_MANAGER_CFG sm1;
    ECAT_SYNC_MANAGER_CFG sm2;
    ECAT_SYNC_MANAGER_CFG sm3;
} ECAT_SYNC_M_INFO;

#pragma pack(pop)  // 恢复之前的对齐设置



typedef enum {
    SERVO_TYPE = 1,
    GREE_AXIS6_TYPE,
    GREE_AXIS4_TYPE,
    CONTROL_PANEL_TYPE,
    IO_MODEL_TYPE,
    IO_EXPANSION_TYPE,
    UNKNOWN_TYPE
} DEVICE_TYPE;

/* 设备类型不再用硬编码表匹配——改查设备字典（devdict + devices.json，
 * 见 src/Greemaster/devices.json）。匹配逻辑在 get_device_types_from_info()。 */

#define MAX_DEVICE_NUM 30   //假设最多 30 个设备
typedef struct {
    TxEntry_Unit statusWord;
    TxEntry_Unit act_pos;
    TxEntry_Unit error_code;
    TxEntry_Unit act_speed;
    TxEntry_Unit act_torque;
    TxEntry_Unit act_mode;
    RxEntry_Unit Control_word;
    RxEntry_Unit target_pos;
    RxEntry_Unit target_speed;
    RxEntry_Unit Modes_of_operation;
    RxEntry_Unit target_torque;
}slave_addr;

typedef struct {
    RxEntry_Unit output_addr[64];
    TxEntry_Unit input_addr[64];
    uint8_t out_count;
    uint8_t in_count;
}control_addr;

typedef struct {
    RxEntry_Unit io_output_addr[64];
    TxEntry_Unit io_input_addr[64];
    uint8_t io_out_count;
    uint8_t io_in_count;
}io_addr;

// 统一的联合体：所有设备数据结构的“容器”
typedef union {
    slave_addr      slave;
    control_addr    control;
    io_addr         io;
} device_data_t;

// 用于存储查找请求和结果的结构体
typedef struct {
    uint16_t index;       // pdo_entry_index
    uint8_t subindex;     // pdo_entry_subindex
    uint32_t pdo_pos;     // 匹配结果：PDO 位置
    uint32_t entrypos;    // 匹配结果：条目在 PDO 中的位置
} pdo_lookup_result;

extern device_data_t g_device_data[MAX_DEVICE_NUM];  // 假设最多 30 个设备
// 辅助结构体：用于定义需要添加的目标
typedef struct {
    uint16_t index;
    uint8_t subIndex;
    uint8_t bitLen;
} TARGET_ENTRY;

/* 一台从站在 g_device_data 里占几个槽（多轴驱动器占多个：axis6→6，axis4→4）。
 * 装配与建表两处都必须走这个规则，各写一遍就会整体错位，而且不报错。 */
int device_slot_count(DEVICE_TYPE type);
void device_reset(void);

/* 某台从站的 EEPROM 身份（按【从站号】索引）。越界返回一个全零的静态对象。 */
const DEVICE_BASIC_INFO* device_identity_get(int slave_pos);

int get_device_info_from_eeprom(int slave_num, DEVICE_TYPE* types);
int device_match(Slave_info* list, DEVICE_TYPE* type, int slave_num);
#endif

