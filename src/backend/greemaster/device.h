#ifndef _DEVICE_H
#define _DEVICE_H
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "/opt/GreeMaster/include/libGREEMASTER/GreeMasterAPI.h"
#include "/opt/GreeMaster/include/libGREEMASTER/gm_errcode.h"

typedef struct{
   int slave_pos;
   int slave_total;
   uint32_t ID;
   uint32_t CODE;
   uint32_t Revision;
   uint32_t Serial;
   int type;
}DEVICE_BASIC_INFO;

#define EEPROM_FIXED_WORDS      64  // Word 数（128 Bytes）
#define WORD_TO_BYTES(w)        ((w) * 2)
#define TYPE_END                0xFFFF
#define TYPE_TX_PDO             0x0032
#define TYPE_RX_PDO             0x0033
#define TYPE_SyncM              0x0029
#define DC_MODE                 0x003C
#define MAX_PDO_COUNT           10   // 最多有效 PDO Entry 数
#define MAX_ENTRY_PER_PDO       20   // 每个 PDO 最多条目数

typedef struct {
    unsigned short int type;      // 数据类型 (Word 1)
    unsigned short int length;    // 内容长度：**单位 Word**，字节数 = 该值 × 2
} ECAT_EEPROM_CLASS_HEADER;

// PDO 块头，8 Bytes：PDO_ENTRY(2B) EntryCount(1B) SM(1B) DCRef(1B) NameIdx(1B) Flags(2B)
typedef struct {
    unsigned short int pdoEntry;      // PDO Entry 值 (如 0x1B00)
    unsigned char entryCount;    // 条目数目 (如 12)
    unsigned char smIndex;       // 同步管理器 SM (如 3)
    unsigned char dcRef;         // DC 参考
    unsigned char nameIdx;       // Name IDx
    unsigned short int flags;         // Flags
} PDO_BLOCK_HEADER;

// 条目，每个 8 Bytes
typedef struct {
    unsigned short int index;         // 对象字典索引
    unsigned char subIndex;      // 子索引
    unsigned char nameIdx;       // NameIdx
    unsigned char dataType;      // 数据类型
    unsigned char bitLen;        // 位长
    unsigned short int flags;         // Flags
} PDO_ENTRY_ITEM;

// 解析结果容器
typedef struct {
    PDO_BLOCK_HEADER headerInfo; 
    PDO_ENTRY_ITEM entries[MAX_ENTRY_PER_PDO];  
    int entryCount;                // 实际条目数
} VALID_PDO_INFO;

typedef struct {
    VALID_PDO_INFO validPdos[MAX_PDO_COUNT];
    int count; // 有效 PDO 数
} PDO_RESULT;

/* 同步管理器块 (SM0 - SM3)，1 字节对齐 */
#pragma pack(push, 1)  

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

#pragma pack(pop)  



typedef enum {
    SERVO_TYPE = 1,
    GREE_AXIS6_TYPE,
    GREE_AXIS4_TYPE,
    CONTROL_PANEL_TYPE,
    IO_MODEL_TYPE,
    IO_EXPANSION_TYPE,
    UNKNOWN_TYPE
} DEVICE_TYPE;

#define MAX_DEVICE_NUM 30   // 从站/槽容量上限
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

// 所有设备数据结构的容器（按 type 取成员）
typedef union {
    slave_addr      slave;
    control_addr    control;
    io_addr         io;
} device_data_t;

// PDO 对象查找请求与结果
typedef struct {
    uint16_t index;       // pdo_entry_index
    uint8_t subindex;     // pdo_entry_subindex
    uint32_t pdo_pos;     // 匹配结果：PDO 位置
    uint32_t entrypos;    // 匹配结果：条目在 PDO 中的位置
} pdo_lookup_result;

extern device_data_t g_device_data[MAX_DEVICE_NUM];  // 按【槽】索引
// 待查对象的目标描述
typedef struct {
    uint16_t index;
    uint8_t subIndex;
    uint8_t bitLen;
} TARGET_ENTRY;

/* 一台从站占几个槽（多轴驱动器占多个：axis6→6，axis4→4）。
 * 装配与建表必须走同一规则；各写一遍会整体错位且不报错。 */
int device_slot_count(DEVICE_TYPE type);
void device_reset(void);

/* 某台从站的 EEPROM 身份（按【从站号】索引）；越界返回全零静态对象。 */
const DEVICE_BASIC_INFO* device_identity_get(int slave_pos);

int get_device_info_from_eeprom(int slave_num, DEVICE_TYPE* types,
                              uint32_t cycle_us, int dc_enable);
int device_match(Slave_info* list, DEVICE_TYPE* type, int slave_num);
const char* device_bind_error(void);
#endif
