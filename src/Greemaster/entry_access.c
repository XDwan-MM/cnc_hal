#include "entry_access.h"
#include "device.h"
#include "device_table.h"

/* 取语义角色对应的 Entry 句柄地址。返回 NULL = 该角色没绑。
 * 「没绑」用 bit_length == 0 判定：g_device_data 是零初始化的，
 * 真正绑上的 Entry 位长必在 1..32 之间。 */
static void* servo_entry(int slot, DevDictRole role, int* is_tx) {
    if (!DeviceTable_IsServo(slot)) return NULL;

    switch (role) {
    case DEV_DICT_ROLE_STATUS_WORD:   *is_tx = 1; return &g_device_data[slot].slave.statusWord;
    case DEV_DICT_ROLE_ACTUAL_POS:    *is_tx = 1; return &g_device_data[slot].slave.act_pos;
    case DEV_DICT_ROLE_ACTUAL_SPEED:  *is_tx = 1; return &g_device_data[slot].slave.act_speed;
    case DEV_DICT_ROLE_ACTUAL_TORQUE: *is_tx = 1; return &g_device_data[slot].slave.act_torque;
    case DEV_DICT_ROLE_MODE_DISPLAY:  *is_tx = 1; return &g_device_data[slot].slave.act_mode;
    case DEV_DICT_ROLE_ERROR_CODE:    *is_tx = 1; return &g_device_data[slot].slave.error_code;
    case DEV_DICT_ROLE_CONTROL_WORD:  *is_tx = 0; return &g_device_data[slot].slave.Control_word;
    case DEV_DICT_ROLE_TARGET_POS:    *is_tx = 0; return &g_device_data[slot].slave.target_pos;
    case DEV_DICT_ROLE_TARGET_SPEED:  *is_tx = 0; return &g_device_data[slot].slave.target_speed;
    case DEV_DICT_ROLE_TARGET_TORQUE: *is_tx = 0; return &g_device_data[slot].slave.target_torque;
    case DEV_DICT_ROLE_OP_MODE:       *is_tx = 0; return &g_device_data[slot].slave.Modes_of_operation;
    default:                          return NULL;
    }
}

MASTER_API int Master_ServoRead(int slot, DevDictRole role, uint32_t* out) {
    if (!out) return -1;

    int is_tx = 0;
    TxEntry_Unit* h = (TxEntry_Unit*)servo_entry(slot, role, &is_tx);
    if (!h || !is_tx || h->bit_length == 0) return -1;

    const size_t nbytes = (size_t)((h->bit_length + 7) / 8);
    if (nbytes > sizeof(*out)) return -1;

    *out = 0;
    return (GM_TxPdoEntry_Read(*h, out, nbytes) != 0) ? -1 : 0;
}

MASTER_API int Master_ServoWrite(int slot, DevDictRole role, uint32_t value) {
    int is_tx = 0;
    RxEntry_Unit* h = (RxEntry_Unit*)servo_entry(slot, role, &is_tx);
    if (!h || is_tx || h->bit_length == 0 || h->bit_length > 32) return -1;

    return (GM_RxPdoEntry_Write(*h, value) != 0) ? -1 : 0;
}

/* 词设备的有效词数与种类（面板 / IO 模块的 union 成员不同）。 */
static int io_info(int slot, int* out_n, int* in_n, int* is_panel) {
    const DeviceSlot* slots = NULL;
    const int n = DeviceTable_Get(&slots);
    if (!slots || slot < 0 || slot >= n) return -1;

    switch (slots[slot].type) {
    case CONTROL_PANEL_TYPE:
        *out_n = (int)slots[slot].entries.control.out_count;
        *in_n  = (int)slots[slot].entries.control.in_count;
        *is_panel = 1;
        return 0;
    case IO_MODEL_TYPE:
    case IO_EXPANSION_TYPE:
        *out_n = (int)slots[slot].entries.io.io_out_count;
        *in_n  = (int)slots[slot].entries.io.io_in_count;
        *is_panel = 0;
        return 0;
    default:
        return -1;
    }
}

MASTER_API int Master_IoReadEntry(int slot, int index, uint32_t* value, int* bit_length) {
    if (!value) return -1;

    int out_n = 0, in_n = 0, is_panel = 0;
    if (io_info(slot, &out_n, &in_n, &is_panel) != 0) return -1;
    if (index < 0 || index >= in_n) return -1;

    const TxEntry_Unit h = is_panel ? g_device_data[slot].control.input_addr[index]
                                    : g_device_data[slot].io.io_input_addr[index];
    if (h.bit_length == 0 || h.bit_length > 32) return -1;

    uint32_t v = 0;
    if (GM_TxPdoEntry_Read(h, &v, (size_t)((h.bit_length + 7) / 8)) != 0) return -1;

    *value = h.bit_length == 32 ? v : v & ((1u << h.bit_length) - 1u);
    if (bit_length) *bit_length = (int)h.bit_length;
    return 0;
}

MASTER_API int Master_IoWriteEntry(int slot, int index, uint32_t value) {
    int out_n = 0, in_n = 0, is_panel = 0;
    if (io_info(slot, &out_n, &in_n, &is_panel) != 0) return -1;
    if (index < 0 || index >= out_n) return -1;

    const RxEntry_Unit h = is_panel ? g_device_data[slot].control.output_addr[index]
                                    : g_device_data[slot].io.io_output_addr[index];
    if (h.bit_length == 0 || h.bit_length > 32) return -1;

    /* 按实际位长掩码：多出来的高位不该冲掉邻居。
     * bit_length == 32 时 1u<<32 是未定义行为，单独处理。 */
    const uint32_t mask = (h.bit_length >= 32) ? 0xFFFFFFFFu
                                               : ((1u << h.bit_length) - 1u);
    return (GM_RxPdoEntry_Write(h, value & mask) != 0) ? -1 : 0;
}
