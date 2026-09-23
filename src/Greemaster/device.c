#include "device.h"
#include "slave_list.h"
#include "common/devdict.h"

static int position = 0;   /* 装配期的槽推进游标，仅本文件用 */

/* 一台从站在 g_device_data 里占几个槽。
 * 多轴驱动器一台占多个槽（axis6 → 6，axis4 → 4），所以"槽号"与"从站号"
 * 是两个不同的量，不能互相替代。COERequestResult()、device_match() 与
 * DeviceTable_Build() 三处推进必须走同一个规则，否则会整体错位且不报错。 */
int device_slot_count(DEVICE_TYPE type) {
    switch (type) {
    case SERVO_TYPE:         return 1;
    case GREE_AXIS6_TYPE:    return 6;
    case GREE_AXIS4_TYPE:    return 4;
    case CONTROL_PANEL_TYPE: return 1;
    case IO_MODEL_TYPE:      return 1;
    default:                 return 0;   /* UNKNOWN_TYPE / IO_EXPANSION_TYPE 不占槽 */
    }
}

/* 字典里的 type 字符串 → DEVICE_TYPE。未识别返回 UNKNOWN_TYPE。 */
static DEVICE_TYPE device_type_from_string(const char* s) {
    if (strcmp(s, "servo") == 0)        return SERVO_TYPE;
    if (strcmp(s, "gree6") == 0)        return GREE_AXIS6_TYPE;
    if (strcmp(s, "gree4") == 0)        return GREE_AXIS4_TYPE;
    if (strcmp(s, "panel") == 0)        return CONTROL_PANEL_TYPE;
    if (strcmp(s, "io") == 0)           return IO_MODEL_TYPE;
    if (strcmp(s, "io_expansion") == 0) return IO_EXPANSION_TYPE;
    return UNKNOWN_TYPE;
}

/* 按 (厂商, 产品, 版本) 查设备字典定类型。字典未命中的设备也会被上报，
 * 类型为 UNKNOWN_TYPE（由调用方决定是否装配）。 */
DEVICE_TYPE get_device_types_from_info(const DEVICE_BASIC_INFO* device_info) {
    DevDictEntry e;
    if (!DevDict_Lookup((uint32_t)device_info->ID,
                        (uint32_t)device_info->CODE,
                        (uint32_t)device_info->Revision, &e)) {
        return UNKNOWN_TYPE;
    }
    return device_type_from_string(e.type);
}

/**
 * @brief 在指定的 PDO_RESULT 中，检查某个条目是否已存在于任何一个 PDO 中
 *
 * @param result_ptr 指向 PDO_RESULT 的指针
 * @param target 目标条目
 * @return int 1: 存在, 0: 不存在
 */
static int is_entry_exists_in_result(PDO_RESULT* result_ptr, const TARGET_ENTRY* target) {
    if (result_ptr == NULL || result_ptr->count == 0) {
        return 0;
    }

    for (int i = 0; i < result_ptr->count; i++) {
        VALID_PDO_INFO* current_pdo = &result_ptr->validPdos[i];
        for (int j = 0; j < current_pdo->entryCount; j++) {
            // 只要 Index 和 SubIndex 匹配，就认为条目存在
            if (current_pdo->entries[j].index == target->index &&
                current_pdo->entries[j].subIndex == target->subIndex) {
                return 1;
            }
        }
    }
    return 0;
}

/**
 * @brief 如果条目不存在，则追加到最后一个 PDO
 *
 * @param result_ptr 指向 PDO_RESULT 的指针
 * @param target 目标条目
 * @return int 1: 成功追加, 0: 已存在或无法追加(无PDO或空间满), -1: 参数错误
 */
static int ensure_entry_in_last_pdo(PDO_RESULT* result_ptr, const TARGET_ENTRY* target) {
    if (result_ptr == NULL || result_ptr->count == 0) {
        return 0; // 没有 PDO 可追加
    }

    // 1. 检查是否已存在
    if (is_entry_exists_in_result(result_ptr, target)) {
        return 0; // 已存在，无需操作
    }

    // 2. 获取最后一个 PDO
    VALID_PDO_INFO* last_pdo = &result_ptr->validPdos[result_ptr->count - 1];

    // 3. 检查空间
    if (last_pdo->entryCount >= MAX_ENTRY_PER_PDO) {
        printf("Warning: Last PDO 0x%04X is full. Cannot add entry %04X:%02X.\n",
            last_pdo->headerInfo.pdoEntry, target->index, target->subIndex);
        return 0;
    }

    // 4. 执行追加
    int idx = last_pdo->entryCount;
    last_pdo->entries[idx].index = target->index;
    last_pdo->entries[idx].subIndex = target->subIndex;
    last_pdo->entries[idx].bitLen = target->bitLen;

    last_pdo->entryCount++;
    last_pdo->headerInfo.entryCount = (unsigned char)last_pdo->entryCount;

    return 1;
}

static int ParsePdosInBlock(const unsigned char* data, size_t bytes,
                            unsigned sm_index, PDO_RESULT* result) {
    PDO_RESULT parsed = *result;
    size_t offset = 0;
    while (offset < bytes) {
        PDO_BLOCK_HEADER header;
        if (bytes - offset < sizeof(header)) return -1;
        memcpy(&header, data + offset, sizeof(header));
        const size_t block_bytes = sizeof(header) +
                                   (size_t)header.entryCount * sizeof(PDO_ENTRY_ITEM);
        if (block_bytes > bytes - offset) return -1;
        if (header.smIndex == sm_index) {
            if (header.entryCount > MAX_ENTRY_PER_PDO ||
                parsed.count >= MAX_PDO_COUNT) return -1;
            VALID_PDO_INFO* pdo = &parsed.validPdos[parsed.count++];
            pdo->headerInfo = header;
            pdo->entryCount = header.entryCount;
            memcpy(pdo->entries, data + offset + sizeof(header),
                   (size_t)header.entryCount * sizeof(PDO_ENTRY_ITEM));
        }
        offset += block_bytes;
    }
    *result = parsed;
    return 0;
}

/* 结束标记只需一个 Word；块长度不含分类头，单位为 Word。 */
static int parse_eeprom_pdos(const unsigned char* data, size_t bytes,
                             PDO_RESULT* tx, PDO_RESULT* rx,
                             ECAT_SYNC_M_INFO* sync, int* have_tx, int* have_rx) {
    size_t offset = WORD_TO_BYTES(EEPROM_FIXED_WORDS);
    if (!data || bytes < offset) return -1;
    while (bytes - offset >= sizeof(uint16_t)) {
        ECAT_EEPROM_CLASS_HEADER header;
        memcpy(&header.type, data + offset, sizeof(header.type));
        if (header.type == TYPE_END) return 0;
        if (bytes - offset < sizeof(header)) return -1;
        memcpy(&header, data + offset, sizeof(header));
        offset += sizeof(header);
        const size_t length = (size_t)header.length * 2;
        if (length > bytes - offset) return -1;
        switch (header.type) {
        case TYPE_SyncM:
            if (length < sizeof(*sync)) return -1;
            memcpy(sync, data + offset, sizeof(*sync));
            break;
        case TYPE_TX_PDO:
            if (ParsePdosInBlock(data + offset, length, 3, tx) != 0) return -1;
            *have_tx = 1;
            break;
        case TYPE_RX_PDO:
            if (ParsePdosInBlock(data + offset, length, 2, rx) != 0) return -1;
            *have_rx = 1;
            break;
        default:
            break;
        }
        offset += length;
    }
    return -1;
}

static int extract_pdos(const Slave_info* slave, uint32_t direction, PDO_RESULT* result) {
    memset(result, 0, sizeof(*result));
    if (!slave) return -1;
    for (const sm_info* sm = slave->sm_list; sm; sm = sm->next) {
        if (sm->sync_index != direction) continue;
        for (const pdo_info* pdo = sm->pdo_list; pdo; pdo = pdo->next) {
            if (result->count >= MAX_PDO_COUNT) return -1;
            VALID_PDO_INFO* dst = &result->validPdos[result->count++];
            dst->headerInfo.pdoEntry = pdo->pdo_index;
            for (const pdo_entry_info* entry = pdo->entry_list; entry; entry = entry->next) {
                if (dst->entryCount >= MAX_ENTRY_PER_PDO) return -1;
                PDO_ENTRY_ITEM* item = &dst->entries[dst->entryCount++];
                item->index = entry->pdo_entry_index;
                item->subIndex = entry->pdo_entry_subindex;
                item->bitLen = entry->pdo_entry_bit_length;
            }
            if ((uint32_t)dst->entryCount != pdo->pdo_entry_num) return -1;
            dst->headerInfo.entryCount = (uint8_t)dst->entryCount;
        }
    }
    return 0;
}

int calculate_tx_bit(PDO_RESULT* TX_pdo_result) {
    int tx_bit = 0;
    for (int i = 0; i < TX_pdo_result->count; i++) {
        for (int j = 0; j < TX_pdo_result->validPdos[i].entryCount; j++) {
            tx_bit = tx_bit + TX_pdo_result->validPdos[i].entries[j].bitLen;
            printf("TX_pdo_result->validPdos[%d].entries[%d].index = %x\n", i,j,TX_pdo_result->validPdos[i].entries[j].index);
        }
    }
    return tx_bit;
}

int calcute_rx_bit(PDO_RESULT* RX_pdo_result) {
    int rx_bit = 0;
    for (int i = 0; i < RX_pdo_result->count; i++) {
        for (int j = 0; j < RX_pdo_result->validPdos[i].entryCount; j++) {
            rx_bit = rx_bit + RX_pdo_result->validPdos[i].entries[j].bitLen;
            printf("RX_pdo_result->validPdos[%d].entries[%d].index = %x\n", i,j,RX_pdo_result->validPdos[i].entries[j].index);
        }
    }
    return rx_bit;
}

int build_chain_from_raw_data(Slave_info** slave_ptr,
    PDO_RESULT* tx_data,
    PDO_RESULT* rx_data,
    ECAT_SYNC_M_INFO* sync_info, int tx_bit, int rx_bit) {
    uint32_t current_sm_bit_offset = 0;
    if (!slave_ptr || !*slave_ptr) return -1;
    Slave_info* current_slave = *slave_ptr;

    if (rx_data->count != 0) {
        // 创建 SM2 (Rx)
        sm_info* sm2_rx = create_empty_sm();
        if (!sm2_rx) return -1;
        sm2_rx->sync_index = 0x1c12;
        sm2_rx->sm_startadress = sync_info->sm2.wPhysAddr;
        sm2_rx->sm_controlbyte = sync_info->sm2.wConfigReg;
        sm2_rx->sm_Enable = 1;
        sm2_rx->sm_TR = 1;
        sm2_rx->sm_len = rx_bit;
        sm2_rx->sm_size_btye = (rx_bit + 7) / 8;
        sm2_rx->pdo_num = rx_data->count;

        // 将 SM_RX 加入 Slave
        add_sm_to_slave(current_slave, sm2_rx);
        // ==========================================
        // 【关键】维护 SM 级别的位偏移量 (Rx 从 0 开始)

        for (int i = 0; i < rx_data->count; i++) {
            VALID_PDO_INFO* pdo_data = &rx_data->validPdos[i];

            uint32_t current_pdo_len_bits = 0;
            for (int k = 0; k < pdo_data->entryCount; k++) {
                current_pdo_len_bits += pdo_data->entries[k].bitLen;
            }

            // 创建 PDO 节点
            pdo_info* new_pdo = create_empty_pdo();
            if (!new_pdo) return -1;
            new_pdo->pdo_index = pdo_data->headerInfo.pdoEntry;
            new_pdo->pdo_len = current_pdo_len_bits;
            new_pdo->pdo_pos = i;
            new_pdo->pdo_entry_num = pdo_data->entryCount;

            // 将 PDO 添加到 SM_RX
            add_pdo_to_sm(sm2_rx, new_pdo);

            uint32_t current_pdo_bit_offset = 0;

            for (int j = 0; j < pdo_data->entryCount; j++) {
                PDO_ENTRY_ITEM* entry = &pdo_data->entries[j];

                if (!add_pdo_entry_to_pdo(new_pdo,
                    entry->index,
                    entry->subIndex,
                    entry->bitLen,
                    current_pdo_bit_offset,
                    current_sm_bit_offset
                )) return -1;

                current_pdo_bit_offset += entry->bitLen;
                current_sm_bit_offset += entry->bitLen;
            }
        }
    }
    // ==========================================
    // 1. 处理 TX PDOs -> 挂载到 sm3 (Sync Index 1, 0x1C13)
    // ==========================================
    // 创建 SM3
    if (tx_data->count != 0) {
        sm_info* sm3_tx = create_empty_sm();
        if (!sm3_tx) return -1;
        sm3_tx->sync_index = 0x1c13;
        sm3_tx->sm_startadress = sync_info->sm3.wPhysAddr;
        sm3_tx->sm_controlbyte = sync_info->sm3.wConfigReg;
        sm3_tx->sm_Enable = 1;
        sm3_tx->sm_TR = 0;
        sm3_tx->sm_len = tx_bit;
        sm3_tx->sm_size_btye = (tx_bit + 7) / 8;
        sm3_tx->pdo_num = tx_data->count;
        add_sm_to_slave(current_slave, sm3_tx);

        // 【关键】维护 SM 级别的位偏移量
        current_sm_bit_offset = 0;

        for (int i = 0; i < tx_data->count; i++) {
            VALID_PDO_INFO* pdo_data = &tx_data->validPdos[i];

            // 创建 PDO 节点
            // 估算 PDO 长度：sum(entry.bitLen)
            uint32_t current_pdo_len_bits = 0;
            for (int k = 0; k < pdo_data->entryCount; k++) {
                current_pdo_len_bits += pdo_data->entries[k].bitLen;
            }

            // 创建 PDO 节点
            pdo_info* new_pdo = create_empty_pdo();
            if (!new_pdo) return -1;
            new_pdo->pdo_index = pdo_data->headerInfo.pdoEntry;
            new_pdo->pdo_len = current_pdo_len_bits;
            new_pdo->pdo_pos = i;
            new_pdo->pdo_entry_num = pdo_data->entryCount;

            // 将 PDO 添加到 SM_TX
            add_pdo_to_sm(sm3_tx, new_pdo);

            // 【关键】维护 PDO 级别的位偏移量
            uint32_t current_pdo_bit_offset = 0;

            // 填充该 PDO 下的 Entries
            for (int j = 0; j < pdo_data->entryCount; j++) {
                PDO_ENTRY_ITEM* entry = &pdo_data->entries[j];

                // 直接调用带返回值的 add 函数，传入计算好的偏移量
                if (!add_pdo_entry_to_pdo(new_pdo,
                    entry->index,
                    entry->subIndex,
                    entry->bitLen,
                    current_pdo_bit_offset,       // bit_pos: 相对于 PDO 开头
                    current_sm_bit_offset         // bit_offset_sm: 相对于 SM 开头
                )) return -1;

                // 更新计数器
                current_pdo_bit_offset += entry->bitLen;
                current_sm_bit_offset += entry->bitLen;
            }
        }
    }
    return 0;
}

static int COERequestResult(const DEVICE_BASIC_INFO* info, const unsigned char* data,
                            int slave_pos, size_t bytes) {
    const int slots = device_slot_count((DEVICE_TYPE)info->type);
    if (position < 0 || slots > MAX_DEVICE_NUM - position) return -1;
    int have_tx = 0, have_rx = 0;
    ECAT_SYNC_M_INFO sync = {0};
    PDO_RESULT tx = {0}, rx = {0};
    if (parse_eeprom_pdos(data, bytes, &tx, &rx, &sync, &have_tx, &have_rx) != 0)
        return -1;

    if (!have_tx || !have_rx) {
        int interrupt = 0;
        Slave_info* fallback = GM_PDO_Map_Get(slave_pos, 10, &interrupt);
        if (!fallback) return -1;
        const int rc = extract_pdos(fallback, 0x1c13, &tx) ||
                       extract_pdos(fallback, 0x1c12, &rx);
        for (const sm_info* sm = fallback->sm_list; sm; sm = sm->next) {
            ECAT_SYNC_MANAGER_CFG* dst = sm->sync_index == 0x1c12 ? &sync.sm2 :
                                         sm->sync_index == 0x1c13 ? &sync.sm3 : NULL;
            if (dst) {
                dst->wPhysAddr = sm->sm_startadress;
                dst->wDataLen = sm->sm_size_btye;
                dst->wConfigReg = sm->sm_controlbyte;
            }
        }
        const int freed = GM_Free_PDO_Map(fallback);
        if (rc != 0 || freed != 0) return -1;
    }
    if (info->type == SERVO_TYPE) {
        DevDictEntry dict;
        if (!DevDict_Lookup(info->ID, info->CODE, info->Revision, &dict)) return -1;
        /* 自定义对象号也用于动态补映射，不能写回标准对象号。 */
        const DevDictRole roles[] = { DEV_DICT_ROLE_ACTUAL_SPEED, DEV_DICT_ROLE_ERROR_CODE,
                                     DEV_DICT_ROLE_TARGET_SPEED, DEV_DICT_ROLE_OP_MODE };
        const TARGET_ENTRY standard[] = {{0x606C,0,32}, {0x603F,0,16},
                                          {0x60FF,0,32}, {0x6060,0,8}};
        for (int i = 0; i < 4; ++i) {
            TARGET_ENTRY target = standard[i];
            if (strcmp(dict.profile, "custom") == 0) {
                DevDictObject object;
                if (!DevDict_FindObject(&dict, roles[i], &object)) return -1;
                target.index = object.index;
                target.subIndex = object.sub;
                target.bitLen = object.bits;
            }
            PDO_RESULT* result = i < 2 ? &tx : &rx;
            if (!is_entry_exists_in_result(result, &target) &&
                ensure_entry_in_last_pdo(result, &target) != 1) return -1;
        }
    }

    const int tx_bits = calculate_tx_bit(&tx), rx_bits = calcute_rx_bit(&rx);
    Slave_info* slave = create_empty_slave(slave_pos);
    if (!slave) return -1;
    slave->Vendorid = info->ID;
    slave->ProductCode = info->CODE;
    slave->sync_assign_activate = 0x300; // TODO 某些io面板不支持DC，暂时写死0x300，后续可根据设备类型判断
    slave->sync0_cycle = SYNC0_CYCLE;
    slave->sync0_shift = SYNC0_SHIFT;
    slave->sync1_cycle = SYNC1_CYCLE;
    slave->sync1_shift = SYNC1_SHIFT;
    slave->slave_total = info->slave_total;
    slave->sm_num = (tx.count != 0) + (rx.count != 0);
    slave->slave_len = tx_bits + rx_bits;
    if (build_chain_from_raw_data(&slave, &tx, &rx, &sync, tx_bits, rx_bits) != 0) {
        free_slave_chain(slave);
        return -1;
    }
    add_slave_to_global_list(slave);
    position += slots;
    return 0;
}

/* DS402 标准对象号 —— profile="ds402" 时用它；字典里不写 objects。
 * 这组值对应原硬编码的 6 个 Tx + 4 个 Rx 请求，行为等价。
 * 注意原实现没有绑 target_torque（0x6071），这里也不绑。 */
static const DevDictObject kDs402Std[] = {
    {DEV_DICT_ROLE_STATUS_WORD,   0x6041, 0x00, 16, 1},
    {DEV_DICT_ROLE_ACTUAL_POS,    0x6064, 0x00, 32, 1},
    {DEV_DICT_ROLE_ERROR_CODE,    0x603F, 0x00, 16, 1},
    {DEV_DICT_ROLE_MODE_DISPLAY,  0x6061, 0x00,  8, 1},
    {DEV_DICT_ROLE_ACTUAL_SPEED,  0x606C, 0x00, 32, 1},
    {DEV_DICT_ROLE_ACTUAL_TORQUE, 0x6077, 0x00, 16, 1},
    {DEV_DICT_ROLE_CONTROL_WORD,  0x6040, 0x00, 16, 0},
    {DEV_DICT_ROLE_TARGET_POS,    0x607A, 0x00, 32, 0},
    {DEV_DICT_ROLE_TARGET_SPEED,  0x60FF, 0x00, 32, 0},
    {DEV_DICT_ROLE_OP_MODE,       0x6060, 0x00,  8, 0},
};

/* 每台从站的 EEPROM 身份，按【从站号】索引。
 * get_device_info_from_eeprom 里那个 device_info 是局部变量、用完即丢，
 * 而 device_match 阶段查设备字典需要 (厂商, 产品, 版本)，所以在这里留一份。 */
static DEVICE_BASIC_INFO g_slave_identity[MAX_DEVICE_NUM];

const DEVICE_BASIC_INFO* device_identity_get(int slave_pos) {
    static const DEVICE_BASIC_INFO kEmpty;
    if (slave_pos < 0 || slave_pos >= MAX_DEVICE_NUM) return &kEmpty;
    return &g_slave_identity[slave_pos];
}

static TxEntry_Unit* tx_role(slave_addr* axis, DevDictRole role) {
    switch (role) {
    case DEV_DICT_ROLE_STATUS_WORD: return &axis->statusWord;
    case DEV_DICT_ROLE_ACTUAL_POS: return &axis->act_pos;
    case DEV_DICT_ROLE_ERROR_CODE: return &axis->error_code;
    case DEV_DICT_ROLE_MODE_DISPLAY: return &axis->act_mode;
    case DEV_DICT_ROLE_ACTUAL_SPEED: return &axis->act_speed;
    case DEV_DICT_ROLE_ACTUAL_TORQUE: return &axis->act_torque;
    default: return NULL;
    }
}

static RxEntry_Unit* rx_role(slave_addr* axis, DevDictRole role) {
    switch (role) {
    case DEV_DICT_ROLE_CONTROL_WORD: return &axis->Control_word;
    case DEV_DICT_ROLE_TARGET_POS: return &axis->target_pos;
    case DEV_DICT_ROLE_TARGET_SPEED: return &axis->target_speed;
    case DEV_DICT_ROLE_TARGET_TORQUE: return &axis->target_torque;
    case DEV_DICT_ROLE_OP_MODE: return &axis->Modes_of_operation;
    default: return NULL;
    }
}

static int bind_servo_role(Slave_info* list, uint32_t slave_pos, uint32_t slot,
                           const DevDictObject* object, uint32_t pdo, uint32_t entry,
                           int by_index) {
    if (slot >= MAX_DEVICE_NUM) return -1;
    slave_addr* axis = &g_device_data[slot].slave;
    if (object->is_tx) {
        TxEntry_Unit* dst = tx_role(axis, object->role);
        TxEntry_Unit handle = {0};
        if (!dst) return -1;
        int rc = by_index
            ? GM_TxPdoEntry_Get_By_Index(&handle, list, slave_pos, pdo, object->index, object->sub)
            : GM_TxPdoEntry_Get_By_Pos(&handle, list, slave_pos, pdo, entry);
        if (rc != 0 || handle.bit_length != object->bits) return -1;
        *dst = handle;
    } else {
        RxEntry_Unit* dst = rx_role(axis, object->role);
        RxEntry_Unit handle = {0};
        if (!dst) return -1;
        int rc = by_index
            ? GM_RxPdoEntry_Get_By_Index(&handle, list, slave_pos, pdo, object->index, object->sub)
            : GM_RxPdoEntry_Get_By_Pos(&handle, list, slave_pos, pdo, entry);
        if (rc != 0 || handle.bit_length != object->bits) return -1;
        *dst = handle;
    }
    return 0;
}

static Slave_info* find_slave(Slave_info* list, uint32_t slave_pos) {
    for (; list; list = list->next)
        if (list->slave_pos == slave_pos) return list;
    return NULL;
}

static int find_object(const Slave_info* slave, const DevDictObject* object,
                       uint32_t* pdo_pos, uint32_t* entry_pos) {
    int found = 0;
    uint32_t pdo_index = 0;
    for (const sm_info* sm = slave->sm_list; sm; sm = sm->next) {
        if (!sm->sm_Enable || sm->sm_TR != (uint32_t)(object->is_tx ? Tx : Rx)) continue;
        for (const pdo_info* pdo = sm->pdo_list; pdo; pdo = pdo->next, ++pdo_index) {
            uint32_t entry_index = 0;
            for (const pdo_entry_info* entry = pdo->entry_list; entry;
                 entry = entry->next, ++entry_index) {
                if (entry->pdo_entry_index != object->index ||
                    entry->pdo_entry_subindex != object->sub) continue;
                if (found || entry->pdo_entry_bit_length != object->bits) return -1;
                *pdo_pos = pdo_index;
                *entry_pos = entry_index;
                found = 1;
            }
        }
    }
    return found ? 0 : -1;
}

int servo_addr_config(Slave_info* list, uint32_t slave_pos, uint32_t slot) {
    if (slave_pos >= MAX_DEVICE_NUM || slot >= MAX_DEVICE_NUM) return -1;
    Slave_info* slave = find_slave(list, slave_pos);
    if (!slave) return -1;
    const DEVICE_BASIC_INFO* id = &g_slave_identity[slave_pos];
    DevDictEntry dict;
    if (!DevDict_Lookup(id->ID, id->CODE, id->Revision, &dict)) return -1;
    const DevDictObject* objects = kDs402Std;
    int count = sizeof(kDs402Std) / sizeof(kDs402Std[0]);
    if (strcmp(dict.profile, "custom") == 0) {
        objects = dict.objects;
        count = dict.object_count;
    } else if (strcmp(dict.profile, "ds402") != 0) {
        return -1;
    }
    for (int i = 0; i < count; ++i) {
        uint32_t pdo, entry;
        if (find_object(slave, &objects[i], &pdo, &entry) != 0 ||
            bind_servo_role(list, slave_pos, slot, &objects[i], pdo, entry, 0) != 0)
            return -1;
    }
    return 0;
}

static int bind_multi_axis(Slave_info* list, uint32_t slot, uint32_t slave_pos, int axes) {
    if (!list || slave_pos >= MAX_DEVICE_NUM || slot > MAX_DEVICE_NUM - (uint32_t)axes)
        return -1;
    /* axis4 按 PDO 内位置；axis6 按每轴对象号偏移。顺序对应 kDs402Std。 */
    static const uint8_t axis4_entries[] = {0, 2, 1, 5, 3, 4, 0, 2, 3, 1};
    for (int axis = 0; axis < axes; ++axis) {
        for (size_t i = 0; i < sizeof(kDs402Std) / sizeof(kDs402Std[0]); ++i) {
            DevDictObject object = kDs402Std[i];
            uint32_t pdo = axis;
            if (axes == 6) {
                object.index += 2048 * axis;
                pdo = (object.is_tx ? 0x1A00 : 0x1600) + 16 * axis;
            }
            if (bind_servo_role(list, slave_pos, slot + axis, &object,
                                pdo, axis4_entries[i], axes == 6) != 0) return -1;
        }
    }
    return 0;
}

int servo_addr_axis6_config(Slave_info* list, uint32_t slot, uint32_t slave_pos) {
    return bind_multi_axis(list, slot, slave_pos, 6);
}

int servo_addr_axis4_config(Slave_info* list, uint32_t slot, uint32_t slave_pos) {
    return bind_multi_axis(list, slot, slave_pos, 4);
}

static int bind_io(Slave_info* list, uint32_t slave_pos, int slot, int panel) {
    Slave_info* slave = find_slave(list, slave_pos);
    if (!slave) return -1;
    TxEntry_Unit* inputs = panel ? g_device_data[slot].control.input_addr
                                : g_device_data[slot].io.io_input_addr;
    RxEntry_Unit* outputs = panel ? g_device_data[slot].control.output_addr
                                 : g_device_data[slot].io.io_output_addr;
    unsigned counts[2] = {0}, pdos[2] = {0};
    for (const sm_info* sm = slave->sm_list; sm; sm = sm->next) {
        if (!sm->sm_Enable) continue;
        if (sm->sm_TR != Tx && sm->sm_TR != Rx) return -1;
        const unsigned direction = sm->sm_TR;
        for (const pdo_info* pdo = sm->pdo_list; pdo; pdo = pdo->next, ++pdos[direction]) {
            unsigned index = 0;
            for (const pdo_entry_info* entry = pdo->entry_list; entry; entry = entry->next, ++index) {
                const unsigned n = counts[direction];
                const unsigned bits = entry->pdo_entry_bit_length;
                if (n >= 64 || bits == 0 || bits > 32) return -1;
                if (direction == Tx) {
                    if (GM_TxPdoEntry_Get_By_Pos(&inputs[n], list, slave_pos, pdos[Tx], index) != 0 ||
                        inputs[n].bit_length != bits) return -1;
                } else {
                    if (GM_RxPdoEntry_Get_By_Pos(&outputs[n], list, slave_pos, pdos[Rx], index) != 0 ||
                        outputs[n].bit_length != bits) return -1;
                }
                ++counts[direction];
            }
            if (index != pdo->pdo_entry_num) return -1;
        }
    }
    if (panel) {
        g_device_data[slot].control.in_count = counts[Tx];
        g_device_data[slot].control.out_count = counts[Rx];
    } else {
        g_device_data[slot].io.io_in_count = counts[Tx];
        g_device_data[slot].io.io_out_count = counts[Rx];
    }
    return 0;
}

int device_match(Slave_info* list, DEVICE_TYPE* types, int slave_num) {
    if (slave_num < 0 || slave_num > MAX_DEVICE_NUM || (slave_num && !types)) return -1;
    int total = 0;
    for (int i = 0; i < slave_num; ++i) {
        int n = device_slot_count(types[i]);
        if (n > MAX_DEVICE_NUM - total) return -1;
        total += n;
    }
    memset(g_device_data, 0, sizeof(g_device_data));
    int slot = 0;
    for (int i = 0; i < slave_num; ++i) {
        int rc = 0;
        switch (types[i]) {
        case SERVO_TYPE: rc = servo_addr_config(list, i, slot); break;
        case GREE_AXIS6_TYPE: rc = servo_addr_axis6_config(list, slot, i); break;
        case GREE_AXIS4_TYPE: rc = servo_addr_axis4_config(list, slot, i); break;
        case CONTROL_PANEL_TYPE: rc = bind_io(list, i, slot, 1); break;
        case IO_MODEL_TYPE: rc = bind_io(list, i, slot, 0); break;
        default: break;
        }
        if (rc != 0) {
            memset(g_device_data, 0, sizeof(g_device_data));
            return -1;
        }
        slot += device_slot_count(types[i]);
    }
    return 0;
}

void device_reset(void) {
    memset(g_device_data, 0, sizeof(g_device_data));
    memset(g_slave_identity, 0, sizeof(g_slave_identity));
    position = 0;
}

static uint32_t read_u32_le(const unsigned char* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

int get_device_info_from_eeprom(int slave_num, DEVICE_TYPE* types) {
    if (slave_num < 0 || slave_num > MAX_DEVICE_NUM || (slave_num && !types)) return -1;
    device_reset();
    const size_t buffer_size = 4 * 1024 * 1024;
    for (int i = 0; i < slave_num; ++i) {
        unsigned char* memory = calloc(buffer_size, 1);
        if (!memory) return -1;
        int interrupt = 0;
        int rc = GM_EEPROM_Get(i, memory, buffer_size, 10, &interrupt);
        if (rc > 0 && (size_t)rc <= buffer_size) {
            const size_t actual_bytes = (size_t)rc;
            if (actual_bytes < WORD_TO_BYTES(EEPROM_FIXED_WORDS)) {
                free(memory);
                return -1;
            }
            DEVICE_BASIC_INFO info = {0};
            info.ID = read_u32_le(memory + 16);
            info.CODE = read_u32_le(memory + 20);
            info.Revision = read_u32_le(memory + 24);
            info.Serial = read_u32_le(memory + 28);
            info.slave_pos = i;
            info.slave_total = slave_num;
            types[i] = get_device_types_from_info(&info);
            info.type = types[i];
            g_slave_identity[i] = info;
            rc = COERequestResult(&info, memory, i, actual_bytes);
        } else {
            rc = -1;
        }
        free(memory);
        if (rc != 0) return -1;
    }
    return 0;
}
