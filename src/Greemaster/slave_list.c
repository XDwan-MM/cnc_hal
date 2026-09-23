#include "slave_list.h"
Slave_info* slave_list = NULL;          // 定义从站信息结构体以存储解析结果


Slave_info* create_empty_slave(uint32_t pos) {
    Slave_info* node = calloc(1, sizeof(*node));
    if (node) node->slave_pos = pos;
    return node;
}

sm_info* create_empty_sm(void) { return calloc(1, sizeof(sm_info)); }
pdo_info* create_empty_pdo(void) { return calloc(1, sizeof(pdo_info)); }
pdo_entry_info* create_empty_pdo_entry(void) { return calloc(1, sizeof(pdo_entry_info)); }

/* 仅释放本层以 calloc 创建的映射链。 */
void free_slave_chain(Slave_info* slave) {
    while (slave) {
        Slave_info* next_slave = slave->next;
        sm_info* sm = slave->sm_list;
        while (sm) {
            sm_info* next_sm = sm->next;
            pdo_info* pdo = sm->pdo_list;
            while (pdo) {
                pdo_info* next_pdo = pdo->next;
                pdo_entry_info* entry = pdo->entry_list;
                while (entry) {
                    pdo_entry_info* next_entry = entry->next;
                    free(entry);
                    entry = next_entry;
                }
                free(pdo);
                pdo = next_pdo;
            }
            free(sm);
            sm = next_sm;
        }
        free(slave);
        slave = next_slave;
    }
}

// ================= 专用链表追加操作 =================

/**
 * @brief 将新 Slave 添加到全局链表尾部
 */
void add_slave_to_global_list(Slave_info* new_slave) {
    if (!new_slave) return;
    
    if (slave_list == NULL) {
        slave_list = new_slave;
    } else {
        Slave_info* tail = slave_list;
        while(tail->next != NULL) {
            tail = tail->next;
        }
        tail->next = new_slave;
    }
}

/**
 * @brief 将新 SM 添加到指定 Slave 的 SM 链表尾部
 */
void add_sm_to_slave(Slave_info* slave, sm_info* new_sm) {
    if (!slave || !new_sm) return;

    if (!slave->sm_list) {
        slave->sm_list = new_sm;
    } else {
        sm_info* tail = slave->sm_list;
        while(tail->next != NULL) {
            tail = tail->next;
        }
        tail->next = new_sm;
    }
}

/**
 * @brief 将新 PDO 添加到指定 SM 的 PDO 链表尾部
 */
void add_pdo_to_sm(sm_info* sm, pdo_info* new_pdo) {
    if (!sm || !new_pdo) return;

    if (!sm->pdo_list) {
        sm->pdo_list = new_pdo;
    } else {
        pdo_info* tail = sm->pdo_list;
        while(tail->next != NULL) {
            tail = tail->next;
        }
        tail->next = new_pdo;
    }
}

/**
 * @brief 添加 PDO Entry 并返回新创建的节点指针
 */
pdo_entry_info* add_pdo_entry_to_pdo(pdo_info* pdo, uint16_t index, uint8_t subindex, 
                                     uint8_t bit_len, uint32_t bit_pos, uint32_t bit_offset_sm) {
    pdo_entry_info* new_entry = create_empty_pdo_entry();
    if (!new_entry) return NULL;
    
    new_entry->pdo_entry_index = index;
    new_entry->pdo_entry_subindex = subindex;
    new_entry->pdo_entry_bit_length = bit_len;
    new_entry->bit_pos = bit_pos;       
    new_entry->bit_offset_sm = bit_offset_sm; 

    if (!pdo->entry_list) {
        pdo->entry_list = new_entry;
    } else {
        pdo_entry_info* tail = pdo->entry_list;
        while(tail->next != NULL) {
            tail = tail->next;
        }
        tail->next = new_entry;
    }
    
    return new_entry;
}

/**
 * @brief 打印单个 PDO Entry
 */
static void print_pdo_entry(pdo_entry_info* entry, int indent_level) {
    if (!entry) return;

    // 生成缩进空格
    char indent[100] = {0};
    for(int i = 0; i < indent_level * 2; i++) {
        indent[i] = ' ';
    }

    printf("%s[Entry] Index: 0x%04X, SubIdx: 0x%02X, Len: %d bits\n"
           "%s       |-> bit_pos(PDO): %u, bit_offset_sm(SM): %u\n",
           indent, 
           entry->pdo_entry_index, 
           entry->pdo_entry_subindex, 
           entry->pdo_entry_bit_length,
           indent,
           (unsigned int)entry->bit_pos,
           (unsigned int)entry->bit_offset_sm);
}

/**
 * @brief 打印单个 PDO
 */
static void print_pdo(pdo_info* pdo, int indent_level) {
    if (!pdo) return;

    char indent[100] = {0};
    for(int i = 0; i < indent_level * 2; i++) {
        indent[i] = ' ';
    }

    printf("%s[PDO] Index: 0x%04X, Entries: %u, TotalLen: %u bits\n"
           "%s       |-- StartBit: %u\n",
           indent,
           pdo->pdo_index,
           pdo->pdo_entry_num,
           pdo->pdo_len,
           indent,
           (unsigned int)pdo->pdo_pos);

    // 遍历 Entry
    pdo_entry_info* curr = pdo->entry_list;
    while (curr != NULL) {
        print_pdo_entry(curr, indent_level + 1);
        curr = curr->next;
    }
}

/**
 * @brief 打印单个 SM
 */
static void print_sm(sm_info* sm, int indent_level) {
    if (!sm) return;

    char indent[100] = {0};
    for(int i = 0; i < indent_level * 2; i++) {
        indent[i] = ' ';
    }

    const char* direction = (sm->sm_TR == 1) ? "Tx" : "Rx";
    
    printf("%s[SM%d] SyncIdx: %u, Addr: 0x%04X, Ctrl: 0x%02X, Dir: %s\n"
           "%s       |-- Enabled: %u, Length: %u bits (%u bytes), PDO Count: %u\n",
           indent,
           sm->sync_index,
           sm->sync_index,
           sm->sm_startadress,
           sm->sm_controlbyte,
           direction,
           indent,
           sm->sm_Enable,
           sm->sm_len,
           sm->sm_size_btye,
           sm->pdo_num);

    // 遍历 PDO
    pdo_info* curr = sm->pdo_list;
    while (curr != NULL) {
        print_pdo(curr, indent_level + 1);
        curr = curr->next;
    }
}

/**
 * @brief 打印整个 Slave 信息
 */
void print_slave_info(Slave_info* slave) {
    if (!slave) {
        printf("Slave Info: NULL\n");
        return;
    }

    printf("\n========================================\n"
           "| Slave Position: %u\n"
           "| VendorID: 0x%08X, ProductCode: 0x%08X\n"
           "| Total Length: %u bits\n"
           "| SM Count: %u\n"
           "========================================\n",
           slave->slave_pos,
           slave->Vendorid, 
           slave->ProductCode,
           slave->slave_len,
           slave->sm_num);

    // 遍历 SM
    sm_info* curr = slave->sm_list;
    while (curr != NULL) {
        print_sm(curr, 1); 
        curr = curr->next;
    }
    
    printf("----------------------------------------\n\n");
}

/**
 * @brief 打印全局从站链表的所有信息
 */
void print_all_slaves(void) {
    if (slave_list == NULL) {
        printf("[INFO] Global slave_list is NULL.\n");
        return;
    }

    printf("\n=================== Global Slave List Dump ===================\n");

    Slave_info* curr = slave_list;
    int index = 0;
    
    while (curr != NULL) {
        print_slave_info(curr);
        
        if (curr->next == NULL) {
            break;
        }
        curr = curr->next;
        index++;
    }

    printf("Total Slaves Printed: %d\n", index);
}