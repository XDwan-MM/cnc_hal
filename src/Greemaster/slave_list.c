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
