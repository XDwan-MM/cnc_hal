#ifndef _SLAVE_LIST_H
#define _SLAVE_LIST_H
#include "/opt/GreeMaster/include/libGREEMASTER/GreeMasterAPI.h"
#include "/opt/GreeMaster/include/libGREEMASTER/gm_errcode.h"
#include <string.h>
// ================= 辅助宏：颜色输出 (可选，使日志更清晰) =================
#ifndef COLOR_RESET
#define COLOR_RESET   "\033[0m"
#define COLOR_GREEN   "\033[32m" // Slave
#define COLOR_BLUE    "\033[34m" // SM
#define COLOR_YELLOW  "\033[33m" // PDO
#define COLOR_CYAN    "\033[36m" // Entry
#endif

/*void append_to_list(void** head_ptr, void* new_node);
pdo_entry_info* create_empty_pdo_entry();
void add_pdo_entry_to_pdo(pdo_info* pdo, uint16_t index, uint8_t subindex, uint8_t bit_len, uint32_t bit_pos, uint32_t bit_offset_sm);
pdo_info* create_empty_pdo();
void add_pdo_to_sm(sm_info* sm, uint32_t pdo_index, uint32_t pdo_len, uint32_t pdo_pos, uint32_t pdo_entry_num);
sm_info* create_empty_sm();
void add_sm_to_slave(Slave_info* slave, uint32_t sync_index, uint16_t start_addr, uint16_t control_byte, uint16_t enable, uint32_t direction, uint32_t sm_len, uint32_t sm_size_btye, uint32_t pdo_num);
Slave_info* create_empty_slave(uint32_t pos);
void add_slave_to_global_list(Slave_info* new_slave);
void free_pdo_entries(pdo_entry_info* head);
void free_pdos(pdo_info* head);
void free_sms(sm_info* head);
void free_slaves(void);*/

void free_slave_chain(Slave_info* slave);

Slave_info* create_empty_slave(uint32_t pos);
sm_info* create_empty_sm();
pdo_info* create_empty_pdo();
pdo_entry_info* create_empty_pdo_entry();

// 专用追加函数
void add_slave_to_global_list(Slave_info* new_slave);
void add_sm_to_slave(Slave_info* slave, sm_info* new_sm);
void add_pdo_to_sm(sm_info* sm, pdo_info* new_pdo);
pdo_entry_info* add_pdo_entry_to_pdo(pdo_info* pdo, uint16_t index, uint8_t subindex, uint8_t bit_len, uint32_t bit_pos, uint32_t bit_offset_sm);

/**
 * @brief 打印单个 Slave 的信息
 */
void print_slave_info(Slave_info* slave);

/**
 * @brief 打印全局从站链表的所有信息
 */
void print_all_slaves(void);
extern Slave_info* slave_list;
#endif
