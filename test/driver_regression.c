/* SDK 替身：验证调用顺序、失败回滚与内存边界，不接硬件。 */
#include "../src/Greemaster/device.c"
#include "Greemaster/main_demo.h"
#include "Greemaster/device_table.h"
#include "Greemaster/servo_step.h"
#include "Greemaster/entry_access.h"
#include <assert.h>
#include <time.h>

static uint64_t fake_ms;
static uint32_t start_seconds, active_seconds, op_seconds, sync_seconds;
static int advance_start_ms, stop_during_wait;
int __wrap_clock_gettime(clockid_t id, struct timespec* ts) {
    ts->tv_sec = (time_t)(fake_ms / 1000);
    ts->tv_nsec = (long)(fake_ms % 1000) * 1000000;
    return 0;
}

static unsigned checks;
#define CHECK(expr) do { ++checks; if (!(expr)) {     fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #expr); abort(); } } while (0)

static int calloc_fail_at, calloc_calls;
void* __real_calloc(size_t count, size_t size);
void* __wrap_calloc(size_t count, size_t size) {
    if (calloc_fail_at && ++calloc_calls == calloc_fail_at) return NULL;
    return __real_calloc(count, size);
}

static const char* fail_call;
static int fail_code = -7, fail_index, fail_nth = 1;
static int sdk_calls, allocations, releases, closes, io_releases, handshakes;
static int slaves = 1, bad_eeprom, missing_mode, wrong_width, use_fallback;
static int actual_eeprom_length, fallback_reads, fallback_frees;
static int unknown_device;
static uint16_t status_word = 0x23, read_fail_index, write_fail_index;
static uint8_t actual_mode = 8;
static uint16_t written_index[64];
static uint32_t written_value[64];
static unsigned write_count;
static int servo_reads;

static int called(const char* name) {
    ++sdk_calls;
    if (fail_call && strcmp(fail_call, name) == 0 && ++fail_index == fail_nth)
        return fail_code;
    return 0;
}
#define CALL() called(__func__)

static void clear_failure(void) {
    fail_call = NULL; fail_index = 0; fail_nth = 1; fail_code = -7;
    read_fail_index = write_fail_index = 0;
    write_count = 0; servo_reads = 0;
}

int GM_Resource_Allocation(void) { ++allocations; return CALL(); }
int GM_Resource_Release(void) { ++releases; return CALL(); }
int GM_Master_Init(void) { return CALL(); }
int GM_Master_Close(void) { ++closes; return CALL(); }
_Version_ GM_Get_Version(void) { _Version_ v = {0}; return v; }
int Err_Fun_Register(CallbackFunc cb) { return CALL(); }
int Err_Code_Get(Err_info* err) { memset(err, 0, sizeof(*err)); return 0; }
int Err_Info_Get(Err_info err, FILE* out) { return 0; }
int PdoWar_Info_Get(FILE* out, PdoWarn* warn) { return 0; }
int GM_Master_Start(uint32_t timeout, int* interrupt) {
    start_seconds = timeout;
    fake_ms += advance_start_ms;
    CHECK(*interrupt == 0); return CALL();
}
int GM_Slave_Num_Get(void) { int rc = CALL(); return rc ? rc : slaves; }
int GM_Calculate_Config_Info(Slave_info** list) { return CALL(); }
int GM_PDO_Map_Print(Slave_info* list, uint32_t num) { return 0; }
int GM_Master_Set_Cycle(uint32_t period) { return CALL(); }
int GM_DC_Enable(void) { return CALL(); }
int GM_Reg_Resource_Init(uint32_t count, uint32_t* sizes) { return CALL(); }
int GM_Sdo_Datagram_Enable(void) { return CALL(); }
int GM_Config_Download_And_Active(Slave_info* list, uint32_t* crc, uint32_t* timeouts,
                                uint32_t timeout, int* interrupt, Fmmu_Manul* fmmu) {
    active_seconds = timeout;
    CHECK(*interrupt == 0); return CALL();
}
int GM_Master_Wait_OP(uint32_t timeout, int* interrupt) {
    op_seconds = timeout;
    CHECK(*interrupt == 0); return CALL();
}
int GM_Master_Receive(int pdo, PdoWarn* warn) {
    memset(warn, 0, sizeof(*warn)); warn->CRCErrCount = 9; return CALL();
}
int GM_Master_Send(int pdo) { return CALL(); }
int GM_Master_OP_Valid(void) { ++handshakes; return CALL(); }
int GM_Wait_Master_Sync(uint32_t timeout) {
    sync_seconds = timeout;
    if (stop_during_wait) Master_RequestStop();
    return CALL();
}
int IO_Resource_Allocation_Scanf(void) { return CALL(); }
int IO_Resource_Release(void) { ++io_releases; return CALL(); }
void Fmmu_Manul_Free(Fmmu_Manul* ptr) { free(ptr); }

static void append_category(unsigned char* buffer, size_t* offset, uint16_t type,
                            const void* data, size_t bytes) {
    ECAT_EEPROM_CLASS_HEADER h = {type, bytes/2};
    memcpy(buffer + *offset, &h, sizeof(h)); *offset += sizeof(h);
    memcpy(buffer + *offset, data, bytes); *offset += bytes;
}

static size_t make_eeprom(unsigned char* data, int six_axis) {
    memset(data, 0, 4096);
    const uint32_t vendor = unknown_device ? 12345 : six_axis ? 2252 : 441;
    const uint32_t product = six_axis ? 6 : 2;
    memcpy(data + 16, &vendor, 4); memcpy(data + 20, &product, 4);
    size_t offset = 128;
    ECAT_SYNC_M_INFO sm = {0};
    sm.sm2.wPhysAddr = 0x1100; sm.sm3.wPhysAddr = 0x1200;
    append_category(data, &offset, TYPE_SyncM, &sm, sizeof(sm));
    if (!use_fallback) {
        for (int dir = 0; dir < 2; ++dir) {
            unsigned char block[1024] = {0};
            size_t length = 0;
            for (int axis = 0; axis < (six_axis ? 6 : 1); ++axis) {
                PDO_BLOCK_HEADER header = {0};
                header.pdoEntry = (dir == Tx ? 0x1A00 : 0x1600) + 16 * axis;
                header.smIndex = dir == Tx ? 3 : 2;
                size_t at = length; length += sizeof(header);
                for (size_t j = 0; j < sizeof(kDs402Std)/sizeof(kDs402Std[0]); ++j) {
                    const DevDictObject* obj = &kDs402Std[j];
                    if (obj->is_tx != (dir == Tx)) continue;
                    if (missing_mode && obj->role == DEV_DICT_ROLE_MODE_DISPLAY) continue;
                    PDO_ENTRY_ITEM entry = {0};
                    entry.index = obj->index + 2048 * axis;
                    entry.bitLen = wrong_width && obj->role == DEV_DICT_ROLE_ACTUAL_POS ? 16 : obj->bits;
                    memcpy(block + length, &entry, sizeof(entry)); length += sizeof(entry);
                    ++header.entryCount;
                }
                memcpy(block + at, &header, sizeof(header));
            }
            append_category(data, &offset, dir == Tx ? TYPE_TX_PDO : TYPE_RX_PDO, block, length);
        }
    }
    uint16_t end = TYPE_END;
    memcpy(data + offset, &end, 2);
    return offset + 2;
}

int GM_EEPROM_Get(int pos, unsigned char* out, size_t size, uint32_t timeout, int* interrupt) {
    int rc = CALL(); if (rc) return rc;
    CHECK(size >= 4096);
    size_t n = make_eeprom(out, slaves > 1);
    if (bad_eeprom) n = 130; /* 分类头截断 */
    if (actual_eeprom_length >= 0) return actual_eeprom_length;
    return (int)n;
}

static Slave_info* fallback_map;
Slave_info* GM_PDO_Map_Get(int pos, int timeout, int* interrupt) {
    ++fallback_reads;
    Slave_info* result = fallback_map;
    fallback_map = NULL;
    return result;
}
int GM_Free_PDO_Map(Slave_info* list) {
    ++fallback_frees;
    free_slave_chain(list);
    return CALL();
}

static pdo_entry_info* lookup(Slave_info* list, unsigned slave, unsigned pdo_pos,
                              unsigned entry_pos, int tx) {
    Slave_info* device = find_slave(list, slave);
    if (!device) return NULL;
    for (sm_info* sm = device->sm_list; sm; sm = sm->next) {
        if (!sm->sm_Enable || sm->sm_TR != (uint32_t)(tx ? Tx : Rx)) continue;
        for (pdo_info* pdo = sm->pdo_list; pdo; pdo = pdo->next) {
            if (pdo_pos--) continue;
            pdo_entry_info* entry = pdo->entry_list;
            while (entry && entry_pos--) entry = entry->next;
            return entry;
        }
    }
    return NULL;
}
static pdo_entry_info* lookup_index(Slave_info* list, unsigned slave, unsigned pdo_index,
                                    unsigned entry_index, unsigned sub, int tx) {
    Slave_info* device = find_slave(list, slave);
    if (!device) return NULL;
    for (sm_info* sm = device->sm_list; sm; sm = sm->next) {
        if (sm->sm_TR != (uint32_t)(tx ? Tx : Rx)) continue;
        for (pdo_info* pdo = sm->pdo_list; pdo; pdo = pdo->next) {
            if (pdo->pdo_index != pdo_index) continue;
            for (pdo_entry_info* e = pdo->entry_list; e; e = e->next)
                if (e->pdo_entry_index == entry_index && e->pdo_entry_subindex == sub) return e;
        }
    }
    return NULL;
}
#define FILL_HANDLE(dst,e) do { \
    if (!(e)) { return -1; } \
    memset(dst, 0, sizeof(*(dst))); \
    (dst)->index = (e)->pdo_entry_index; \
    (dst)->subindex = (e)->pdo_entry_subindex; \
    (dst)->bit_length = (e)->pdo_entry_bit_length; \
    return 0; \
} while (0)

int GM_TxPdoEntry_Get_By_Pos(TxEntry_Unit* dst, Slave_info* list, uint32_t slave,
                            uint32_t pdo, uint32_t entry) {
    int rc = CALL(); if (rc) return rc;
    pdo_entry_info* e = lookup(list,slave,pdo,entry,1); FILL_HANDLE(dst,e);
}
int GM_RxPdoEntry_Get_By_Pos(RxEntry_Unit* dst, Slave_info* list, uint32_t slave,
                            uint32_t pdo, uint32_t entry) {
    int rc = CALL(); if (rc) return rc;
    pdo_entry_info* e = lookup(list,slave,pdo,entry,0); FILL_HANDLE(dst,e);
}
int GM_TxPdoEntry_Get_By_Index(TxEntry_Unit* dst, Slave_info* list, uint32_t slave,
                              uint32_t pdo, uint32_t entry, uint32_t sub) {
    int rc = CALL(); if (rc) return rc;
    pdo_entry_info* e = lookup_index(list,slave,pdo,entry,sub,1); FILL_HANDLE(dst,e);
}
int GM_RxPdoEntry_Get_By_Index(RxEntry_Unit* dst, Slave_info* list, uint32_t slave,
                              uint32_t pdo, uint32_t entry, uint32_t sub) {
    int rc = CALL(); if (rc) return rc;
    pdo_entry_info* e = lookup_index(list,slave,pdo,entry,sub,0); FILL_HANDLE(dst,e);
}
int GM_TxPdoEntry_Read(TxEntry_Unit entry, void* out, size_t size) {
    ++servo_reads;
    if (entry.index == read_fail_index) return fail_code;
    uint32_t value = entry.index == 0x6041 ? status_word :
                     entry.index == 0x6061 ? actual_mode :
                     entry.index == 0x6064 ? 1234 : UINT32_MAX;
    memcpy(out, &value, size);
    return 0;
}
int GM_RxPdoEntry_Write(RxEntry_Unit entry, uint32_t value) {
    CHECK(write_count < 64);
    written_index[write_count] = entry.index;
    written_value[write_count++] = value;
    return entry.index == write_fail_index ? fail_code : 0;
}

static MasterConfig config = MASTER_CONFIG_DEFAULT;

static void assert_closed(void) {
    const DeviceSlot* slots = (void*)1;
    CHECK(Master_StopFlag() != 0);
    CHECK(DeviceTable_Get(&slots) == 0 && slots == NULL);
    CHECK(slave_list == NULL);
    CHECK(g_device_data[0].slave.statusWord.bit_length == 0);
    CHECK(Master_WaitCycle() < 0 && Master_CommitCycle() < 0);
    CHECK(Master_ServoStep(0, DS402_REQ_ENABLE, 8, NULL) < 0);
    CHECK(DevDict_EntryCount() == 0);
}

static void lifecycle_tests(void) {
    actual_eeprom_length = -1;
    advance_start_ms = 2001;
    CHECK(ethercat_init(&config) == 0);
    CHECK(start_seconds == 120 && active_seconds == 118 && op_seconds == 118);
    CHECK(Master_WaitCycle() == 0 && sync_seconds == 5);
    stop_during_wait = 1;
    CHECK(Master_WaitCycle() == MASTER_STOP_REQUESTED);
    stop_during_wait = 0;
    CHECK(ethercat_close() == 0);
    advance_start_ms = 120000;
    CHECK(ethercat_init(&config) == MASTER_START_TIMEOUT);
    advance_start_ms = 0;
    config.cycle_timeout_ms = 1001;
    CHECK(ethercat_init(&config) == 0);
    CHECK(Master_WaitCycle() == 0 && sync_seconds == 2);
    CHECK(ethercat_close() == 0);
    config.cycle_timeout_ms = 1;
    CHECK(ethercat_init(&config) == 0);
    CHECK(Master_WaitCycle() == 0 && sync_seconds == 1);
    CHECK(ethercat_close() == 0);
    config.cycle_timeout_ms = 5000;
    assert_closed();
    int before = sdk_calls;
    CHECK(ethercat_init(NULL) < 0);
    MasterConfig invalid = config; invalid.cycle_us = 249;
    CHECK(ethercat_init(&invalid) < 0 && sdk_calls == before);
    CHECK(ethercat_close() == 0 && sdk_calls == before);

    const char* stages[] = {
        "GM_Resource_Allocation", "Err_Fun_Register", "GM_Master_Init", "GM_Master_Start",
        "GM_Slave_Num_Get", "GM_EEPROM_Get", "GM_Calculate_Config_Info",
        "GM_TxPdoEntry_Get_By_Pos", "GM_RxPdoEntry_Get_By_Pos",
        "GM_Master_Set_Cycle", "GM_DC_Enable", "GM_Reg_Resource_Init",
        "GM_Sdo_Datagram_Enable", "GM_Config_Download_And_Active", "GM_Master_Wait_OP",
        "GM_Master_Receive", "IO_Resource_Allocation_Scanf", "GM_Master_Send", "GM_Master_OP_Valid"
    };
    for (unsigned sign = 0; sign < 2; ++sign) {
        for (size_t i = 0; i < sizeof(stages)/sizeof(stages[0]); ++i) {
            if (sign && (!strcmp(stages[i],"GM_EEPROM_Get") || !strcmp(stages[i],"GM_Slave_Num_Get")))
                continue; /* 这两个 API 的正值是成功数据。 */
            clear_failure(); fail_call = stages[i]; fail_code = sign ? 7 : -7;
            CHECK(ethercat_init(&config) < 0);
            assert_closed();
            clear_failure();
            CHECK(ethercat_init(&config) == 0);
            CHECK(Master_StopFlag() == 0);
            CHECK(DeviceTable_Get(NULL) == 1);
            CHECK(Master_WaitCycle() == 0 && Master_CommitCycle() == 0);
            before = sdk_calls;
            CHECK(ethercat_init(&config) < 0 && sdk_calls == before);
            CHECK(ethercat_close() == 0);
            before = sdk_calls;
            CHECK(ethercat_close() == 0 && sdk_calls == before);
        }
    }
    CHECK(allocations == releases);
    slaves = 31; before = handshakes;
    CHECK(ethercat_init(&config) < 0 && handshakes == before);
    assert_closed();
    slaves = 6; /* 六台 axis6 = 36 槽 */
    CHECK(ethercat_init(&config) < 0 && handshakes == before);
    assert_closed();
    slaves = 5;
    CHECK(ethercat_init(&config) == 0 && DeviceTable_Get(NULL) == 30);
    const DeviceSlot* all_slots;
    CHECK(DeviceTable_Get(&all_slots) == 30);
    for (int k = 0; k < 30; ++k)
        CHECK(all_slots[k].slave_pos == k / 6 && all_slots[k].axis_index == k % 6);
    CHECK(ethercat_close() == 0);
    slaves = 1;
    int* bad[] = {&bad_eeprom, &missing_mode, &wrong_width};
    for (size_t i = 0; i < 3; ++i) {
        *bad[i] = 1; before = handshakes;
        CHECK(ethercat_init(&config) < 0 && handshakes == before);
        assert_closed(); *bad[i] = 0;
    }
    const int lengths[] = {0, 20, 4*1024*1024+1};
    for (size_t i = 0; i < 3; ++i) {
        actual_eeprom_length = lengths[i];
        CHECK(ethercat_init(&config) < 0); assert_closed();
    }
    actual_eeprom_length = -1;
    CHECK(ethercat_init(&config) == 0);
    CHECK(Master_ServoSetModeSwitch(0, DS402_MODESW_IN_PLACE) == 0);
    Master_RequestStop(); CHECK(Master_StopFlag() != 0);
    before = sdk_calls;
    CHECK(Master_WaitCycle() == MASTER_STOP_REQUESTED);
    CHECK(Master_CommitCycle() == MASTER_STOP_REQUESTED && sdk_calls == before);
    CHECK(ethercat_close() == 0 && ethercat_init(&config) == 0);
    Ds402ModeSwitch policy;
    CHECK(Master_ServoGetModeSwitch(0,&policy) == 0 && policy == DS402_MODESW_DISABLE_FIRST);
    fail_call = "GM_Master_Close"; fail_code = -17;
    CHECK(ethercat_close() == -17);
    assert_closed(); clear_failure();
    MasterBusHealth health;
    CHECK(Master_BusHealth(&health) == 0 && health.crc_err_count == 0);
}

static void servo_tests(void) {
    clear_failure(); CHECK(ethercat_init(&config) == 0);
    for (int sign = 0; sign < 2; ++sign) {
        clear_failure(); fail_code = sign ? 7 : -7;
        read_fail_index = 0x6061; status_word = 0x23;
        CHECK(Master_ServoStep(0,DS402_REQ_ENABLE,8,NULL) < 0 && write_count == 0);
        read_fail_index = 0x6064; status_word = 0x21;
        CHECK(Master_ServoStep(0,DS402_REQ_ENABLE,8,NULL) < 0 && write_count == 0);
        read_fail_index = 0; write_fail_index = 0x607A;
        CHECK(Master_ServoStep(0,DS402_REQ_ENABLE,8,NULL) < 0 && write_count == 1);
        CHECK(written_index[0] == 0x607A);
        write_count = 0; write_fail_index = 0x6060; status_word = 0x23; actual_mode = 9;
        CHECK(Master_ServoStep(0,DS402_REQ_ENABLE,8,NULL) < 0 && write_count == 2);
        CHECK(written_index[0] == 0x607A && written_index[1] == 0x6060);
        write_count = 0; write_fail_index = 0x6040; actual_mode = 8;
        CHECK(Master_ServoStep(0,DS402_REQ_ENABLE,8,NULL) < 0 && write_count == 1);
        CHECK(written_value[0] == 0x0F);
    }
    clear_failure(); status_word = 0x23; actual_mode = 8;
    g_device_data[0].slave.act_mode.bit_length = 0;
    CHECK(Master_ServoStep(0,DS402_REQ_ENABLE,8,NULL) < 0 && write_count == 0);
    status_word = 0x237; read_fail_index = 0x6061;
    CHECK(Master_ServoStep(0,DS402_REQ_DROP_VOLTAGE,0,NULL) == 0);
    CHECK(write_count == 1 && written_value[0] == 0);
    g_device_data[0].slave.act_mode.bit_length = 8;
    clear_failure(); actual_mode = 9;
    CHECK(Master_ServoSetModeSwitch(0,DS402_MODESW_IN_PLACE) == 0);
    CHECK(Master_ServoStep(0,DS402_REQ_ENABLE,8,NULL) == 0);
    CHECK(write_count == 2 && written_index[0] == 0x607A &&
          written_value[0] == 1234 && written_index[1] == 0x6060);
    clear_failure();
    CHECK(Master_ServoStep(30,DS402_REQ_ENABLE,8,NULL) < 0);
    CHECK(Master_ServoStep(1,DS402_REQ_ENABLE,8,NULL) < 0);
    CHECK(Master_ServoStep(0,(Ds402Request)99,8,NULL) < 0);
    CHECK(Master_ServoSetModeSwitch(0,(Ds402ModeSwitch)99) < 0);
    CHECK(Master_ServoStep(0,DS402_REQ_ENABLE,99,NULL) < 0);
    CHECK(servo_reads == 0 && write_count == 0);
    CHECK(ethercat_close() == 0);
}

static void parser_tests(void) {
    unsigned char block[8+21*8] = {0};
    PDO_BLOCK_HEADER h = {0}; h.smIndex = 3; h.entryCount = 21;
    memcpy(block,&h,sizeof(h));
    PDO_RESULT result = {0};
    CHECK(ParsePdosInBlock(block,sizeof(block),3,&result) < 0 && result.count == 0);
    h.entryCount = 20; memcpy(block,&h,sizeof(h));
    CHECK(ParsePdosInBlock(block,sizeof(block)-8,3,&result) == 0);
    CHECK(result.count == 1 && result.validPdos[0].entryCount == 20);
    result.count = 0;
    CHECK(ParsePdosInBlock(block,7,3,&result) < 0);
    CHECK(ParsePdosInBlock(block,8,3,&result) < 0);
    unsigned char many[11*8] = {0}; h.entryCount = 0;
    for (int i=0;i<11;++i) memcpy(many+i*8,&h,8);
    CHECK(ParsePdosInBlock(many,sizeof(many),3,&result) < 0 && result.count == 0);
    CHECK(ParsePdosInBlock(many,10*8,3,&result) == 0 && result.count == 10);

    unsigned char eeprom[512] = {0};
    PDO_RESULT tx={0},rx={0}; ECAT_SYNC_M_INFO sm={0};
    int ht=0,hr=0;
    CHECK(parse_eeprom_pdos(eeprom,127,&tx,&rx,&sm,&ht,&hr) < 0);
    CHECK(parse_eeprom_pdos(eeprom,128,&tx,&rx,&sm,&ht,&hr) < 0);
    uint16_t end=TYPE_END; memcpy(eeprom+128,&end,2);
    CHECK(parse_eeprom_pdos(eeprom,130,&tx,&rx,&sm,&ht,&hr) == 0);
    ECAT_EEPROM_CLASS_HEADER category={TYPE_SyncM,1}; memcpy(eeprom+128,&category,4);
    CHECK(parse_eeprom_pdos(eeprom,134,&tx,&rx,&sm,&ht,&hr) < 0);
    category.type=TYPE_TX_PDO;category.length=100;memcpy(eeprom+128,&category,4);
    CHECK(parse_eeprom_pdos(eeprom,136,&tx,&rx,&sm,&ht,&hr) < 0);

    clear_failure();
    DEVICE_TYPE types[6] = {GREE_AXIS6_TYPE,GREE_AXIS6_TYPE,GREE_AXIS6_TYPE,
                            GREE_AXIS6_TYPE,GREE_AXIS6_TYPE,SERVO_TYPE};
    int before=sdk_calls;
    CHECK(device_match(NULL,types,6) < 0 && sdk_calls == before);
    CHECK(servo_addr_axis6_config(NULL,25,0) < 0 && sdk_calls == before);
    CHECK(servo_addr_axis4_config(NULL,27,0) < 0 && sdk_calls == before);
    for(int i=0;i<6;++i) g_slave_identity[i].type=types[i];
    CHECK(DeviceTable_Build(6) < 0 && DeviceTable_Get(NULL) == 0);
    CHECK(DeviceTable_Build(-1) < 0 && DeviceTable_Build(31) < 0);
    device_reset();
}

static Slave_info* io_map(int entries) {
    Slave_info* slave=create_empty_slave(0);
    for(int dir=0;dir<2;++dir) {
        sm_info* sm=create_empty_sm();sm->sm_Enable=1;sm->sm_TR=dir;
        sm->sync_index=dir==Tx?0x1c13:0x1c12;
        add_sm_to_slave(slave,sm);
        int n=0;
        while(n<entries) {
            pdo_info* pdo=create_empty_pdo();add_pdo_to_sm(sm,pdo);
            pdo->pdo_index=(dir==Tx?0x1a00:0x1600)+n/2;
            for(int j=0;j<2 && n<entries;++j,++n) {
                CHECK(add_pdo_entry_to_pdo(pdo,0x2000+dir*256+n,0,n%2?12:1,0,0)!=NULL);
                ++pdo->pdo_entry_num;
            }
        }
    }
    return slave;
}

static void io_tests(void) {
    for(int panel=0;panel<2;++panel) {
        device_reset();
        Slave_info* list=io_map(4);
        DEVICE_TYPE type=panel?CONTROL_PANEL_TYPE:IO_MODEL_TYPE;
        g_slave_identity[0].type=type;
        CHECK(device_match(list,&type,1)==0 && DeviceTable_Build(1)==1);
        const DeviceSlot* slots;CHECK(DeviceTable_Get(&slots)==1);
        CHECK(slots[0].axis_index == -1);
        CHECK((panel?slots[0].entries.control.in_count:slots[0].entries.io.io_in_count)==4);
        for(int i=0;i<4;++i) {
            uint32_t value;int bits;
            CHECK(Master_IoReadEntry(0,i,&value,&bits)==0);
            CHECK(bits==(i%2?12:1) && value==(i%2?4095u:1u));
            clear_failure();
            CHECK(Master_IoWriteEntry(0,i,UINT32_MAX)==0);
            CHECK(write_count==1 && written_index[0]==0x2100+i && written_value[0]==value);
        }
        uint32_t value;
        CHECK(Master_IoReadEntry(0,4,&value,NULL)<0);
        CHECK(Master_ServoRead(0,DEV_DICT_ROLE_STATUS_WORD,&value)<0);
        CHECK(Master_ServoStep(0,DS402_REQ_ENABLE,8,NULL)<0);
        fail_call="GM_TxPdoEntry_Get_By_Pos";fail_nth=3;
        CHECK(device_match(list,&type,1)<0);
        CHECK(g_device_data[0].io.io_input_addr[0].bit_length==0);
        clear_failure();free_slave_chain(list);
    }
    device_reset();
    Slave_info* list=io_map(65);DEVICE_TYPE type=IO_MODEL_TYPE;
    CHECK(device_match(list,&type,1)<0);free_slave_chain(list);
    list=io_map(64);CHECK(device_match(list,&type,1)==0);free_slave_chain(list);

    PDO_RESULT tx={0},rx={0};ECAT_SYNC_M_INFO sync={0};
    tx.count=1;tx.validPdos[0].entryCount=1;tx.validPdos[0].entries[0].bitLen=1;
    rx.count=1;rx.validPdos[0].entryCount=1;rx.validPdos[0].entries[0].bitLen=12;
    list=create_empty_slave(0);
    CHECK(build_chain_from_raw_data(&list,&tx,&rx,&sync,1,12)==0);
    CHECK(list->sm_list->sm_size_btye==2 && list->sm_list->next->sm_size_btye==1);
    free_slave_chain(list);

    list=io_map(22);PDO_RESULT result;
    CHECK(extract_pdos(list,0x1c13,&result)<0); /* 11 PDO */
    free_slave_chain(list);
    list=io_map(2);
    pdo_info* first=list->sm_list->pdo_list;
    for(int i=2;i<21;++i){CHECK(add_pdo_entry_to_pdo(first,0x3000+i,0,8,0,0)!=NULL);++first->pdo_entry_num;}
    CHECK(extract_pdos(list,0x1c13,&result)<0);
    free_slave_chain(list);
    DeviceTable_Reset();device_reset();
}

static void fallback_tests(void) {
    clear_failure(); use_fallback=1; unknown_device=1;
    fallback_map=io_map(4);
    fallback_map->sm_list->sm_startadress=0x1800;
    fallback_map->sm_list->next->sm_startadress=0x1900;
    int before=fallback_reads, freed=fallback_frees;
    CHECK(ethercat_init(&config)==0);
    CHECK(fallback_reads==before+1 && fallback_frees==freed+1);
    CHECK(slave_list->sm_list->sm_startadress==0x1900);
    CHECK(slave_list->sm_list->next->sm_startadress==0x1800);
    CHECK(ethercat_close()==0);
    fallback_map=io_map(22);
    before=handshakes;freed=fallback_frees;
    CHECK(ethercat_init(&config)<0 && handshakes==before);
    CHECK(fallback_frees==freed+1 && fallback_map==NULL);
    assert_closed();
    CHECK(ethercat_init(&config)<0);assert_closed();
    use_fallback=0;unknown_device=0;
}

static void allocation_tests(void) {
    int succeeded=0;
    for(int n=1;n<100;++n) {
        calloc_fail_at=n;calloc_calls=0;
        int before=handshakes;
        int rc=ethercat_init(&config);
        calloc_fail_at=0;
        if(rc==0) { CHECK(ethercat_close()==0); succeeded=1; break; }
        CHECK(handshakes==before);
        assert_closed();
    }
    CHECK(succeeded);
}

int main(void) {
    lifecycle_tests();
    servo_tests();
    parser_tests();
    io_tests();
    fallback_tests();
    allocation_tests();
    printf("driver_regression: %u checks passed\n",checks);
    return 0;
}
