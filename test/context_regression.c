#include "hal_c_api.h"
#include "Greemaster/main_demo.h"
#include "Greemaster/device_table.h"
#include "Greemaster/servo_step.h"
#include "Greemaster/entry_access.h"
#include <assert.h>
#include <math.h>
#include <pthread.h>

static unsigned checks;
#define CHECK(x) do { ++checks; if (!(x)) { fprintf(stderr, "line %d: %s\n", __LINE__, #x); abort(); } } while (0)
static DeviceSlot slots[4];
static uint32_t inputs[4][64], output[4][64], servo[2][DEV_DICT_ROLE_COUNT];
static int opened, starts, closes, sends, reads, writes, steps, waits;
static int fail_init, fail_read, fail_write, fail_step, fail_wait, fail_send, fail_calloc;
static int wrong_type, bad_width;
static int omit_optional, omit_spindle_speed;
static int stuck_fault;      /* 非零 = 连复位边沿也清不掉故障，用来测超时 */
static MasterBusHealth health;
static pthread_barrier_t entered, resume_wait;
static int block_wait;
static MasterConfig received_config;

void* __real_calloc(size_t n, size_t size);
void* __wrap_calloc(size_t n, size_t size) {
    return fail_calloc ? NULL : __real_calloc(n, size);
}

int ethercat_init(const MasterConfig* cfg) {
    ++starts;
    if (fail_init) return fail_init;
    received_config = *cfg;
    opened = 1;
    memset(slots, 0, sizeof(slots));
    memset(servo, 0, sizeof(servo));
    memset(output, 0, sizeof(output));
    for (int i = 0; i < 2; ++i) {
        slots[i].type = GREE_AXIS6_TYPE;
        slots[i].slave_pos = 0; slots[i].axis_index = i;
        slots[i].vendor_id = 123; slots[i].product_code = 456;
        slots[i].serial = 99;
        strcpy(slots[i].name, "Dual axis");
        slots[i].entries.slave.statusWord.bit_length = 16;
        slots[i].entries.slave.act_pos.bit_length = 32;
        slots[i].entries.slave.act_mode.bit_length = 8;
        slots[i].entries.slave.Control_word.bit_length = 16;
        slots[i].entries.slave.target_pos.bit_length = 32;
        slots[i].entries.slave.Modes_of_operation.bit_length = 8;
        slots[i].entries.slave.act_speed.bit_length = 32;
        slots[i].entries.slave.target_speed.bit_length = 32;
        slots[i].entries.slave.error_code.bit_length = 16;
        if ((i == 1 && omit_optional) || (i == 0 && omit_spindle_speed)) {
            slots[i].entries.slave.act_speed.bit_length = 0;
            slots[i].entries.slave.target_speed.bit_length = 0;
            slots[i].entries.slave.error_code.bit_length = 0;
        }
        servo[i][DEV_DICT_ROLE_ACTUAL_POS] = 100;
        servo[i][DEV_DICT_ROLE_STATUS_WORD] = 0x40;
        servo[i][DEV_DICT_ROLE_MODE_DISPLAY] = i ? 8 : 9;
        servo[i][DEV_DICT_ROLE_OP_MODE] = i ? 8 : 9;
    }
    slots[2].type = wrong_type ? SERVO_TYPE : IO_MODEL_TYPE;
    slots[2].slave_pos = 1; slots[2].axis_index = -1;
    slots[2].entries.io.io_in_count = 4;
    const int in_bits[] = {3, 12, 1, 32};
    for (int i = 0; i < 4; ++i) slots[2].entries.io.io_input_addr[i].bit_length = in_bits[i];
    if (bad_width) slots[2].entries.io.io_input_addr[0].bit_length = 0;
    slots[2].entries.io.io_out_count = 2;
    slots[2].entries.io.io_output_addr[0].bit_length = 5;
    slots[2].entries.io.io_output_addr[1].bit_length = 12;
    slots[3].type = CONTROL_PANEL_TYPE;
    slots[3].slave_pos = 2; slots[3].axis_index = -1;
    slots[3].entries.control.in_count = slots[3].entries.control.out_count = 1;
    slots[3].entries.control.input_addr[0].bit_length = 9;
    slots[3].entries.control.output_addr[0].bit_length = 9;
    inputs[2][0] = 5; inputs[2][1] = 0xABC; inputs[2][2] = 1; inputs[2][3] = 0x89ABCDEF;
    inputs[3][0] = 0x101;
    return 0;
}
int ethercat_close(void) { ++closes; opened = 0; return 0; }
const char* Master_StartupError(void) { return ""; }
int Master_SlaveCount(void) { return opened ? 3 : 0; }
const DEVICE_BASIC_INFO* device_identity_get(int pos) {
    static DEVICE_BASIC_INFO ids[3] = {
        {.ID=123, .CODE=456, .Serial=99},
        {.ID=2252, .CODE=269418497, .Revision=1},
        {.ID=2252, .CODE=269418498, .Revision=1}
    };
    return &ids[pos];
}
int DeviceTable_Get(const DeviceSlot** out) { if (out) *out = opened ? slots : NULL; return opened ? 4 : 0; }
int Master_WaitCycle(void) {
    ++waits;
    if (block_wait) { pthread_barrier_wait(&entered); pthread_barrier_wait(&resume_wait); }
    return fail_wait ? -1 : 0;
}
int Master_CommitCycle(void) {
    ++sends;
    if (fail_send) return -1;
    for (int i = 0; i < 2; ++i) {
        switch (servo[i][DEV_DICT_ROLE_CONTROL_WORD]) {
        case 0: servo[i][DEV_DICT_ROLE_STATUS_WORD] = 0x40; break;
        case 6: servo[i][DEV_DICT_ROLE_STATUS_WORD] = 0x21; break;
        case 7: servo[i][DEV_DICT_ROLE_STATUS_WORD] = 0x23; break;
        case 15: servo[i][DEV_DICT_ROLE_STATUS_WORD] = 0x237; break;
        /* 0x80 = 故障复位边沿。真实驱动器收到边沿且故障确实消失后，会回到
         * SwitchOnDisabled。stuck_fault 用来模拟"边沿发了但故障还在"。 */
        case 128: if (!stuck_fault) servo[i][DEV_DICT_ROLE_STATUS_WORD] = 0x40; break;
        }
        servo[i][DEV_DICT_ROLE_MODE_DISPLAY] = servo[i][DEV_DICT_ROLE_OP_MODE];
    }
    return 0;
}
int Master_ServoRead(int slot, DevDictRole role, uint32_t* out) {
    ++reads;
    if (role == DEV_DICT_ROLE_ERROR_CODE && !slots[slot].entries.slave.error_code.bit_length) return -1;
    if (role == DEV_DICT_ROLE_ACTUAL_SPEED && !slots[slot].entries.slave.act_speed.bit_length) return -1;
    if (fail_read) return -1;
    *out = servo[slot][role]; return 0;
}
int Master_ServoWrite(int slot, DevDictRole role, uint32_t value) {
    ++writes;
    if (role == DEV_DICT_ROLE_TARGET_SPEED && !slots[slot].entries.slave.target_speed.bit_length) return -1;
    if (fail_write) return -1;
    servo[slot][role] = value; return 0;
}
int Master_ServoStep(int slot, Ds402Request req, uint16_t mode, uint16_t* out) {
    ++steps;
    if (fail_step) return -1;
    const uint16_t sw = servo[slot][DEV_DICT_ROLE_STATUS_WORD];
    if (out) *out = sw;
    Ds402Step s = Ds402_NextStepReq(sw, servo[slot][DEV_DICT_ROLE_MODE_DISPLAY], req, mode, DS402_MODESW_DISABLE_FIRST);
    if (s.preset_target && Master_ServoWrite(slot, DEV_DICT_ROLE_TARGET_POS, servo[slot][DEV_DICT_ROLE_ACTUAL_POS])) return -1;
    if (s.op_mode != 0xFFFF && Master_ServoWrite(slot, DEV_DICT_ROLE_OP_MODE, s.op_mode)) return -1;
    if (s.kind != DS402_STEP_NONE && s.kind != DS402_STEP_DONE && s.kind != DS402_STEP_SWITCH_MODE)
        return Master_ServoWrite(slot, DEV_DICT_ROLE_CONTROL_WORD, s.control_word);
    return 0;
}
int Master_BusHealth(MasterBusHealth* out) {
    if (!out) return -1;
    *out = health;
    return 0;
}
int Master_IoReadEntry(int slot, int index, uint32_t* out, int* bits) {
    ++reads;
    if (fail_read) return -1;
    *out = inputs[slot][index];
    *bits = slot == 3 ? slots[slot].entries.control.input_addr[index].bit_length : slots[slot].entries.io.io_input_addr[index].bit_length;
    return 0;
}
int Master_IoWriteEntry(int slot, int index, uint32_t value) {
    ++writes;
    if (fail_write) return -1;
    output[slot][index] = value; return 0;
}

static HalCConfig config(void) {
    HalCConfig c = {0};
    c.abi_major = HAL_C_ABI_MAJOR; c.abi_minor = HAL_C_ABI_MINOR; c.struct_size = sizeof(c);
    c.cycle_us = 1000; c.start_timeout_ms = 120000; c.cycle_timeout_ms = 2000;
    c.fault_reset_timeout_ms = 2000; c.dc_enable = 1;
    c.axis_count = 1;
    HalCAxisCfg* a = &c.axes[0];
    a->axis_index = 1; a->logical_axis = 7; a->estop_action = HAL_ESTOP_DISABLE_OPERATION;
    a->work_mode = HAL_WORK_POSITION; a->encoder_type = HAL_ENC_ABSOLUTE;
    a->feedback_pulses_per_rev = 1000;
    a->command_units_per_count = .1; a->feedback_units_per_count = .2;
    c.spindle_count = 1;
    c.spindles[0].axis = *a; c.spindles[0].axis.axis_index = 0;
    c.spindles[0].axis.logical_axis = 3;
    c.spindles[0].axis.work_mode = HAL_WORK_VELOCITY;
    c.spindles[0].axis.estop_action = HAL_ESTOP_DISABLE_VOLTAGE;
    c.spindles[0].axis.command_units_per_count = .5;
    c.spindles[0].axis.feedback_units_per_count = .25;
    c.spindles[0].axis.feedback_wrap = HAL_WRAP_MODULAR;
    c.spindles[0].max_speed = 1000; c.spindles[0].speed_window = .5;
    c.io_count = c.panel_count = 1;
    c.ios[0].slave_pos = 1; c.ios[0].x_start = 2; c.ios[0].y_start = 1;
    c.panels[0].slave_pos = 2; c.panels[0].x_start = 10; c.panels[0].y_start = 6;
    return c;
}
static HalContext* create(const HalCConfig* cfg) {
    HalContext* c = NULL; char err[128];
    CHECK(hal_context_create(cfg, &c, err, sizeof(err)) == 0 && c && !err[0]);
    return c;
}
static void begin(HalContext* c) { CHECK(hal_rt_wait_cycle(c) == 0); CHECK(hal_rt_begin_cycle(c) == 0); }
static void cycle(HalContext* c) { begin(c); CHECK(hal_rt_commit_cycle(c) == 0); }
static void enable(HalContext* c) {
    CHECK(hal_rt_axis_enable(c, 7, 1) == 0);
    CHECK(hal_rt_spindle_enable(c, 3, 1) == 0);
    for (int i = 0; i < 4; ++i) cycle(c);
}

static void lifecycle(void) {
    HalCConfig cfg = config();
    HalContext* c = create(&cfg);
    CHECK(starts == 0);
    CHECK(hal_axis_count(c) == 2);
    HalAxisId id;
    CHECK(hal_axis_resolve(c, 7, &id) == 0 && id == 7);
    CHECK(hal_axis_resolve(c, -1, &id) == HAL_ERROR_ARGUMENT && id == HAL_INVALID_ID);
    fail_init = 1;
    CHECK(hal_context_start(c, NULL, 0) == HAL_ERROR_BUS);
    fail_init = MASTER_DEVICE_DICTIONARY_ERROR;
    char dict_error[128];
    CHECK(hal_context_start(c, dict_error, sizeof(dict_error)) == HAL_ERROR_CONFIG);
    CHECK(strstr(dict_error, "CNC_HAL_DEVICES_JSON") != NULL);
    fail_init = 0;
    CHECK(hal_context_start(c, NULL, 0) == 0);
    CHECK(hal_slave_count(c) == 3);
    HalCSlaveInfo slave;
    CHECK(hal_slave_info(c, 0, &slave) == 0 && slave.axis_count == 2 && slave.identity.serial == 99);
    CHECK(hal_slave_info(c, 1, &slave) == 0 && slave.axis_count == 0 && slave.identity.type == IO_MODEL_TYPE);
    CHECK(received_config.cycle_us == 1000 && received_config.cycle_timeout_ms == 2000);
    HalContext* other = create(&cfg);
    const int before = starts;
    CHECK(hal_context_start(other, NULL, 0) == HAL_ERROR_BUSY && starts == before);
    const int close_before = closes;
    hal_context_destroy(other);
    CHECK(closes == close_before && opened);
    HalCIdentity ident;
    CHECK(hal_device_identity(c, 7, &ident) == 0 && ident.serial == 99 && ident.device_id == 99);
    HalCAxisStatus status;
    CHECK(hal_rt_axis_read_status(c, 7, &status) == HAL_ERROR_NOT_RUNNING);
    CHECK(hal_rt_begin_cycle(c) == HAL_ERROR_STATE);
    cycle(c);
    CHECK(hal_rt_axis_write_pos(c, 7, 1) == HAL_ERROR_NOT_RUNNING);
    CHECK(hal_context_request_stop(c) == 0);
    const int waits_before = waits;
    CHECK(hal_rt_wait_cycle(c) == HAL_ERROR_STOPPED && waits == waits_before);
    CHECK(hal_rt_commit_cycle(c) == HAL_ERROR_STOPPED);
    CHECK(hal_context_stop(c) == 0 && !opened);
    CHECK(hal_context_stop(c) == 0 && closes == close_before + 1);
    CHECK(hal_context_start(c, NULL, 0) == 0);
    cycle(c);
    CHECK(hal_rt_axis_read_status(c, 7, &status) == 0 && !status.enabled);
    hal_context_destroy(c);
    fail_calloc = 1; c = (void*)1;
    CHECK(hal_context_create(&cfg, &c, NULL, 0) == HAL_ERROR_MEMORY && !c);
    fail_calloc = 0;
    cfg.topology_fingerprint = 123;
    CHECK(hal_context_create(&cfg, &c, NULL, 0) == HAL_ERROR_CONFIG && !c);
}

static void binding(void) {
    for (int kind = 0; kind < 5; ++kind) {
        HalCConfig cfg = config();
        if (kind == 0) cfg.axes[0].axis_index = 5;
        if (kind == 1) cfg.panels[0].x_start = 7;
        if (kind == 2) cfg.panels[0].y_start = 3;
        wrong_type = kind == 3; bad_width = kind == 4;
        HalContext* c = create(&cfg);
        const int close_before = closes;
        char error[100];
        CHECK(hal_context_start(c, error, sizeof(error)) == HAL_ERROR_CONFIG && error[0]);
        CHECK(!opened && closes == close_before + 1);
        hal_context_destroy(c);
    }
    wrong_type = bad_width = 0;
    HalCConfig cfg = config(); cfg.panels[0].x_start = 8; cfg.panels[0].y_start = 4;
    HalContext* c = create(&cfg);
    CHECK(hal_context_start(c, NULL, 0) == 0);
    hal_context_destroy(c);
}

static void optional_capabilities(void) {
    HalCConfig cfg = config();
    omit_optional = 1;
    HalContext* c = create(&cfg);
    CHECK(hal_context_start(c, NULL, 0) == 0);
    HalCCapability cap;
    CHECK(hal_axis_capability(c, 7, HAL_FUNC_POSITION, &cap) == 0 && cap.state == HAL_CAP_READY);
    CHECK(hal_axis_capability(c, 7, HAL_FUNC_SPEED, &cap) == 0 && cap.state == HAL_CAP_UNSUPPORTED);
    CHECK(cap.slave_pos == 0 && cap.axis_index == 1 && cap.reason[0]);
    CHECK(hal_axis_capability(c, 7, HAL_FUNC_ERROR_CODE, &cap) == 0 && cap.state == HAL_CAP_UNSUPPORTED);
    CHECK(hal_axis_capability(c, 7, HAL_FUNC_ALARM_CONTROL, &cap) == HAL_ERROR_ARGUMENT);
    CHECK(hal_slave_capability(c, 0, HAL_FUNC_ALARM_CONTROL, &cap) == 0 &&
          cap.state == HAL_CAP_NOT_CONFIGURED && cap.axis_index == -1);
    CHECK(hal_axis_capability(c, 3, HAL_FUNC_SPEED, &cap) == 0 && cap.state == HAL_CAP_READY);
    cycle(c);
    HalCAxisStatus status;
    CHECK(hal_rt_axis_read_status(c, 7, &status) == 0 && !status.error_code_valid);
    CHECK(hal_rt_axis_read_status(c, 3, &status) == 0 && status.error_code_valid);
    hal_context_destroy(c);
    omit_optional = 0;

    omit_spindle_speed = 1;
    c = create(&cfg);
    char error[160];
    CHECK(hal_context_start(c, error, sizeof(error)) == HAL_ERROR_CONFIG);
    CHECK(strstr(error, "从站 0 轴 0") && strstr(error, "主轴速度"));
    hal_context_destroy(c);
    omit_spindle_speed = 0;
}

static void motion_io(void) {
    HalCConfig cfg = config();
    cfg.axes[0].command_invert = 1;
    HalContext* c = create(&cfg);
    CHECK(hal_context_start(c, NULL, 0) == 0);
    enable(c);
    CHECK(servo[1][DEV_DICT_ROLE_TARGET_POS] == (uint32_t)-200);
    begin(c);
    const int read_before = reads, step_before = steps;
    HalCAxisStatus a;
    CHECK(hal_rt_axis_read_status(c, 7, &a) == 0 && a.enabled && a.actual_pos == 20);
    CHECK(hal_rt_axis_read_status(c, 7, &a) == 0 && reads == read_before && steps == step_before);
    CHECK(hal_rt_axis_write_pos(c, 7, 25) == 0);
    CHECK(hal_rt_axis_write_pos(c, 7, NAN) == HAL_ERROR_ARGUMENT);
    CHECK(hal_rt_axis_write_pos(c, 7, 1e300) == HAL_ERROR_ARGUMENT);
    CHECK(hal_rt_axis_set_pos(c, 7, 120) == 0);
    CHECK(hal_rt_axis_read_status(c, 7, &a) == 0 && a.actual_pos == 120 && a.command_pos == 125);
    uint8_t x[14]; memset(x, 0xAA, sizeof(x));
    CHECK(hal_rt_io_snapshot_inputs(c, x, sizeof(x)) == 0 && reads == read_before);
    const uint8_t expected[] = {0,0,0xE5,0xD5,0xEF,0xCD,0xAB,0x89,0,0,1,1,0,0};
    CHECK(memcmp(x, expected, sizeof(x)) == 0);
    CHECK(hal_rt_io_snapshot_inputs(c, x, 11) == HAL_ERROR_ARGUMENT);
    const uint8_t y[] = {0,0xA5,0x5A,1,0,0,0xAA,1};
    CHECK(hal_rt_io_flush_outputs(c, y, sizeof(y)) == 0);
    CHECK(hal_rt_io_flush_outputs(c, y, 7) == HAL_ERROR_ARGUMENT);
    CHECK(hal_rt_spindle_write_speed(c, 3, 10, -1) == 0);
    CHECK(hal_rt_commit_cycle(c) == 0);
    CHECK(servo[1][DEV_DICT_ROLE_TARGET_POS] == (uint32_t)-250);
    /* 速度 PDO 与 rpm 是 1:1：指令 10 rpm、dir=-1，写进 PDO 的就该是 -10。
     * 此前按 counts/s 假设会写成 10*(-1)*6/0.5 = -120。 */
    CHECK(servo[0][DEV_DICT_ROLE_TARGET_SPEED] == (uint32_t)-10);
    CHECK(output[2][0] == 5 && output[2][1] == 0xAD5 && output[3][0] == 0x1AA);
    HalCSpindleStatus s;
    /* 读侧同样 1:1：喂一个 counts/s 换算得不出来的值，证明没偷偷乘当量。
     * 反馈当量 .25，旧式 -1234*.25/6 会得到 -51。 */
    servo[0][DEV_DICT_ROLE_ACTUAL_SPEED] = (uint32_t)-1234;
    begin(c);
    CHECK(hal_rt_spindle_read_status(c, 3, &s) == 0 && s.actual_speed == -1234 && !s.at_speed);
    /* 贴近指令时 at_speed 置位：指令 -10 rpm，转速窗口 .5。
     * 上一拍已 begin 未 commit，必须先提交才能进下一次 wait。 */
    CHECK(hal_rt_commit_cycle(c) == 0);
    servo[0][DEV_DICT_ROLE_ACTUAL_SPEED] = (uint32_t)-10;
    begin(c);
    CHECK(hal_rt_spindle_read_status(c, 3, &s) == 0 && s.actual_speed == -10 && s.at_speed);
    CHECK(hal_rt_spindle_request_mode(c, 3, HAL_SPINDLE_CSP) == 0);
    CHECK(hal_rt_spindle_write_speed(c, 3, 10, 1) == HAL_ERROR_NOT_RUNNING);
    CHECK(hal_rt_spindle_estop(c, 3) == 0);
    CHECK(hal_rt_axis_write_pos(c, 7, 130) == 0);
    CHECK(hal_rt_axis_estop(c, 7) == 0);
    CHECK(hal_rt_commit_cycle(c) == 0);
    CHECK(servo[0][DEV_DICT_ROLE_CONTROL_WORD] == 0 && servo[0][DEV_DICT_ROLE_TARGET_SPEED] == 0);
    CHECK(servo[1][DEV_DICT_ROLE_CONTROL_WORD] == 7 && servo[1][DEV_DICT_ROLE_TARGET_POS] == (uint32_t)-250);
    for (int i = 0; i < 3; ++i) cycle(c);
    CHECK(servo[0][DEV_DICT_ROLE_OP_MODE] == 9 && servo[0][DEV_DICT_ROLE_CONTROL_WORD] == 0);
    CHECK(servo[1][DEV_DICT_ROLE_CONTROL_WORD] == 7);
    CHECK(hal_rt_spindle_request_mode(c, 3, HAL_SPINDLE_CSP) == 0);
    CHECK(hal_rt_spindle_enable(c, 3, 1) == 0);
    for (int i = 0; i < 5; ++i) cycle(c);
    begin(c);
    CHECK(hal_rt_spindle_read_status(c, 3, &s) == 0 && s.mode == HAL_SPINDLE_CSP && s.enabled);
    CHECK(hal_rt_spindle_write_pos(c, 3, 30) == 0);
    CHECK(hal_rt_commit_cycle(c) == 0 && servo[0][DEV_DICT_ROLE_TARGET_POS] == 60);
    hal_context_destroy(c);
}

static void wrap_and_faults(void) {
    HalCConfig cfg = config();
    HalContext* c = create(&cfg);
    CHECK(hal_context_start(c, NULL, 0) == 0);
    servo[0][DEV_DICT_ROLE_ACTUAL_POS] = 900;
    servo[1][DEV_DICT_ROLE_ACTUAL_POS] = INT32_MAX;
    cycle(c);
    servo[0][DEV_DICT_ROLE_ACTUAL_POS] = 100;
    servo[1][DEV_DICT_ROLE_ACTUAL_POS] = (uint32_t)INT32_MAX + 1u;
    cycle(c);
    double pos;
    CHECK(hal_rt_axis_read_pos(c, 3, &pos) == 0 && pos == 275);
    CHECK(hal_rt_axis_read_pos(c, 7, &pos) == 0 && fabs(pos - 429496729.6) < 1e-6);
    CHECK(hal_rt_spindle_enable(c, 7, 1) == HAL_ERROR_ARGUMENT);
    hal_context_destroy(c);
    int* failures[] = {&fail_wait, &fail_read, &fail_step, &fail_write, &fail_send};
    for (int i = 0; i < 5; ++i) {
        c = create(&cfg); CHECK(hal_context_start(c, NULL, 0) == 0);
        *failures[i] = 1;
        int rc = hal_rt_wait_cycle(c);
        if (!rc) rc = hal_rt_begin_cycle(c);
        if (!rc) rc = hal_rt_commit_cycle(c);
        CHECK(rc == HAL_ERROR_BUS);
        *failures[i] = 0;
        const int before = waits + reads + writes + steps + sends;
        CHECK(hal_rt_wait_cycle(c) == HAL_ERROR_BUS);
        CHECK(hal_rt_axis_enable(c, 7, 1) == HAL_ERROR_BUS);
        CHECK(hal_rt_axis_read_pos(c, 7, &pos) == HAL_ERROR_BUS);
        CHECK(hal_rt_axis_estop(c, 7) == 0 && hal_rt_spindle_estop(c, 3) == 0);
        CHECK(before == waits + reads + writes + steps + sends);
        CHECK(hal_context_stop(c) == 0 && hal_context_start(c, NULL, 0) == 0);
        cycle(c); hal_context_destroy(c);
    }
}

static void stop_overrides_staged_enable(void) {
    HalCConfig cfg = config();
    HalContext* c = create(&cfg);
    CHECK(hal_context_start(c, NULL, 0) == 0);
    CHECK(hal_rt_axis_enable(c, 7, 1) == 0);
    CHECK(hal_rt_spindle_enable(c, 3, 1) == 0);
    cycle(c); cycle(c);
    begin(c);
    CHECK(servo[0][DEV_DICT_ROLE_CONTROL_WORD] == 15 && servo[1][DEV_DICT_ROLE_CONTROL_WORD] == 15);
    CHECK(hal_rt_axis_estop(c, 7) == 0 && hal_rt_spindle_estop(c, 3) == 0);
    CHECK(hal_rt_commit_cycle(c) == 0);
    CHECK(servo[0][DEV_DICT_ROLE_CONTROL_WORD] == 0 && servo[1][DEV_DICT_ROLE_CONTROL_WORD] == 7);
    CHECK(servo[0][DEV_DICT_ROLE_STATUS_WORD] == 0x40 && servo[1][DEV_DICT_ROLE_STATUS_WORD] == 0x23);
    cycle(c);
    CHECK(servo[0][DEV_DICT_ROLE_CONTROL_WORD] == 0 && servo[1][DEV_DICT_ROLE_CONTROL_WORD] == 7);
    begin(c);
    const uint8_t y[8] = {0};
    CHECK(hal_rt_io_flush_outputs(c, y, sizeof(y)) == 0);
    const int before = sends;
    fail_write = 1;
    CHECK(hal_rt_commit_cycle(c) == HAL_ERROR_BUS && sends == before);
    fail_write = 0;
    CHECK(hal_rt_commit_cycle(c) == HAL_ERROR_BUS && sends == before);
    hal_context_destroy(c);
}

static void* wait_thread(void* c) { return (void*)(intptr_t)hal_rt_wait_cycle(c); }
static void concurrent_stop(void) {
    HalCConfig cfg = config();
    HalContext* c = create(&cfg);
    CHECK(hal_context_start(c, NULL, 0) == 0);
    CHECK(pthread_barrier_init(&entered, NULL, 2) == 0);
    CHECK(pthread_barrier_init(&resume_wait, NULL, 2) == 0);
    block_wait = 1;
    pthread_t thread; CHECK(pthread_create(&thread, NULL, wait_thread, c) == 0);
    pthread_barrier_wait(&entered);
    const int before = closes;
    CHECK(hal_context_request_stop(c) == 0 && closes == before);
    pthread_barrier_wait(&resume_wait);
    void* rc; CHECK(pthread_join(thread, &rc) == 0 && (intptr_t)rc == HAL_ERROR_STOPPED);
    block_wait = 0;
    pthread_barrier_destroy(&entered); pthread_barrier_destroy(&resume_wait);
    CHECK(hal_context_stop(c) == 0 && closes == before + 1);
    hal_context_destroy(c);
}

/* 累计反馈越过 int32 后，重新使能仍用原始 PDO 预置，不能误闭锁总线。 */
static void preset_overflow_probe(void) {
    HalCConfig cfg = config();
    cfg.axes[0].feedback_units_per_count = cfg.axes[0].command_units_per_count;
    HalContext* c = create(&cfg);
    CHECK(hal_context_start(c, NULL, 0) == 0);

    const int slot = 1;                      /* 逻辑轴 7 → 从站 0 物理轴 1 */

    cycle(c);                                /* 第一拍取 counts 初值 */
    for (int i = 0; i < 3; ++i) {            /* 每拍 +1e9 计数（< 2^31，增量合法） */
        servo[slot][DEV_DICT_ROLE_ACTUAL_POS] += 1000000000u;
        cycle(c);
    }

    double pos = -1;
    CHECK(hal_rt_axis_read_pos(c, 7, &pos) == 0 && pos > 0);
    CHECK(hal_rt_axis_enable(c, 7, 1) == 0);
    cycle(c);                                /* 第一拍 SHUTDOWN */
    cycle(c);                                /* 预置拍 */
    CHECK(servo[slot][DEV_DICT_ROLE_TARGET_POS] == servo[slot][DEV_DICT_ROLE_ACTUAL_POS]);
    cycle(c);
    cycle(c);
    CHECK(hal_rt_axis_read_pos(c, 7, &pos) == 0 && pos > 0);
    begin(c);
    CHECK(hal_rt_axis_write_pos(c, 7, pos + 1.0) == 0);
    CHECK(hal_rt_commit_cycle(c) == 0);
    CHECK(servo[slot][DEV_DICT_ROLE_TARGET_POS] == servo[slot][DEV_DICT_ROLE_ACTUAL_POS] + 10u);

    hal_context_stop(c);
    hal_context_destroy(c);

    cfg = config(); /* 不同计数尺度仍不可直接复制原始 PDO。 */
    c = create(&cfg);
    CHECK(hal_context_start(c, NULL, 0) == 0);
    cycle(c);
    for (int i = 0; i < 3; ++i) {
        servo[slot][DEV_DICT_ROLE_ACTUAL_POS] += 1000000000u;
        cycle(c);
    }
    CHECK(hal_rt_axis_enable(c, 7, 1) == 0);
    cycle(c);
    CHECK(hal_rt_wait_cycle(c) == 0);
    const int prior_steps = steps;
    CHECK(hal_rt_begin_cycle(c) == HAL_ERROR_ARGUMENT);
    CHECK(steps == prior_steps); /* 所有轴都未写本拍控制 PDO。 */
    CHECK(hal_rt_axis_read_pos(c, 7, &pos) == 0);
    CHECK(hal_context_stop(c) == 0);
    hal_context_destroy(c);
}

static void spindle_origin_angle(void) {
    HalCConfig cfg = config();
    cfg.spindles[0].axis.command_units_per_count = 1.0;
    cfg.spindles[0].axis.feedback_units_per_count = 1.0;
    cfg.spindles[0].axis.feedback_wrap = HAL_WRAP_LINEAR;
    HalContext* c = create(&cfg);
    CHECK(hal_context_start(c, NULL, 0) == 0);
    cycle(c);
    CHECK(hal_rt_axis_set_pos(c, 3, 0) == 0);
    CHECK(hal_rt_spindle_request_mode(c, 3, HAL_SPINDLE_CSP) == 0);
    CHECK(hal_rt_spindle_enable(c, 3, 1) == 0);
    for (int i = 0; i < 5; ++i) cycle(c);

    begin(c);
    CHECK(hal_rt_spindle_write_pos(c, 3, -5) == 0);
    CHECK(hal_rt_commit_cycle(c) == 0);
    CHECK(servo[0][DEV_DICT_ROLE_TARGET_POS] == 95u); /* 从零点反转到 -5° */

    begin(c);
    CHECK(hal_rt_spindle_write_pos(c, 3, 350) == 0);
    CHECK(hal_rt_commit_cycle(c) == 0);
    CHECK(servo[0][DEV_DICT_ROLE_TARGET_POS] == 450u); /* 沿正向到 +350° */

    begin(c);
    CHECK(hal_rt_spindle_write_pos(c, 3, 360) == 0);
    CHECK(hal_rt_commit_cycle(c) == 0);
    CHECK(servo[0][DEV_DICT_ROLE_TARGET_POS] == 810u);

    begin(c);
    CHECK(hal_rt_spindle_write_pos(c, 3, -360) == 0);
    CHECK(hal_rt_commit_cycle(c) == 0);
    CHECK(servo[0][DEV_DICT_ROLE_TARGET_POS] == 450u);

    servo[0][DEV_DICT_ROLE_ACTUAL_POS] = 450;
    cycle(c); /* 当前相对启动零点 +350° */
    begin(c);
    CHECK(hal_rt_spindle_write_pos(c, 3, 10) == 0);
    CHECK(hal_rt_commit_cycle(c) == 0);
    CHECK(servo[0][DEV_DICT_ROLE_TARGET_POS] == 470u); /* +20° 到下一圈的 10° */

    servo[0][DEV_DICT_ROLE_ACTUAL_POS] = 470;
    cycle(c);
    begin(c);
    CHECK(hal_rt_spindle_write_pos(c, 3, 5) == 0);
    CHECK(hal_rt_commit_cycle(c) == 0);
    CHECK(servo[0][DEV_DICT_ROLE_TARGET_POS] == 825u); /* +10° → +5°，正转 355° */

    begin(c);
    CHECK(hal_rt_spindle_write_pos(c, 3, 5) == 0);
    CHECK(hal_rt_commit_cycle(c) == 0);
    CHECK(servo[0][DEV_DICT_ROLE_TARGET_POS] == 825u); /* 重复普通刻度不再转圈 */

    begin(c);
    CHECK(hal_rt_spindle_write_pos(c, 3, 361) == HAL_ERROR_ARGUMENT);
    CHECK(hal_rt_spindle_write_pos(c, 3, NAN) == HAL_ERROR_ARGUMENT);
    CHECK(hal_rt_commit_cycle(c) == 0);

    hal_context_destroy(c);

    c = create(&cfg);
    CHECK(hal_context_start(c, NULL, 0) == 0);
    cycle(c);
    CHECK(hal_rt_axis_set_pos(c, 3, 0) == 0);
    for (int i = 0; i < 3; ++i) {
        servo[0][DEV_DICT_ROLE_ACTUAL_POS] += 1000000000u;
        cycle(c);
    }
    CHECK(hal_rt_spindle_request_mode(c, 3, HAL_SPINDLE_CSP) == 0);
    CHECK(hal_rt_spindle_enable(c, 3, 1) == 0);
    for (int i = 0; i < 5; ++i) cycle(c);
    CHECK(servo[0][DEV_DICT_ROLE_TARGET_POS] == servo[0][DEV_DICT_ROLE_ACTUAL_POS]);
    begin(c);
    CHECK(hal_rt_spindle_write_pos(c, 3, 10) == 0);
    CHECK(hal_rt_commit_cycle(c) == 0);
    CHECK(servo[0][DEV_DICT_ROLE_TARGET_POS] == servo[0][DEV_DICT_ROLE_ACTUAL_POS] + 250u);
    hal_context_destroy(c);

    c = create(&cfg);
    CHECK(hal_context_start(c, NULL, 0) == 0);
    servo[0][DEV_DICT_ROLE_ACTUAL_POS] = 0xFFFFFFFEu;
    cycle(c);
    CHECK(hal_rt_axis_set_pos(c, 3, 0) == 0);
    CHECK(hal_rt_spindle_request_mode(c, 3, HAL_SPINDLE_CSP) == 0);
    CHECK(hal_rt_spindle_enable(c, 3, 1) == 0);
    for (int i = 0; i < 5; ++i) cycle(c);
    servo[0][DEV_DICT_ROLE_ACTUAL_POS] = 0xFFFFFFFFu;
    cycle(c);
    begin(c);
    CHECK(hal_rt_spindle_write_pos(c, 3, 10) == 0);
    CHECK(hal_rt_commit_cycle(c) == 0);
    CHECK(servo[0][DEV_DICT_ROLE_TARGET_POS] == 8u);
    hal_context_destroy(c);

    c = create(&cfg);
    CHECK(hal_context_start(c, NULL, 0) == 0);
    servo[0][DEV_DICT_ROLE_ACTUAL_POS] = 0x7FFFFFFEu;
    cycle(c);
    CHECK(hal_rt_axis_set_pos(c, 3, 0) == 0);
    CHECK(hal_rt_spindle_request_mode(c, 3, HAL_SPINDLE_CSP) == 0);
    CHECK(hal_rt_spindle_enable(c, 3, 1) == 0);
    for (int i = 0; i < 5; ++i) cycle(c);
    servo[0][DEV_DICT_ROLE_ACTUAL_POS] = 0x7FFFFFFFu;
    cycle(c);
    begin(c);
    CHECK(hal_rt_spindle_write_pos(c, 3, 10) == 0);
    CHECK(hal_rt_commit_cycle(c) == 0);
    CHECK(servo[0][DEV_DICT_ROLE_TARGET_POS] == 0x80000008u);
    hal_context_destroy(c);
}


/* 小数刻度、取消后的基准，以及上机用例所用的提交前限幅流程。 */
static HalContext* angle_context(double origin, int estop) {
    HalCConfig cfg = config();
    cfg.spindles[0].axis.command_units_per_count = .001;
    cfg.spindles[0].axis.feedback_units_per_count = .001;
    cfg.spindles[0].axis.feedback_wrap = HAL_WRAP_LINEAR;
    cfg.spindles[0].axis.estop_action = estop;
    HalContext* c = create(&cfg);
    CHECK(hal_context_start(c, NULL, 0) == 0);
    cycle(c);
    CHECK(hal_rt_axis_set_pos(c, 3, origin) == 0);
    CHECK(hal_rt_spindle_request_mode(c, 3, HAL_SPINDLE_CSP) == 0);
    enable(c);
    return c;
}
static void angle_regressions(void) {
    HalCAxisStatus status;
    for (int sign = -1; sign <= 1; sign += 2) {
        HalContext* c = angle_context(sign * 720., HAL_ESTOP_DISABLE_OPERATION);
        uint32_t first = 0;
        for (int i = 0; i < 4; ++i) {
            begin(c);
            CHECK(hal_rt_spindle_write_pos(c, 3, sign * .2) == 0);
            CHECK(hal_rt_axis_read_status(c, 3, &status) == 0);
            CHECK(fabs(status.command_pos - sign * 720.2) < 1e-9);
            CHECK(hal_rt_spindle_write_pos(c, 3, NAN) == HAL_ERROR_ARGUMENT);
            CHECK(hal_rt_commit_cycle(c) == 0);
            if (!i) first = servo[0][DEV_DICT_ROLE_TARGET_POS];
            CHECK(servo[0][DEV_DICT_ROLE_TARGET_POS] == first);
        }
        /* 累计位置写入和设零必须使旧刻度缓存失效。 */
        begin(c);
        CHECK(hal_rt_axis_write_pos(c, 3, sign * 730.) == 0);
        CHECK(hal_rt_spindle_write_pos(c, 3, sign * .2) == 0);
        CHECK(hal_rt_axis_read_status(c, 3, &status) == 0);
        CHECK(fabs(status.command_pos - sign * 1080.2) < 1e-9);
        CHECK(hal_rt_axis_set_pos(c, 3, sign * 721.) == 0);
        CHECK(hal_rt_spindle_write_pos(c, 3, sign * .2) == 0);
        CHECK(hal_rt_axis_read_status(c, 3, &status) == 0);
        CHECK(fabs(status.command_pos - sign * 1440.2) < 1e-9);
        for (int i = 0; i < 2; ++i)
            CHECK(hal_rt_spindle_write_pos(c, 3, sign * 360.) == 0);
        CHECK(hal_rt_axis_read_status(c, 3, &status) == 0);
        CHECK(fabs(status.command_pos - sign * 2160.2) < 1e-9);
        CHECK(hal_rt_commit_cycle(c) == 0);
        hal_context_destroy(c);
    }
    for (int action = 0; action < 4; ++action) {
        HalContext* c = angle_context(0, action == 2 ? HAL_ESTOP_DISABLE_VOLTAGE : HAL_ESTOP_DISABLE_OPERATION);
        const uint32_t old_target = servo[0][DEV_DICT_ROLE_TARGET_POS];
        begin(c);
        CHECK(hal_rt_spindle_write_pos(c, 3, 350) == 0);
        if (action == 0) CHECK(hal_rt_spindle_enable(c, 3, 0) == 0);
        else if (action == 3) CHECK(hal_rt_spindle_request_mode(c, 3, HAL_SPINDLE_CSV) == 0);
        else CHECK(hal_rt_spindle_estop(c, 3) == 0);
        CHECK(hal_rt_commit_cycle(c) == 0);
        CHECK(servo[0][DEV_DICT_ROLE_TARGET_POS] == old_target);
        for (int i = 0; i < 4; ++i) cycle(c);
        CHECK(hal_rt_spindle_request_mode(c, 3, HAL_SPINDLE_CSP) == 0);
        CHECK(hal_rt_spindle_enable(c, 3, 1) == 0);
        for (int i = 0; i < 5; ++i) cycle(c);
        begin(c);
        CHECK(hal_rt_spindle_write_pos(c, 3, 5) == 0);
        CHECK(hal_rt_axis_read_status(c, 3, &status) == 0);
        CHECK(fabs(status.command_pos - 5) < 1e-9);
        CHECK(hal_rt_commit_cycle(c) == 0);
        hal_context_destroy(c);
    }
    HalContext* c = angle_context(.1, HAL_ESTOP_DISABLE_OPERATION);
    const uint32_t old_target = servo[0][DEV_DICT_ROLE_TARGET_POS];
    servo[0][DEV_DICT_ROLE_ACTUAL_POS] -= 200u; /* 使能后从 +.1 漂到 -.1 */
    begin(c);
    CHECK(hal_rt_spindle_write_pos(c, 3, .05) == 0);
    CHECK(hal_rt_axis_read_status(c, 3, &status) == 0);
    CHECK(fabs(status.actual_pos + .1) < 1e-9);
    CHECK(fabs(status.command_pos - 360.05) < 1e-9);
    CHECK(fabs(status.command_pos - status.actual_pos) > 1.0);
    CHECK(hal_rt_spindle_estop(c, 3) == 0); /* 提交前超限取消 */
    CHECK(hal_rt_commit_cycle(c) == 0);
    CHECK(servo[0][DEV_DICT_ROLE_TARGET_POS] == old_target);
    hal_context_destroy(c);
}

static void fault_reset_and_bus_health(void) {
    /* ---- 总线健康：HAL 只搬运不判断，逐字段核对映射 ---- */
    memset(&health, 0, sizeof(health));
    health.pdo_warn = 1; health.pdo_warn_code = 7; health.pdo_warn_para = 0x1234;
    health.dc_warn = 1; health.dc_warn_code = 9; health.dc_warn_para = 0x5678;
    health.crc_err_count = 11; health.frame_timeout_count = 22;
    health.expect_wkc_tx = 33; health.expect_wkc_rx = 44;

    HalCConfig cfg = config();
    HalContext* c = create(&cfg);
    CHECK(hal_context_start(c, NULL, 0) == 0);

    HalCBusHealth h;
    CHECK(hal_bus_health(c, &h) == HAL_OK);
    CHECK(h.pdo_warn == 1 && h.pdo_warn_code == 7 && h.pdo_warn_para == 0x1234);
    CHECK(h.dc_warn == 1 && h.dc_warn_code == 9 && h.dc_warn_para == 0x5678);
    CHECK(h.crc_error_count == 11 && h.frame_timeout_count == 22);
    CHECK(h.expect_wkc_tx == 33 && h.expect_wkc_rx == 44);
    CHECK(hal_bus_health(NULL, &h) == HAL_ERROR_ARGUMENT);
    CHECK(hal_bus_health(c, NULL) == HAL_ERROR_ARGUMENT);

    /* ---- 故障复位：请求 → 边沿 → 反馈离开 Fault → DONE ---- */
    int32_t st = -1;
    CHECK(hal_rt_axis_fault_reset_state(c, 7, &st) == 0 && st == HAL_RESET_NONE);
    CHECK(hal_rt_axis_fault_reset_state(c, 999, &st) == HAL_ERROR_ARGUMENT);

    cycle(c);
    /* 把轴推进故障态：状态字置 Fault 位（bit3）。控制字设成 commit 不处理的值，
     * 免得被它按控制字重算状态字覆盖掉。 */
    servo[1][DEV_DICT_ROLE_STATUS_WORD] = 0x0008;   /* 轴 7 绑 slots[1]，不是 slots[0] */
    servo[1][DEV_DICT_ROLE_CONTROL_WORD] = 0xFF;
    begin(c);
    HalCAxisStatus s;
    CHECK(hal_rt_axis_read_status(c, 7, &s) == 0 && (s.raw_status & 0x0008u));

    /* 只允许在 begin 与 commit 之间受理（request 只有周期线程能写） */
    CHECK(hal_rt_axis_fault_reset(c, 7) == 0);
    CHECK(hal_rt_axis_fault_reset_state(c, 7, &st) == 0 && st == HAL_RESET_PENDING);
    /* 复位期间不能写位置：request 不是 ENABLE，motion_ready 为假 */
    CHECK(hal_rt_axis_write_pos(c, 7, 10) == HAL_ERROR_NOT_RUNNING);
    CHECK(hal_rt_commit_cycle(c) == 0);

    /* 下一拍 begin 才发出 0x80 边沿，该拍 commit 里 mock 让驱动器离开 Fault */
    cycle(c);
    /* 再下一拍 begin 采样到已离开 Fault → DONE，且请求被改成 DISABLE */
    begin(c);
    CHECK(hal_rt_axis_fault_reset_state(c, 7, &st) == 0 && st == HAL_RESET_DONE);
    CHECK(hal_rt_axis_read_status(c, 7, &s) == 0 && !s.enabled);
    CHECK(hal_rt_commit_cycle(c) == 0);
    hal_context_destroy(c);

    /* 阶段外调用被拒 */
    c = create(&cfg);
    CHECK(hal_context_start(c, NULL, 0) == 0);
    CHECK(hal_rt_axis_fault_reset(c, 7) == HAL_ERROR_STATE);
    hal_context_destroy(c);

    /* ---- 超时：边沿发了但故障一直不消失 ---- */
    stuck_fault = 1;
    HalCConfig cfg2 = config();
    cfg2.fault_reset_timeout_ms = 4;   /* 4ms ÷ 1ms 周期 = 4 拍 */
    HalContext* c2 = create(&cfg2);
    CHECK(hal_context_start(c2, NULL, 0) == 0);
    cycle(c2);
    servo[1][DEV_DICT_ROLE_STATUS_WORD] = 0x0008;
    servo[1][DEV_DICT_ROLE_CONTROL_WORD] = 0xFF;
    begin(c2);
    CHECK(hal_rt_axis_fault_reset(c2, 7) == 0);
    CHECK(hal_rt_commit_cycle(c2) == 0);
    st = HAL_RESET_PENDING;
    for (int i = 0; i < 10 && st == HAL_RESET_PENDING; ++i) {
        begin(c2);
        CHECK(hal_rt_axis_fault_reset_state(c2, 7, &st) == 0);
        CHECK(hal_rt_commit_cycle(c2) == 0);
    }
    CHECK(st == HAL_RESET_TIMEOUT);
    /* 超时同样必须留在未使能态 */
    CHECK(hal_rt_axis_read_status(c2, 7, &s) == 0 && !s.enabled);
    stuck_fault = 0;
    hal_context_destroy(c2);
}

int main(void) {
    lifecycle(); binding(); optional_capabilities(); motion_io(); wrap_and_faults();
    stop_overrides_staged_enable(); concurrent_stop();
    preset_overflow_probe(); spindle_origin_angle(); angle_regressions();
    fault_reset_and_bus_health();
    printf("context_regression: %u checks passed\n", checks);
}
