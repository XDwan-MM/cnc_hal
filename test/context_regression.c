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
static pthread_barrier_t entered, resume_wait;
static int block_wait;
static MasterConfig received_config;

void* __real_calloc(size_t n, size_t size);
void* __wrap_calloc(size_t n, size_t size) {
    return fail_calloc ? NULL : __real_calloc(n, size);
}

int ethercat_init(const MasterConfig* cfg) {
    ++starts;
    if (fail_init) return -1;
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
        }
        servo[i][DEV_DICT_ROLE_MODE_DISPLAY] = servo[i][DEV_DICT_ROLE_OP_MODE];
    }
    return 0;
}
int Master_ServoRead(int slot, DevDictRole role, uint32_t* out) {
    ++reads;
    if (fail_read) return -1;
    *out = servo[slot][role]; return 0;
}
int Master_ServoWrite(int slot, DevDictRole role, uint32_t value) {
    ++writes;
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
    c.cycle_us = 1000; c.start_timeout_ms = 120000; c.cycle_timeout_ms = 2000; c.dc_enable = 1;
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
    fail_init = 0;
    CHECK(hal_context_start(c, NULL, 0) == 0);
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
    CHECK(servo[0][DEV_DICT_ROLE_TARGET_SPEED] == (uint32_t)-120);
    CHECK(output[2][0] == 5 && output[2][1] == 0xAD5 && output[3][0] == 0x1AA);
    servo[0][DEV_DICT_ROLE_ACTUAL_SPEED] = (uint32_t)-240;
    begin(c);
    HalCSpindleStatus s;
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

int main(void) {
    lifecycle(); binding(); motion_io(); wrap_and_faults();
    stop_overrides_staged_enable(); concurrent_stop();
    printf("context_regression: %u checks passed\n", checks);
}
