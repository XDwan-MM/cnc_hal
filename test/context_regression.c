#include "hal_c_api.h"
#include "backend/greemaster/master.h"
#include "backend/greemaster/device_table.h"
#include "backend/greemaster/servo_step.h"
#include "backend/greemaster/entry_access.h"
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
static int wide_mode_bits;    /* 非零 = 模式字按 16 位建句柄（模拟蓝海华腾那类 ESI） */
static int narrow_pos_bits;   /* 非零 = 位置 PDO 只给 16 位（应被拒） */
static int omit_optional, omit_spindle_speed, omit_spindle_position, omit_feed_position;
static int stuck_fault;      /* 非零 = 连复位边沿也清不掉故障，用来测超时 */
static int force_serial_zero; /* 非零 = 桩上报 serial=0，用来测 device_id 的哈希回退 */
static uint32_t previous_control[2];
static unsigned reset_edges[2];
static int ignore_first_reset_edge;
/* 驱动层的停止标志，**独立于 HAL 那份**——真实驱动层有自己的一个（master.c:17），
 * Master_RequestStop() 置它、Master_WaitCycle() 前后查它。这里照抄这个结构，
 * 才能验出"HAL 的 request_stop 有没有透传到驱动层"。 */
static int driver_stop;
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
    driver_stop = 0;   /* 与真实 ethercat_init 一致：每次启动清停止标志 */
    memset(slots, 0, sizeof(slots));
    memset(servo, 0, sizeof(servo));
    memset(previous_control, 0, sizeof(previous_control));
    memset(reset_edges, 0, sizeof(reset_edges));
    memset(output, 0, sizeof(output));
    for (int i = 0; i < 2; ++i) {
        slots[i].type = GREE_AXIS6_TYPE;
        slots[i].slave_pos = 0; slots[i].axis_index = i;
        slots[i].vendor_id = 123; slots[i].product_code = 456;
        slots[i].serial = force_serial_zero ? 0 : 99;
        strcpy(slots[i].name, "Dual axis");
        slots[i].entries.slave.statusWord.bit_length = 16;
        slots[i].entries.slave.act_pos.bit_length =
            (i == 0 && omit_spindle_position) || (i == 1 && omit_feed_position) ? 0 :
            narrow_pos_bits ? 16 : 32;
        slots[i].entries.slave.act_mode.bit_length = wide_mode_bits ? 16 : 8;
        slots[i].entries.slave.Control_word.bit_length = 16;
        slots[i].entries.slave.target_pos.bit_length =
            (i == 0 && omit_spindle_position) || (i == 1 && omit_feed_position) ? 0 : 32;
        slots[i].entries.slave.Modes_of_operation.bit_length = wide_mode_bits ? 16 : 8;
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
/* 从站身份表放在文件作用域：测试要把它改成「同型号第 2 台」来验证 family_index。 */
static DEVICE_BASIC_INFO g_ids[3] = {
    {.ID=123, .CODE=456, .Serial=99},
    {.ID=2252, .CODE=269418497, .Revision=1},
    {.ID=2252, .CODE=269418498, .Revision=1}
};
const DEVICE_BASIC_INFO* device_identity_get(int pos) { return &g_ids[pos]; }
int DeviceTable_Get(const DeviceSlot** out) { if (out) *out = opened ? slots : NULL; return opened ? 4 : 0; }
void Master_RequestStop(void) { driver_stop = 1; }
int  Master_StopFlag(void) { return driver_stop != 0; }

int Master_WaitCycle(void) {
    ++waits;
    /* 与真实驱动层同构：等待前后都查停止标志，置了就立刻返回 */
    if (driver_stop) return MASTER_STOP_REQUESTED;
    if (block_wait) { pthread_barrier_wait(&entered); pthread_barrier_wait(&resume_wait); }
    if (driver_stop) return MASTER_STOP_REQUESTED;
    return fail_wait ? -1 : 0;
}
int Master_CommitCycle(void) {
    ++sends;
    if (fail_send) return -1;
    for (int i = 0; i < 2; ++i) {
        const uint32_t control = servo[i][DEV_DICT_ROLE_CONTROL_WORD];
        const int rising_reset = (control & 128u) && !(previous_control[i] & 128u);
        if (rising_reset) ++reset_edges[i];
        if (servo[i][DEV_DICT_ROLE_STATUS_WORD] & 8u) {
            /* 故障只能由复位上升沿解除；0/7/15 不能把故障伪装成已恢复。 */
            if (rising_reset && !stuck_fault &&
                (!ignore_first_reset_edge || reset_edges[i] > 1))
                servo[i][DEV_DICT_ROLE_STATUS_WORD] = 0x40;
        } else switch (control) {
        case 0: servo[i][DEV_DICT_ROLE_STATUS_WORD] = 0x40; break;
        case 6: servo[i][DEV_DICT_ROLE_STATUS_WORD] = 0x21; break;
        case 7: servo[i][DEV_DICT_ROLE_STATUS_WORD] = 0x23; break;
        case 15: servo[i][DEV_DICT_ROLE_STATUS_WORD] = 0x237; break;
        }
        previous_control[i] = control;
        servo[i][DEV_DICT_ROLE_MODE_DISPLAY] = servo[i][DEV_DICT_ROLE_OP_MODE];
    }
    return 0;
}
int Master_ServoRead(int slot, DevDictRole role, uint32_t* out) {
    ++reads;
    if (role == DEV_DICT_ROLE_ACTUAL_POS && !slots[slot].entries.slave.act_pos.bit_length) return -1;
    if (role == DEV_DICT_ROLE_ERROR_CODE && !slots[slot].entries.slave.error_code.bit_length) return -1;
    if (role == DEV_DICT_ROLE_ACTUAL_SPEED && !slots[slot].entries.slave.act_speed.bit_length) return -1;
    if (fail_read) return -1;
    *out = servo[slot][role]; return 0;
}
int Master_ServoWrite(int slot, DevDictRole role, uint32_t value) {
    ++writes;
    if (role == DEV_DICT_ROLE_TARGET_POS && !slots[slot].entries.slave.target_pos.bit_length) return -1;
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
    if (s.preset_target && slots[slot].entries.slave.act_pos.bit_length == 32 &&
        slots[slot].entries.slave.target_pos.bit_length == 32 &&
        Master_ServoWrite(slot, DEV_DICT_ROLE_TARGET_POS, servo[slot][DEV_DICT_ROLE_ACTUAL_POS])) return -1;
    if (s.op_mode != 0xFFFF && Master_ServoWrite(slot, DEV_DICT_ROLE_OP_MODE, s.op_mode)) return -1;
    if (Ds402_StepWritesControl(s))
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
    c.spindles[0].max_speed = 1000;
    /* 定向：值与位都是厂商专有的，这里用台架那套（0x040F + 状态字 bit10）当样例。 */
    c.spindles[0].axis.vendor_orient_control_word = 0x040Fu;
    c.spindles[0].axis.vendor_orient_done_mask = 0x0400u;
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
    CHECK(hal_rt_axis_read_status(c, 7, &status) == 0 && !status.error_code_valid &&
          !status.linear_speed_valid && status.actual_speed_mm_min == 0);
    CHECK(hal_rt_axis_read_status(c, 3, &status) == 0 && status.error_code_valid);
    /* 实际速度也要能从**轴状态**里读到：进给轴拿不到 hal_rt_spindle_read_speed（它要求
     * 主轴），而进给率显示要这个数。上面 capability 已经断言过这两根轴一个有速度 PDO、
     * 一个没有，正好用来钉住"没映射"与"速度为 0"必须能区分——
     * 分不开的话上层会把"读不到"当成"停着"。 */
    servo[0][DEV_DICT_ROLE_ACTUAL_SPEED] = 1234;
    cycle(c);
    CHECK(hal_rt_axis_read_status(c, 3, &status) == 0 &&
          status.speed_valid && status.actual_speed == 1234);
    CHECK(hal_rt_axis_read_status(c, 7, &status) == 0 &&
          !status.speed_valid && status.actual_speed == 0);
    hal_context_destroy(c);
    omit_optional = 0;

    omit_spindle_speed = 1;
    c = create(&cfg);
    char error[160];
    /* 缺必需 PDO 报 UNSUPPORTED：bind_axis 有意把它和"找不到匹配的轴"(CONFIG) 分开，
     * start 必须原样报出来，不能一律折成 CONFIG。 */
    CHECK(hal_context_start(c, error, sizeof(error)) == HAL_ERROR_UNSUPPORTED);
    CHECK(strstr(error, "从站 0 轴 0") && strstr(error, "主轴速度"));
    hal_context_destroy(c);
    omit_spindle_speed = 0;
}

static void feed_speed_mm_min(void) {
    HalCConfig cfg = config();
    cfg.axes[0].command_pulses_per_rev = 2000;
    HalContext* c = create(&cfg);
    CHECK(hal_context_start(c, NULL, 0) == 0);
    servo[1][DEV_DICT_ROLE_ACTUAL_SPEED] = 120;
    cycle(c);
    HalCAxisStatus status;
    CHECK(hal_rt_axis_read_status(c, 7, &status) == 0);
    CHECK(status.speed_valid && status.linear_speed_valid);
    CHECK(status.actual_speed == 120 && status.actual_speed_rpm == 120);
    CHECK(status.actual_speed_mm_min == 24000);
    CHECK(hal_rt_axis_read_status(c, 3, &status) == 0 &&
          !status.linear_speed_valid && status.actual_speed_mm_min == 0);
    hal_context_destroy(c);

    cfg.axes[0].speed_unit = HAL_SPEED_PULSES_PER_S;
    cfg.axes[0].feedback_invert = 1;
    c = create(&cfg);
    CHECK(hal_context_start(c, NULL, 0) == 0);
    servo[1][DEV_DICT_ROLE_ACTUAL_SPEED] = 1000;
    cycle(c);
    CHECK(hal_rt_axis_read_status(c, 7, &status) == 0);
    CHECK(status.actual_speed == -1000 && status.actual_speed_rpm == -30);
    CHECK(status.linear_speed_valid && status.actual_speed_mm_min == -6000);
    hal_context_destroy(c);

    cfg.axes[0].speed_unit = HAL_SPEED_RPM;
    cfg.axes[0].command_pulses_per_rev = 0;
    c = create(&cfg);
    CHECK(hal_context_start(c, NULL, 0) == 0);
    cycle(c);
    CHECK(hal_rt_axis_read_status(c, 7, &status) == 0 &&
          status.speed_valid && !status.linear_speed_valid &&
          status.actual_speed_mm_min == 0);
    hal_context_destroy(c);
}

static void velocity_only_spindle(void) {
    HalCConfig cfg = config();
    omit_spindle_position = 1;
    HalContext* c = create(&cfg);
    CHECK(hal_context_start(c, NULL, 0) == 0);
    HalCCapability cap;
    CHECK(hal_axis_capability(c, 3, HAL_FUNC_POSITION, &cap) == 0 &&
          cap.state == HAL_CAP_UNSUPPORTED);
    CHECK(hal_axis_capability(c, 3, HAL_FUNC_SPEED, &cap) == 0 && cap.state == HAL_CAP_READY);
    enable(c);
    begin(c);
    HalCAxisStatus status;
    CHECK(hal_rt_axis_read_status(c, 3, &status) == 0 && !status.position_valid &&
          status.enabled && status.speed_valid);
    CHECK(hal_rt_spindle_write_speed(c, 3, 100, 1) == 0);
    CHECK(hal_rt_spindle_request_mode(c, 3, HAL_SPINDLE_CSP) == HAL_ERROR_UNSUPPORTED);
    CHECK(hal_rt_spindle_prepare_csp(c, 3, 1000) == HAL_ERROR_UNSUPPORTED);
    CHECK(hal_rt_spindle_write_pos(c, 3, 90) == HAL_ERROR_UNSUPPORTED);
    CHECK(hal_rt_axis_write_pos(c, 3, 90) == HAL_ERROR_UNSUPPORTED);
    CHECK(hal_rt_axis_set_pos(c, 3, 0) == HAL_ERROR_UNSUPPORTED);
    double position;
    CHECK(hal_rt_axis_read_pos(c, 3, &position) == HAL_ERROR_UNSUPPORTED);
    CHECK(hal_rt_commit_cycle(c) == 0);
    CHECK(servo[0][DEV_DICT_ROLE_TARGET_SPEED] == 100);
    hal_context_destroy(c);
    omit_spindle_position = 0;

    omit_feed_position = 1;
    c = create(&cfg);
    CHECK(hal_context_start(c, NULL, 0) == HAL_ERROR_UNSUPPORTED);
    hal_context_destroy(c);
    omit_feed_position = 0;
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
    CHECK(hal_rt_spindle_read_status(c, 3, &s) == 0 && s.actual_speed == -1234);
    /* 上一拍已 begin 未 commit，必须先提交才能进下一次 wait。 */
    CHECK(hal_rt_commit_cycle(c) == 0);
    servo[0][DEV_DICT_ROLE_ACTUAL_SPEED] = (uint32_t)-10;
    begin(c);
    CHECK(hal_rt_spindle_read_status(c, 3, &s) == 0 && s.actual_speed == -10);
    CHECK(hal_rt_spindle_request_mode(c, 3, HAL_SPINDLE_CSP) == 0);
    CHECK(hal_rt_spindle_write_speed(c, 3, 10, 1) == HAL_ERROR_NOT_ENABLED);
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
    /* request_stop 必须同时通知驱动层。只置 HAL 那份的话，PDO 线程卡在
     * GM_Wait_Master_Sync 里时叫不醒，要白等一个 cycle_timeout_ms。 */
    CHECK(driver_stop == 1);
    /* 且驱动层在"进入等待之前"就该看到它，不必进等待 */
    CHECK(Master_WaitCycle() == MASTER_STOP_REQUESTED);
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
    /* 复位期间不能写位置：request 不是 ENABLE，轴也未使能（2026-10-08 起分码为 NOT_ENABLED） */
    CHECK(hal_rt_axis_write_pos(c, 7, 10) == HAL_ERROR_NOT_ENABLED);
    CHECK(hal_rt_commit_cycle(c) == 0);

    /* 下一拍 begin 才发出 0x80 边沿，该拍 commit 里 mock 让驱动器离开 Fault */
    cycle(c); /* 先发送低电平 */
    cycle(c); /* 再发送复位上升沿 */
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

static void io_image_size(void) {
    HalCConfig cfg = config();
    HalContext* c = create(&cfg);
    uint32_t xs = 99, ys = 99;
    /* 未 start 时设备还没枚举，段长未知 */
    CHECK(hal_io_image_size(c, &xs, &ys) == HAL_ERROR_NOT_RUNNING);
    CHECK(hal_context_start(c, NULL, 0) == 0);
    CHECK(hal_io_image_size(c, &xs, &ys) == HAL_OK);
    /* 按测试配置手算一遍，确认 HAL 的累加口径：
     *   IO 段 ：x_start=2，4 个 Entry 位长 3+12+1+32 = 48 位 → 6 字节 → 末尾 8
     *           y_start=1，2 个 Entry 位长 5+12   = 17 位 → 3 字节 → 末尾 4
     *   面板段：x_start=10，1 个 Entry 9 位 → 2 字节 → 末尾 12
     *           y_start=6 ，1 个 Entry 9 位 → 2 字节 → 末尾 8
     *   x_size = 各 X 段末尾最大值 = 12；y_size = 8 */
    CHECK(xs == 12 && ys == 8);
    CHECK(hal_io_image_size(NULL, &xs, &ys) == HAL_ERROR_ARGUMENT);
    CHECK(hal_io_image_size(c, NULL, &ys) == HAL_ERROR_ARGUMENT);
    CHECK(hal_io_image_size(c, &xs, NULL) == HAL_ERROR_ARGUMENT);
    hal_context_destroy(c);
}

/* 2026-10-04 审查：隔离停止、重使能、复位边沿/超时/急停组合和速度方向。 */
static void review_regressions(void) {
    HalCConfig cfg = config();
    HalContext* c = create(&cfg);
    HalContext* idle = create(&cfg);
    CHECK(hal_context_start(c, NULL, 0) == 0);
    CHECK(hal_context_request_stop(idle) == 0 && !driver_stop);
    cycle(c);
    hal_context_destroy(idle);
    hal_context_destroy(c);

    cfg.axes[0].feedback_units_per_count = cfg.axes[0].command_units_per_count;
    c = create(&cfg); CHECK(hal_context_start(c, NULL, 0) == 0);
    servo[1][DEV_DICT_ROLE_STATUS_WORD] = 0x23;
    servo[1][DEV_DICT_ROLE_CONTROL_WORD] = 7;
    servo[1][DEV_DICT_ROLE_TARGET_POS] = 9000;
    begin(c); CHECK(hal_rt_axis_enable(c, 7, 1) == 0);
    CHECK(hal_rt_commit_cycle(c) == 0);
    begin(c);
    CHECK(servo[1][DEV_DICT_ROLE_TARGET_POS] == 100);
    CHECK(servo[1][DEV_DICT_ROLE_CONTROL_WORD] == 15);
    CHECK(hal_rt_commit_cycle(c) == 0);
    begin(c); CHECK(hal_rt_axis_write_pos(c, 7, 900) == 0);
    CHECK(hal_rt_commit_cycle(c) == 0);
    begin(c); CHECK(hal_rt_axis_enable(c, 7, 0) == 0);
    CHECK(hal_rt_commit_cycle(c) == 0);
    begin(c); CHECK(hal_rt_axis_enable(c, 7, 1) == 0);
    CHECK(hal_rt_commit_cycle(c) == 0);
    begin(c);
    CHECK(servo[1][DEV_DICT_ROLE_TARGET_POS] == 100);
    CHECK(hal_rt_commit_cycle(c) == 0);
    hal_context_destroy(c);

    for (int action = 0; action < 3; ++action) {
        cfg = config(); cfg.fault_reset_timeout_ms = 5;
        if (action == 2) cfg.axes[0].estop_action = HAL_ESTOP_DISABLE_VOLTAGE;
        c = create(&cfg); CHECK(hal_context_start(c, NULL, 0) == 0);
        begin(c);
        if (!action) CHECK(hal_rt_axis_enable(c, 7, 0) == 0);
        else CHECK(hal_rt_axis_estop(c, 7) == 0);
        CHECK(hal_rt_axis_fault_reset(c, 7) == HAL_ERROR_STATE); /* 同拍停止优先 */
        CHECK(hal_rt_commit_cycle(c) == 0);
        for (int attempt = 0; attempt < 2; ++attempt) {
            servo[1][DEV_DICT_ROLE_STATUS_WORD] = 8;
            begin(c); CHECK(hal_rt_axis_fault_reset(c, 7) == 0);
            CHECK(hal_rt_commit_cycle(c) == 0);
            int32_t state = HAL_RESET_PENDING;
            for (int n = 0; n < 10 && state == HAL_RESET_PENDING; ++n) {
                cycle(c); CHECK(hal_rt_axis_fault_reset_state(c, 7, &state) == 0);
            }
            CHECK(state == HAL_RESET_DONE);
            CHECK(reset_edges[1] == (unsigned)attempt + 1);
            CHECK(!(servo[1][DEV_DICT_ROLE_CONTROL_WORD] & 128u));
            HalCAxisStatus status;
            CHECK(hal_rt_axis_read_status(c, 7, &status) == 0 && !status.enabled);
        }
        hal_context_destroy(c);
    }

    cfg = config(); cfg.fault_reset_timeout_ms = 1;
    c = create(&cfg); CHECK(hal_context_start(c, NULL, 0) == 0);
    stuck_fault = 1; servo[1][DEV_DICT_ROLE_STATUS_WORD] = 8;
    begin(c); CHECK(hal_rt_axis_fault_reset(c, 7) == 0);
    CHECK(hal_rt_commit_cycle(c) == 0);
    cycle(c); CHECK(reset_edges[1] == 0);
    cycle(c); CHECK(reset_edges[1] == 1);
    cycle(c);
    int32_t state;
    CHECK(hal_rt_axis_fault_reset_state(c, 7, &state) == 0 && state == HAL_RESET_TIMEOUT);
    CHECK(!(servo[1][DEV_DICT_ROLE_CONTROL_WORD] & 128u));
    stuck_fault = 0; hal_context_destroy(c);

    cfg = config(); cfg.cycle_us = 1500; cfg.fault_reset_timeout_ms = 2;
    c = create(&cfg); CHECK(hal_context_start(c, NULL, 0) == 0);
    stuck_fault = 1; servo[1][DEV_DICT_ROLE_STATUS_WORD] = 8;
    begin(c); CHECK(hal_rt_axis_fault_reset(c, 7) == 0);
    CHECK(hal_rt_commit_cycle(c) == 0);
    cycle(c); cycle(c); cycle(c);
    CHECK(hal_rt_axis_fault_reset_state(c, 7, &state) == 0 && state == HAL_RESET_PENDING);
    cycle(c);
    CHECK(hal_rt_axis_fault_reset_state(c, 7, &state) == 0 && state == HAL_RESET_TIMEOUT);
    stuck_fault = 0; hal_context_destroy(c);

    cfg = config(); cfg.fault_reset_timeout_ms = 10;
    c = create(&cfg); CHECK(hal_context_start(c, NULL, 0) == 0);
    ignore_first_reset_edge = 1; servo[1][DEV_DICT_ROLE_STATUS_WORD] = 8;
    begin(c); CHECK(hal_rt_axis_fault_reset(c, 7) == 0);
    CHECK(hal_rt_commit_cycle(c) == 0);
    for (int n = 0; n < 6; ++n) cycle(c);
    CHECK(reset_edges[1] == 2);
    CHECK(hal_rt_axis_fault_reset_state(c, 7, &state) == 0 && state == HAL_RESET_DONE);
    ignore_first_reset_edge = 0;
    begin(c); CHECK(hal_context_request_stop(c) == 0);
    CHECK(hal_rt_axis_fault_reset(c, 7) == HAL_ERROR_STOPPED);
    hal_context_destroy(c);

    c = create(&cfg); CHECK(hal_context_start(c, NULL, 0) == 0);
    begin(c); fail_send = 1;
    CHECK(hal_rt_commit_cycle(c) == HAL_ERROR_BUS); fail_send = 0;
    CHECK(hal_rt_axis_fault_reset(c, 7) == HAL_ERROR_BUS);
    hal_context_destroy(c);

    for (int stop = 0; stop < 3; ++stop) {
        cfg = config();
        if (stop == 2) cfg.axes[0].estop_action = HAL_ESTOP_DISABLE_VOLTAGE;
        c = create(&cfg); CHECK(hal_context_start(c, NULL, 0) == 0);
        stuck_fault = 1; servo[1][DEV_DICT_ROLE_STATUS_WORD] = 8;
        begin(c); CHECK(hal_rt_axis_fault_reset(c, 7) == 0);
        CHECK(hal_rt_axis_enable(c, 7, 1) == HAL_ERROR_NOT_RUNNING);
        CHECK(hal_rt_commit_cycle(c) == 0);
        cycle(c); begin(c); /* 本拍已暂存高电平，停止必须在 commit 覆盖它 */
        CHECK(servo[1][DEV_DICT_ROLE_CONTROL_WORD] == 128);
        if (!stop) CHECK(hal_rt_axis_enable(c, 7, 0) == 0);
        else CHECK(hal_rt_axis_estop(c, 7) == 0);
        CHECK(hal_rt_axis_fault_reset_state(c, 7, &state) == 0 && state == HAL_RESET_NONE);
        CHECK(hal_rt_axis_fault_reset(c, 7) == HAL_ERROR_STATE);
        CHECK(hal_rt_commit_cycle(c) == 0);
        const uint32_t stop_word = stop == 2 ? 0 : 7;
        CHECK(servo[1][DEV_DICT_ROLE_CONTROL_WORD] == stop_word && reset_edges[1] == 0);
        cycle(c); CHECK(servo[1][DEV_DICT_ROLE_CONTROL_WORD] == stop_word);
        stuck_fault = 0; hal_context_destroy(c);
    }

    /* 固定原始反馈 +100 是合成输入，只检查命令/反馈的符号换算。
     * 模拟器不随命令生成真实速度；四组合包含命令与反馈方向不一致的输入。
     * 这组符号关系不能证明任意方向组合在实机上有效；实际极性须按接线核对。 */
    for (int command = 0; command < 2; ++command) for (int feedback = 0; feedback < 2; ++feedback) {
        cfg = config(); cfg.spindles[0].axis.command_invert = command;
        cfg.spindles[0].axis.feedback_invert = feedback;
        c = create(&cfg); CHECK(hal_context_start(c, NULL, 0) == 0); enable(c);
        servo[0][DEV_DICT_ROLE_ACTUAL_SPEED] = 100;
        begin(c); double rpm;
        CHECK(hal_rt_spindle_read_speed(c, 3, &rpm) == 0 && rpm == (feedback ? -100 : 100));
        /* 轴状态里的 actual_speed 必须与主轴那条**同源**（含 feedback_invert 的符号）。
         * 两条路各算一遍的话，同一个读数会给出不同的数。 */
        HalCAxisStatus sp_axis;
        CHECK(hal_rt_axis_read_status(c, 3, &sp_axis) == 0 &&
              sp_axis.speed_valid && sp_axis.actual_speed == rpm);
        CHECK(hal_rt_spindle_write_speed(c, 3, 100, 1) == 0);
        HalCSpindleStatus speed_status;
        CHECK(hal_rt_spindle_read_status(c, 3, &speed_status) == 0);
        CHECK(speed_status.command_speed == 100);
        CHECK(hal_rt_commit_cycle(c) == 0);
        CHECK(servo[0][DEV_DICT_ROLE_TARGET_SPEED] == (command ? (uint32_t)-100 : 100));
        begin(c); CHECK(hal_rt_spindle_write_speed(c, 3, 100, -1) == 0);
        CHECK(hal_rt_spindle_read_status(c, 3, &speed_status) == 0);
        CHECK(speed_status.command_speed == -100);
        CHECK(hal_rt_commit_cycle(c) == 0);
        CHECK(servo[0][DEV_DICT_ROLE_TARGET_SPEED] == (command ? 100 : (uint32_t)-100));
        hal_context_destroy(c);
    }
}

static void voltage_stop_priority(void) {
    for (int order = 0; order < 2; ++order) {
        HalCConfig cfg = config(); cfg.axes[0].estop_action = HAL_ESTOP_DISABLE_VOLTAGE;
        HalContext* c = create(&cfg);
        CHECK(hal_context_start(c, NULL, 0) == 0); enable(c);
        begin(c);
        if (order) CHECK(hal_rt_axis_enable(c, 7, 0) == 0);
        CHECK(hal_rt_axis_estop(c, 7) == 0);
        if (!order) CHECK(hal_rt_axis_enable(c, 7, 0) == 0);
        CHECK(hal_rt_commit_cycle(c) == 0);
        CHECK(servo[1][DEV_DICT_ROLE_CONTROL_WORD] == 0);
        for (int n = 0; n < 3; ++n) {
            begin(c); CHECK(hal_rt_axis_enable(c, 7, 0) == 0);
            CHECK(hal_rt_commit_cycle(c) == 0);
            CHECK(servo[1][DEV_DICT_ROLE_CONTROL_WORD] == 0);
            CHECK(servo[1][DEV_DICT_ROLE_STATUS_WORD] == 0x40);
        }
        /* 显式重新上使能仍可解除历史停止；去使能不能代替这个显式动作。 */
        CHECK(hal_rt_axis_enable(c, 7, 1) == 0);
        for (int n = 0; n < 4; ++n) cycle(c);
        HalCAxisStatus status;
        CHECK(hal_rt_axis_read_status(c, 7, &status) == 0 && status.enabled);
        hal_context_destroy(c);
    }
}

static void standard_enabled_status(void) {
    const uint16_t words[] = {0x0027, 0x0037, 0x0227, 0x0237};
    for (unsigned i = 0; i < sizeof(words) / sizeof(words[0]); ++i) {
        HalCConfig cfg = config(); HalContext* c = create(&cfg);
        CHECK(hal_context_start(c, NULL, 0) == 0); enable(c);
        servo[0][DEV_DICT_ROLE_STATUS_WORD] = servo[1][DEV_DICT_ROLE_STATUS_WORD] = words[i];
        begin(c);
        HalCAxisStatus status; HalCSpindleStatus spindle;
        CHECK(hal_rt_axis_read_status(c, 7, &status) == 0 && status.enabled);
        CHECK(hal_rt_spindle_read_status(c, 3, &spindle) == 0 && spindle.enabled);
        CHECK(hal_rt_axis_write_pos(c, 7, 25) == 0);
        CHECK(hal_rt_spindle_write_speed(c, 3, 100, 1) == 0);
        CHECK(hal_rt_commit_cycle(c) == 0);
        servo[0][DEV_DICT_ROLE_STATUS_WORD] = servo[1][DEV_DICT_ROLE_STATUS_WORD] = 0x0007;
        begin(c);
        CHECK(hal_rt_axis_read_status(c, 7, &status) == 0 && !status.enabled);
        CHECK(hal_rt_axis_write_pos(c, 7, 25) == HAL_ERROR_NOT_ENABLED);
        CHECK(hal_rt_spindle_write_speed(c, 3, 100, 1) == HAL_ERROR_NOT_ENABLED);
        CHECK(hal_rt_commit_cycle(c) == 0);
        hal_context_destroy(c);
    }
}

/* 2026-10-04 覆盖补强：下面四处原先被改坏也不会让任何用例变红。 */
static void coverage_gaps(void) {
    HalCConfig cfg;
    HalContext* c;

    /* 1) 启动超时码：MASTER_START_TIMEOUT 必须翻成 HAL_ERROR_TIMEOUT，
     *    不是笼统的 HAL_ERROR_BUS ——上层的报警号是按码分的。 */
    cfg = config(); c = create(&cfg);
    fail_init = MASTER_START_TIMEOUT;
    CHECK(hal_context_start(c, NULL, 0) == HAL_ERROR_TIMEOUT);
    fail_init = 0;
    CHECK(hal_context_start(c, NULL, 0) == 0);
    hal_context_destroy(c);

    /* 2) family_index：把第 2 台从站改成与第 1 台同 vendor+product，
     *    「同型号第几台」必须从 0 变成 1。 */
    g_ids[1].ID = g_ids[0].ID; g_ids[1].CODE = g_ids[0].CODE;
    cfg = config(); c = create(&cfg);
    CHECK(hal_context_start(c, NULL, 0) == 0);
    HalCSlaveInfo slave;
    CHECK(hal_slave_info(c, 0, &slave) == 0 && slave.identity.family_index == 0);
    CHECK(hal_slave_info(c, 1, &slave) == 0 && slave.identity.family_index == 1);
    CHECK(hal_slave_info(c, 2, &slave) == 0 && slave.identity.family_index == 0);
    hal_context_destroy(c);
    g_ids[1].ID = 2252; g_ids[1].CODE = 269418497;

    /* 3) device_id 的哈希回退：serial==0 时必须是 FNV(vendor,product,revision)。
     *    这里断言**确切数值**——只断言「非零」或「两个设备不同」都挡不住常量被改。 */
    force_serial_zero = 1;
    cfg = config(); c = create(&cfg);
    CHECK(hal_context_start(c, NULL, 0) == 0);
    HalCIdentity ident;
    CHECK(hal_device_identity(c, 7, &ident) == 0 && ident.serial == 0);
    CHECK(ident.device_id == UINT64_C(18014831272581144209));
    hal_context_destroy(c);
    force_serial_zero = 0;

    /* 4) 位置写入的 CSP 闸门：主轴处于 CSV 使能态时，通用位置写入必须被拒，
     *    否则 0x607A 会被下发给速度模式驱动器。
     *    2026-10-08：模式不符与未使能已拆成两个错误码，这里断言的是模式那一个。 */
    cfg = config(); c = create(&cfg);
    CHECK(hal_context_start(c, NULL, 0) == 0);
    CHECK(hal_rt_spindle_enable(c, 3, 1) == 0);
    for (int i = 0; i < 5; ++i) cycle(c);
    begin(c);
    HalCSpindleStatus ss;
    CHECK(hal_rt_spindle_read_status(c, 3, &ss) == 0 && ss.enabled && ss.mode == HAL_SPINDLE_CSV);
    CHECK(hal_rt_axis_write_pos(c, 3, 0.0) == HAL_ERROR_MODE_MISMATCH);
    CHECK(hal_rt_commit_cycle(c) == 0);
    hal_context_destroy(c);
}

/* 定向：发控制字后要**先等够**再采完成位。
 *
 * 为什么必须等：完成位在别的模式下可能是别的含义。蓝海华腾 VTS 手册附录 C 写着
 * 6041h.bit10 在 CSV 模式下表示"速度达到设定转速"，发 0x40F 后延时 10ms 才表示
 * "定位完成"。不等的话主轴正常转到速时 bit10 本来就是 1 → 一请求就判 DONE，
 * M19 看起来"秒完成"，其实驱动器还没开始定位。 */
static void spindle_orient_fresh_edge(void) {
    for (int delay = 0; delay <= 5; delay += 5) {
        HalCConfig cfg = config();
        cfg.spindles[0].axis.vendor_orient_delay_ms = delay;
        /* A different mask ensures the handshake is not hard-coded to bit10. */
        cfg.spindles[0].axis.vendor_orient_done_mask = 0x0800u;
        HalContext* c = create(&cfg);
        CHECK(hal_context_start(c, NULL, 0) == 0);
        enable(c);
        int32_t os;
        for (int round = 0; round < 3; ++round) {
            servo[0][DEV_DICT_ROLE_STATUS_WORD] = 0x0037u;
            begin(c); // A low before request/commit must not arm completion.
            if (round == 1) CHECK(hal_rt_spindle_release_orient(c, 3) == 0);
            CHECK(hal_rt_spindle_request_orient(c, 3) == 0);
            CHECK(hal_rt_commit_cycle(c) == 0);
            servo[0][DEV_DICT_ROLE_STATUS_WORD] = 0x0837u;
            for (int i = 0; i < 12; ++i) cycle(c);
            CHECK(hal_rt_spindle_orient_state(c, 3, &os) == 0 && os == HAL_ORIENT_RUNNING);
            servo[0][DEV_DICT_ROLE_STATUS_WORD] = 0x0037u;
            cycle(c);
            servo[0][DEV_DICT_ROLE_STATUS_WORD] = 0x0837u;
            cycle(c);
            CHECK(hal_rt_spindle_orient_state(c, 3, &os) == 0 && os == HAL_ORIENT_DONE);
        }
        begin(c);
        CHECK(hal_rt_spindle_release_orient(c, 3) == 0);
        CHECK(hal_rt_spindle_request_orient(c, 3) == 0);
        CHECK(hal_rt_commit_cycle(c) == 0);
        servo[0][DEV_DICT_ROLE_STATUS_WORD] = 0x0037u;
        cycle(c); // Low during the delay must be remembered.
        servo[0][DEV_DICT_ROLE_STATUS_WORD] = 0x0837u;
        cycle(c);
        CHECK(hal_rt_spindle_orient_state(c, 3, &os) == 0 &&
              os == (delay ? HAL_ORIENT_RUNNING : HAL_ORIENT_DONE));
        for (int i = 0; i < 6; ++i) cycle(c);
        CHECK(hal_rt_spindle_orient_state(c, 3, &os) == 0 && os == HAL_ORIENT_DONE);
        begin(c);
        CHECK(hal_rt_spindle_request_orient(c, 3) == 0);
        CHECK(hal_rt_commit_cycle(c) == 0);
        servo[0][DEV_DICT_ROLE_STATUS_WORD] = 0x0008u;
        cycle(c); // A fault's low must not qualify as a handshake.
        servo[0][DEV_DICT_ROLE_STATUS_WORD] = 0x0837u;
        for (int i = 0; i < 12; ++i) cycle(c);
        CHECK(hal_rt_spindle_orient_state(c, 3, &os) == 0 && os == HAL_ORIENT_RUNNING);
        hal_context_destroy(c);
    }
}

static void spindle_orient_delay(void) {
    HalCConfig cfg = config();
    cfg.spindles[0].axis.vendor_orient_delay_ms = 5;   /* 周期 1ms → 等 5 拍 */
    HalContext* c = create(&cfg);
    CHECK(hal_context_start(c, NULL, 0) == 0);
    CHECK(hal_rt_spindle_enable(c, 3, 1) == 0);
    for (int i = 0; i < 5; ++i) cycle(c);

    /* 驱动器把完成位**先**报成 1（CSV 到速就是这个位）——不能立刻判 DONE。 */
    servo[0][DEV_DICT_ROLE_STATUS_WORD] = 0x0437u;
    begin(c);
    CHECK(hal_rt_spindle_request_orient(c, 3) == 0);
    int32_t os = -1;
    CHECK(hal_rt_spindle_orient_state(c, 3, &os) == 0 && os == HAL_ORIENT_RUNNING);
    CHECK(hal_rt_commit_cycle(c) == 0);

    /* 等待期内：完成位已经是 1，仍必须是 RUNNING */
    cycle(c);
    begin(c);
    CHECK(hal_rt_spindle_orient_state(c, 3, &os) == 0 && os == HAL_ORIENT_RUNNING);
    CHECK(hal_rt_commit_cycle(c) == 0);

    /* 等够也不能把旧高电平当完成。 */
    for (int i = 0; i < 6; ++i) cycle(c);
    begin(c);
    CHECK(hal_rt_spindle_orient_state(c, 3, &os) == 0 && os == HAL_ORIENT_RUNNING);
    CHECK(hal_rt_commit_cycle(c) == 0);
    servo[0][DEV_DICT_ROLE_STATUS_WORD] = 0x0037u;
    cycle(c);
    servo[0][DEV_DICT_ROLE_STATUS_WORD] = 0x0437u;
    begin(c);
    CHECK(hal_rt_spindle_orient_state(c, 3, &os) == 0 && os == HAL_ORIENT_DONE);
    CHECK(hal_rt_commit_cycle(c) == 0);
    hal_context_destroy(c);
}

/* 速度单位：驱动器是 pulse/s 时，对外仍然全是 rpm。
 *
 * 蓝海华腾 VTS 的 C2-00 出厂默认就是 pulse/s：不换算的话，写"300 rpm"会变成
 * 300 脉冲/秒（主轴几乎不转），而"停稳"判据（转速<5）永远不成立。 */
static void speed_unit_pulses_per_second(void) {
    HalCConfig cfg = config();
    cfg.spindles[0].axis.speed_unit = HAL_SPEED_PULSES_PER_S;
    cfg.spindles[0].axis.command_pulses_per_rev = 10000;   /* 每转 1 万脉冲 */
    HalContext* c = create(&cfg);
    CHECK(hal_context_start(c, NULL, 0) == 0); enable(c);

    /* 读：50000 脉冲/秒 = 300 rpm */
    servo[0][DEV_DICT_ROLE_ACTUAL_SPEED] = 50000;
    begin(c);
    double rpm = 0.0;
    CHECK(hal_rt_spindle_read_speed(c, 3, &rpm) == 0 && rpm > 299.9 && rpm < 300.1);
    HalCAxisStatus st;
    CHECK(hal_rt_axis_read_status(c, 3, &st) == 0 && st.speed_valid);
    CHECK(st.actual_speed == 50000);                                  /* 原始值保留 */
    CHECK(st.actual_speed_rpm > 299.9 && st.actual_speed_rpm < 300.1);/* 换算后 */

    /* 写：300 rpm = 300/60×10000 = 50000 脉冲/秒；对外读回来仍是 rpm */
    CHECK(hal_rt_spindle_write_speed(c, 3, 300, 1) == 0);
    HalCSpindleStatus ss;
    CHECK(hal_rt_spindle_read_status(c, 3, &ss) == 0 && ss.command_speed == 300);
    CHECK(hal_rt_commit_cycle(c) == 0);
    CHECK(servo[0][DEV_DICT_ROLE_TARGET_SPEED] == 50000u);
    hal_context_destroy(c);

    /* 填 pulse/s 却不给每转脉冲数 → 配置直接拒，不按 1:1 静默走 */
    cfg = config();
    cfg.spindles[0].axis.speed_unit = HAL_SPEED_PULSES_PER_S;
    cfg.spindles[0].axis.command_pulses_per_rev = 0;
    {
        HalContext* bad = NULL; char err[128];
        CHECK(hal_context_create(&cfg, &bad, err, sizeof(err)) == HAL_ERROR_CONFIG && !bad);
    }
    /* 单位枚举非法也一样 */
    cfg.spindles[0].axis.speed_unit = 7;
    cfg.spindles[0].axis.command_pulses_per_rev = 10000;
    {
        HalContext* bad = NULL; char err[128];
        CHECK(hal_context_create(&cfg, &bad, err, sizeof(err)) == HAL_ERROR_CONFIG && !bad);
    }
}

/* 定向中途故障复位：复位序列不能被定向控制字盖掉；复位之后定向不再成立。 */
static void orient_then_fault_reset(void) {
    HalCConfig cfg = config();
    cfg.fault_reset_timeout_ms = 50;
    HalContext* c = create(&cfg);
    CHECK(hal_context_start(c, NULL, 0) == 0);
    enable(c);

    /* 先进定向：主轴槽 3 在伺服索引 0 */
    servo[0][DEV_DICT_ROLE_STATUS_WORD] = 0x0437u;
    begin(c);
    CHECK(hal_rt_spindle_request_orient(c, 3) == 0);
    CHECK(hal_rt_commit_cycle(c) == 0);
    CHECK(servo[0][DEV_DICT_ROLE_CONTROL_WORD] == 0x040Fu);

    /* 驱动器报故障，请求复位 */
    servo[0][DEV_DICT_ROLE_STATUS_WORD] = 0x0048u;   /* bit3 = Fault */
    int32_t st = -1;
    begin(c);
    CHECK(hal_rt_axis_fault_reset(c, 3) == 0);
    CHECK(hal_rt_axis_fault_reset_state(c, 3, &st) == 0 && st == HAL_RESET_PENDING);
    CHECK(hal_rt_commit_cycle(c) == 0);

    /* 复位低电平是**下一拍 begin** 才写下去的：那一拍的 commit 不得再用定向控制字
     * 盖掉它（没修之前就是这里把复位序列吃掉，一直等到超时）。 */
    cycle(c);
    CHECK(servo[0][DEV_DICT_ROLE_CONTROL_WORD] != 0x040Fu);
    CHECK((servo[0][DEV_DICT_ROLE_CONTROL_WORD] & 128u) == 0u);
    /* 定向状态被复位清掉（否则复位收口后会再把 0x040F 发下去，把轴重新使能） */
    int32_t os = -1;
    CHECK(hal_rt_spindle_orient_state(c, 3, &os) == 0 && os == HAL_ORIENT_NONE);

    /* 第二拍给高电平（复位上升沿） */
    begin(c); CHECK(hal_rt_commit_cycle(c) == 0);
    CHECK((servo[0][DEV_DICT_ROLE_CONTROL_WORD] & 128u) != 0u);
    CHECK(servo[0][DEV_DICT_ROLE_CONTROL_WORD] != 0x040Fu);

    /* 故障清掉 → 复位收口，且**保持未使能** */
    servo[0][DEV_DICT_ROLE_STATUS_WORD] = 0x0040u;
    for (int i = 0; i < 4; ++i) cycle(c);
    CHECK(hal_rt_axis_fault_reset_state(c, 3, &st) == 0 && st == HAL_RESET_DONE);
    HalCAxisStatus stt;
    CHECK(hal_rt_axis_read_status(c, 3, &stt) == 0 && stt.enabled == 0);
    hal_context_destroy(c);
}

/* 主轴定向：厂商控制字 + 状态字完成位。 */
static void spindle_orient(void) {
    HalCConfig cfg = config(); HalContext* c = create(&cfg);
    CHECK(hal_context_start(c, NULL, 0) == 0);
    CHECK(hal_rt_spindle_enable(c, 3, 1) == 0);
    for (int i = 0; i < 5; ++i) cycle(c);

    /* 还没请求时是 NONE；定向期间主轴的伺服在索引 0。 */
    int32_t os = -1;
    begin(c);
    CHECK(hal_rt_spindle_orient_state(c, 3, &os) == 0 && os == HAL_ORIENT_NONE);
    CHECK(hal_rt_spindle_request_orient(c, 3) == 0);
    CHECK(hal_rt_spindle_orient_state(c, 3, &os) == 0 && os == HAL_ORIENT_RUNNING);
    /* 控制字要到 commit 才下发（HAL 的周期契约：write 只暂存）。 */
    CHECK(hal_rt_commit_cycle(c) == 0);
    CHECK(servo[0][DEV_DICT_ROLE_CONTROL_WORD] == 0x040Fu);
    CHECK(servo[0][DEV_DICT_ROLE_TARGET_SPEED] == 0u);

    /* 本轮提交后先见低，再报完成位；继续发控制字保持对准。 */
    servo[0][DEV_DICT_ROLE_STATUS_WORD] = 0x0037u;
    cycle(c);
    servo[0][DEV_DICT_ROLE_STATUS_WORD] = 0x0437u;
    begin(c);
    CHECK(hal_rt_spindle_orient_state(c, 3, &os) == 0 && os == HAL_ORIENT_DONE);
    CHECK(hal_rt_commit_cycle(c) == 0);
    CHECK(servo[0][DEV_DICT_ROLE_CONTROL_WORD] == 0x040Fu);

    /* 释放：状态回 NONE，控制字不再被定向机制接管。
     * 断言"回到 DS402 标准值"而不是某个具体值——后者要跟着桩的状态机细节走，
     * 太脆。这里要证的只是"厂商控制字已经被撤销"。 */
    servo[0][DEV_DICT_ROLE_STATUS_WORD] = 0x0023u;
    cycle(c);
    begin(c);
    CHECK(hal_rt_spindle_release_orient(c, 3) == 0);
    CHECK(hal_rt_spindle_orient_state(c, 3, &os) == 0 && os == HAL_ORIENT_NONE);
    CHECK(hal_rt_commit_cycle(c) == 0);
    {
        const uint32_t cw = servo[0][DEV_DICT_ROLE_CONTROL_WORD];
        CHECK(cw == 0u || cw == 6u || cw == 7u || cw == 15u);
    }
    hal_context_destroy(c);

    /* 没配定向控制字的轴必须明确拒绝，不能静默当成成功。 */
    cfg = config();
    cfg.spindles[0].axis.vendor_orient_control_word = 0;
    cfg.spindles[0].axis.vendor_orient_done_mask = 0;
    c = create(&cfg);
    CHECK(hal_context_start(c, NULL, 0) == 0);
    CHECK(hal_rt_spindle_enable(c, 3, 1) == 0);
    for (int i = 0; i < 5; ++i) cycle(c);
    begin(c);
    CHECK(hal_rt_spindle_request_orient(c, 3) == HAL_ERROR_UNSUPPORTED);
    CHECK(hal_rt_commit_cycle(c) == 0);
    hal_context_destroy(c);

    /* 进给轴不是主轴：同样的调用必须被拒。 */
    cfg = config(); c = create(&cfg);
    CHECK(hal_context_start(c, NULL, 0) == 0);
    begin(c);
    CHECK(hal_rt_spindle_request_orient(c, 7) == HAL_ERROR_ARGUMENT);
    CHECK(hal_rt_commit_cycle(c) == 0);
    hal_context_destroy(c);
}

/* hal_rt_axis_readiness：**只读**说明"还差什么才能受理运动指令"。
 * 这个接口此前是公共 API 却一条用例都没有 —— 补上，顺便把它的口径钉住。 */
static void axis_readiness(void) {
    uint32_t r = 0xFFFFFFFFu;
    HalCConfig cfg = config();
    HalContext* c = create(&cfg);
    CHECK(hal_context_start(c, NULL, 0) == 0);

    /* 没跑过 begin：不在 begin 与 commit 之间 */
    CHECK(hal_rt_axis_readiness(c, 7, &r) == 0 && (r & HAL_READY_PHASE));

    begin(c);
    /* 窗口内、但还没使能 */
    CHECK(hal_rt_axis_readiness(c, 7, &r) == 0);
    CHECK((r & HAL_READY_PHASE) == 0);
    CHECK(r & HAL_READY_NOT_ENABLED);
    CHECK(hal_rt_commit_cycle(c) == 0);

    /* 使能并等状态机爬到 OperationEnabled。进给轴配置就是位置模式，
     * 模式两位也该清掉，最终全 0 = 可受理。 */
    CHECK(hal_rt_axis_enable(c, 7, 1) == 0);
    for (int i = 0; i < 20; ++i) {
        cycle(c);
        begin(c);
        CHECK(hal_rt_axis_readiness(c, 7, &r) == 0);
        CHECK(hal_rt_commit_cycle(c) == 0);
        if (r == 0) break;
    }
    CHECK(r == 0);

    /* 主轴：配置的是速度模式。请求切到位置模式后，在驱动器 0x6061 跟上来之前
     * MODE_ACTUAL 置位（"切换还没生效"）；跟上之后全 0 —— 这时 write_pos 才受理。
     * 这就是"先 request_mode、等全 0、再下发"那条用法。 */
    begin(c);   /* 必须在窗口里：上面循环最后一次已经 commit 了 */
    CHECK(hal_rt_spindle_request_mode(c, 3, HAL_SPINDLE_CSP) == 0);
    CHECK(hal_rt_spindle_enable(c, 3, 1) == 0);
    CHECK(hal_rt_axis_readiness(c, 3, &r) == 0);
    CHECK(r & HAL_READY_MODE_ACTUAL);
    CHECK(hal_rt_commit_cycle(c) == 0);

    for (int i = 0; i < 20; ++i) {
        cycle(c);
        begin(c);
        CHECK(hal_rt_axis_readiness(c, 3, &r) == 0);
        CHECK(hal_rt_commit_cycle(c) == 0);
        if (r == 0) break;
    }
    CHECK(r == 0);

    /* 轴不存在 / 出参为空 */
    CHECK(hal_rt_axis_readiness(c, 99, &r) == HAL_ERROR_ARGUMENT);
    CHECK(hal_rt_axis_readiness(c, 7, NULL) == HAL_ERROR_ARGUMENT);
    hal_context_destroy(c);
}

/* PDO 位长判据：位置类必须正好 32 位，其余不窄于语义宽度就收。
 * 依据是"驱动层读回来右对齐"——位长宽了数值不变，窄了才丢位。 */
static void pdo_width_policy(void) {
    HalCConfig cfg = config();
    char error[160];

    /* 模式字 16 位应当被接受：右对齐读回来照样是 8/9，写进去也一样。
     * 真机例子：蓝海华腾的 0x6060/0x6061 在 ESI 里就是 16 位。 */
    wide_mode_bits = 1;
    HalContext* c = create(&cfg);
    CHECK(hal_context_start(c, error, sizeof(error)) == 0);
    hal_context_destroy(c);
    wide_mode_bits = 0;

    /* 位置 PDO 只有 16 位必须拒：signed32() 会把 0xFFFF 当成 +65535 而不是 -1，
     * 负位置读成正的，位置环直接飞车。 */
    narrow_pos_bits = 1;
    c = create(&cfg);
    CHECK(hal_context_start(c, error, sizeof(error)) == HAL_ERROR_UNSUPPORTED);
    hal_context_destroy(c);
    narrow_pos_bits = 0;
}

/* 释放定向**不得**替上层决定使能意图。
 *
 * 场景：定向完成后上层明确要求去使能，紧接着释放定向。
 * 如果 release_orient 把 request 硬掰回 ENABLE，上层刚下的去使能就被静默吃掉——
 * 释放只该"把控制字交回状态机"，要不要上力矩是上层的事。
 *
 * 断言方式：释放后再跑两拍，看状态机把控制字推到哪。
 *   尊重意图 → 停在未使能族（0x00/0x06/0x07）
 *   吃掉意图 → 爬到 0x0F（Enable operation）
 */
static void orient_release_keeps_enable_intent(void) {
    HalCConfig cfg = config(); HalContext* c = create(&cfg);
    CHECK(hal_context_start(c, NULL, 0) == 0);
    CHECK(hal_rt_axis_enable(c, 3, 1) == 0);
    for (int i = 0; i < 5; ++i) cycle(c);

    begin(c);
    CHECK(hal_rt_spindle_request_orient(c, 3) == 0);
    CHECK(hal_rt_commit_cycle(c) == 0);
    servo[0][DEV_DICT_ROLE_STATUS_WORD] = 0x0037u;
    cycle(c);
    servo[0][DEV_DICT_ROLE_STATUS_WORD] = 0x0437u;
    cycle(c);

    int32_t os = -1;
    begin(c);
    CHECK(hal_rt_spindle_orient_state(c, 3, &os) == 0 && os == HAL_ORIENT_DONE);

    /* 上层改主意：去使能，然后释放定向。 */
    servo[0][DEV_DICT_ROLE_STATUS_WORD] = 0x0023u;
    CHECK(hal_rt_axis_enable(c, 3, 0) == 0);
    CHECK(hal_rt_spindle_release_orient(c, 3) == 0);
    CHECK(hal_rt_commit_cycle(c) == 0);

    cycle(c); cycle(c);
    {
        const uint32_t cw = servo[0][DEV_DICT_ROLE_CONTROL_WORD];
        if (!(cw == 0u || cw == 6u || cw == 7u)) {
            fprintf(stderr, "line %d: 释放定向把去使能意图吃掉了，控制字=0x%02X\n",
                    __LINE__, cw);
            abort();
        }
        ++checks;
    }
    hal_context_destroy(c);
}

static void orient_release_clears_vendor_control(void) {
    for (int action = 0; action < 4; ++action) {
        HalCConfig cfg = config();
        HalContext* c = create(&cfg);
        CHECK(hal_context_start(c, NULL, 0) == 0);
        enable(c);
        begin(c);
        CHECK(hal_rt_spindle_request_orient(c, 3) == 0);
        CHECK(hal_rt_commit_cycle(c) == 0);
        servo[0][DEV_DICT_ROLE_STATUS_WORD] = 0x0037u;
        cycle(c);
        servo[0][DEV_DICT_ROLE_STATUS_WORD] = 0x1737u;
        begin(c);
        int32_t os;
        CHECK(hal_rt_spindle_orient_state(c, 3, &os) == 0 && os == HAL_ORIENT_DONE);
        CHECK(servo[0][DEV_DICT_ROLE_CONTROL_WORD] == 0x040Fu);
        CHECK(hal_rt_spindle_release_orient(c, 3) == 0);
        /* release 只暂存，不能在 commit 之前写 PDO。 */
        CHECK(servo[0][DEV_DICT_ROLE_CONTROL_WORD] == 0x040Fu);
        if (action == 1) CHECK(hal_rt_spindle_enable(c, 3, 0) == 0);
        if (action == 2) CHECK(hal_rt_spindle_estop(c, 3) == 0);
        if (action == 3) {
            servo[0][DEV_DICT_ROLE_STATUS_WORD] = 8u;
            CHECK(hal_rt_axis_fault_reset(c, 3) == 0);
        }
        CHECK(hal_rt_commit_cycle(c) == 0);
        if (action == 0) {
            CHECK(servo[0][DEV_DICT_ROLE_CONTROL_WORD] == DS402_CW_ENABLE_OP);
            cycle(c);
            begin(c);
            CHECK(hal_rt_spindle_orient_state(c, 3, &os) == 0 && os == HAL_ORIENT_NONE);
            CHECK(hal_rt_spindle_write_speed(c, 3, 100, 1) == 0);
            CHECK(hal_rt_commit_cycle(c) == 0);
            CHECK(servo[0][DEV_DICT_ROLE_CONTROL_WORD] == DS402_CW_ENABLE_OP);
            CHECK(servo[0][DEV_DICT_ROLE_TARGET_SPEED] == 100u);
        } else if (action == 3) {
            cycle(c);
            CHECK(servo[0][DEV_DICT_ROLE_CONTROL_WORD] == 0u);
            cycle(c);
            CHECK(servo[0][DEV_DICT_ROLE_CONTROL_WORD] == DS402_CW_FAULT_RESET);
            cycle(c); cycle(c);
            begin(c);
            HalCAxisStatus st;
            CHECK(hal_rt_axis_read_status(c, 3, &st) == 0 && !st.enabled);
            CHECK(hal_rt_commit_cycle(c) == 0);
            CHECK(servo[0][DEV_DICT_ROLE_CONTROL_WORD] == 0u);
        } else {
            CHECK(servo[0][DEV_DICT_ROLE_CONTROL_WORD] ==
                (action == 1 ? DS402_CW_SWITCH_ON : DS402_CW_DISABLE_VOLTAGE));
            CHECK(servo[0][DEV_DICT_ROLE_TARGET_SPEED] == 0u);
        }
        hal_context_destroy(c);
    }
}

static void stopped_orientation_does_not_resume(void) {
    for (int action = 0; action < 4; ++action) {
        HalCConfig cfg = config();
        if (action == 2) cfg.spindles[0].axis.estop_action = HAL_ESTOP_DISABLE_OPERATION;
        HalContext* c = create(&cfg);
        CHECK(hal_context_start(c, NULL, 0) == 0);
        enable(c);
        begin(c);
        CHECK(hal_rt_spindle_request_orient(c, 3) == 0);
        CHECK(hal_rt_commit_cycle(c) == 0);
        servo[0][DEV_DICT_ROLE_STATUS_WORD] = 0x1737u;
        begin(c);
        if (action == 0) CHECK(hal_rt_spindle_enable(c, 3, 0) == 0);
        else if (action < 3) CHECK(hal_rt_spindle_estop(c, 3) == 0);
        else CHECK(hal_rt_spindle_request_mode(c, 3, HAL_SPINDLE_CSP) == 0);
        int32_t os;
        CHECK(hal_rt_spindle_orient_state(c, 3, &os) == 0 && os == HAL_ORIENT_NONE);
        if (action < 3) CHECK(hal_rt_spindle_request_orient(c, 3) == HAL_ERROR_NOT_ENABLED);
        CHECK(hal_rt_commit_cycle(c) == 0);
        CHECK(servo[0][DEV_DICT_ROLE_CONTROL_WORD] != 0x040Fu);
        if (action < 3) CHECK(hal_rt_spindle_enable(c, 3, 1) == 0);
        for (int i = 0; i < 8; ++i) {
            cycle(c);
            CHECK(servo[0][DEV_DICT_ROLE_CONTROL_WORD] != 0x040Fu);
        }
        begin(c);
        HalCSpindleStatus st;
        CHECK(hal_rt_spindle_read_status(c, 3, &st) == 0 && st.enabled);
        CHECK(st.mode == (action == 3 ? HAL_SPINDLE_CSP : HAL_SPINDLE_CSV));
        CHECK(hal_rt_spindle_orient_state(c, 3, &os) == 0 && os == HAL_ORIENT_NONE);
        /* 新的显式请求仍可启动定向。 */
        CHECK(hal_rt_spindle_request_orient(c, 3) == 0);
        CHECK(hal_rt_commit_cycle(c) == 0);
        CHECK(servo[0][DEV_DICT_ROLE_CONTROL_WORD] == 0x040Fu);
        hal_context_destroy(c);
    }
}

static void spindle_prepare_csp(void) {
    HalCConfig cfg = config();
    HalContext* c = create(&cfg);
    CHECK(hal_context_start(c, NULL, 0) == HAL_OK);
    int32_t state, result;
    CHECK(hal_rt_spindle_prepare_csp(c, 3, 100) == HAL_ERROR_NOT_RUNNING);
    enable(c);
    servo[0][DEV_DICT_ROLE_ACTUAL_SPEED] = 100;
    begin(c);
    CHECK(hal_rt_spindle_prepare_csp(c, 7, 100) == HAL_ERROR_ARGUMENT);
    CHECK(hal_rt_spindle_prepare_csp(c, 3, 0) == HAL_ERROR_ARGUMENT);
    CHECK(hal_rt_axis_set_pos(c, 3, 125) == HAL_OK);
    CHECK(hal_rt_spindle_write_speed(c, 3, 100, 1) == HAL_OK);
    CHECK(hal_rt_spindle_prepare_csp(c, 3, 100) == HAL_OK);
    CHECK(hal_rt_spindle_prepare_csp(c, 3, 1000) == HAL_ERROR_BUSY);
    CHECK(hal_rt_spindle_write_speed(c, 3, 10, 1) == HAL_ERROR_BUSY);
    CHECK(hal_rt_spindle_write_pos(c, 3, 10) == HAL_ERROR_BUSY);
    CHECK(hal_rt_axis_set_pos(c, 3, 0) == HAL_ERROR_BUSY);
    CHECK(hal_rt_spindle_request_mode(c, 3, HAL_SPINDLE_CSP) == HAL_ERROR_BUSY);
    CHECK(hal_rt_spindle_request_orient(c, 3) == HAL_ERROR_BUSY);
    CHECK(hal_rt_commit_cycle(c) == HAL_OK);
    CHECK(servo[0][DEV_DICT_ROLE_TARGET_SPEED] == 0);
    for (int i = 0; i < 12; ++i) cycle(c);
    CHECK(hal_rt_spindle_prepare_csp_state(c, 3, &state, &result) == HAL_OK);
    CHECK(state == HAL_CSP_PREPARE_STOPPING && result == HAL_ERROR_BUSY);
    CHECK(servo[0][DEV_DICT_ROLE_OP_MODE] == DS402_MODE_CSV);
    servo[0][DEV_DICT_ROLE_ACTUAL_SPEED] = 0;
    for (int i = 0; i < 35; ++i) {
        cycle(c);
        CHECK(servo[0][DEV_DICT_ROLE_CONTROL_WORD] != 0x040f);
    }
    CHECK(hal_rt_spindle_prepare_csp_state(c, 3, &state, &result) == HAL_OK);
    CHECK(state == HAL_CSP_PREPARE_DONE && result == HAL_OK);
    HalCAxisStatus axis;
    CHECK(hal_rt_axis_read_status(c, 3, &axis) == HAL_OK);
    CHECK(axis.enabled && axis.actual_pos == 125 && axis.command_pos == 125);
    CHECK(servo[0][DEV_DICT_ROLE_TARGET_POS] == 50); /* Separate command/feedback scales. */
    begin(c);
    CHECK(hal_rt_spindle_prepare_csp(c, 3, 100) == HAL_OK);
    CHECK(hal_rt_spindle_estop(c, 3) == HAL_OK);
    CHECK(hal_rt_spindle_prepare_csp_state(c, 3, &state, &result) == HAL_OK);
    CHECK(state == HAL_CSP_PREPARE_CANCELLED && result == HAL_ERROR_STOPPED);
    CHECK(hal_rt_spindle_prepare_csp(c, 3, 100) == HAL_ERROR_NOT_RUNNING);
    CHECK(hal_rt_commit_cycle(c) == HAL_OK);
    hal_context_destroy(c);

    c = create(&cfg);
    CHECK(hal_context_start(c, NULL, 0) == HAL_OK);
    begin(c); /* Cold, disabled CSV must also prepare successfully. */
    CHECK(hal_rt_spindle_prepare_csp(c, 3, 100) == HAL_OK);
    CHECK(hal_rt_commit_cycle(c) == HAL_OK);
    for (int i = 0; i < 40; ++i) cycle(c);
    CHECK(hal_rt_spindle_prepare_csp_state(c, 3, &state, &result) == HAL_OK);
    CHECK(state == HAL_CSP_PREPARE_DONE);
    servo[0][DEV_DICT_ROLE_ACTUAL_SPEED] = 10;
    begin(c);
    CHECK(hal_rt_spindle_prepare_csp(c, 3, 100) == HAL_ERROR_BUSY);
    CHECK(hal_rt_commit_cycle(c) == HAL_OK);
    hal_context_destroy(c);

    for (int fault = 0; fault < 2; ++fault) {
        c = create(&cfg);
        CHECK(hal_context_start(c, NULL, 0) == HAL_OK);
        enable(c);
        servo[0][DEV_DICT_ROLE_ACTUAL_SPEED] = (uint32_t)-100;
        begin(c);
        CHECK(hal_rt_spindle_prepare_csp(c, 3, 5) == HAL_OK);
        CHECK(hal_rt_commit_cycle(c) == HAL_OK);
        if (fault) servo[0][DEV_DICT_ROLE_ERROR_CODE] = 0xff43;
        for (int i = 0; i < 7; ++i) cycle(c);
        CHECK(hal_rt_spindle_prepare_csp_state(c, 3, &state, &result) == HAL_OK);
        CHECK(state == HAL_CSP_PREPARE_ERROR);
        CHECK(result == (fault ? HAL_ERROR_NOT_RUNNING : HAL_ERROR_TIMEOUT));
        CHECK(servo[0][DEV_DICT_ROLE_TARGET_SPEED] == 0);
        CHECK(servo[0][DEV_DICT_ROLE_CONTROL_WORD] == DS402_CW_SWITCH_ON);
        hal_context_destroy(c);
    }
    for (int reset = 0; reset < 2; ++reset) {
        c = create(&cfg);
        CHECK(hal_context_start(c, NULL, 0) == HAL_OK);
        begin(c);
        CHECK(hal_rt_spindle_prepare_csp(c, 3, 100) == HAL_OK);
        CHECK((reset ? hal_rt_axis_fault_reset(c, 3) :
                       hal_rt_spindle_enable(c, 3, 0)) == HAL_OK);
        CHECK(hal_rt_spindle_prepare_csp_state(c, 3, &state, &result) == HAL_OK);
        CHECK(state == HAL_CSP_PREPARE_CANCELLED);
        CHECK(hal_rt_commit_cycle(c) == HAL_OK);
        for (int i = 0; i < 30; ++i) cycle(c);
        CHECK(hal_rt_axis_read_status(c, 3, &axis) == HAL_OK && !axis.enabled);
        hal_context_destroy(c);
    }
    omit_spindle_speed = 1;
    c = create(&cfg);
    CHECK(hal_context_start(c, NULL, 0) == HAL_ERROR_UNSUPPORTED);
    hal_context_destroy(c);
    omit_spindle_speed = 0;
}

int main(void) {
    lifecycle(); binding(); optional_capabilities(); feed_speed_mm_min(); velocity_only_spindle(); motion_io(); wrap_and_faults();
    stop_overrides_staged_enable(); concurrent_stop();
    preset_overflow_probe(); spindle_origin_angle(); angle_regressions();
    pdo_width_policy();
    fault_reset_and_bus_health(); io_image_size(); review_regressions();
    voltage_stop_priority(); standard_enabled_status(); coverage_gaps();
    spindle_orient(); spindle_orient_delay(); spindle_orient_fresh_edge(); orient_then_fault_reset();
    speed_unit_pulses_per_second();
    orient_release_keeps_enable_intent(); orient_release_clears_vendor_control();
    stopped_orientation_does_not_resume(); axis_readiness();
    spindle_prepare_csp();
    printf("context_regression: %u checks passed\n", checks);
    return 0;
}
