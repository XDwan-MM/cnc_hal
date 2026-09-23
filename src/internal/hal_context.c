/* HAL 运行层：context 生命周期 + 周期三原语（wait / begin / commit）。
 * 契约与设计推导见 docs/runtime.md、docs/rt-interface.md。 */

#include "hal_c_api.h"
#include "Greemaster/main_demo.h"
#include "Greemaster/device_table.h"
#include "Greemaster/servo_step.h"
#include "Greemaster/entry_access.h"
#include <math.h>
#include <stdatomic.h>
#include <time.h>

/* 一根轴的运行期状态 = 配置副本 + 采样快照 + 未提交的指令。
 * 轴与主轴共用这个结构（HalAxisId 就落在这里的下标上），转速侧字段对进给轴无意义。 */
typedef struct {
    HalCAxisCfg    cfg;          /* 生效的配置副本；主轴取自 spindle_cfg.axis */
    HalCSpindleCfg spindle_cfg;  /* 仅主轴有效：转速上限与到位窗口来自这里 */
    HalCIdentity   identity;     /* 绑定时填；slot == -1 时为零值 */

    int slot;          /* DeviceTable 槽号；-1 = 未绑定（绑不上就不让启动） */
    int spindle;       /* 非零 = 这是主轴（下标 >= config.axis_count） */
    int sampled;       /* 本 context 内是否采过样：决定 counts 取初值还是取增量 */
    int mode;          /* 驱动器回报的当前模式（0x6061）；-1 = 还没采到 */
    int desired_mode;  /* HAL 要求驱动器处在的模式（CSP/CSV），每拍传给状态机 */

    Ds402Request request;  /* 推进方向。每拍都要传，直到状态字显示已达成为止 */

    int pos_dirty;      /* 有未下发的目标位置，commit 时写 0x607A */
    int speed_dirty;    /* 有未下发的目标速度，commit 时写 0x60FF */
    int zero_speed;     /* 需要一次性把速度目标清 0（使能/取消运动/急停后），发完清掉 */
    int stop_override;  /* 非零 = 急停覆盖控制字：HAL_ESTOP_* 决定写 0x07 还是 0x00 */
    int motion_ready;   /* 上一拍使能且状态机到位；write_pos/speed 的前置条件 */

    uint32_t raw_pos;        /* 上一拍实际位置的原始计数（32 位），增量靠它算 */
    uint32_t pos_command;    /* 待下发的目标位置原始计数 */
    uint32_t speed_command;  /* 待下发的目标速度原始计数（deg/s ÷ 命令当量） */

    double counts;           /* 展开后的累计反馈计数；线性按 32 位回绕，模按每转脉冲数 */
    double feedback_offset;  /* set_pos 平移坐标的偏置（用户单位），不写驱动器 */
    double command_offset;   /* 命令侧同上；两侧必须一起平移，否则下一次写位置会跳 */

    HalCAxisStatus   status;  /* 最近一次采样的轴快照，read_status 直接返回它 */
    HalCSpindleStatus speed;  /* 主轴转速侧快照；进给轴也填，但没有转速语义 */
} Axis;

/* 一块 IO 模块或面板的运行期状态。两者共用，差别只在绑定时的类型过滤
 * 与 Entry 来源（面板取 entries.control，普通 IO 取 entries.io）。 */
typedef struct {
    int slot;       /* 槽号；-1 = 未绑定 */
    int in_count;   /* 输入 Entry 个数（不是位数，也不是字节数） */
    int out_count;  /* 输出 Entry 个数 */

    uint32_t x, y;                  /* 输入/输出段在 PLC 地址域里的【字节】基址 */
    uint32_t in_bytes, out_bytes;   /* 各 Entry 位长求和后向上取整的字节数 */

    uint8_t  in_bits[64];   /* 每个输入 Entry 的位长（1..32）；采样时拿它校验实际位长 */
    uint8_t  out_bits[64];  /* 每个输出 Entry 的位长；打包时按它取位 */
    uint8_t  inputs[256];   /* 本拍输入按位序打包的映像；64 Entry × 32 位 = 上限 */
    uint32_t outputs[64];   /* 待下发的输出 Entry 值，每 Entry 一个词 */

    int dirty;  /* 本轮 outputs 有改动；到 commit 才写总线 */
} Io;

struct HalContext {
    HalCConfig config;          /* create 时拷入，运行期只读 */
    atomic_int stop_requested;  /* 其它线程可写；周期线程在 wait/commit 前后查它 */
    int running;                /* start 成功后置 1；stop 或启动回滚时清 0 */
    int phase;                  /* 周期阶段 0 空闲 / 1 已 wait / 2 已 begin，用来卡调用顺序 */
    int sampled;                /* 跑过至少一轮 begin；read_* 在此之前拒绝 */
    int fault;                  /* 粘性总线错误码，gate 会一直返回它，直到 stop/start */
    int axis_count;             /* 轴 + 主轴总数；HalAxisId 就在这个下标空间里 */
    int io_count;               /* IO + 面板总数 */
    uint32_t x_size, y_size;    /* 输入/输出映像所需字节数 = 各段末尾的最大值 */
    Axis axes[HAL_C_MAX_DEV];   /* 前 axis_count 个有效 */
    Io   ios[HAL_C_MAX_DEV];    /* 前 io_count 个有效 */
};

/* SDK 的主站是进程全局单例；所有 context 生命周期操作由调用方串行。 */
static HalContext* owner;

/* 统一的「填错误文本 + 返回码」出口，只用于非 RT 路径（RT 路径不碰 err 缓冲）。 */
static int report(int code, char* err, uint32_t len, const char* detail) {
    if (err && len) snprintf(err, len, "%s", detail);
    return code;
}

/* 周期原语与轴指令的公共前检，次序即优先级：空指针 → 未运行 → 已请求停止 → 粘性故障。 */
static int gate(HalContext* c) {
    if (!c) return HAL_ERROR_ARGUMENT;
    if (!c->running) return HAL_ERROR_NOT_RUNNING;
    if (atomic_load(&c->stop_requested)) return HAL_ERROR_STOPPED;
    return c->fault;
}

/* 只记第一个总线错误：后续每一拍都栽在同一个码上，不被新错误覆盖。 */
static int bus_error(HalContext* c) {
    if (!c->fault) c->fault = HAL_ERROR_BUS;
    return c->fault;
}

/* 按逻辑轴号找轴。主轴也在 axes[] 里，一并命中——HalAxisId 对两者是同一个东西。 */
static Axis* find_axis(const HalContext* c, HalAxisId id) {
    if (!c || id == HAL_INVALID_ID) return NULL;
    for (int i = 0; i < c->axis_count; ++i)
        if (c->axes[i].cfg.logical_axis == (int32_t)id) return (Axis*)&c->axes[i];
    return NULL;
}

/* 32 位原始计数按二进制补码解释成有符号值。 */
static double signed32(uint32_t value) {
    return value <= INT32_MAX ? (double)value : (double)value - 4294967296.0;
}

/* 命令侧当量（用户单位/计数），带方向符号。 */
static double command_scale(const Axis* a) {
    return a->cfg.command_units_per_count * (a->cfg.command_invert ? -1 : 1);
}

/* 反馈侧当量，带方向符号。与命令侧分开——外接光栅尺可能只需要翻反馈。 */
static double feedback_scale(const Axis* a) {
    return a->cfg.feedback_units_per_count * (a->cfg.feedback_invert ? -1 : 1);
}

/* 用户单位 → 原始计数：四舍五入；非有限值或超出 int32 一律拒绝，不静默饱和或截断。 */
static int to_counts(double value, uint32_t* out) {
    if (!isfinite(value)) return HAL_ERROR_ARGUMENT;
    const double rounded = round(value);
    if (rounded < INT32_MIN || rounded > INT32_MAX) return HAL_ERROR_ARGUMENT;
    *out = (uint32_t)(int32_t)rounded;
    return HAL_OK;
}

/* 驱动器不给序列号时的设备 id：FNV-1a(vendor, product, revision)。 */
static uint64_t identity_hash(const DeviceSlot* s) {
    uint64_t h = UINT64_C(14695981039346656037);
    const uint32_t values[] = {s->vendor_id, s->product_code, s->revision};
    for (int i = 0; i < 3; ++i)
        for (int byte = 0; byte < 4; ++byte) {
            h ^= (values[i] >> (byte * 8)) & 255u;
            h *= UINT64_C(1099511628211);
        }
    return h;
}

/* 按 (从站位置, 从站内轴序号) 在槽表里认领本轴，顺带填身份和「同型号第几台」。
 * 找不到或类型不是伺服 → 配置错误：启动失败，不做降级。 */
static int bind_axis(Axis* a, const DeviceSlot* slots, int count) {
    a->slot = -1;
    for (int i = 0; i < count; ++i) {
        const DeviceSlot* s = &slots[i];
        if (s->slave_pos != a->cfg.slave_pos || s->axis_index != a->cfg.axis_index) continue;
        if (s->type != SERVO_TYPE && s->type != GREE_AXIS6_TYPE && s->type != GREE_AXIS4_TYPE) continue;
        a->slot = i;
        a->identity.type = s->type;
        a->identity.vendor_id = s->vendor_id;
        a->identity.product_code = s->product_code;
        a->identity.revision = s->revision;
        a->identity.serial = s->serial;
        memcpy(a->identity.name, s->name, sizeof(a->identity.name));
        a->identity.name[sizeof(a->identity.name) - 1] = 0;
        a->identity.device_id = s->serial ? s->serial : identity_hash(s);
        int last_slave = -1;
        for (int j = 0; j < i; ++j)
            if (slots[j].slave_pos != s->slave_pos && slots[j].slave_pos != last_slave &&
                slots[j].vendor_id == s->vendor_id && slots[j].product_code == s->product_code) {
                ++a->identity.family_index;
                last_slave = slots[j].slave_pos;
            }
        return HAL_OK;
    }
    return HAL_ERROR_CONFIG;
}

/* 按从站号认领面板/IO 槽，累计各 Entry 位长得字节数。
 * 位长必须在 1..32、Entry 数 ≤ 64 —— 这里放宽会让映像长度算错。 */
static int bind_io(Io* io, const DeviceSlot* slots, int n, int slave, int panel) {
    for (int i = 0; i < n; ++i) {
        const DeviceSlot* s = &slots[i];
        if (s->slave_pos != slave) continue;
        if (panel ? s->type != CONTROL_PANEL_TYPE :
            (s->type != IO_MODEL_TYPE && s->type != IO_EXPANSION_TYPE)) continue;
        io->slot = i;
        io->in_count = panel ? s->entries.control.in_count : s->entries.io.io_in_count;
        io->out_count = panel ? s->entries.control.out_count : s->entries.io.io_out_count;
        if (io->in_count > 64 || io->out_count > 64) return HAL_ERROR_CONFIG;
        uint32_t in = 0, out = 0;
        for (int j = 0; j < io->in_count; ++j) {
            const int bits = panel ? s->entries.control.input_addr[j].bit_length : s->entries.io.io_input_addr[j].bit_length;
            if (bits < 1 || bits > 32) return HAL_ERROR_CONFIG;
            io->in_bits[j] = (uint8_t)bits; in += (uint32_t)bits;
        }
        for (int j = 0; j < io->out_count; ++j) {
            const int bits = panel ? s->entries.control.output_addr[j].bit_length : s->entries.io.io_output_addr[j].bit_length;
            if (bits < 1 || bits > 32) return HAL_ERROR_CONFIG;
            io->out_bits[j] = (uint8_t)bits; out += (uint32_t)bits;
        }
        io->in_bytes = (in + 7) / 8; io->out_bytes = (out + 7) / 8;
        return HAL_OK;
    }
    return HAL_ERROR_CONFIG;
}

/* 两个字节段是否相交。长度为 0 的段不参与——没用到的段不算重叠。 */
static int overlap(uint32_t a, uint32_t an, uint32_t b, uint32_t bn) {
    return an && bn && (uint64_t)a < (uint64_t)b + bn && (uint64_t)b < (uint64_t)a + an;
}

/* 清回「未启动」。create 与 start 都调，使重启动严格等价于首次启动：
 * 旧请求、坐标基准、快照与错误闭锁一律不跨重启保留。 */
static void reset_runtime(HalContext* c) {
    c->running = c->phase = c->sampled = c->fault = 0;
    c->axis_count = c->config.axis_count + c->config.spindle_count;
    c->io_count = c->config.io_count + c->config.panel_count;
    c->x_size = c->y_size = 0;
    memset(c->axes, 0, sizeof(c->axes));
    memset(c->ios, 0, sizeof(c->ios));
    for (int i = 0; i < c->axis_count; ++i) {
        Axis* a = &c->axes[i];
        a->spindle = i >= c->config.axis_count;
        if (a->spindle) {
            a->spindle_cfg = c->config.spindles[i - c->config.axis_count];
            a->cfg = a->spindle_cfg.axis;
        } else a->cfg = c->config.axes[i];
        a->slot = -1;
        a->mode = -1;
        /* 兼容 RT 的 enc_off：显示坐标=原始反馈坐标-enc_off；命令反向补回。 */
        a->feedback_offset = -a->cfg.enc_off;
        a->command_offset = -a->cfg.enc_off;
        a->desired_mode = a->cfg.work_mode == HAL_WORK_POSITION ? DS402_MODE_CSP : DS402_MODE_CSV;
        a->request = DS402_REQ_DISABLE;
        a->zero_speed = 1;
        a->speed.mode = -1;
    }
}

/* 只做静态校验 + 分配，不碰硬件。失败时 *out 保持 NULL。 */
int32_t hal_context_create(const HalCConfig* config, HalContext** out, char* err, uint32_t len) {
    if (!out) return report(HAL_ERROR_ARGUMENT, err, len, "context 出参为空");
    *out = NULL;
    int rc = hal_config_validate(config, err, len);
    if (rc) return rc;
    HalContext* c = calloc(1, sizeof(*c));
    if (!c) return report(HAL_ERROR_MEMORY, err, len, "context 分配失败");
    c->config = *config;
    atomic_init(&c->stop_requested, 0);
    reset_runtime(c);
    *out = c;
    return HAL_OK;
}

/* 唯一会碰主站的地方：起主站 → 按配置认领设备 → 算 X/Y 映像尺寸。
 * 任何一步失败都整体回滚（关主站、清运行状态），不留半启动的 context。
 * 主站是进程全局单例，所以同时只允许一个启动中的 context（owner）。
 * 轴数、PDO 位宽、X/Y 段重叠都在这里才知道，故只有 create 的静态校验是不够的。 */
int32_t hal_context_start(HalContext* c, char* err, uint32_t len) {
    if (err && len) err[0] = 0;
    if (!c) return report(HAL_ERROR_ARGUMENT, err, len, "context 为空");
    if (owner) return report(HAL_ERROR_BUSY, err, len, "主站已被启动的 context 占用");
    reset_runtime(c);
    atomic_store(&c->stop_requested, 0);
    owner = c;
    struct timespec started, ended;
    if (clock_gettime(CLOCK_MONOTONIC, &started) != 0) {
        owner = NULL;
        return report(HAL_ERROR_BUS, err, len, "无法读取启动时钟");
    }
    MasterConfig cfg = {c->config.cycle_us, c->config.start_timeout_ms,
                        c->config.cycle_timeout_ms, c->config.dc_enable};
    const int driver_rc = ethercat_init(&cfg);
    if (driver_rc != 0) {
        owner = NULL;
        return report(driver_rc == MASTER_START_TIMEOUT ? HAL_ERROR_TIMEOUT : HAL_ERROR_BUS,
                      err, len, "主站启动失败，驱动已回滚");
    }
    const DeviceSlot* slots = NULL;
    const int n = DeviceTable_Get(&slots);
    int result = HAL_ERROR_CONFIG;
    const char* detail = "设备表无效";
    if (n < 0 || n > MAX_DEVICE_NUM || (n && !slots)) goto bad;
    detail = "配置物理轴未找到或类型不匹配";
    for (int i = 0; i < c->axis_count; ++i)
        if (bind_axis(&c->axes[i], slots, n)) goto bad;
    for (int i = 0; i < c->io_count; ++i) {
        const int panel = i >= c->config.io_count;
        const int j = panel ? i - c->config.io_count : i;
        Io* io = &c->ios[i];
        io->x = (uint32_t)(panel ? c->config.panels[j].x_start : c->config.ios[j].x_start);
        io->y = (uint32_t)(panel ? c->config.panels[j].y_start : c->config.ios[j].y_start);
        const int slave = panel ? c->config.panels[j].slave_pos : c->config.ios[j].slave_pos;
        detail = "IO/面板未找到、类型不匹配或 PDO 位宽无效";
        if (bind_io(io, slots, n, slave, panel)) goto bad;
        detail = "IO 与面板的 X/Y 地址段重叠";
        for (int k = 0; k < i; ++k)
            if (overlap(io->x, io->in_bytes, c->ios[k].x, c->ios[k].in_bytes) ||
                overlap(io->y, io->out_bytes, c->ios[k].y, c->ios[k].out_bytes)) goto bad;
        if (io->in_bytes && io->x + io->in_bytes > c->x_size) c->x_size = io->x + io->in_bytes;
        if (io->out_bytes && io->y + io->out_bytes > c->y_size) c->y_size = io->y + io->out_bytes;
    }
    detail = "启动总预算已耗尽";
    result = HAL_ERROR_TIMEOUT;
    if (clock_gettime(CLOCK_MONOTONIC, &ended) != 0 ||
        ((double)(ended.tv_sec - started.tv_sec) * 1000.0 +
         (double)(ended.tv_nsec - started.tv_nsec) / 1000000.0) >= c->config.start_timeout_ms) goto bad;
    c->running = 1;
    return HAL_OK;
bad:
    (void)ethercat_close();
    owner = NULL;
    reset_runtime(c);
    return report(result, err, len, detail);
}

/* 只写原子通知，不关主站——资源留到 stop。可与周期调用并发。 */
int32_t hal_context_request_stop(HalContext* c) {
    if (!c) return HAL_ERROR_ARGUMENT;
    atomic_store(&c->stop_requested, 1);
    return HAL_OK;
}

/* 关主站并清状态，重复调用无害。调用方必须先确认所有周期调用已退出。 */
int32_t hal_context_stop(HalContext* c) {
    if (!c) return HAL_ERROR_ARGUMENT;
    atomic_store(&c->stop_requested, 1);
    int rc = 0;
    if (owner == c) { rc = ethercat_close(); owner = NULL; }
    reset_runtime(c);
    return rc ? HAL_ERROR_BUS : HAL_OK;
}

/* stop + free。参数可空。 */
void hal_context_destroy(HalContext* c) {
    if (c) { (void)hal_context_stop(c); free(c); }
}

/* 逻辑轴号 → HalAxisId：校验存在后原样返回。留这个口子是为了让
 * 「逻辑轴号 ≠ 装配下标」有个落点，将来真加映射表时不动调用方。 */
int32_t hal_axis_resolve(const HalContext* c, int32_t logical, HalAxisId* out) {
    if (out) *out = HAL_INVALID_ID;
    if (!c || !out || logical < 0 || logical >= 32) return HAL_ERROR_ARGUMENT;
    if (!find_axis(c, (HalAxisId)logical)) return HAL_ERROR_ARGUMENT;
    *out = (HalAxisId)logical;
    return HAL_OK;
}

/* 可寻址的逻辑轴数量：只数 logical_axis >= 0 的，主轴也算在内。 */
int32_t hal_axis_count(const HalContext* c) {
    if (!c) return -HAL_ERROR_ARGUMENT;
    int n = 0;
    for (int i = 0; i < c->axis_count; ++i) n += c->axes[i].cfg.logical_axis >= 0;
    return n;
}

/* 身份是启动认领设备时才填的，所以未启动返回 NOT_RUNNING。 */
int32_t hal_device_identity(const HalContext* c, HalAxisId id, HalCIdentity* out) {
    Axis* a = find_axis(c, id);
    if (!a || !out) return HAL_ERROR_ARGUMENT;
    if (!c->running) return HAL_ERROR_NOT_RUNNING;
    *out = a->identity;
    return HAL_OK;
}

/* 周期第一拍，也是唯一的阻塞点。phase 必须为 0（上一拍已 commit）。 */
int32_t hal_rt_wait_cycle(HalContext* c) {
    int rc = gate(c); if (rc) return rc;
    if (c->phase != 0) return HAL_ERROR_STATE;
    rc = Master_WaitCycle();
    if (atomic_load(&c->stop_requested) || rc == MASTER_STOP_REQUESTED) return HAL_ERROR_STOPPED;
    if (rc) return bus_error(c);
    c->phase = 1;
    return HAL_OK;
}

/* 采一拍：读五个 Tx 角色 → 把原始位置展开成累计计数 → 填轴与主轴快照。
 * 线性反馈按 32 位回绕累计，模反馈按每转脉冲数取最短差值，两者都要求相邻采样
 * 位移小于半个计数周期——否则丢掉的整转从单个模计数里认不出来。任一读失败即总线故障。 */
static int sample_axis(Axis* a) {
    uint32_t raw, sw, mode, error, velocity; // 实际位置 状态字 当前运行模式 错误码 实际速度
    if (Master_ServoRead(a->slot, DEV_DICT_ROLE_ACTUAL_POS, &raw) ||
        Master_ServoRead(a->slot, DEV_DICT_ROLE_STATUS_WORD, &sw) ||
        Master_ServoRead(a->slot, DEV_DICT_ROLE_MODE_DISPLAY, &mode) ||
        Master_ServoRead(a->slot, DEV_DICT_ROLE_ERROR_CODE, &error) ||
        Master_ServoRead(a->slot, DEV_DICT_ROLE_ACTUAL_SPEED, &velocity)) return HAL_ERROR_BUS;
    if (!a->sampled) a->counts = signed32(raw);
    else {
        double delta = signed32(raw - a->raw_pos);
        if (a->cfg.feedback_wrap == HAL_WRAP_MODULAR) {
            const double period = a->cfg.feedback_pulses_per_rev;
            delta = remainder(delta, period);
        }
        a->counts += delta;
    }
    a->raw_pos = raw;
    a->status.actual_pos = a->counts * feedback_scale(a) + a->feedback_offset;
    a->status.raw_status = (uint16_t)sw;
    a->status.error_code = (uint16_t)error;
    a->status.enabled = (sw & DS402_SW_MASK_OP_ENABLED) == DS402_SW_VAL_OP_ENABLED;
    a->status.position_valid = 1;
    if (!a->sampled) a->status.command_pos = a->status.actual_pos;
    a->sampled = 1;
    a->mode = (int)mode;
    a->speed.mode = mode == DS402_MODE_CSP ? HAL_SPINDLE_CSP : mode == DS402_MODE_CSV ? HAL_SPINDLE_CSV : -1;
    a->speed.enabled = a->status.enabled;
    a->speed.raw_status = (uint16_t)sw;
    a->speed.position_deg = a->status.actual_pos;
    a->speed.actual_speed = signed32(velocity) * feedback_scale(a) / 6.0;
    if (!isfinite(a->status.actual_pos) || !isfinite(a->speed.actual_speed)) return HAL_ERROR_BUS;
    a->speed.at_speed = a->spindle && a->status.enabled && mode == DS402_MODE_CSV &&
        fabs(a->speed.actual_speed - a->speed.command_speed) <= a->spindle_cfg.speed_window;
    return HAL_OK;
}

/* 周期第二拍：采所有轴、各推一拍 DS402、读 IO 输入。此后 read_* 才有效。
 * 使能/取消运动/急停留下的 zero_speed 在这里发一次速度目标 0，
 * 防止上一轮的转速在本拍被重新写下去。 */
int32_t hal_rt_begin_cycle(HalContext* c) {
    int rc = gate(c); if (rc) return rc;
    if (c->phase != 1) return HAL_ERROR_STATE;
    for (int i = 0; i < c->axis_count; ++i) {
        Axis* a = &c->axes[i];
        if (sample_axis(a)) return bus_error(c);
        if (a->zero_speed) {
            if (Master_ServoWrite(a->slot, DEV_DICT_ROLE_TARGET_SPEED, 0)) return bus_error(c);
            a->zero_speed = 0;
        }
        const Ds402Step step = Ds402_NextStepReq(a->status.raw_status, (uint16_t)a->mode,
            a->request, (uint16_t)a->desired_mode, DS402_MODESW_DISABLE_FIRST);
        uint32_t preset = 0;
        if (step.preset_target && to_counts((a->status.actual_pos - a->command_offset) / command_scale(a), &preset))
            return bus_error(c);
        if (Master_ServoStep(a->slot, a->request, (uint16_t)a->desired_mode, NULL)) return bus_error(c);
        /* 驱动原语只认识原始脉冲；命令/反馈当量不同时由 HAL 修正预置。 */
        if (step.preset_target) {
            if (Master_ServoWrite(a->slot, DEV_DICT_ROLE_TARGET_POS, preset)) return bus_error(c);
            a->status.command_pos = a->status.actual_pos;
        }
        if (a->request == DS402_REQ_ENABLE) a->stop_override = 0;
        a->motion_ready = a->request == DS402_REQ_ENABLE && step.kind == DS402_STEP_DONE;
    }
    for (int i = 0; i < c->io_count; ++i) {
        Io* io = &c->ios[i];
        memset(io->inputs, 0, sizeof(io->inputs));
        uint32_t bit = 0;
        for (int j = 0; j < io->in_count; ++j) {
            uint32_t value; int bits;
            if (Master_IoReadEntry(io->slot, j, &value, &bits) || bits != io->in_bits[j]) return bus_error(c);
            for (int b = 0; b < bits; ++b, ++bit)
                io->inputs[bit / 8] |= (uint8_t)(((value >> b) & 1u) << (bit % 8));
        }
    }
    c->sampled = 1;
    c->phase = 2;
    return HAL_OK;
}

/* 周期第三拍：下发攒下的轴指令与 IO 输出，然后提交主站。
 * 急停若在 begin 之后才到达，这一拍要覆盖 begin 时写下的旧控制字和速度目标——
 * 否则本拍会把旧的使能动作照发出去。 */
int32_t hal_rt_commit_cycle(HalContext* c) {
    int rc = gate(c); if (rc) return rc;
    if (c->phase != 2) return HAL_ERROR_STATE;
    for (int i = 0; i < c->axis_count; ++i) {
        Axis* a = &c->axes[i];
        /* 急停可在 begin 后到达；提交前覆盖本拍旧控制字和速度目标。 */
        if (a->zero_speed || a->stop_override) {
            if (Master_ServoWrite(a->slot, DEV_DICT_ROLE_TARGET_SPEED, 0)) return bus_error(c);
            a->zero_speed = 0;
        }
        if (a->stop_override && Master_ServoWrite(a->slot, DEV_DICT_ROLE_CONTROL_WORD,
            a->stop_override == HAL_ESTOP_DISABLE_VOLTAGE ? DS402_CW_DISABLE_VOLTAGE : DS402_CW_SWITCH_ON))
            return bus_error(c);
        if (a->pos_dirty && Master_ServoWrite(a->slot, DEV_DICT_ROLE_TARGET_POS, a->pos_command)) return bus_error(c);
        if (a->speed_dirty && Master_ServoWrite(a->slot, DEV_DICT_ROLE_TARGET_SPEED, a->speed_command)) return bus_error(c);
        a->pos_dirty = a->speed_dirty = 0;
    }
    for (int i = 0; i < c->io_count; ++i) {
        Io* io = &c->ios[i];
        if (io->dirty) for (int j = 0; j < io->out_count; ++j)
            if (Master_IoWriteEntry(io->slot, j, io->outputs[j])) return bus_error(c);
        io->dirty = 0;
    }
    if (atomic_load(&c->stop_requested)) return HAL_ERROR_STOPPED;
    rc = Master_CommitCycle();
    if (atomic_load(&c->stop_requested) || rc == MASTER_STOP_REQUESTED) return HAL_ERROR_STOPPED;
    if (rc) return bus_error(c);
    c->phase = 0;
    return HAL_OK;
}

/* 取消尚未提交的运动：丢脏标志、清速度目标、撤销 motion_ready。
 * 只动 HAL 侧的暂存，驱动器下一拍才会因为速度目标变 0 而减速。 */
static void cancel_motion(Axis* a) {
    a->pos_dirty = a->speed_dirty = 0;
    a->motion_ready = 0;
    a->zero_speed = 1;
    a->speed.command_speed = 0;
    a->speed.at_speed = 0;
}

/* on/off 二值，其它值拒绝。请求真的变了才取消运动，重复调用不会反复丢指令。
 * off 停在 SwitchedOn（可收指令、不带载），不是断电——要断电走 estop。 */
int32_t hal_rt_axis_enable(HalContext* c, HalAxisId id, int32_t on) {
    int rc = gate(c); if (rc) return rc;
    Axis* a = find_axis(c, id);
    if (!a || (on != 0 && on != 1)) return HAL_ERROR_ARGUMENT;
    const Ds402Request req = on ? DS402_REQ_ENABLE : DS402_REQ_DISABLE;
    if (a->request != req) cancel_motion(a);
    a->request = req;
    if (!on) a->stop_override = HAL_ESTOP_DISABLE_OPERATION;
    return HAL_OK;
}

/* 有意不走 gate()：总线故障闭锁时仍接受停止意图并记下来，但闭锁下不再发 PDO，
 * 所以「受理成功」不等于失联硬件真的执行了。 */
int32_t hal_rt_axis_estop(HalContext* c, HalAxisId id) {
    /* 总线故障闭锁时仍接受停止意图，但无法保证失联硬件执行。 */
    if (!c) return HAL_ERROR_ARGUMENT;
    if (!c->running) return HAL_ERROR_NOT_RUNNING;
    Axis* a = find_axis(c, id);
    if (!a) return HAL_ERROR_ARGUMENT;
    cancel_motion(a);
    a->request = a->cfg.estop_action == HAL_ESTOP_DISABLE_OPERATION ? DS402_REQ_DISABLE : DS402_REQ_DROP_VOLTAGE;
    a->stop_override = a->cfg.estop_action;
    a->desired_mode = a->mode == DS402_MODE_CSV ? DS402_MODE_CSV : DS402_MODE_CSP;
    return HAL_OK;
}

/* 只在 CSP + 已使能 + 状态机到位时受理，否则返回 NOT_RUNNING（沿用公共码，不为
 * 「还没准备好」新造一个）。写的是命令侧坐标，按 command_offset 和命令当量换算。 */
int32_t hal_rt_axis_write_pos(HalContext* c, HalAxisId id, double pos) {
    int rc = gate(c); if (rc) return rc;
    Axis* a = find_axis(c, id);
    if (!a || !isfinite(pos)) return HAL_ERROR_ARGUMENT;
    if (c->phase != 2 || !a->motion_ready || !a->status.enabled || a->request != DS402_REQ_ENABLE ||
        a->desired_mode != DS402_MODE_CSP || a->mode != DS402_MODE_CSP) return HAL_ERROR_NOT_RUNNING;
    rc = to_counts((pos - a->command_offset) / command_scale(a), &a->pos_command);
    if (rc) return rc;
    a->pos_dirty = 1;
    a->status.command_pos = pos;
    return HAL_OK;
}

/* 返回最近一次 begin 的快照，本身不触发任何总线访问。 */
int32_t hal_rt_axis_read_status(HalContext* c, HalAxisId id, HalCAxisStatus* out) {
    int rc = gate(c); if (rc) return rc;
    Axis* a = find_axis(c, id);
    if (!a || !out) return HAL_ERROR_ARGUMENT;
    if (!c->sampled) return HAL_ERROR_NOT_RUNNING;
    *out = a->status;
    return HAL_OK;
}

/* read_status 的取坐标快捷方式。 */
int32_t hal_rt_axis_read_pos(HalContext* c, HalAxisId id, double* out) {
    if (!out) return HAL_ERROR_ARGUMENT;
    HalCAxisStatus status;
    const int rc = hal_rt_axis_read_status(c, id, &status);
    if (!rc) *out = status.actual_pos;
    return rc;
}

/* 平移坐标基准，不写运动目标：命令侧与反馈侧偏置一起移，已暂存的原始命令保持不变，
 * 所以驱动器不会因为「设坐标」而动。三个加法都查有限性——偏置溢出不报错会静默丢坐标。 */
int32_t hal_rt_axis_set_pos(HalContext* c, HalAxisId id, double pos) {
    int rc = gate(c); if (rc) return rc;
    Axis* a = find_axis(c, id);
    if (!a || !isfinite(pos)) return HAL_ERROR_ARGUMENT;
    if (!c->sampled) return HAL_ERROR_NOT_RUNNING;
    const double delta = pos - a->status.actual_pos;
    if (!isfinite(delta) || !isfinite(a->feedback_offset + delta) ||
        !isfinite(a->command_offset + delta) || !isfinite(a->status.command_pos + delta)) return HAL_ERROR_ARGUMENT;
    a->feedback_offset += delta;
    a->command_offset += delta;
    a->status.actual_pos = pos;
    a->status.command_pos += delta;
    a->speed.position_deg = pos;
    return HAL_OK;
}

/* 主轴专有入口的公共前检：先过 gate，再确认这个 id 确实是主轴。 */
static int spindle_gate(HalContext* c, HalAxisId id) {
    const int rc = gate(c); if (rc) return rc;
    const Axis* a = find_axis(c, id);
    return a && a->spindle ? HAL_OK : HAL_ERROR_ARGUMENT;
}

/* 转发到轴的使能路径，主轴侧没有额外语义。 */
int32_t hal_rt_spindle_enable(HalContext* c, HalAxisId id, int32_t on) {
    const int rc = spindle_gate(c, id);
    return rc ? rc : hal_rt_axis_enable(c, id, on);
}

/* 也只转发；断电还是去使能由该主轴的 estop_action 配置决定。 */
int32_t hal_rt_spindle_estop(HalContext* c, HalAxisId id) {
    const Axis* a = find_axis(c, id);
    if (!a || !a->spindle) return HAL_ERROR_ARGUMENT;
    return hal_rt_axis_estop(c, id);
}

/* 只改期望模式，不动使能——急停中的轴不会因为切模式被重新使能。
 * 期望模式真的变了才取消运动，重复调用不会反复丢指令。 */
int32_t hal_rt_spindle_request_mode(HalContext* c, HalAxisId id, HalSpindleMode mode) {
    const int rc = spindle_gate(c, id); if (rc) return rc;
    if (mode != HAL_SPINDLE_CSV && mode != HAL_SPINDLE_CSP) return HAL_ERROR_ARGUMENT;
    Axis* a = find_axis(c, id);
    const int desired = mode == HAL_SPINDLE_CSP ? DS402_MODE_CSP : DS402_MODE_CSV;
    if (a->desired_mode != desired) cancel_motion(a);
    a->desired_mode = desired;
    return HAL_OK;
}

/* 只在 CSV + 已使能 + 状态机到位时受理。rpm 与 dir 分开传：正负号是转向语义
 * （M03/M04），所以 rpm 本身不许为负。换算 ×6 把 rpm 折成 deg/s（360°/60s）。
 * at_speed 在这里先按指令值算一遍，下一拍采样再按实测刷新。 */
int32_t hal_rt_spindle_write_speed(HalContext* c, HalAxisId id, double rpm, int32_t dir) {
    int rc = spindle_gate(c, id); if (rc) return rc;
    Axis* a = find_axis(c, id);
    if (!isfinite(rpm) || rpm < 0 || rpm > a->spindle_cfg.max_speed || dir < -1 || dir > 1) return HAL_ERROR_ARGUMENT;
    if (c->phase != 2 || !a->motion_ready || !a->status.enabled || a->request != DS402_REQ_ENABLE ||
        a->desired_mode != DS402_MODE_CSV || a->mode != DS402_MODE_CSV) return HAL_ERROR_NOT_RUNNING;
    rc = to_counts(rpm * dir * 6.0 / command_scale(a), &a->speed_command);
    if (rc) return rc;
    a->speed_dirty = 1;
    a->speed.command_speed = rpm * dir;
    a->speed.at_speed = fabs(a->speed.actual_speed - a->speed.command_speed) <= a->spindle_cfg.speed_window;
    return HAL_OK;
}

/* 返回最近一次 begin 的转速快照。 */
int32_t hal_rt_spindle_read_status(HalContext* c, HalAxisId id, HalCSpindleStatus* out) {
    const int rc = spindle_gate(c, id); if (rc) return rc;
    if (!out) return HAL_ERROR_ARGUMENT;
    if (!c->sampled) return HAL_ERROR_NOT_RUNNING;
    *out = find_axis(c, id)->speed;
    return HAL_OK;
}

/* read_status 的取实际转速快捷方式。 */
int32_t hal_rt_spindle_read_speed(HalContext* c, HalAxisId id, double* out) {
    if (!out) return HAL_ERROR_ARGUMENT;
    HalCSpindleStatus status;
    const int rc = hal_rt_spindle_read_status(c, id, &status);
    if (!rc) *out = status.actual_speed;
    return rc;
}

/* 转发到轴的位置写入——刚性攻丝就是走这条路下发角度，
 * 所以主轴配置里 logical_axis 必须 >= 0，否则这里找不到轴。 */
int32_t hal_rt_spindle_write_pos(HalContext* c, HalAxisId id, double deg) {
    const int rc = spindle_gate(c, id);
    return rc ? rc : hal_rt_axis_write_pos(c, id, deg);
}

/* 把本拍输入映像拷到调用方的 X 区：段内覆盖、段外补 0，调用方不必先清。
 * len < x_size 直接报错——映像不够长说明调用方和 HAL 对地址域的理解不一致。 */
int32_t hal_rt_io_snapshot_inputs(HalContext* c, uint8_t* image, uint32_t len) {
    const int rc = gate(c); if (rc) return rc;
    if (len < c->x_size || (len && !image)) return HAL_ERROR_ARGUMENT;
    if (!c->sampled) return HAL_ERROR_NOT_RUNNING;
    if (len) memset(image, 0, len);
    for (int i = 0; i < c->io_count; ++i)
        if (c->ios[i].in_bytes) memcpy(image + c->ios[i].x, c->ios[i].inputs, c->ios[i].in_bytes);
    return HAL_OK;
}

/* 把调用方的 Y 区按位拆进各输出 Entry 并置 dirty，真正下发在 commit。
 * HAL 不做读改写：总线按整 Entry 下发，要改其中几位由调用方自己保证。 */
int32_t hal_rt_io_flush_outputs(HalContext* c, const uint8_t* image, uint32_t len) {
    const int rc = gate(c); if (rc) return rc;
    if (len < c->y_size || (len && !image)) return HAL_ERROR_ARGUMENT;
    if (c->phase != 2) return HAL_ERROR_STATE;
    for (int i = 0; i < c->io_count; ++i) {
        Io* io = &c->ios[i];
        uint32_t bit = 0;
        for (int j = 0; j < io->out_count; ++j) {
            uint32_t value = 0;
            for (int b = 0; b < io->out_bits[j]; ++b, ++bit)
                value |= ((uint32_t)((image[io->y + bit / 8] >> (bit % 8)) & 1u)) << b;
            io->outputs[j] = value;
        }
        io->dirty = 1;
    }
    return HAL_OK;
}
