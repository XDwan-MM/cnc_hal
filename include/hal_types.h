#pragma once

/* ============================================================================
 * CNC HAL 公共 C ABI —— 类型与版本闸门
 *
 * 本目录（include/）只放公共 C ABI，内部 C++ 全在 src/ 下。
 * 硬约束：传递依赖只能 <stdint.h>，不得出现 C++ 类型、STL、引用、模板、异常、
 * 厂商 SDK 类型；字符串一律定长数组，整数一律定宽。
 * 版本闸门：abi_major（+ abi_minor）与 struct_size 必须精确相等，不能用「至少」；
 * MAJOR = 契约/语义变更（函数增删、返回码含义变），MINOR = 结构布局变更（字段增删改序）。
 * 字段增删改序都要动这两个宏，并同步改 RT 侧 port_work/src/hal/rt_hal_config.c 的
 * 编译期 #error 闸门。
 * ========================================================================== */

#include <stdint.h>

#define HAL_C_ABI_MAJOR  7u     /* 契约/语义版本：函数增删、返回码含义变 */
#define HAL_C_ABI_MINOR  3u     /* 结构布局版本：字段增删改序 */

/* ---- HAL 发行版本 ---- */
#define HAL_VERSION_MAJOR  0u       /* 主版本 */
#define HAL_VERSION_MINOR  1u       /* 次版本 */
#define HAL_VERSION_PATCH  0u       /* 修订号 */
#define HAL_VERSION_STRING "0.1"    /* 字符串形式，日志与上报用；须与上面三个一致 */

#define HAL_C_MAX_DEV     30u   /* 轴 / IO / 面板 各自的容量上限 */
#define HAL_C_MAX_SPINDLE 4u    /* 主轴容量上限 */
#define HAL_C_NAME_LEN    64u   /* 定长名称缓冲长度；字符串一律定长数组（见文件头） */
#define HAL_INVALID_ID    UINT16_MAX  /* 无效 HalAxisId 的哨兵；0 是合法轴号 */

/* ---- 不透明句柄 ---- */
typedef struct HalContext HalContext;

/* ---- 设备标识 ----*/
typedef uint16_t HalAxisId;      /* 就是逻辑轴号本身，不是装配下标 */

/* 按轴实例查询的功能码 */
#define HAL_FUNC_POSITION       1u  /* 位置：0x607A 目标位置 + 0x6064 实际位置 */
#define HAL_FUNC_SPEED          2u  /* 速度：0x60FF 目标转速 + 0x606C 实际转速（两个都要有） */
#define HAL_FUNC_ERROR_CODE     3u  /* 故障码 0x603F */
#define HAL_FUNC_ALARM_CONTROL  4u  /* 厂商专有报警控制 0x6FFF；当前无设备定义，恒不可用 */
#define HAL_CAP_READY           1u  /* 该功能可用 */
#define HAL_CAP_UNSUPPORTED     2u  /* 设备不支持，或没绑这个 PDO；reason 里写明缺什么 */
#define HAL_CAP_NOT_CONFIGURED  3u  /* HAL 侧尚未配置该功能（非设备问题） */

typedef struct {
    uint32_t function;
    uint32_t state;
    int32_t  slave_pos;
    int32_t  axis_index;
    char     reason[96];
} HalCCapability;

/* ---- 参数取值 ---- */
#define HAL_WORK_POSITION     1   /* 位置增量模式（CSP） */
#define HAL_WORK_VELOCITY     3   /* 速度模式（CSV） */
#define HAL_ENC_INCREMENTAL_Z 1   /* 增量式、带 Z 相；回零策略不归 HAL */
#define HAL_ENC_ABSOLUTE      3   /* 绝对式 */

/* 驱动器速度对象（0x606C 读 / 0x60FF 写）的单位。HAL 同时提供原始速度与换算后的 rpm：
 *   HAL_SPEED_RPM          —— 驱动器本身就是 rpm（蓝海华腾 C2-00 = 0）；
 *   HAL_SPEED_PULSES_PER_S —— 驱动器是脉冲/秒（蓝海华腾 C2-00 出厂默认），
 *                             HAL 用 command_pulses_per_rev 双向换算。
 * 不填就是 0（rpm）。 */
#define HAL_SPEED_RPM           0
#define HAL_SPEED_PULSES_PER_S  1
#define HAL_WRAP_LINEAR       0   /* 线性计数 */
#define HAL_WRAP_MODULAR      1   /* 模循环（主轴：模每转脉冲数） */

/* 急停时对该轴做什么。0 无效，必须由机床配置显式选择。 */
#define HAL_ESTOP_DISABLE_OPERATION 1   /* 只去使能：停在 SwitchedOn（写 0x07，可收指令、不带载） */
#define HAL_ESTOP_DISABLE_VOLTAGE   2   /* 撤销电压（写 0x00，断电），最彻底 */
/* 主轴期望模式：HAL 自己的取值，与驱动器 0x6060 不同（映射见括号） */
#define HAL_SPINDLE_CSV 0               /* 速度模式，映射到 0x6060 = 9 */
#define HAL_SPINDLE_CSP 1               /* 位置模式，映射到 0x6060 = 8 */
typedef int32_t HalSpindleMode;

/* ============================================================================
 * 配置
 *
 * 扁平结构，启动期传一次。没有「通道」这一维：通道是加工组织，不是硬件属性，归上层。
 * ========================================================================== */

/* 通用轴属性 —— 进给轴和主轴共用；转速侧的量是主轴专有（见 HalCSpindleCfg） */
typedef struct {
    int32_t slave_pos;                  /* 绑定的从站位置（总线拓扑，运行期不变） */
    int32_t axis_index;                 /* 从站内物理轴序号，从 0 起 */
    int32_t estop_action;               /* HAL_ESTOP_*，不得省略 */ 
    int32_t logical_axis;               /* 逻辑轴号；-1 = 不占轴槽 */
    int32_t work_mode;                  /* HAL_WORK_* */
    int32_t encoder_type;               /* HAL_ENC_* */
    int32_t feedback_pulses_per_rev;    /* 每转反馈脉冲数 */
    int32_t feedback_wrap;              /* HAL_WRAP_* */
    double  command_units_per_count;    /* 命令当量（用户单位/脉冲），> 0；1 mm = 10000 命令计数则填 0.0001 */
    double  feedback_units_per_count;   /* 反馈当量，> 0。可与命令不同 */
    int32_t command_invert;             /* 非零 = 位置及主轴 CSV 转速命令方向取反 */
    int32_t feedback_invert;            /* 非零 = 位置及速度反馈方向取反；与命令分开 */
    double  enc_off;                    /* 已保存的坐标偏置（原始位置 - enc_off） */

    /* ---- 主轴定向（厂商专有，可选）----
     * 定向不是标准 DS402 动作：驱动器收到厂商规定的控制字后自行减速并对准，完成标志在
     * 状态字的某一位上。值与位都是驱动器专有的，由设备字典配。
     * 两个都为 0 = 该轴不支持定向，此时定向接口返回 HAL_ERROR_UNSUPPORTED（不静默当成功）。
     * 进给轴保持 0。 */
    uint16_t vendor_orient_control_word; /* 如 0x040F */
    uint16_t vendor_orient_done_mask;    /* 如 0x0400（状态字 bit10） */
    int32_t  vendor_orient_delay_ms;     /* 发出控制字后至少等这么久（ms）才判完成；<= 0 = 不等。
                                          * 延时内仍记录低电平；每轮必须先低后高，延时不能代替边沿。
                                          * 完成位在别的模式下可能是别的含义：蓝海华腾 VTS 6041h.bit10 在 CSV 模式下
                                          * 表示「速度到达」，发 0x40F 后延时 10ms 才表示「定位完成」。 */

    /* ---- 速度对象单位（驱动器专有）----
     * 取值见 HAL_SPEED_*；默认 0（rpm）。填 pulse/s 时必须同时给
     * command_pulses_per_rev（不配会被拒）；进给轴返回 mm/min 也需要它。 */
    int32_t  speed_unit;                 /* 0 = rpm；1 = pulse/s */
    int32_t  command_pulses_per_rev;     /* 每转指令脉冲数；pulse/s↔rpm 及进给轴 mm/min 换算使用 */
} HalCAxisCfg;

/* 主轴 —— 通用轴属性之上加转速上限（定向与运行期 CSV↔CSP 切换是主轴专有）。
 * @note axis.logical_axis 必须 >= 0 —— 主轴角度接口也用 HalAxisId 寻址。 */
typedef struct {
    HalCAxisCfg axis;                   /* 通用属性全在这，不重复 */
    double      max_speed;              /* rpm */
} HalCSpindleCfg;

typedef struct {
    int32_t slave_pos;
    int32_t x_start;                    /* 输入段起始**字节**地址（PLC 域 X），>= 0 */
    int32_t y_start;                    /* 输出段起始**字节**地址（PLC 域 Y），>= 0 */
} HalCIoCfg;

typedef struct {
    int32_t slave_pos;
    int32_t panel_type;                 /* HMI 按型号解释语义布局；HAL 不解读 */
    int32_t x_start;                    /* 与普通 IO 共用 PLC X 字节地址域 */
    int32_t y_start;                    /* 与普通 IO 共用 PLC Y 字节地址域 */
} HalCPanelCfg;

typedef struct {
    /* ---- 版本闸门（必须在最前，且顺序不变）---- */
    uint16_t abi_major;                 /* 必须 == HAL_C_ABI_MAJOR */
    uint16_t abi_minor;                 /* 必须 == HAL_C_ABI_MINOR */
    uint16_t struct_size;               /* 必须 == sizeof(HalCConfig)，精确相等 */
    uint16_t reserved;                  /* 必须为 0。显式填充，不依赖编译器 */

    uint64_t topology_fingerprint;      /* 当前必须为 0（指纹算法未定） */

    uint32_t cycle_us;
    uint32_t start_timeout_ms;          /* 整个启动过程共用的总预算 */
    uint32_t cycle_timeout_ms;
    uint32_t fault_reset_timeout_ms;    /* 故障复位从发起算起的等待上限（ms），必须 > 0。
                                           超过即判 HAL_RESET_TIMEOUT，并把该轴留在未使能态。 */
    int32_t  dc_enable;                 /* 0 或 1；运行期间配置不可变 */

    /* ---- 设备数组，前 count 个有效 ---- */
    int32_t        axis_count;
    HalCAxisCfg    axes[HAL_C_MAX_DEV];
    int32_t        spindle_count;
    HalCSpindleCfg spindles[HAL_C_MAX_SPINDLE];
    int32_t        io_count;
    HalCIoCfg      ios[HAL_C_MAX_DEV];
    int32_t        panel_count;
    HalCPanelCfg   panels[HAL_C_MAX_DEV];
} HalCConfig;

/* 状态（出参；非 RT 读取须由调用方与周期线程同步） */

typedef struct {
    int32_t  enabled;                   /* 非零 = DS402 已到 OperationEnabled */
    int32_t  position_valid;            /* 非零 = 实际位置与目标位置 PDO 可用；速度型主轴可能为 0 */
    double   actual_pos;                /* 用户单位 */
    double   command_pos;               /* 用户单位 */
    double   actual_speed;              /* 实际速度原始值：单位即驱动器速度对象（0x606C）的单位，HAL 不换算。
                                           speed_valid 为 0 时恒为 0，表示不可得，而不是静止。 */
    double   actual_speed_rpm;          /* 同一读数的 rpm 口径，按该轴 speed_unit 换算；显示与比较一律用这个。
                                           speed_valid 为 0 时无意义。 */
    int32_t  speed_valid;               /* 非零 = 该轴映射了速度 PDO（0x606C 与 0x60FF 都是 32 位），actual_speed 才有意义；
                                           主轴上恒为 1（HAL 在 bind 阶段就拒绝没有速度 PDO 的主轴）。 */
    uint16_t raw_status;                /* 原始状态字 0x6041，诊断用 */
    uint16_t error_code;                /* 驱动器故障码 0x603F */
    int32_t  error_code_valid;          /* 0x603F 未映射时为 0，error_code 此时无效 */
    double   actual_speed_mm_min;       /* 进给轴实际线速度，mm/min；主轴及无效时为 0 */
    int32_t  linear_speed_valid;        /* 非零 = 进给轴有速度 PDO 且配置了每转指令脉冲数 */
} HalCAxisStatus;

typedef struct {
    int32_t  enabled;
    int32_t  mode;                      /* HAL_SPINDLE_CSV / _CSP，来自 0x6061 */
    double   command_speed;             /* rpm，带符号：正=正转 M03，负=反转 M04 */
    double   actual_speed;              /* rpm（HAL 已按 speed_unit 换算）——「停稳」判据（转速 < 5）用它，必须是 rpm */
    double   position_deg;              /* 累计转角（度），不取模；无位置能力时为 0 */
    uint16_t raw_status;
} HalCSpindleStatus;

typedef struct {
    int32_t  type;                      /* 设备类型（字典里的 type 映射而来） */
    uint32_t vendor_id;
    uint32_t product_code;
    uint32_t revision;
    uint32_t serial;
    char     name[HAL_C_NAME_LEN];      /* 设备字典查出；未接入前为空串 */
    int32_t  family_index;              /* 同 vendor+product 的第几台（0 起，按总线顺序） */
    uint64_t device_id;                 /* serial 非零取 serial，否则 FNV(vendor,product,revision) */
} HalCIdentity;

typedef struct {
    int32_t slave_pos;                  /* 实际扫描顺序；包含未配置和未识别的从站 */
    int32_t axis_count;                 /* 已装配的物理轴数 */
    HalCIdentity identity;
} HalCSlaveInfo;

/* 总线健康快照（出参）。只记录不判断：原始计数与标志，什么算异常、要不要报警由上层决定。
 * 全部为累计/瞬时原始值，未做单位换算；还没收过帧时全零。 */
typedef struct {
    int32_t  pdo_warn;                  /* 非零 = 最近一帧有 PDO 警告 */
    int32_t  pdo_warn_code;             /* PDO 警告号 */
    uint32_t pdo_warn_para;             /* 警告参数 */
    int32_t  dc_warn;                   /* 非零 = 有分布式时钟警告 */
    int32_t  dc_warn_code;
    uint32_t dc_warn_para;
    uint32_t crc_error_count;           /* CRC 错误累计次数 */
    uint32_t frame_timeout_count;       /* 帧超时累计次数 */
    uint32_t expect_wkc_tx;             /* 期望工作计数器（Tx 侧） */
    uint32_t expect_wkc_rx;             /* 期望工作计数器（Rx 侧） */
} HalCBusHealth;
