#pragma once

/* ============================================================================
 * CNC HAL 公共 C ABI —— 类型与版本闸门
 *
 * 本目录（include/）**只放公共 C ABI**，内部 C++ 全在 src/ 下。
 *
 * 硬约束（本目录下所有头都必须遵守）：
 *   - 传递依赖只能是 <stdint.h>：不得出现 C++ 类型、STL、引用、模板、异常、厂商 SDK 类型
 *   - 字符串一律定长数组，不用 char*（libstdc++ 的布局跨版本不稳定）
 *   - 整数一律定宽，不用 int/long
 *   - 字段增删改序都是布局变更，必须动 HAL_C_ABI_MAJOR/MINOR
 *
 * 版本策略：
 *   MAJOR —— 契约/语义变更（函数增删、返回码含义变）
 *   MINOR —— 结构布局变更（字段增删改序）
 * 真正的闸门是**两个都要对**：abi_major（+ abi_minor）与 struct_size **精确相等**。
 * struct_size 用精确相等而不是"至少"——它是唯一能挡住字段插入的检查。
 * ========================================================================== */

#include <stdint.h>

/* 5.0：新增 hal_io_image_size()（函数增按本文件的策略是 MAJOR）。
 * 4.0：新增故障复位与总线健康查询两个公共函数，并新增
 *      HalCConfig.fault_reset_timeout_ms 字段。 */
#define HAL_C_ABI_MAJOR  5u
#define HAL_C_ABI_MINOR  0u

#define HAL_C_MAX_DEV     30u   /* 轴 / IO / 面板 各自的容量上限 */
#define HAL_C_MAX_SPINDLE 4u    /* 主轴容量上限（一台机床几根就够） */
#define HAL_C_NAME_LEN    64u
#define HAL_INVALID_ID    UINT16_MAX

/* ---- 不透明句柄 ---- */
typedef struct HalContext HalContext;

/* ---- 设备标识 ----
 * 只有轴号一种。**主轴也用这个寻址**——它就是一根轴，只是多了转速语义；
 * 再给一个 HalSpindleId 就是同一个东西两个名字。 */
typedef uint16_t HalAxisId;      /* **就是逻辑轴号本身**，不是装配下标 */

/* 按轴实例查询的功能。0x6FFF 报警控制尚无设备定义，因此不会宣告可用。 */
#define HAL_FUNC_POSITION       1u
#define HAL_FUNC_SPEED          2u
#define HAL_FUNC_ERROR_CODE     3u
#define HAL_FUNC_ALARM_CONTROL  4u
#define HAL_CAP_READY           1u
#define HAL_CAP_UNSUPPORTED     2u
#define HAL_CAP_NOT_CONFIGURED  3u

typedef struct {
    uint32_t function;
    uint32_t state;
    int32_t  slave_pos;
    int32_t  axis_index;
    char     reason[96];
} HalCCapability;

/* ---- 参数取值 ----
 * 用宏而不是 enum：C 的 enum 底层宽度由实现定义，不适合跨 ABI。 */
#define HAL_WORK_POSITION     1   /* 位置增量模式（CSP） */
#define HAL_WORK_VELOCITY     3   /* 速度模式（CSV） */
#define HAL_ENC_INCREMENTAL_Z 1   /* 增量式、带 Z 相；是否回零由机床策略决定 */
#define HAL_ENC_ABSOLUTE      3   /* 绝对式 */
#define HAL_WRAP_LINEAR       0   /* 线性计数 */
#define HAL_WRAP_MODULAR      1   /* 模循环（主轴：模每转脉冲数） */

/* 0 无效，必须由机床配置显式选择。 */
#define HAL_ESTOP_DISABLE_OPERATION 1
#define HAL_ESTOP_DISABLE_VOLTAGE   2
#define HAL_SPINDLE_CSV 0
#define HAL_SPINDLE_CSP 1
typedef int32_t HalSpindleMode;

/* ============================================================================
 * 配置
 *
 * 保留**扁平结构**：启动期传一次，几 KB 的代价无关紧要。按设备种类的两级闸门
 * 留作将来演进（触发条件：出现第三个消费方，或设备类型的变化开始逼着无关方重编）。
 *
 * **没有"通道"这一维**：通道是加工组织（一条独立的加工执行流——自己的程序、
 * 轴组、主轴、坐标系），不是硬件属性。删掉实时端业务层，「轴 3 属于通道 2」
 * 就没意义了 → 归上层。一根轴自己不知道自己在哪个通道里，那是机床配置说的。
 * ========================================================================== */

/* 通用轴属性 —— 进给轴和主轴共用。凡是"带编码器的驱动轴"都有的东西都在这。
 * 转速侧的量不在：那是主轴专有（见 HalCSpindleCfg）。 */
typedef struct {
    int32_t slave_pos;                  /* 绑定的从站位置（总线拓扑，运行期不变） */
    int32_t axis_index;                 /* 从站内物理轴序号，从 0 起 */
    int32_t estop_action;               /* HAL_ESTOP_*，不得省略 1-去使能 2-撤销电压*/ 
    int32_t logical_axis;               /* 逻辑轴号；-1 = 不占轴槽 */
    int32_t work_mode;                  /* HAL_WORK_* */
    int32_t encoder_type;               /* HAL_ENC_* */
    int32_t feedback_pulses_per_rev;    /* 每转反馈脉冲数 */
    int32_t feedback_wrap;              /* HAL_WRAP_* */
    double  command_units_per_count;    /* 命令当量（用户单位/脉冲），> 0 如确定 1 mm 对应 10000 个命令计数，就传入 command_units_per_count = 0.0001*/
    double  feedback_units_per_count;   /* 反馈当量，> 0。可与命令不同 */
    int32_t command_invert;             /* 非零 = 命令方向取反 */
    int32_t feedback_invert;            /* 非零 = 反馈方向取反。**与命令分开**——
                                           外接编码器/光栅尺可能只需要翻反馈 */
    double  enc_off;                    /* 已保存的坐标偏置；旧 RT 语义：原始位置 - enc_off */
} HalCAxisCfg;

/* 主轴 —— 在通用轴属性之上加转速侧的几个量。
 * 单独一张表，不塞进 axes[]：主轴的固有属性和进给轴不同（转速窗口、定向、
 * 运行期 CSV↔CSP 切换），混在一起会把轴表越弄越脏。
 * @note axis.logical_axis **必须 >= 0** —— 主轴角度接口也用 HalAxisId 寻址。 */
typedef struct {
    HalCAxisCfg axis;                   /* 通用属性全在这，不重复 */
    double      max_speed;              /* rpm */
    double      accel;                  /* 预留的主轴加速度参数；当前仅校验 >= 0，不生成加减速曲线，单位未定 */
    double      speed_window;           /* 转速到位窗口（rpm）；HAL 据此给 at_speed */
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

    uint64_t topology_fingerprint;      /* TODO 当前必须为 0；算法未定，不静默忽略非零值 */// TODO 指纹算法未定暂时都填0

    uint32_t cycle_us;
    uint32_t start_timeout_ms;          /* 整个启动过程共用的总预算 */
    uint32_t cycle_timeout_ms;
    uint32_t fault_reset_timeout_ms;    /* 故障复位从发起算起的等待上限（ms），必须 > 0。
                                           超过即判 HAL_RESET_TIMEOUT 并把该轴留在未使能态。 */
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

/* ============================================================================
 * 状态（出参；非 RT 读取须由调用方与周期线程同步）
 * ========================================================================== */

typedef struct {
    int32_t  enabled;                   /* 非零 = DS402 已到 OperationEnabled */
    int32_t  position_valid;            /* 非零 = 坐标可信。**当前恒为 1**：
                                           绝对式编码器不等于坐标可信，但驱动层现在还
                                           无从得知（要走 SDO）。字段保留，上层按
                                           「可能不可信」写，将来真值出现时不用改。 */
    double   actual_pos;                /* 用户单位 */
    double   command_pos;               /* 用户单位 */
    uint16_t raw_status;                /* 原始状态字 0x6041，诊断用 */
    uint16_t error_code;                /* 驱动器故障码 0x603F */
    int32_t  error_code_valid;          /* 0x603F 未映射时为 0，error_code 此时无效 */
} HalCAxisStatus;

typedef struct {
    int32_t  enabled;
    int32_t  at_speed;                  /* 非零 = |指令 - 实际| <= 转速窗口 */
    int32_t  mode;                      /* HAL_SPINDLE_CSV / _CSP，来自 0x6061 */
    double   command_speed;             /* rpm，带符号：正=正转 M03，负=反转 M04 */
    double   actual_speed;
    double   position_deg;              /* 累计转角（度）。HAL **不取模** */
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

/* 总线健康快照（出参）。
 *
 * **只记录不判断**——这里给的是原始计数与标志，什么算异常、要不要报警由上层决定。
 * 对应旧 RT 的 PdoWar_Info_Get() + Master_Hardware_Error 那条链：旧代码把 CRC 错误
 * 和帧超时直接映射成 ALARM_ETHCAT_SV，HAL 不做这个映射。
 *
 * 全部为累计/瞬时原始值，未做任何单位换算。还没收过帧时全零。 */
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
