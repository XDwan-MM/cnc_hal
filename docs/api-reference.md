# CNC HAL C API 接口文档

适用范围：当前公开 C ABI 3.0（`HAL_C_ABI_MAJOR=3`，`HAL_C_ABI_MINOR=0`）。本文以 `include/` 中的声明及 `src/internal/` 中的实现为准。厂商 SDK、`src/Greemaster/` 内部函数和 `examples/` 测试辅助函数不属于公开 API。

## 1. 引入与调用流程

```c
#include "hal_c_api.h"
```

`hal_c_api.h` 汇总配置、控制、周期接口及公共类型。使用时按以下顺序：

1. 清零并填写 `HalCConfig`；设置 ABI 版本和 `struct_size`，填写已确认的设备、换算参数及 IO 地址。
2. `hal_config_validate()` 做静态检查；`hal_context_create()` 拷贝配置并分配 context，不访问硬件。
3. `hal_context_start()` 启动主站，扫描实际从站，按设备字典和配置绑定设备/PDO；成功后可查询身份和能力。
4. 由**一个周期线程**反复调用 `hal_rt_wait_cycle()` → `hal_rt_begin_cycle()` → 读写本拍数据 → `hal_rt_commit_cycle()`。
5. 退出时可从其他线程调用 `hal_context_request_stop()`；等待周期线程退出后调用 `hal_context_stop()` 和 `hal_context_destroy()`。

```c
HalCConfig cfg = {0};
cfg.abi_major = HAL_C_ABI_MAJOR;
cfg.abi_minor = HAL_C_ABI_MINOR;
cfg.struct_size = sizeof(cfg);
/* 继续填写 cycle_us、超时、设备数组、轴换算参数等；参见第 2 节。 */

char err[256];
HalContext *ctx = NULL;
int32_t rc = hal_context_create(&cfg, &ctx, err, sizeof(err));
if (rc != HAL_OK) { /* err 给出原因 */ }
if (rc == HAL_OK) {
    rc = hal_context_start(ctx, err, sizeof(err));
    if (rc == HAL_OK) {
        /* 在单个周期线程执行 wait → begin → read/write → commit。 */
        hal_context_stop(ctx);
    }
    hal_context_destroy(ctx);
}
```

这段代码只示意生命周期，`cfg` 尚未具备可启动的设备配置。完整的现场配置和调用示例见 `examples/hal_hardware_test_config.c` 与 `examples/hal_hardware_test.c`。`start` 需要目标主站、SDK 和设备；`create` 与静态校验不需要硬件。

## 2. 配置与公共类型

以下字段**按结构体声明顺序**排列。表中「要求」列写的是 `hal_config_validate()` 会检查的内容 ——
不满足会在 `create` 阶段就被拒绝，不会走到硬件。

### `HalCConfig`

顶层配置。前四个成员构成版本闸门，**必须在最前且顺序不变**。

| 成员 | 类型 | 要求与含义 |
| --- | --- | --- |
| `abi_major` | `uint16_t` | 必须 `== HAL_C_ABI_MAJOR`（当前 **3**）。契约／语义变更时升它。 |
| `abi_minor` | `uint16_t` | 必须 `== HAL_C_ABI_MINOR`（当前 **0**）。**结构布局变更**（字段增删改序）时升它。 |
| `struct_size` | `uint16_t` | 必须 `== sizeof(HalCConfig)`，**精确相等**而不是「至少」—— 它是唯一能挡住字段插入的检查。 |
| `reserved` | `uint16_t` | 必须为 `0`。显式填充位，不依赖编译器对齐。 |
| `topology_fingerprint` | `uint64_t` | **当前必须为 `0`**。指纹算法未定，非零值会被直接拒绝，**不会静默跳过比对**。 |
| `cycle_us` | `uint32_t` | 主站通讯周期，单位微秒。**至少 `250`**。 |
| `start_timeout_ms` | `uint32_t` | 非零。**整个启动过程共用的总预算**，不是每阶段各用一次完整超时。各阶段共用一份单调时钟截止时间，后续等待只消耗剩余预算。 |
| `cycle_timeout_ms` | `uint32_t` | 非零。等待周期节拍的超时，单位毫秒。 |
| `dc_enable` | `int32_t` | `0` 或 `1`，是否启用分布式时钟。**启动时确定，运行期间不可改**。 |
| `axis_count` | `int32_t` | `axes[]` 的有效项数。 |
| `axes[]` | `HalCAxisCfg[HAL_C_MAX_DEV]` | **进给轴**配置数组，容量 **30**。只使用前 `axis_count` 项。进给轴只支持 `HAL_WORK_POSITION`。 |
| `spindle_count` | `int32_t` | `spindles[]` 的有效项数。 |
| `spindles[]` | `HalCSpindleCfg[HAL_C_MAX_SPINDLE]` | **主轴**配置数组，容量 **4**。主轴**单独一张表**，与 `axes[]` 平级 —— 转速窗口、模式切换这些主轴专有的量不污染轴表。 |
| `io_count` | `int32_t` | `ios[]` 的有效项数。 |
| `ios[]` | `HalCIoCfg[HAL_C_MAX_DEV]` | **普通 IO** 配置数组，容量 **30**。 |
| `panel_count` | `int32_t` | `panels[]` 的有效项数。 |
| `panels[]` | `HalCPanelCfg[HAL_C_MAX_DEV]` | **操作面板**配置数组，容量 **30**。与普通 IO 共用 X/Y 地址域。 |

四类设备的**总数**不得超过 `HAL_C_MAX_DEV`（**30**），不是各自 30。
`slave_pos` 是**实际总线扫描位置**，从 0 起；同一物理轴 `(slave_pos, axis_index)` 不能重复绑定，
IO／面板也不能与伺服重复占用同一从站。

**关于 ABI 闸门**：`struct_size` 用精确相等而非「至少」，是因为配置会在**两个组件之间传递**
（例如 Qt 构造、RT 读取）。字段插入会让两边对同一个偏移量的理解差几个字节，
「至少」拦不住这种情况。改了配置字段，两侧都要重编。

### `HalCAxisCfg`

**进给轴与主轴共用**的通用轴属性。凡是「带编码器的驱动轴」都有的东西都在这；
转速侧的量不在这里（那是主轴专有，见 `HalCSpindleCfg`）。

| 成员 | 类型 | 要求与含义 |
| --- | --- | --- |
| `slave_pos` | `int32_t` | 绑定的从站位置，从 `0` 起，即总线扫描顺序。属于总线拓扑，**运行期不变**。 |
| `axis_index` | `int32_t` | 从站**内部**的物理轴序号，从 `0` 起。单轴伺服只能填 `0`；多轴从站按实际轴数校验范围。`(slave_pos, axis_index)` 共同定位一根物理轴，不得重复绑定。 |
| `estop_action` | `int32_t` | **必须显式选择**，填 `0` 会被拒绝：`HAL_ESTOP_DISABLE_OPERATION`(1) 去使能，`HAL_ESTOP_DISABLE_VOLTAGE`(2) 撤销电压。不提供未经现场确认的默认动作。 |
| `logical_axis` | `int32_t` | 对外的逻辑轴号 `0..31`，不得重复。进给轴可填 `-1` 表示**不占轴槽**（存在、会被绑定和采样，但不可经轴号寻址）；**主轴必须 `>= 0`**。`HalAxisId` 就是这个号码，**不是数组下标**。 |
| `work_mode` | `int32_t` | `HAL_WORK_POSITION`(1，CSP 位置模式) 或 `HAL_WORK_VELOCITY`(3，CSV 速度模式)。**进给轴只支持 CSP**；主轴两者皆可配置。 |
| `encoder_type` | `int32_t` | `HAL_ENC_INCREMENTAL_Z`(1) 增量式带 Z 相或 `HAL_ENC_ABSOLUTE`(3) 绝对式。该字段目前只做配置校验；是否找 Z 相或把上电位置设为零由机床策略决定，HAL 不因编码器类型自动回零。 |
| `feedback_pulses_per_rev` | `int32_t` | 反馈**每转计数周期数**，须 `> 0`。**不能仅凭编码器标称分辨率推断** —— 要算上电子齿轮比。模循环反馈的跨圈展开依赖它。 |
| `feedback_wrap` | `int32_t` | `HAL_WRAP_LINEAR`(0) 线性计数 / `HAL_WRAP_MODULAR`(1) 模循环。见下方说明。 |
| `command_units_per_count` | `double` | **命令当量**，单位「用户单位／命令计数」，有限且 `> 0`。例：已知 1 mm 对应 10000 个命令计数，就填 `0.0001`。 |
| `feedback_units_per_count` | `double` | **反馈当量**，单位「用户单位／反馈计数」，有限且 `> 0`。**可与命令当量不同** —— 外接编码器或光栅尺可能只翻反馈侧。 |
| `command_invert` | `int32_t` | 非零 = 命令方向取反。 |
| `feedback_invert` | `int32_t` | 非零 = 反馈方向取反。**与命令分开配置**，理由同上。 |
| `enc_off` | `double` | 已保存的坐标偏置，单位 mm（直线轴）或 deg（回转轴），须有限。**初始显示坐标 = 原始反馈换算值 − 该偏置**，沿用旧 RT 的符号约定。它是**软件偏置，不会写入驱动器的电子齿轮对象**。运行时重新标定请用 `hal_rt_axis_set_pos()`。 |

**关于 `feedback_wrap` 的选型**：

- `HAL_WRAP_LINEAR`：原始计数按 32 位线性回绕展开。
- `HAL_WRAP_MODULAR`：按 `feedback_pulses_per_rev` 取模展开，**假定相邻采样位移不超过半个计数周期**，丢整转无法识别。

选哪个取决于驱动器位置对象（`0x6064`）的实际语义，**需要按设备手册和实测确认**，配错会导致角度或位置跳变。

### `HalCSpindleCfg`

**主轴专有属性**，在通用轴属性之上加转速侧的几个量。主轴单独一张表，与 `axes[]` 平级。

| 成员 | 类型 | 要求与含义 |
| --- | --- | --- |
| `axis` | `HalCAxisCfg` | 通用属性**整体复用**，不重复定义。其中 `axis.logical_axis` **必须 `>= 0`** —— 主轴角度接口也用 `HalAxisId` 寻址。 |
| `max_speed` | `double` | 最高转速，单位 rpm，有限且 `> 0`。`hal_rt_spindle_write_speed()` 的 `rpm` 上限。 |
| `accel` | `double` | **预留字段**。当前仅校验为有限且 `>= 0`，**不生成加减速曲线，单位也未定义**。不要按「已实现」使用。 |
| `speed_window` | `double` | 转速到位判定窗口，单位 rpm，有限且 `>= 0`。HAL 用 `|指令 − 实际| <= speed_window` 给出 `at_speed`。填 `0` 时几乎永远不会判为到速。 |

### `HalCIoCfg` 与 `HalCPanelCfg`

普通 IO 模块与操作面板。**两者共用同一块 PLC X/Y 地址域**，段长度由实际 PDO 布局在启动时确定。

| 成员 | 类型 | 要求与含义 |
| --- | --- | --- |
| `slave_pos` | `int32_t` | 从站位置，从 `0` 起。 |
| `x_start` | `int32_t` | 输入段在 PLC **X 域**的起始**字节**地址，须 `>= 0`。 |
| `y_start` | `int32_t` | 输出段在 PLC **Y 域**的起始**字节**地址，须 `>= 0`。 |

`HalCPanelCfg` 在上述三项之外**多一个成员**：

| 成员 | 类型 | 要求与含义 |
| --- | --- | --- |
| `panel_type` | `int32_t` | 面板型号。**按键与指示灯的语义由 PLC／HMI 按型号解释，HAL 不解读。** |

**地址域的两条规则**：

- **X 与 Y 是各自独立的地址域。** 分别检查 X 段之间、Y 段之间是否相交；输入地址与输出地址**不互相比较**。
- **静态校验只检查地址合法性**（非负）。实际段长度及互不重叠要等 `start` 绑定 PDO 之后才能检查 —— 段长度取决于现场设备的实际 PDO 布局，配置里写不出来。没有输入或输出的空段不占地址。

## 3. 配置与生命周期接口

除特别说明外，下列 `int32_t` 函数成功返回 `HAL_OK`（0），失败返回 `HAL_ERROR_*`。`err` 可以为 `NULL`；传入非空缓冲和正长度时，返回的诊断文本以 NUL 结尾。

### 3.1 配置与错误

#### 3.1.1 配置校验函数hal_config_validate()

**函数原型**

```c
int32_t hal_config_validate(const HalCConfig *config, char *err, uint32_t err_len);
```

**函数功能**

对传入的配置做纯静态检查，不启动主站、不打开任何设备。检查项包括：

- **ABI 版本闸门**：`abi_major`、`abi_minor` 是否匹配，`struct_size` 是否**精确相等**，`reserved` 是否为 0
- **设备数量**：`axes` / `spindles` / `ios` / `panels` 各自 `count` 是否在容量上限内，四类总数是否超出 `HAL_C_MAX_DEV`
- **静态绑定**：逻辑轴号是否重复、物理轴 `(slave_pos, axis_index)` 是否重复绑定、IO 与面板是否与伺服占用同一从站
- **数值范围**：当量与每转脉冲数是否有限且大于 0、急停动作是否已显式选择、主轴 `max_speed` 等
- **地址合法性**：`x_start` / `y_start` 是否为非负

**函数参数及返回值**

| 参数名 | 类型 | 说明 |
| ------ | ---- | ---- |
| config | `const HalCConfig*` | 待校验的配置，不可为空 |
| err | `char*` | 诊断文本输出缓冲，可为 `NULL` |
| err_len | `uint32_t` | 缓冲长度；为 0 时不写入 |
| 返回值 | `int32_t` | `HAL_OK`；失败为 `HAL_ERROR_ABI` / `HAL_ERROR_CONFIG` / `HAL_ERROR_ARGUMENT` |

**示例用法**

```c
HalCConfig cfg = {0};
cfg.abi_major   = HAL_C_ABI_MAJOR;
cfg.abi_minor   = HAL_C_ABI_MINOR;
cfg.struct_size = sizeof(cfg);
/* ... 继续填写主站参数、设备数组、换算参数 ... */

char err[256];
int32_t rc = hal_config_validate(&cfg, err, sizeof(err));
if (rc != HAL_OK) {
    /* err 里是中文原因，可直接显示给操作员 */
}
```

**注意**

- 只校验静态配置，**不能证明现场真的有那根轴**。实际轴数、PDO 长度、必需对象是否绑定、X/Y 地址段是否重叠，都要等 `hal_context_start()` 才能确认。
- 拓扑指纹算法尚未确定，当前非零的 `topology_fingerprint` 会被直接拒绝，不会静默忽略。
- `hal_context_create()` 内部会再调用一次本函数，因此单独调用属于前置拦截，不是必需步骤。

#### 3.1.2 错误文本函数hal_error_text()

**函数原型**

```c
void hal_error_text(int32_t code, char *out, uint32_t out_len);
```

**函数功能**

把返回码转换成简短的中文描述文本，便于日志和界面提示。

**函数参数及返回值**

| 参数名 | 类型 | 说明 |
| ------ | ---- | ---- |
| code | `int32_t` | 待转换的返回码，取 `HAL_OK` 或任一 `HAL_ERROR_*` |
| out | `char*` | 输出缓冲 |
| out_len | `uint32_t` | 输出缓冲长度 |
| 返回值 | `void` | 无返回值 |

**示例用法**

```c
char text[64];
hal_error_text(rc, text, sizeof(text));
printf("返回码 0x%04X：%s\n", rc, text);
```

**注意**

- `out` 为空或 `out_len` 为 0 时不写入任何内容，不会越界。
- 只给出返回码的通用解释。**具体失败原因要看 `create` / `start` 的 `err`** —— 例如同样是 `HAL_ERROR_CONFIG`，可能是字典没加载、设备类型不对或必需 PDO 缺失。

### 3.2 生命周期

#### 3.2.1 创建函数hal_context_create()

**函数原型**

```c
int32_t hal_context_create(const HalCConfig *config, HalContext **out, char *err, uint32_t err_len);
```

**函数功能**

创建一次 HAL 实例。内部依次完成：静态校验 → 分配 context → **复制整份配置** → 初始化运行数据、清除错误与周期阶段 → 把 `axes[]` 和 `spindles[]` 整理进内部轴记录（每根轴初始请求为「去使能」）。

**不访问硬件，不打开主站。**

**函数参数及返回值**

| 参数名 | 类型 | 说明 |
| ------ | ---- | ---- |
| config | `const HalCConfig*` | 配置，不可为空 |
| out | `HalContext**` | 带出句柄；**失败时置 `NULL`** |
| err | `char*` | 诊断文本缓冲，可为 `NULL` |
| err_len | `uint32_t` | 缓冲长度 |
| 返回值 | `int32_t` | `HAL_OK`；失败为 `HAL_ERROR_ABI` / `HAL_ERROR_CONFIG` / `HAL_ERROR_ARGUMENT` / `HAL_ERROR_MEMORY` |

**示例用法**

```c
HalContext *ctx = NULL;
char err[256];
int32_t rc = hal_context_create(&cfg, &ctx, err, sizeof(err));
if (rc != HAL_OK) {
    printf("创建失败：%s\n", err);
    return;     /* ctx 保持 NULL，无需 destroy */
}
/* 成功只是「配置副本和运行数据准备好了」，主站还没打开 */
```

**注意**

- 成功后**还没打开主站**，此时既不能查设备身份，也没有任何快照。
- 配置是**拷贝**的，调用方之后可以释放自己那份 `HalCConfig`。
- 允许创建多个 context，但 SDK 是单主站，同一时刻只有一个能 `start` 成功。
- 不用了这个对象要调 `hal_context_destroy()`。

#### 3.2.2 启动函数hal_context_start()

**函数原型**

```c
int32_t hal_context_start(HalContext *c, char *err, uint32_t err_len);
```

**函数功能**

启动主站并把配置里的设备接上线。内部顺序：

1. 检查全局主站是否已被其他 context 占用
2. 清除旧运行状态与停止通知
3. 从公共配置构造驱动的 `MasterConfig`
4. 启动驱动：加载设备字典 → 读 EEPROM → 装配 PDO → 完成握手
5. 取得实际设备槽表
6. 按 `(slave_pos, axis_index)` 绑定每根轴，按从站位置绑定每块 IO / 面板
7. 校验 X 段之间、Y 段之间地址不重叠
8. 检查启动总预算，置运行标志

**函数参数及返回值**

| 参数名 | 类型 | 说明 |
| ------ | ---- | ---- |
| c | `HalContext*` | 已创建的 context |
| err | `char*` | 诊断文本缓冲，可为 `NULL` |
| err_len | `uint32_t` | 缓冲长度 |
| 返回值 | `int32_t` | `HAL_OK`；失败为 `HAL_ERROR_BUSY` / `HAL_ERROR_CONFIG` / `HAL_ERROR_TIMEOUT` / `HAL_ERROR_BUS` 等 |

**示例用法**

```c
int32_t rc = hal_context_start(ctx, err, sizeof(err));
if (rc != HAL_OK) {
    printf("启动失败：%s\n", err);
    hal_context_destroy(ctx);      /* 对象仍在，必须释放 */
    return;
}
/* 只有到这里才允许查询设备身份、跑周期 */
```

**注意**

- **启动失败会回滚**：关闭驱动、释放主站占用、清除运行状态，不留下半可用的设备对象。
- 启动成功**不代表轴已使能**，只代表可以进入周期调用。使能要在后续 `begin` 里由 DS402 状态机逐拍推进（约 3~4 拍）。
- 设备字典路径取环境变量 `CNC_HAL_DEVICES_JSON`，未设置时用编译期固化的路径。**字典加载失败会直接返回，不会把总线拉到 OP。**
- 需要目标主站、SDK 和真实设备；`create` 和静态校验不需要硬件。
- 启动各阶段共用一份单调时钟预算，不是每阶段各用一次完整超时。

#### 3.2.3 请求停止函数hal_context_request_stop()

**函数原型**

```c
int32_t hal_context_request_stop(HalContext *c);
```

**函数功能**

非阻塞地发出停止通知。只设置 context 内的一个原子标志，不关闭主站、不释放资源、不等待。

**函数参数及返回值**

| 参数名 | 类型 | 说明 |
| ------ | ---- | ---- |
| c | `HalContext*` | 运行中的 context |
| 返回值 | `int32_t` | `HAL_OK`；其余见错误码章节 |

**示例用法**

```c
/* 控制线程：发出停机意图 */
hal_context_request_stop(ctx);

/* 周期线程：当前拍或下一拍拿到 STOPPED 后退出循环 */
while (running) {
    if (hal_rt_wait_cycle(ctx) == HAL_ERROR_STOPPED) break;
    if (hal_rt_begin_cycle(ctx))  break;
    /* ... 运算 ... */
    if (hal_rt_commit_cycle(ctx)) break;
}

/* 控制线程：确认周期线程已退出（例如 pthread_join）后，才继续 */
hal_context_stop(ctx);
hal_context_destroy(ctx);
```

**注意**

- **这是唯一允许与周期调用并发的接口**，可以从其他线程调用。其余生命周期调用必须串行。
- 只发通知，**不释放任何资源**；后续仍必须走 `stop` / `destroy`。
- **不等于急停**：它不会遍历各轴下发急停控制字。要停轴请用 `hal_rt_axis_estop()`。
- 等待中的 `wait_cycle` 依赖 SDK 的周期等待返回后才能检查通知。当前 SDK 等待参数是秒粒度且不可中断，所以**不承诺立即唤醒**，最坏要等一个等待周期返回。

#### 3.2.4 停止函数hal_context_stop()

**函数原型**

```c
int32_t hal_context_stop(HalContext *c);
```

**函数功能**

关闭主站并清除运行状态：调用驱动的关闭流程释放主站资源，随后清除快照、错误闭锁、周期阶段与旧指令。**保留配置**。

**函数参数及返回值**

| 参数名 | 类型 | 说明 |
| ------ | ---- | ---- |
| c | `HalContext*` | 要停止的 context |
| 返回值 | `int32_t` | `HAL_OK`；其余见错误码章节 |

**示例用法**

```c
/* 前提：所有周期调用已经退出 */
hal_context_stop(ctx);

/* 之后可以用同一个 context 再次启动 */
hal_context_start(ctx, err, sizeof(err));
```

**注意**

- 调用前**必须确保所有周期调用已经结束**（由调用方自己 join 周期线程）。**HAL 不做这个等待。**
- 可重复调用，幂等。
- 之后再次 `start` 时，**旧的快照、请求、坐标偏置和故障闭锁都不会保留**，坐标基准回到配置里的 `enc_off`。
- **这是清除总线故障闭锁的唯一途径** —— 闭锁后只有 `stop` + `start` 能清。

#### 3.2.5 销毁函数hal_context_destroy()

**函数原型**

```c
void hal_context_destroy(HalContext *c);
```

**函数功能**

释放 context 占用的内存，内部包含一次 `stop`。

**函数参数及返回值**

| 参数名 | 类型 | 说明 |
| ------ | ---- | ---- |
| c | `HalContext*` | 要释放的 context；**允许传 `NULL`** |
| 返回值 | `void` | 无返回值 |

**示例用法**

```c
hal_context_stop(ctx);       /* 建议显式先停 */
hal_context_destroy(ctx);    /* 再释放 */
ctx = NULL;                  /* 避免悬空指针 */
```

**注意**

- 允许传入 `NULL`，此时什么也不做。
- 若尚有周期线程使用该句柄，**必须先等待其退出**。不允许与周期调用并发销毁。
- 不向 C 边界抛异常。
- `create` 失败时 `ctx` 是 `NULL`，此时不需要也不必调用本函数。

### 3.3 轴与从站查询

#### 3.3.1 逻辑轴号解析函数hal_axis_resolve()

**函数原型**

```c
int32_t hal_axis_resolve(const HalContext *c, int32_t logical_axis, HalAxisId *out);
```

**函数功能**

校验一个逻辑轴号是否在本 context 的配置中真实存在，并把同一个号码原样带出。

这个函数看起来"什么都没做"——因为 `HalAxisId` 就是逻辑轴号本身，不是数组下标。它的价值在于把「轴号是否存在」变成一次**显式检查**：上层从 G 代码或 PLC 拿到一个轴号后先解析一次，后续所有 `hal_rt_*` 调用就可以信任这个号码。

**函数参数及返回值**

| 参数名 | 类型 | 说明 |
| ------ | ---- | ---- |
| c | `const HalContext*` | 已创建的 context |
| logical_axis | `int32_t` | 待校验的逻辑轴号 |
| out | `HalAxisId*` | 带出解析结果（与入参同值）；失败时置 `HAL_INVALID_ID` |
| 返回值 | `int32_t` | `HAL_OK`；轴号不存在或 `out` 为空为 `HAL_ERROR_ARGUMENT` |

**示例用法**

```c
HalAxisId axis;
if (hal_axis_resolve(ctx, 7, &axis) != HAL_OK) {
    printf("轴号 7 不可寻址（配置里 logical_axis = -1？）\n");
    return;
}
/* 此后可以放心用 axis 调周期接口 */
hal_rt_axis_read_status(ctx, axis, &status);
```

**注意**

- **可以在 `create` 之后调用**，不需要 `start` —— 它只查配置副本，不碰硬件。
- 返回的 `HalAxisId` 与入参**是同一个数字**。不要把它当数组下标去索引 `axes[]`。
- 失败时若 `out` 非空，会被置成 `HAL_INVALID_ID`（`UINT16_MAX`）。
- 解析一次即可在后续周期里复用，不必每拍调用。

#### 3.3.2 轴数查询函数hal_axis_count()

**函数原型**

```c
int32_t hal_axis_count(const HalContext *c);
```

**函数功能**

返回本 context 中**可经轴号寻址**的轴数量，包含主轴。

**函数参数及返回值**

| 参数名 | 类型 | 说明 |
| ------ | ---- | ---- |
| c | `const HalContext*` | context 句柄 |
| 返回值 | `int32_t` | **非负** = 轴数；**负数** = 错误码 |

**示例用法**

```c
const int32_t n = hal_axis_count(ctx);
if (n < 0) {
    char text[64];
    hal_error_text(n, text, sizeof(text));
    printf("查询失败：%s\n", text);
    return;
}
printf("可寻址轴共 %d 根\n", n);
```

**注意**

- **返回值约定与其他函数相反**：成功返回非负数量，失败返回**负**错误码。空句柄返回负的 `HAL_ERROR_ARGUMENT`。
- **不含 `logical_axis = -1` 的进给轴** —— 这类轴存在、会被绑定和采样，但不占轴号，不能用 `HalAxisId` 寻址。
- 数量**包含主轴**。主轴也占一个逻辑轴号，不是额外的分类。
- 可在 `create` 后调用，不需要 `start`。

#### 3.3.3 设备身份查询函数hal_device_identity()

**函数原型**

```c
int32_t hal_device_identity(const HalContext *c, HalAxisId id, HalCIdentity *out);
```

**函数功能**

返回一根**已绑定轴**所对应设备的身份信息，来源是启动时扫描到的从站与设备字典的匹配结果。

带出的 `HalCIdentity` 字段：

| 字段 | 含义 |
| --- | --- |
| `type` | 设备类型（由字典里的 type 映射而来） |
| `vendor_id` / `product_code` / `revision` | EtherCAT 从站身份三元组 |
| `serial` | 序列号，可能为 0 |
| `name` | 设备字典查出的名称；未识别时为空串 |
| `family_index` | 同 vendor+product 的第几台（0 起，按总线顺序） |
| `device_id` | `serial` 非零时取 `serial`，否则按 vendor/product/revision 算哈希 |

**函数参数及返回值**

| 参数名 | 类型 | 说明 |
| ------ | ---- | ---- |
| c | `const HalContext*` | 已启动的 context |
| id | `HalAxisId` | 逻辑轴号 |
| out | `HalCIdentity*` | 带出身份信息，不可为空 |
| 返回值 | `int32_t` | `HAL_OK`；未启动或轴号无效为 `HAL_ERROR_NOT_RUNNING` / `HAL_ERROR_ARGUMENT` |

**示例用法**

```c
HalCIdentity dev;
if (hal_device_identity(ctx, axis, &dev) == HAL_OK) {
    printf("设备：%s  vendor=0x%08X product=0x%08X rev=0x%08X（同型号第 %d 台）\n",
           dev.name[0] ? dev.name : "未识别",
           dev.vendor_id, dev.product_code, dev.revision, dev.family_index);
}
```

**注意**

- **需要 `start` 成功**才能调用。
- `device_id` 在 `serial` 为 0 时是**哈希值**，不是真实序列号 —— 不要把它当"设备序列号"显示给操作员。
- 未识别的从站仍会带出 vendor/product/revision 数字，但 `name` 为空串，且不因此自动获得任何功能。

#### 3.3.4 轴能力查询函数hal_axis_capability()

**函数原型**

```c
int32_t hal_axis_capability(const HalContext *c, HalAxisId id, uint32_t function, HalCCapability *out);
```

**函数功能**

查询一根**已配置轴**上某项功能的可用状态。`function` 取：

| 宏 | 值 | 含义 |
| --- | ---: | --- |
| `HAL_FUNC_POSITION` | 1 | 位置功能（CSP） |
| `HAL_FUNC_SPEED` | 2 | 速度功能（CSV） |
| `HAL_FUNC_ERROR_CODE` | 3 | 故障码读取 |
| `HAL_FUNC_ALARM_CONTROL` | 4 | 报警控制（轴级无定义，见 3.3.7） |

**只读启动时的绑定结果，不访问总线。**

带出的 `HalCCapability` 含 `function`、`state`、`slave_pos`、`axis_index`、`reason[96]`。
`state` 取 `HAL_CAP_READY`(1) / `HAL_CAP_UNSUPPORTED`(2) / `HAL_CAP_NOT_CONFIGURED`(3)。

**函数参数及返回值**

| 参数名 | 类型 | 说明 |
| ------ | ---- | ---- |
| c | `const HalContext*` | 已启动的 context |
| id | `HalAxisId` | 逻辑轴号 |
| function | `uint32_t` | `HAL_FUNC_*` 之一 |
| out | `HalCCapability*` | 带出能力状态 |
| 返回值 | `int32_t` | `HAL_OK` = **查询本身成功**，不是"能力可用" |

**示例用法**

```c
HalCCapability cap;
if (hal_axis_capability(ctx, axis, HAL_FUNC_SPEED, &cap) == HAL_OK) {
    if (cap.state == HAL_CAP_READY) {
        /* 可以走 CSV 转速接口 */
    } else {
        printf("速度功能不可用（state=%u）：%s\n", cap.state, cap.reason);
    }
}
```

**注意**

- **函数返回 0 只代表「查询成功」，不代表能力可用** —— 必须再看 `out->state`。这是最容易误判的一点。
- **位置功能**：缺少必需 PDO 会让整个 `start` 失败，因此成功启动的已配置轴查位置**一律为 `READY`**。
- **速度功能**：要求实际速度与目标速度 PDO **都**绑定。主轴缺它们会导致 `start` 失败；进给轴缺则返回 `UNSUPPORTED`。
- **故障码功能**：要求 `0x603F` 已绑定，缺则返回 `UNSUPPORTED`，同时轴状态里的 `error_code_valid = 0`。
- 结果来自**现有设备字典 + 启动时装配的 PDO 句柄**。当前版本**不解析 ESI，也不按 ESI 自动配置 PDO**，所以查询结果不等同于对全部 CoE 字典对象的在线核验。

#### 3.3.5 从站数量查询函数hal_slave_count()

**函数原型**

```c
int32_t hal_slave_count(const HalContext *c);
```

**函数功能**

返回启动时**实际扫描到**的从站数量。

**函数参数及返回值**

| 参数名 | 类型 | 说明 |
| ------ | ---- | ---- |
| c | `const HalContext*` | 已启动的 context |
| 返回值 | `int32_t` | **非负** = 从站数；**负数** = 错误码 |

**示例用法**

```c
const int32_t total = hal_slave_count(ctx);
if (total < 0) return;
printf("总线上扫到 %d 个从站\n", total);
for (int32_t i = 0; i < total; ++i) {
    HalCSlaveInfo info;
    if (hal_slave_info(ctx, i, &info) == HAL_OK)
        printf("  [%d] %s（装配 %d 轴）\n", info.slave_pos,
               info.identity.name[0] ? info.identity.name : "未识别",
               info.axis_count);
}
```

**注意**

- **返回值约定同 `hal_axis_count`**：非负数量、负错误码。
- **包含未配置和未识别的从站** —— 这是「总线上实际挂了几个」，不是「配置里用了几个」。两者对比可以发现配置漏项。
- 需要 `start` 成功；未启动返回负错误码。

#### 3.3.6 从站信息查询函数hal_slave_info()

**函数原型**

```c
int32_t hal_slave_info(const HalContext *c, int32_t slave_pos, HalCSlaveInfo *out);
```

**函数功能**

按**从 0 起的扫描位置**查询某个实际从站的信息。

带出的 `HalCSlaveInfo` 含：`slave_pos`（实际扫描顺序）、`axis_count`（**该从站**已装配的物理轴数）、`identity`（同 3.3.3 的 `HalCIdentity`）。

**函数参数及返回值**

| 参数名 | 类型 | 说明 |
| ------ | ---- | ---- |
| c | `const HalContext*` | 已启动的 context |
| slave_pos | `int32_t` | 从站扫描位置，从 0 起 |
| out | `HalCSlaveInfo*` | 带出从站信息 |
| 返回值 | `int32_t` | `HAL_OK`；越界或未启动为 `HAL_ERROR_ARGUMENT` / `HAL_ERROR_NOT_RUNNING` |

**示例用法**

```c
HalCSlaveInfo info;
if (hal_slave_info(ctx, 2, &info) == HAL_OK)
    printf("从站 2：vendor=0x%08X，装配 %d 轴\n",
           info.identity.vendor_id, info.axis_count);
```

**注意**

- `slave_pos` 是**实际扫描顺序**，应与配置里写的 `slave_pos` 一致；不一致说明拓扑变了，这也是启动时绑定失败的常见原因。
- `axis_count` 是**该从站里实际装配的物理轴数**（多轴从站会大于 1），与 `hal_axis_count()` 口径不同 —— 后者数的是可寻址的逻辑轴号。
- 未识别的从站也能查到，只是 `identity.name` 为空串。

#### 3.3.7 从站能力查询函数hal_slave_capability()

**函数原型**

```c
int32_t hal_slave_capability(const HalContext *c, int32_t slave_pos, uint32_t function, HalCCapability *out);
```

**函数功能**

查询某个实际从站上某项功能的可用状态。**当前仅接受 `HAL_FUNC_ALARM_CONTROL`（报警控制）。**

**函数参数及返回值**

| 参数名 | 类型 | 说明 |
| ------ | ---- | ---- |
| c | `const HalContext*` | 已启动的 context |
| slave_pos | `int32_t` | 从站扫描位置，从 0 起 |
| function | `uint32_t` | 当前仅 `HAL_FUNC_ALARM_CONTROL` |
| out | `HalCCapability*` | 带出能力状态 |
| 返回值 | `int32_t` | `HAL_OK` = 查询本身成功 |

**示例用法**

```c
HalCCapability cap;
if (hal_slave_capability(ctx, 0, HAL_FUNC_ALARM_CONTROL, &cap) == HAL_OK)
    printf("报警控制 state=%u：%s\n", cap.state, cap.reason);
/* 当前一定打印 state=3（HAL_CAP_NOT_CONFIGURED） */
```

**注意**

- **当前始终报告 `HAL_CAP_NOT_CONFIGURED`(3)。** 这是**预留能力**，不代表"设备不支持"。
- **库没有写 `0x6FFF` 的公开函数**，也没有提供报警控制的执行接口 —— 这个查询目前只是把状态如实报出来。
- 同 3.3.4：返回 0 只代表查询成功，能力是否可用看 `out->state`。

## 4. 周期接口与线程约束

单个周期必须按下面的顺序执行；`wait` 是这组三拍中唯一的阻塞点：

```text
wait_cycle → begin_cycle → [read_* / write_* / IO 操作] → commit_cycle
                           ↑ 本拍快照          ↑ 暂存输出       ↑ 真正提交
```

周期调用应由同一个 RT 线程串行执行。`read_*` 读最近一次 `begin` 的快照，不主动读取总线；同一拍先写后读，读不到本拍刚写的物理结果。`write_*` 与 IO 输出先暂存，到 `commit` 才下发。`begin` 前没有快照时，读状态返回 `HAL_ERROR_NOT_RUNNING`。`request_stop` 可跨线程通知；其余 context 生命周期调用须串行，`stop`/`destroy` 之前须等待周期线程退出。

全库共用的返回码统一列在 **第 5 章「返回码」**，本节各函数的返回值只写**该函数特有**的失败情形。
另外，本章函数普遍还会返回这几个码，下文不再逐个重复：`HAL_ERROR_ARGUMENT`（context 为空、轴号不存在、入参越界或非有限值）、
`HAL_ERROR_NOT_RUNNING`（未 `start`，或该操作要求的运行状态尚未达成）、
`HAL_ERROR_STOPPED`（已 `request_stop`，或阻塞等待被打断）、
`HAL_ERROR_BUS`（总线失败；**首个错误码被闭锁**，此后所有周期调用持续返回它，直到 `stop`/`start` 清掉）。

### 4.1 周期原语

#### 4.1.1 等待周期函数hal_rt_wait_cycle()

**函数原型**

```c
int32_t hal_rt_wait_cycle(HalContext *c);
```

**函数功能**

周期第一拍：等待主站的下一个周期节拍，并接收一帧数据。驱动内部调用 SDK 的等待与接收，把本帧数据搬进 PDO 缓冲。

**这是整个周期里唯一的阻塞点。**

**函数参数及返回值**

| 参数名 | 类型 | 说明 |
| ------ | ---- | ---- |
| c | `HalContext*` | 已 `start` 的 context |
| 返回值 | `int32_t` | `HAL_OK`；`HAL_ERROR_STATE` = 上一拍未走完（`phase != 0`）；其余见第 5 章 |

**示例用法**

```c
int32_t rc = hal_rt_wait_cycle(ctx);
if (rc == HAL_ERROR_STOPPED) {
    running = 0;            /* 上层请求停止：可识别的正常退出 */
} else if (rc != HAL_OK) {
    running = 0;            /* HAL_ERROR_STATE 顺序错，或 HAL_ERROR_BUS 已闭锁 */
}
```

**注意**

- **上一拍必须已经 `commit`**，否则返回 `HAL_ERROR_STATE`。三拍顺序不能乱。
- 阻塞期间若被 `hal_context_request_stop()` 打断，返回 `HAL_ERROR_STOPPED` —— 这是**可识别的正常停止**，不要与总线故障混为一谈。
- 当前 SDK 的等待参数是秒粒度且不可中断，**不承诺立即唤醒**，最坏要等一个等待周期返回。
- `wait` 只负责收帧，**不运行任何使能状态机**；轴的状态推进在 `begin`。

#### 4.1.2 开始周期函数hal_rt_begin_cycle()

**函数原型**

```c
int32_t hal_rt_begin_cycle(HalContext *c);
```

**函数功能**

周期第二拍，也是**唯一的状态推进点**。按顺序完成：

1. 采样每根轴的实际反馈：实际位置、状态字、实际模式、故障码、实际速度，并预检全部需要换算的预置值
2. 若有「速度目标清零」待发（使能、取消运动或急停留下的），先补发一次速度 0
3. 每根轴推**恰好一拍** DS402 状态机，并判断本轮是否需要预置目标位置
4. 按 Entry 读各 IO 模块与面板的输入，打包进各自 `inputs[]`

全部完成后，本拍快照才可用。

**函数参数及返回值**

| 参数名 | 类型 | 说明 |
| ------ | ---- | ---- |
| c | `HalContext*` | 已 `start` 的 context |
| 返回值 | `int32_t` | `HAL_OK`；`HAL_ERROR_STATE` = 未先 `wait`；`HAL_ERROR_ARGUMENT` = 非同一计数坐标系的预置值超出 int32；其余见第 5 章 |

**示例用法**

```c
if (hal_rt_begin_cycle(ctx) != HAL_OK) { running = 0; return; }

/* 到这里本拍快照才有效：可以读位置、状态、IO 输入 */
double pos;
hal_rt_axis_read_pos(ctx, axis, &pos);
```

**注意**

- **必须先 `wait`**，否则返回 `HAL_ERROR_STATE`。
- 预置换算失败不会闭锁成总线故障，也不会写本拍控制 PDO；周期未进入可提交状态，调用方须停止周期并处置配置。
- **常规 DS402 推进只发生在这里，且每拍恰好一次。** 不要在别处重复推进。
- **本拍采到的状态字是「上一拍命令的结果」。** 即使 `begin` 刚写了「上使能」控制字，也不能马上认为 `status.enabled` 变成了 1 —— 要等后续周期的反馈。
- 所有 `read_*` 读的都是这里采下的快照；`begin` 之前没有快照，读状态会返回 `HAL_ERROR_NOT_RUNNING`。
- 同一拍先写后读，**读不到自己刚写的值**。

#### 4.1.3 提交周期函数hal_rt_commit_cycle()

**函数原型**

```c
int32_t hal_rt_commit_cycle(HalContext *c);
```

**函数功能**

周期第三拍：把本拍暂存的输出真正下发。按顺序处理：

1. **急停覆盖**：必要时清零速度目标、覆盖本拍已写下的旧控制字
2. **轴指令**：把暂存的位置／速度写进对应设备的 PDO 输出缓冲
3. **IO 输出**：把暂存的 IO 输出写进 PDO 缓冲
4. **提交整帧**，发上总线
5. 成功后周期回到空闲相

**函数参数及返回值**

| 参数名 | 类型 | 说明 |
| ------ | ---- | ---- |
| c | `HalContext*` | 已 `start` 的 context |
| 返回值 | `int32_t` | `HAL_OK`；`HAL_ERROR_STATE` = 未先 `begin`；其余见第 5 章 |

**示例用法**

```c
if (hal_rt_commit_cycle(ctx) != HAL_OK) { running = 0; return; }
/* 本拍指令在这一刻才真正到达驱动器 */
```

**注意**

- **必须先 `begin`**，否则返回 `HAL_ERROR_STATE`。
- **这是整帧唯一真正上路的地方。** 此前所有 `write_*` 与 `flush_outputs` 都只是暂存。
- 任何输出写失败都会**闭锁总线错误**，且**不会继续提交本帧**。PDO 缓冲可能已被部分改动，但这一轮不会发出去。
- 提交成功后周期回到空闲相，才能进入下一次 `wait`。

### 4.2 进给轴

#### 4.2.1 轴使能函数hal_rt_axis_enable()

**函数原型**

```c
int32_t hal_rt_axis_enable(HalContext *c, HalAxisId id, int32_t on);
```

**函数功能**

申请使能或去使能一根进给轴。只改变该轴的使能请求，**不等待驱动器完成**。

**函数参数及返回值**

| 参数名 | 类型 | 说明 |
| ------ | ---- | ---- |
| c | `HalContext*` | 已 `start` 的 context |
| id | `HalAxisId` | 逻辑轴号 |
| on | `int32_t` | `1` = 使能，`0` = 去使能；**其它值拒绝** |
| 返回值 | `int32_t` | `HAL_OK`；`HAL_ERROR_ARGUMENT` = 轴号无效或 `on` 非 0/1；其余见第 5 章 |

**示例用法**

```c
/* 申请使能，然后轮询确认——不能假设当拍生效 */
hal_rt_axis_enable(ctx, axis, 1);

while (running) {
    if (hal_rt_wait_cycle(ctx) || hal_rt_begin_cycle(ctx)) break;

    HalCAxisStatus st;
    if (hal_rt_axis_read_status(ctx, axis, &st) == HAL_OK && st.enabled)
        break;                      /* 已到 OperationEnabled，可以下发运动 */

    if (hal_rt_commit_cycle(ctx)) break;
}
```

**注意**

- **非阻塞且幂等。** DS402 状态机在后续 `begin` 里逐拍推进，**约 3~4 拍**才到位。**必须轮询 `read_status().enabled` 确认**，不可假设当拍生效。
- **`on = 0` 停在 DS402 SwitchedOn**（可收指令、不带载），**不是断电**。要断电请用 `hal_rt_axis_estop()`。
- 请求**发生变化**时会丢弃该轴**尚未提交**的运动指令；恢复运动须重新下发目标位置。重复相同请求不会反复丢弃。
- 使能**不会**自动清除驱动器故障。当前没有独立的公共故障复位接口。

#### 4.2.2 轴急停函数hal_rt_axis_estop()

**函数原型**

```c
int32_t hal_rt_axis_estop(HalContext *c, HalAxisId id);
```

**函数功能**

急停一根进给轴。做三件事：

1. 取消尚未提交的运动、使能与模式请求，并置「速度目标清零」
2. 按该轴配置的 `estop_action`，把本拍控制字覆盖成去使能（`0x07`）或撤销电压（`0x00`）
3. 把期望模式改回已采样的实际模式，取消先前未完成的模式请求

**函数参数及返回值**

| 参数名 | 类型 | 说明 |
| ------ | ---- | ---- |
| c | `HalContext*` | 已 `start` 的 context |
| id | `HalAxisId` | 逻辑轴号 |
| 返回值 | `int32_t` | `HAL_OK` = 停止请求已受理；其余见第 5 章 |

**示例用法**

```c
if (hal_rt_axis_estop(ctx, axis) != HAL_OK) {
    /* 请求没被受理 */
}
/* 注意：返回成功只代表「停止意图已记录」，驱动器实际动作要等后续反馈 */
```

**注意**

- **返回成功 ≠ 硬件已停。** 它表示「停止请求已受理」，实际动作仍要经过 PDO 提交和后续反馈。
- **若在 `begin` 之后才到达，`commit` 会用急停值覆盖 `begin` 写下的旧控制字和速度目标** —— 否则本拍仍可能把旧的使能动作发出去。这是唯一的「输出覆盖」例外，不会再跑一遍常规 DS402 推进。
- **重复调用幂等。**
- **恢复必须显式重新 `enable`**，并经后续 `begin` 再次确认就绪后才能受理运动指令。
- **有意不因总线故障闭锁而拒绝**：闭锁时仍记录停止意图，但不再发送 PDO。所以「受理成功」不等于「失联设备真的执行了」。
- 与 `hal_context_request_stop()` 是两件不同的事 —— 后者只结束周期调用，不下发任何轴控制字。

#### 4.2.3 轴位置下发函数hal_rt_axis_write_pos()

**函数原型**

```c
int32_t hal_rt_axis_write_pos(HalContext *c, HalAxisId id, double pos);
```

**函数功能**

在 CSP 模式下下发目标位置（用户单位）。内部把用户位置换算成原始计数并**暂存**，真正的下发在 `commit`。

换算关系：

```text
目标计数 = round((用户位置 − command_offset) ÷ command_scale)
command_scale = command_units_per_count × 命令方向符号
```

**函数参数及返回值**

| 参数名 | 类型 | 说明 |
| ------ | ---- | ---- |
| c | `HalContext*` | 已 `start` 的 context |
| id | `HalAxisId` | 逻辑轴号 |
| pos | `double` | 目标位置（用户单位），须为有限值 |
| 返回值 | `int32_t` | `HAL_OK`；`HAL_ERROR_NOT_RUNNING` = 受理条件不满足；`HAL_ERROR_ARGUMENT` = `pos` 非有限值或换算后超出 int32 计数 |

**示例用法**

```c
/* 必须在 begin 与 commit 之间调用 */
if (hal_rt_axis_write_pos(ctx, axis, 12.5) == HAL_OK) {
    /* 已受理并暂存；此刻驱动器还没收到 */
}
if (hal_rt_commit_cycle(ctx) != HAL_OK) { /* 这一次才真正下发 */ }
```

**注意**

- **受理条件有四条，缺一不可**：本拍已 `begin` 且未 `commit`、该轴**已使能**、状态机已到位、运行模式**确为 CSP**。
  仅看 `status.enabled` 不够 —— 刚请求切模式或刚取消运动时也可能被拒。
- **返回 `HAL_OK` 只代表「被受理并暂存」**，不表示驱动器已经收到。同一拍先写后读，读不到自己刚写的值。
- **坐标相对 `hal_rt_axis_set_pos()` 建立的基准**；启动时还会叠加配置里的 `enc_off`。
- 换算后超出 int32 计数会被拒绝，**不静默饱和或截断**。
- 非有限值（`NaN` / `Inf`）直接返回参数错误。
- 主轴也可用这个通用位置接口，但单位语义是度。

#### 4.2.4 轴位置读取函数hal_rt_axis_read_pos()

**函数原型**

```c
int32_t hal_rt_axis_read_pos(HalContext *c, HalAxisId id, double *pos);
```

**函数功能**

读取最近一次采样的实际位置（用户单位）。**只读快照，不触发总线访问。**

**函数参数及返回值**

| 参数名 | 类型 | 说明 |
| ------ | ---- | ---- |
| c | `HalContext*` | 已 `start` 的 context |
| id | `HalAxisId` | 逻辑轴号 |
| pos | `double*` | 带出实际位置 |
| 返回值 | `int32_t` | `HAL_OK`；`HAL_ERROR_NOT_RUNNING` = 还没跑过 `begin`，没有快照 |

**示例用法**

```c
double pos;
if (hal_rt_axis_read_pos(ctx, axis, &pos) == HAL_OK)
    printf("轴实际位置：%.4f\n", pos);
```

**注意**

- 读的是**最近一次 `begin` 采下的快照**，所以数据最多滞后一拍。
- `begin` 之前调用会返回 `HAL_ERROR_NOT_RUNNING`。
- 位置已按反馈当量和方向换算为用户单位，并叠加了反馈侧坐标基准。
- **`position_valid` 当前恒为 1，不代表已经机械回零或经过独立校准。**

#### 4.2.5 轴状态读取函数hal_rt_axis_read_status()

**函数原型**

```c
int32_t hal_rt_axis_read_status(HalContext *c, HalAxisId id, HalCAxisStatus *out);
```

**函数功能**

读取最近一次采样的轴状态快照。

带出的 `HalCAxisStatus` 含：

| 字段 | 含义 |
| --- | --- |
| `enabled` | 非零 = DS402 已到 OperationEnabled。**取自状态字的判定，不是 HAL 的推断** |
| `position_valid` | 坐标是否可信。**当前恒为 1** |
| `actual_pos` | 实际位置（用户单位） |
| `command_pos` | 最近下发的指令位置（用户单位），由写接口更新 |
| `raw_status` | 原始 DS402 状态字 `0x6041` |
| `error_code` | 驱动器故障码 `0x603F` |
| `error_code_valid` | 故障码是否有效；`0x603F` 未绑定时为 0 |

**函数参数及返回值**

| 参数名 | 类型 | 说明 |
| ------ | ---- | ---- |
| c | `HalContext*` | 已 `start` 的 context |
| id | `HalAxisId` | 逻辑轴号 |
| out | `HalCAxisStatus*` | 带出状态快照 |
| 返回值 | `int32_t` | `HAL_OK`；`HAL_ERROR_NOT_RUNNING` = 还没跑过 `begin` |

**示例用法**

```c
HalCAxisStatus st;
if (hal_rt_axis_read_status(ctx, axis, &st) == HAL_OK) {
    printf("使能=%d 实际=%.4f 指令=%.4f 状态字=0x%04X\n",
           st.enabled, st.actual_pos, st.command_pos, st.raw_status);
    if (st.error_code_valid && st.error_code)
        printf("驱动器故障码：0x%04X\n", st.error_code);
}
```

**注意**

- **`position_valid` 恒为 1，不能据此认定机械坐标已经回零或经过独立校准。** 它的本意是「绝对式编码器不等于坐标可信」，驱动层目前无从得知（要走 SDO）。字段保留，上层按「坐标可能不可信」来写即可。
- `enabled` 来自状态字判定，本拍采的是上一拍命令的结果。
- 检查故障码前先看 `error_code_valid`。

#### 4.2.6 坐标基准设置函数hal_rt_axis_set_pos()

**函数原型**

```c
int32_t hal_rt_axis_set_pos(HalContext *c, HalAxisId id, double pos);
```

**函数功能**

把该轴的当前位置**重新定义为** `pos`（用户单位）。回零、对刀之后设坐标用的。

实现上只平移命令侧与反馈侧的坐标基准，**不写驱动器**，也**不改动已暂存的运动目标** —— 所以调用它本身不会让轴动。

**函数参数及返回值**

| 参数名 | 类型 | 说明 |
| ------ | ---- | ---- |
| c | `HalContext*` | 已 `start` 的 context |
| id | `HalAxisId` | 逻辑轴号 |
| pos | `double` | 新的当前位置（用户单位），须为有限值 |
| 返回值 | `int32_t` | `HAL_OK`；`HAL_ERROR_NOT_RUNNING` = 还没跑过 `begin`；`HAL_ERROR_ARGUMENT` = 非有限值 |

**示例用法**

```c
/* 当前反馈位置 20，把这里定义成 120 */
hal_rt_axis_set_pos(ctx, axis, 120.0);

double pos;
hal_rt_axis_read_pos(ctx, axis, &pos);   /* 现在读到 120 */
/* 驱动器原始位置没变，已暂存的原始运动目标也没变 */
```

**注意**

- **本身不会让轴动**，也不会单独生成新的运动命令。
- **至少要跑过一次 `begin`**（需要有采样才能算出平移量）。
- **使能中也可以调。**
- 只影响当前 context；**重启后恢复配置里的 `enc_off` 基准**，`set_pos` 的结果不会跨 stop/start 保留。
- 与 `write_pos` 的区别：`set_pos` 改的是**坐标基准**，`write_pos` 定的是**运动目标**。

### 4.3 主轴

主轴用 `HalAxisId` 寻址 —— **它就是一根轴**，只是多了转速语义，没有单独的 `HalSpindleId`。
下面这些接口都要求 `id` 指向一根**主轴**；传进给轴轴号会返回 `HAL_ERROR_ARGUMENT`。

#### 4.3.1 主轴使能函数hal_rt_spindle_enable()

**函数原型**

```c
int32_t hal_rt_spindle_enable(HalContext *c, HalAxisId id, int32_t on);
```

**函数功能**

主轴使能／去使能。确认 `id` 对应主轴后，复用进给轴的使能实现。

**函数参数及返回值**

| 参数名 | 类型 | 说明 |
| ------ | ---- | ---- |
| c | `HalContext*` | 已 `start` 的 context |
| id | `HalAxisId` | 逻辑轴号，**且必须是主轴** |
| on | `int32_t` | `1` = 使能，`0` = 去使能 |
| 返回值 | `int32_t` | `HAL_OK`；`HAL_ERROR_ARGUMENT` = 该轴号不是主轴；其余见第 5 章 |

**示例用法**

```c
hal_rt_spindle_enable(ctx, spindle, 1);
/* 同轴使能：非阻塞、幂等，需轮询 read_status().enabled 确认 */
```

**注意**

- 语义与 `hal_rt_axis_enable()` **完全一致**：非阻塞、幂等、约 3~4 拍到位，必须轮询确认。
- `on = 0` 停在 SwitchedOn，**不是断电**；要断电用 `hal_rt_spindle_estop()`。

#### 4.3.2 主轴急停函数hal_rt_spindle_estop()

**函数原型**

```c
int32_t hal_rt_spindle_estop(HalContext *c, HalAxisId id);
```

**函数功能**

主轴急停。语义与 `hal_rt_axis_estop()` 相同，**断电还是去使能由该主轴配置的 `estop_action` 决定**。

**函数参数及返回值**

| 参数名 | 类型 | 说明 |
| ------ | ---- | ---- |
| c | `HalContext*` | 已 `start` 的 context |
| id | `HalAxisId` | 逻辑轴号，**且必须是主轴** |
| 返回值 | `int32_t` | `HAL_OK` = 停止请求已受理；`HAL_ERROR_ARGUMENT` = 不是主轴 |

**示例用法**

```c
hal_rt_spindle_estop(ctx, spindle);
/* 返回成功只代表停止意图已记录 */
```

**注意**

- 复用轴急停的同一份实现，注意事项同 4.2.2：**受理 ≠ 已停**、幂等、恢复须显式使能并确认。
- 总线故障闭锁时仍受理，但不再发送 PDO。

#### 4.3.3 主轴模式请求函数hal_rt_spindle_request_mode()

**函数原型**

```c
int32_t hal_rt_spindle_request_mode(HalContext *c, HalAxisId id, HalSpindleMode mode);
```

**函数功能**

请求切换主轴的运行模式。`mode` 取 `HAL_SPINDLE_CSV`(0，速度模式) 或 `HAL_SPINDLE_CSP`(1，位置模式)。

**只改期望模式，不改变使能状态。** 实际切换在之后的 `begin` 里由 DS402 状态机推进，默认序列是：

```text
先下使能 → 预置目标位置 → 写 0x6060 → 再上使能
```

**函数参数及返回值**

| 参数名 | 类型 | 说明 |
| ------ | ---- | ---- |
| c | `HalContext*` | 已 `start` 的 context |
| id | `HalAxisId` | 逻辑轴号，**且必须是主轴** |
| mode | `HalSpindleMode` | `HAL_SPINDLE_CSV` 或 `HAL_SPINDLE_CSP` |
| 返回值 | `int32_t` | `HAL_OK`；`HAL_ERROR_ARGUMENT` = 不是主轴或模式值非法 |

**示例用法**

```c
/* 刚性攻丝前切到位置模式 */
hal_rt_spindle_request_mode(ctx, spindle, HAL_SPINDLE_CSP);

/* 非阻塞、幂等：轮询实际模式确认，不要假设当拍生效 */
HalCSpindleStatus st;
hal_rt_spindle_read_status(ctx, spindle, &st);
if (st.mode == HAL_SPINDLE_CSP) { /* 切换完成 */ }
```

**注意**

- **不会自动重新使能。** 处于急停或去使能状态的轴，单独调用本函数不会把它重新拉起来 —— 需要显式 `hal_rt_spindle_enable()`。
- **非阻塞、幂等**，语义与 `enable` 同一套：必须轮询 `read_status().mode` 确认。
- **期望模式真的变了才丢弃尚未提交的运动指令**；重复调用无副作用。
- **预置由 HAL 自动完成。** 从速度模式切进位置模式时，驱动器会朝陈旧的目标位置冲过去 —— 漏一次就是事故，所以这一步是 HAL 兜的。
- `read_status().mode` 来自驱动器 `0x6061` 的**实际**模式，可能与期望模式暂时不一致。

#### 4.3.4 主轴转速下发函数hal_rt_spindle_write_speed()

**函数原型**

```c
int32_t hal_rt_spindle_write_speed(HalContext *c, HalAxisId id, double rpm, int32_t dir);
```

**函数功能**

在 CSV 模式下下发转速指令。内部换算并**暂存**，到 `commit` 才下发：

```text
速度命令计数/秒 = rpm × dir × 6 ÷ command_scale(axis)
```

（乘除 6 来自「一转 360 度、一分钟 60 秒」。）

**函数参数及返回值**

| 参数名 | 类型 | 说明 |
| ------ | ---- | ---- |
| c | `HalContext*` | 已 `start` 的 context |
| id | `HalAxisId` | 逻辑轴号，**且必须是主轴** |
| rpm | `double` | 转速**大小**，须有限且 `0 <= rpm <= max_speed` |
| dir | `int32_t` | `1` = 正转（M03），`-1` = 反转（M04），`0` = 停（M05） |
| 返回值 | `int32_t` | `HAL_OK`；`HAL_ERROR_NOT_RUNNING` = 受理条件不满足；其余见第 5 章 |

**示例用法**

```c
/* begin 与 commit 之间调用 */
hal_rt_spindle_write_speed(ctx, spindle, 1000.0, 1);    /* 1000 rpm 正转 */
hal_rt_commit_cycle(ctx);                               /* 这一刻才下发 */

/* 停转 */
hal_rt_spindle_write_speed(ctx, spindle, 0.0, 0);
```

**注意**

- **`rpm` 不接受负值** —— 转向由 `dir` 单独给出。传负 rpm 会被拒绝。
- 受理条件同位置写入：本拍已 `begin` 且未 `commit`、该轴已使能、状态机已到位、**运行模式确为 CSV**。
- 本拍带出的 `at_speed` **先按指令值算**，下一拍采样时才按实测刷新 —— 不要用它当作"已经到速"的证据。
- **返回 `HAL_OK` 只代表受理并暂存。**
- **当前转速换算假设速度 PDO 的单位是计数/秒。** 使用 CSV 前须按设备手册、显示值或外部测速确认这个假设。
- `accel` 字段当前**不参与轨迹生成**，HAL 不在这里规划加减速曲线。

#### 4.3.5 主轴转速读取函数hal_rt_spindle_read_speed()

**函数原型**

```c
int32_t hal_rt_spindle_read_speed(HalContext *c, HalAxisId id, double *rpm);
```

**函数功能**

读取最近一次采样的**带符号**实际转速（rpm）。**只读快照，不触发总线访问。**

**函数参数及返回值**

| 参数名 | 类型 | 说明 |
| ------ | ---- | ---- |
| c | `HalContext*` | 已 `start` 的 context |
| id | `HalAxisId` | 逻辑轴号，**且必须是主轴** |
| rpm | `double*` | 带出实际转速，**带符号：正 = 正转** |
| 返回值 | `int32_t` | `HAL_OK`；`HAL_ERROR_NOT_RUNNING` = 还没跑过 `begin` |

**示例用法**

```c
double rpm;
if (hal_rt_spindle_read_speed(ctx, spindle, &rpm) == HAL_OK)
    printf("主轴实际转速：%.1f rpm（%s）\n", rpm, rpm >= 0 ? "正转" : "反转");
```

**注意**

- 带出的是**带符号**值，与下发时的 `dir` 口径不同（下发是 `rpm` + `dir` 两个参数）。比较时注意符号。
- 读的是最近一次 `begin` 的快照，最多滞后一拍。
- 反馈 rpm 的换算用到 `feedback_scale`；若命令与反馈当量配置不同，两者不会自动抵消。

#### 4.3.6 主轴状态读取函数hal_rt_spindle_read_status()

**函数原型**

```c
int32_t hal_rt_spindle_read_status(HalContext *c, HalAxisId id, HalCSpindleStatus *out);
```

**函数功能**

读取最近一次采样的主轴转速侧快照。

带出的 `HalCSpindleStatus` 含：

| 字段 | 含义 |
| --- | --- |
| `enabled` | 非零 = 已到 OperationEnabled |
| `at_speed` | 非零 = `|指令 − 实际| <= speed_window`（需 CSV、已使能） |
| `mode` | `HAL_SPINDLE_CSV` / `_CSP`，**来自驱动器 `0x6061` 的实际模式** |
| `command_speed` | 指令转速（rpm，带符号） |
| `actual_speed` | 实际转速（rpm，带符号） |
| `position_deg` | **累计转角**（度），**不取模** |
| `raw_status` | 原始 DS402 状态字 |

**函数参数及返回值**

| 参数名 | 类型 | 说明 |
| ------ | ---- | ---- |
| c | `HalContext*` | 已 `start` 的 context |
| id | `HalAxisId` | 逻辑轴号，**且必须是主轴** |
| out | `HalCSpindleStatus*` | 带出转速侧快照 |
| 返回值 | `int32_t` | `HAL_OK`；`HAL_ERROR_NOT_RUNNING` = 还没跑过 `begin` |

**示例用法**

```c
HalCSpindleStatus st;
if (hal_rt_spindle_read_status(ctx, spindle, &st) == HAL_OK) {
    printf("模式=%d 指令=%.1f 实际=%.1f 到速=%d 角度=%.2f°\n",
           st.mode, st.command_speed, st.actual_speed, st.at_speed, st.position_deg);
}
```

**注意**

- **`at_speed` 是判定结果，不是原始数据** —— 用实际与指令转速之差和配置的 `speed_window` 比较得出。窗口为 0 时几乎永远不会为真。
- **`position_deg` 是累计转角、不取模**，要显示 0~360 须上层自行 `fmod`（负角度还要再 `+360`）。
- 反馈角度可以跨多圈累计；相同计数坐标系下的位置命令以本拍原始反馈计数加有符号位移生成目标位型。驱动器在 32 位边界的实际转向仍须现场确认。
- `mode` 来自驱动器反馈，可能与 `request_mode` 的期望值暂时不一致。
- HAL 不生成运动轨迹，也不做速度／转矩限幅。

#### 4.3.7 主轴角度下发函数hal_rt_spindle_write_pos()

**函数原型**

```c
int32_t hal_rt_spindle_write_pos(HalContext *c, HalAxisId id, double deg);
```

**函数功能**

在 CSP 模式下下发相对既定零点的主轴单圈刻度（度）。对于“上电位置为 0°”的主轴，调用方在启动后首次采样成功、主轴尚未运动时调用一次 `hal_rt_axis_set_pos(ctx, spindle, 0)`；此操作不改编码器值，也不要求先找 Z 相。之后切 CSV/CSP 不改变该零点。正数沿正向、负数沿反向到达指定刻度；0 取最近的零刻度。方向从上一条已受理目标计算，切入 CSP 时该目标预置为当前位置；取消运动后再次就绪时，从当时反馈重建计算基准。连续重复相同普通刻度（包括小数）保留原累计目标；设零或调用通用累计位置写入接口会清除这一重复判定。`+360`/`-360` 是分别完整正转/反转一圈的特殊命令。

主轴必须在配置里带有效逻辑轴号（`logical_axis >= 0`）。这个接口不表达累计多圈目标；需要连续多圈轨迹的功能必须另定接口契约。

**函数参数及返回值**

| 参数名 | 类型 | 说明 |
| ------ | ---- | ---- |
| c | `HalContext*` | 已 `start` 的 context |
| id | `HalAxisId` | 逻辑轴号，**且必须是主轴** |
| deg | `double` | 相对既定零点的刻度，范围 [-360, 360]；符号指定到达方向，±360 为单次完整转一圈 |
| 返回值 | `int32_t` | `HAL_OK`；`HAL_ERROR_ARGUMENT` = 不是主轴；`HAL_ERROR_NOT_RUNNING` = 受理条件不满足 |

**示例用法**

```c
/* 从零点正转到 +10° 刻度；等待该命令完成 */
hal_rt_spindle_write_pos(ctx, spindle, 10.0);
/* 下一条命令：继续正转 355° 到下一圈的 +5° 刻度 */
hal_rt_spindle_write_pos(ctx, spindle, 5.0);
/* 另一条独立命令，只调用一次：从上一目标正转完整一圈 */
hal_rt_spindle_write_pos(ctx, spindle, 360.0);
```

**注意**

- `+5°` 表示沿正向到零点的 +5° 刻度，`-5°` 表示沿反向到零点的 -5° 刻度。两者不能按最短路径互换。重复普通目标不会再转一圈；重复 ±360 会再次请求完整一圈。
- **受理条件同 `hal_rt_axis_write_pos()`**：本拍已 `begin` 且未 `commit`、已使能、状态机到位、**模式确为 CSP**。
- 同一计数坐标系下，位置下发以本拍原始反馈计数加有符号位移形成 32 位目标位型；实际驱动器跨界运动方向须现场验证。命令/反馈计数坐标不同时仍走原有绝对计数换算，超出 int32 会返回参数错误。
- 主轴必须配置 `logical_axis >= 0`，不能用 `-1`（"纯主轴"）—— 否则无法经轴号寻址。

### 4.4 IO 与面板

普通 IO 与操作面板**共用同一块 PLC 映像**，都走下面两个函数，不再有独立的 MCP 周期接口。
段在映像里的位置由配置的起始**字节**地址（`x_start` / `y_start`）决定，段外保持 0。

#### 4.4.1 输入映像获取函数hal_rt_io_snapshot_inputs()

**函数原型**

```c
int32_t hal_rt_io_snapshot_inputs(HalContext *c, uint8_t *x_image, uint32_t x_len);
```

**函数功能**

把最近一次 `begin` 采到的全部 IO 与面板输入，拷进调用方的 PLC X 字节映像。

行为是**段内覆盖、段外补 0**，且会先统一清零一次，**所以调用方不必自己先清缓冲**。

**函数参数及返回值**

| 参数名 | 类型 | 说明 |
| ------ | ---- | ---- |
| c | `HalContext*` | 已 `start` 的 context |
| x_image | `uint8_t*` | 目标缓冲 |
| x_len | `uint32_t` | 缓冲长度，必须 `>=` 启动时算出的 `x_size` |
| 返回值 | `int32_t` | `HAL_OK`；`HAL_ERROR_ARGUMENT` = 缓冲不够长；`HAL_ERROR_NOT_RUNNING` = 还没跑过 `begin` |

**示例用法**

```c
uint8_t x_image[64];
if (hal_rt_io_snapshot_inputs(ctx, x_image, sizeof(x_image)) != HAL_OK) {
    /* 缓冲不够长：说明调用方与 HAL 对地址域的理解不一致 */
}
```

**注意**

- **缓冲不够长返回参数错误**，这是刻意设计 —— 通常意味着调用方和 HAL 对 X 地址域的理解不一致，静默截断会更危险。
- 只拷缓存，**不读取硬件**。
- **当前没有公开的映像长度查询函数**，调用方须按现场配置预留足够缓冲（见 4.4.2 末）。
- Entry 不保证整字节，**按低位在前连续打包**。

#### 4.4.2 输出映像下发函数hal_rt_io_flush_outputs()

**函数原型**

```c
int32_t hal_rt_io_flush_outputs(HalContext *c, const uint8_t *y_image, uint32_t y_len);
```

**函数功能**

在 `begin` 与 `commit` 之间调用，把 PLC Y 字节映像按配置的起始地址**拆成各模块的输出 Entry 值并暂存**，等 `commit` 时才随整帧下发。

**函数参数及返回值**

| 参数名 | 类型 | 说明 |
| ------ | ---- | ---- |
| c | `HalContext*` | 已 `start` 的 context |
| y_image | `const uint8_t*` | 源缓冲 |
| y_len | `uint32_t` | 缓冲长度，必须 `>=` 启动时算出的 `y_size` |
| 返回值 | `int32_t` | `HAL_OK`；`HAL_ERROR_STATE` = 不在 `begin` 与 `commit` 之间；`HAL_ERROR_ARGUMENT` = 缓冲不够长 |

**示例用法**

```c
if (hal_rt_begin_cycle(ctx) != HAL_OK) { running = 0; return; }

hal_rt_io_flush_outputs(ctx, y_image, sizeof(y_image));   /* 暂存 */
hal_rt_commit_cycle(ctx);                                 /* 这一刻才下发 */
```

**注意**

- **只能在 `begin` 与 `commit` 之间调用**，否则返回 `HAL_ERROR_STATE`。
- **本拍不调用则本拍不写任何输出**，旧值原样保留 —— 这不是"自动清零"，是"保持不动"。
- **总线按整 Entry 下发**，不提供单 bit 读改写。要只改其中几位，**由调用方自己做读改写**（先取回、改位、再整块下发）。
- **零值不一定是设备的安全态。** 请自行审核完整 Y 映像及急停后的输出行为。
- 调用方须负责完整 Y 映像的内容及每一位的点位含义 —— HAL 只搬运字节。

**关于映像长度**

X/Y 所需的最小长度在 `start` 时由配置的起始地址和实际 PDO 位长算出，但**当前没有公开的长度查询函数**。
调用方应按现场配置预留足够缓冲；缓冲短于实际需求时，两个函数都会返回 `HAL_ERROR_ARGUMENT` 而不是截断。

## 5. 返回码

**全库统一的约定**：0 表示成功，非 0 为错误码，不允许异常跨 C 边界。
第 3、4 章各函数的「返回值」行只写**该函数特有**的失败情形，本表是唯一的完整清单。

| 常量 | 数值 | 常见含义 |
| --- | ---: | --- |
| `HAL_OK` | `0` | 成功。 |
| `HAL_ERROR_STATE` | `0x0001` | 周期 `wait/begin/commit` 顺序错误；IO 输出不在写入窗口。 |
| `HAL_ERROR_NOT_RUNNING` | `0x0003` | 尚未启动、未采样，或运动所需使能/模式/状态尚未到位。 |
| `HAL_ERROR_CONFIG` | `0x0007` | 静态配置、设备字典或启动绑定配置无效；当前实现中启动绑定缺少必需 PDO 也归入此码，具体原因看 `err`。 |
| `HAL_ERROR_BUSY` | `0x0008` | 全局主站已被另一运行中的 context 占用。 |
| `HAL_ERROR_ARGUMENT` | `0x0009` | 空指针、轴号/从站位置、数值或缓冲长度无效。 |
| `HAL_ERROR_ABI` | `0x000A` | ABI 版本、结构大小或保留字段不匹配。 |
| `HAL_ERROR_STOPPED` | `0x000B` | 已请求停止，或阻塞等待被停止打断。 |
| `HAL_ERROR_MEMORY` | `0x000C` | context 分配失败。 |
| `HAL_ERROR_BUS` | `0x0101` | 主站/PDO 访问失败；周期总线故障闭锁后持续返回此码，直至 `stop`/`start`。 |
| `HAL_ERROR_TIMEOUT` | `0x0104` | 启动总预算耗尽。 |
| `HAL_ERROR_UNSUPPORTED` | `0x0105` | 预留的“不支持”错误码；当前 `start` 对绑定失败返回 `HAL_ERROR_CONFIG`，可选能力缺失由能力查询的 `state` 表示。 |

**三个容易用错的地方：**

- **两个例外是「数量」，不是「成功与否」。** `hal_axis_count()` 与 `hal_slave_count()` 成功返回**非负数量**、失败返回**负错误码**。写 `if (rc != HAL_OK)` 会把正常结果也判成失败 —— 这两处要写 `if (rc < 0)`。其余返回码函数直接返回表中正值。
- **查询类函数返回 0 不代表能力可用。** `hal_axis_capability()` 等返回 0 只说明**查询本身成功**，能力是否可用还要看 `HalCCapability.state`（见 3.3.4）。
- **`HAL_ERROR_CONFIG` 需要看 `err` 才能定位。** 字典没加载、设备类型不匹配、启动时缺少必需 PDO 都归入此码，公共返回码本身区分不出来。

错误码分段：公共 `0x00xx` · 总线 `0x01xx` · 轴 `0x20xx` · 主轴 `0x30xx` · DIO `0x40xx`。
后三段是**预留区**，当前版本没有定义具体错误码。

## 6. 构建与设备字典

公开头文件安装在 `include/`，共享库名为 `cnc_hal`。项目 CMake 构建需要 GREEMASTER SDK、`libxml2`、线程库；安装时设备字典 `devices.json` 会安装到 `${CMAKE_INSTALL_DATADIR}/cnc_hal`。从构建树运行时可设置 `CNC_HAL_DEVICES_JSON` 指向源码树中的 `src/Greemaster/devices.json`。具体编译和上机命令见 `examples/测试用例用法.md`。

当前设备字典按 vendor ID、product code、revision 匹配设备类型和既有对象映射。新增设备能否启动取决于身份匹配、设备类型、现有 PDO 及启动时的绑定检查；仅有 CoE 对象定义并不足以让本版自动映射或调用该功能。
