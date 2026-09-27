# CNC HAL C API 接口文档

适用范围：当前 `master` 分支的公开 C ABI 2.3（`HAL_C_ABI_MAJOR=2`，`HAL_C_ABI_MINOR=3`）。本文以 `include/` 中的声明及 `src/internal/` 中的实现为准。厂商 SDK、`src/Greemaster/` 内部函数和 `examples/` 测试辅助函数不属于公开 API。

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

### `HalCConfig`

| 字段 | 要求与含义 |
| --- | --- |
| `abi_major`、`abi_minor`、`struct_size` | 分别等于 `HAL_C_ABI_MAJOR`、`HAL_C_ABI_MINOR`、`sizeof(HalCConfig)`；必须精确匹配。 |
| `reserved`、`topology_fingerprint` | 当前均必须为 `0`；拓扑指纹功能尚未启用。 |
| `cycle_us` | 主站周期，至少 `250` 微秒。 |
| `start_timeout_ms`、`cycle_timeout_ms` | 非零；前者是整个启动过程的总预算，后者用于周期等待。 |
| `dc_enable` | `0` 或 `1`；运行期间配置不变。 |
| `axis_count` / `axes[]` | 进给轴配置数与数组，最多 `HAL_C_MAX_DEV`；进给轴只支持 `HAL_WORK_POSITION`。 |
| `spindle_count` / `spindles[]` | 主轴配置数与数组，最多 `HAL_C_MAX_SPINDLE`；主轴也占逻辑轴号。 |
| `io_count` / `ios[]` | 普通 IO 配置数与数组。 |
| `panel_count` / `panels[]` | 面板配置数与数组。 |

四类配置的**总数**不得超过 `HAL_C_MAX_DEV`（当前为 30）。数组只使用各自前 `count` 项。`slave_pos` 是实际扫描位置，从 0 起；同一物理轴 `(slave_pos, axis_index)` 不能重复绑定，IO/面板不能与伺服重复占用从站。

### `HalCAxisCfg` 与 `HalCSpindleCfg`

| 字段 | 要求与含义 |
| --- | --- |
| `slave_pos`、`axis_index` | 从站位置与从站内物理轴序号；后者从 0 起。 |
| `logical_axis` | 对外的逻辑轴号 `0..31`，不得重复；进给轴可填 `-1` 表示不可经轴号寻址，主轴必须 `>=0`。`HalAxisId` 就是这个号码，不是数组下标。 |
| `work_mode` | 进给轴为 `HAL_WORK_POSITION`（CSP）；主轴可配置 `HAL_WORK_POSITION` 或 `HAL_WORK_VELOCITY`（CSV）。 |
| `estop_action` | 必须显式选 `HAL_ESTOP_DISABLE_OPERATION` 或 `HAL_ESTOP_DISABLE_VOLTAGE`。 |
| `encoder_type` | `HAL_ENC_INCREMENTAL_Z` 或 `HAL_ENC_ABSOLUTE`。 |
| `feedback_pulses_per_rev`、`feedback_wrap` | 前者须大于 0；后者选 `HAL_WRAP_LINEAR` 或 `HAL_WRAP_MODULAR`。模循环展开假定相邻采样位移不超过半个计数周期。 |
| `command_units_per_count`、`feedback_units_per_count` | 有限且大于 0；命令与反馈分别换算，可不同。进给轴为自定用户单位/计数，主轴位置按度/计数使用。 |
| `command_invert`、`feedback_invert` | 非零表示对应方向反转；命令和反馈可独立设置。 |
| `enc_off` | 有限的已保存坐标偏置；初始显示坐标为原始反馈换算值减去该偏置。 |
| `HalCSpindleCfg.max_speed` | 主轴最高转速，rpm，有限且大于 0。 |
| `HalCSpindleCfg.speed_window` | 主轴到速判定窗口，rpm，有限且不小于 0。 |
| `HalCSpindleCfg.accel` | 当前仅校验为有限且不小于 0；**尚不生成加减速曲线**。 |

`HalCIoCfg`、`HalCPanelCfg` 的 `x_start`/`y_start` 是 PLC X/Y 映像的**字节**起始地址，须非负；面板另有 `panel_type`，由上层解释。静态校验检查地址合法性，实际段长度及互不重叠在 `start` 绑定 PDO 后检查。X 与 Y 是各自独立的地址域。

## 3. 配置、生命周期与查询接口

除特别说明外，下列 `int32_t` 函数成功返回 `HAL_OK`（0），失败返回 `HAL_ERROR_*`。`err` 可以为 `NULL`；传入非空缓冲和正长度时，返回的诊断文本以 NUL 结尾。

| 函数 | 用途与关键约束 |
| --- | --- |
| `hal_config_validate(const HalCConfig *config, char *err, uint32_t err_len)` | 检查 ABI、设备数量、静态绑定、数值范围等；不访问主站，**不能**证明实际拓扑/PDO 可用。 |
| `hal_error_text(int32_t code, char *out, uint32_t out_len)` | 把返回码转成简短中文文本；缓冲为空或长度为 0 时不写入。详细启动原因应看 `start` 的 `err`。 |
| `hal_context_create(const HalCConfig *config, HalContext **out, char *err, uint32_t err_len)` | 静态校验、复制配置、分配 context；失败时将有效的 `*out` 置 `NULL`。 |
| `hal_context_start(HalContext *c, char *err, uint32_t err_len)` | 启动全局主站、读取实际从站、按配置绑定轴和 IO，校验必需 PDO 与映像段。失败会回滚；成功后才允许周期与身份查询。一个进程同时只能有一个运行中的 context。 |
| `hal_context_request_stop(HalContext *c)` | 原子设置停止请求，可与周期线程并发；只通知，不释放主站资源。 |
| `hal_context_stop(HalContext *c)` | 关闭主站并清除运行状态；可重复调用。调用前须等所有周期调用退出。之后可用同一 context 再次 `start`，旧快照、请求、坐标偏置和故障闭锁不会保留。 |
| `hal_context_destroy(HalContext *c)` | `stop` 后释放 context；允许 `NULL`。若尚有周期线程使用该句柄，必须先等待其退出。 |
| `hal_axis_resolve(const HalContext *c, int32_t logical_axis, HalAxisId *out)` | 校验逻辑轴号存在并返回同一个号码；失败时若 `out` 非空，置 `HAL_INVALID_ID`。可在 `create` 后使用。 |
| `hal_axis_count(const HalContext *c)` | 返回可寻址轴数，包含主轴，不含 `logical_axis=-1` 的进给轴；空句柄返回负的 `HAL_ERROR_ARGUMENT`。 |
| `hal_device_identity(const HalContext *c, HalAxisId id, HalCIdentity *out)` | 返回已绑定轴的设备身份；需 `start` 成功。 |
| `hal_axis_capability(const HalContext *c, HalAxisId id, uint32_t function, HalCCapability *out)` | 查询轴的 `HAL_FUNC_POSITION`、`HAL_FUNC_SPEED` 或 `HAL_FUNC_ERROR_CODE`；需已启动。函数返回 0 仅表示**查询成功**，还须看 `out->state`。 |
| `hal_slave_count(const HalContext *c)` | 返回扫描到的实际从站数，含未配置/未识别从站；未启动或空句柄返回负错误码。 |
| `hal_slave_info(const HalContext *c, int32_t slave_pos, HalCSlaveInfo *out)` | 按 0 起的扫描位置查实际从站身份和已装配物理轴数；需已启动。 |
| `hal_slave_capability(const HalContext *c, int32_t slave_pos, uint32_t function, HalCCapability *out)` | 当前仅接受 `HAL_FUNC_ALARM_CONTROL`；需已启动。始终报告 `HAL_CAP_NOT_CONFIGURED`，不提供报警控制执行接口。 |

`HalCIdentity` 给出类型、vendor/product/revision/serial、字典名称及同 vendor+product 的扫描序号 `family_index`；`device_id` 在 serial 非零时取 serial，否则按 vendor/product/revision 计算哈希。`HalCSlaveInfo` 还含 `slave_pos` 和已装配物理轴数。未识别的从站仍可见身份数字，但不因此自动获得功能。

### 能力查询的边界

`HalCCapability` 包含 `function`、`state`、`slave_pos`、`axis_index` 和 `reason`。`state` 分别为 `HAL_CAP_READY`、`HAL_CAP_UNSUPPORTED`、`HAL_CAP_NOT_CONFIGURED`。

- 位置功能依赖启动时对 DS402 必需 PDO 的绑定；缺少必需项会使整个 `start` 失败，因此成功启动的已配置轴查询位置为 `READY`。
- 速度功能要求实际速度与目标速度 PDO 都绑定；主轴缺失它们会导致 `start` 失败，进给轴则可返回 `UNSUPPORTED`。
- 故障码功能要求 `0x603F` 已绑定；缺少时返回 `UNSUPPORTED`，轴状态中的 `error_code_valid=0`。
- `HAL_FUNC_ALARM_CONTROL` 是从站级预留能力，当前统一为 `NOT_CONFIGURED`。库没有写 `0x6FFF` 的公开函数。

这些结果来自现有设备字典及启动时装配的 PDO 句柄。当前 `master` **不解析 ESI，也不按 ESI 自动配置 PDO**；查询结果不等同于对全部 CoE 字典对象的在线核验。

## 4. 周期接口与线程约束

单个周期必须按下面的顺序执行；`wait` 是这组三拍中唯一的阻塞点：

```text
wait_cycle → begin_cycle → [read_* / write_* / IO 操作] → commit_cycle
                           ↑ 本拍快照          ↑ 暂存输出       ↑ 真正提交
```

| 函数 | 用途与关键约束 |
| --- | --- |
| `hal_rt_wait_cycle(HalContext *c)` | 等待下一主站周期；上一拍必须已 `commit`。 |
| `hal_rt_begin_cycle(HalContext *c)` | 采样各轴和 IO 输入、推进 DS402 状态机；必须先 `wait`。读接口此后取得本拍快照。 |
| `hal_rt_commit_cycle(HalContext *c)` | 下发暂存的轴指令和 IO 输出，提交本拍；必须先 `begin`。 |

周期调用应由同一个 RT 线程串行执行。`read_*` 读最近一次 `begin` 的快照，不主动读取总线；同一拍先写后读，读不到本拍刚写的物理结果。`write_*` 与 IO 输出先暂存，到 `commit` 才下发。`begin` 前没有快照时，读状态返回 `HAL_ERROR_NOT_RUNNING`。`request_stop` 可跨线程通知；其余 context 生命周期调用须串行，`stop`/`destroy` 之前须等待周期线程退出。

### 进给轴及通用轴

| 函数 | 用途与关键约束 |
| --- | --- |
| `hal_rt_axis_enable(c, id, on)` | `on` 只能为 0/1；申请使能或去使能。`on=0` 停在 DS402 SwitchedOn，不等于断电。需经过后续 `begin` 确认状态，才能下发运动。 |
| `hal_rt_axis_estop(c, id)` | 取消未提交运动，按该轴 `estop_action` 申请去使能或撤销电压；恢复需重新使能并确认。总线已故障时仍可记录停止意图，返回成功不保证失联设备执行。 |
| `hal_rt_axis_write_pos(c, id, pos)` | CSP 目标位置，用户单位；仅在本拍 `begin` 后、`commit` 前，且使能、模式与状态机均到位时受理。非有限值或换算超出 32 位计数返回参数错误。主轴也可用此通用位置接口。 |
| `hal_rt_axis_read_pos(c, id, &pos)` | 读取最近采样的实际位置，用户单位。 |
| `hal_rt_axis_read_status(c, id, &status)` | 读取最近轴状态快照。 |
| `hal_rt_axis_set_pos(c, id, pos)` | 将当前反馈位置重定义为 `pos`；只调整 HAL 的反馈/命令坐标基准，不写驱动器或触发运动；至少要有一次 `begin` 采样。 |

`HalCAxisStatus` 含 `enabled`、`actual_pos`、`command_pos`、原始 DS402 状态字 `raw_status`、`error_code` 与 `error_code_valid`。当前 `position_valid` 恒为 1，**不能据此认定机械坐标已经回零或经过独立校准**。`set_pos` 只影响当前 context；重启后恢复配置中的 `enc_off` 基准。

### 主轴

| 函数 | 用途与关键约束 |
| --- | --- |
| `hal_rt_spindle_enable(c, id, on)` | 主轴使能/去使能，要求 `id` 对应主轴；`on` 为 0/1。 |
| `hal_rt_spindle_estop(c, id)` | 主轴急停，动作由其 `estop_action` 决定。 |
| `hal_rt_spindle_request_mode(c, id, mode)` | 请求 `HAL_SPINDLE_CSV` 或 `HAL_SPINDLE_CSP`；实际切换在后续周期推进，不会自动重新使能。 |
| `hal_rt_spindle_write_speed(c, id, rpm, dir)` | CSV 目标转速；`rpm` 有限且在 `0..max_speed`，`dir` 为 `-1/0/1`，分别表示反转/停/正转。只在本拍 `begin` 与 `commit` 之间、使能且 CSV 到位时受理。 |
| `hal_rt_spindle_read_speed(c, id, &rpm)` | 读取最近采样的带符号实际转速，单位 rpm。 |
| `hal_rt_spindle_read_status(c, id, &status)` | 读取最近主轴快照，含模式、转速、到速判断和累计角度。 |
| `hal_rt_spindle_write_pos(c, id, deg)` | 主轴 CSP 目标角度，单位度；不取模，可发送累计角度；受理条件同轴位置写入。 |

`HalCSpindleStatus.mode` 来自驱动器 `0x6061`；`at_speed` 用实际与命令转速之差和配置窗口判断。当前转速换算假设速度 PDO 为计数/秒；使用 CSV 前须按设备确认命令、反馈当量和 PDO 单位。HAL 不生成运动轨迹或速度/转矩限幅。

### IO 与面板

| 函数 | 用途与关键约束 |
| --- | --- |
| `hal_rt_io_snapshot_inputs(c, x_image, x_len)` | 将最近一次 `begin` 采到的 IO/面板输入拷入 PLC X 字节映像；段外清零。缓冲须覆盖启动时算出的完整 X 映像。 |
| `hal_rt_io_flush_outputs(c, y_image, y_len)` | 在 `begin` 与 `commit` 之间，把 PLC Y 映像拆成各输出 Entry 并暂存；缓冲须覆盖完整 Y 映像。本拍不调用则保持旧输出。 |

X/Y 所需最小长度在 `start` 时由配置的起始地址和实际 PDO 位长计算，当前没有公开的长度查询函数；调用方应按现场配置预留足够缓冲。Entry 内按低位在前打包。输出按**整 Entry** 写入，不提供单 bit 读改写；调用方须负责完整 Y 映像及点位含义。零值不一定是设备的安全态。

## 5. 返回码

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

`hal_axis_count()`、`hal_slave_count()` 返回非负数量，失败返回**负错误码**；其余返回码函数直接返回表中正值。检查能力时区分 API 调用结果和 `HalCCapability.state`。

## 6. 构建与设备字典

公开头文件安装在 `include/`，共享库名为 `cnc_hal`。项目 CMake 构建需要 GREEMASTER SDK、`libxml2`、线程库；安装时设备字典 `devices.json` 会安装到 `${CMAKE_INSTALL_DATADIR}/cnc_hal`。从构建树运行时可设置 `CNC_HAL_DEVICES_JSON` 指向源码树中的 `src/Greemaster/devices.json`。具体编译和上机命令见 `examples/测试用例用法.md`。

当前设备字典按 vendor ID、product code、revision 匹配设备类型和既有对象映射。新增设备能否启动取决于身份匹配、设备类型、现有 PDO 及启动时的绑定检查；仅有 CoE 对象定义并不足以让本版自动映射或调用该功能。
