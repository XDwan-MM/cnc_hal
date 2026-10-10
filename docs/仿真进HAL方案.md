# 方案：把仿真驱动层从 RT 搬进 HAL

> 起手先读 `HAL/docs/续接说明.md`（现状与未决）。本文只讲一件事：**让 HAL 自己带仿真**，
> 把 RT 里那份"扮演 HAL 驱动层"的桩搬走。本文只写方案，不动代码。

## 1. 目标与非目标

**目标**

| | |
|---|---|
| G1 | 仿真实现住进 HAL，RT 不再有"扮演别人内部"的源文件 |
| G2 | RT 的 vendored 副本不再需要 `lib/cnc_hal/internal/`（9 个内部头）→ debug 与 release 包含路径一致 |
| G3 | **实时构建物理上不含仿真**（红线，见 §5） |
| G4 | HAL 自己就能跑一遍完整周期（现在是 RT 的脚手架在替它跑） |

**非目标**

- 不做"运行时切仿真"的开关 —— 环境变量一开就变假数据，机床上是事故级。
- 不改 HAL 对外契约的语义（函数、返回码、结构体含义）。
- 不动 RT 的业务逻辑：本方案只搬"驱动层替身"。

## 2. 现状（都核过，带位置）

| 项 | 现状 |
|---|---|
| 桩 | `port_work/src/hal/rt_hal_sim_backend.c`（约 400 行，2026-10-10 实测 406） |
| 它实现什么 | `ethercat_init/close`、`Master_SlaveCount / WaitCycle / CommitCycle / BusHealth / ServoRead / ServoWrite / ServoStep / IoReadEntry / IoWriteEntry / RequestStop / StopFlag` —— 13 个符号 |
| 怎么生效 | **同名符号顶替**：桩作为 .o 先占住符号，链接器就不会从 `libcnc_hal.a` 里拉真 Greemaster 实现 |
| 编进哪个档 | 只有 `debug_mode`（`CNC_RT.pro:102-108`） |
| 因此多出来的 | `INCLUDEPATH += lib/cnc_hal/internal`（9 个内部头，只有 debug 档有） |
| 故障注入 | `CNC_RT_SIM_FAULT` / `CNC_RT_SIM_FAULT_STUCK` / `CNC_RT_SIM_ERR` |
| 拓扑 | 桩自己写死 `SIM_SLAVE_PANEL/IO/SPINDLE/X/Y/Z` + `build_*_slot()` |

## 3. 为什么要搬（问题清单）

| # | 问题 | 后果 |
|---|---|---|
| P1 | RT 里有个文件在 include HAL 的内部头 | 依赖边界靠"多加一条 INCLUDEPATH"维持，review 时容易被当成越界 |
| P2 | 桩与 HAL 内部类型是**隐式耦合** | HAL 内部一改名，RT 的桩就编不过；编译器不会告诉你"谁在依赖" |
| P3 | 仿真只能被 RT 使唤 | HAL 的 ctest 只有 4 条配置/契约用例，跑不了完整周期 |
| P4 | 从站拓扑有**两份真相** | 桩的 `SIM_SLAVE_*` 与 `rt_hal_config.c` 的 `kBindings` 必须手工保持一致（现在只靠注释提醒） |
| P5 | 故障注入开关挂在 RT 名下 | 它注入的是 HAL 语义的故障，名字却叫 `CNC_RT_SIM_*` |

## 4. 设计

### 4.1 两种做法，先做 A

**A（推荐，改动最小）**：保持"同名符号顶替"，只把**实现搬进 HAL** 并编成独立库。

```
HAL/src/driver/greemaster/   ← 真实现（现在编进 libcnc_hal.a）
HAL/src/driver/sim/          ← 仿真实现（编成 libcnc_hal_sim.a）
```

- HAL 内部代码**一行不改**（还是调 `Master_*`）。
- 两个库**符号同名**，链接时二选一 —— 与现在完全相同的机制，只是产出方从 RT 变成 HAL。
- RT 侧：debug 档链 `libcnc_hal_sim.a`，实时档链 `libcnc_hal.a`。

**B（后续可选）**：把"驱动层"正式定义成函数表（`HalDriverOps`），HAL 内部改调表。
更干净，但要改 HAL 里所有调 `Master_*` 的地方；A 已经能消掉 P1/P2/P3/P5，所以 **B 等 A 稳定后再评**。

### 4.2 拓扑单一来源（顺带解掉 P4）

桩现在写死一份从站表，因为 `ethercat_init(const MasterConfig*)` 拿不到轴配置。二选一：

- **S1（推荐，一次做对）**：HAL 在 `hal_context_start()` 里把 `HalCConfig` 也交给驱动层（sim 用它，真实现忽略）。
  sim **从配置生成**槽表 → 拓扑只剩一份真相。
- **S2（改动更小）**：桩仍自造表，但把"与 `kBindings` 一致"做成**启动断言或 HAL 用例**
  （断言 sim 提供的槽位覆盖配置要求的每个 `(slave_pos, axis_index)`）。

### 4.3 构建与产物

| 产物 | 内容 | 谁链 |
|---|---|---|
| `libcnc_hal.a` | HAL + 真 Greemaster 驱动层 | 实时构建、HAL 自己的用例 |
| `libcnc_hal_sim.a` | HAL 的仿真驱动层 | 只有离线回归档 |

`build_all.sh` 出两个；`sync.sh` 拷两个；`SYNCED.txt` 多加一行 sim 库的 sha256。

### 4.4 版本与 ABI

- 接口放**内部头** → `HAL_C_ABI_*` **不动**（推荐）。
- 若哪天做 B（函数表进公共头）→ 那是一次 MINOR/MAJOR 变更，按 `hal_types.h` 的版本策略走，
  并同步 RT 侧那道编译期闸门。

### 4.5 RT 侧改动清单

- `CNC_RT.pro`：`debug_mode` 档把 `-lcnc_hal` 换成 `-lcnc_hal_sim`；**删掉** `INCLUDEPATH += lib/cnc_hal/internal`。
- 删 `src/hal/rt_hal_sim_backend.c`。
- `sync.sh`：不再拷 `internal/`（那 9 个头留在 HAL 仓里），改拷 sim 库。
- 注入开关改名：`CNC_RT_SIM_*` → `CNC_HAL_SIM_*`。
- **测试脚本里 grep 的仿真日志文本**（`[SIM] ...`、`注入 Fault 态` 等）跟着改。

### 4.6 用例与检查

**HAL ctest 新增**

1. sim 全周期：init → start → N 拍 → 轴使能 → 写位置 → 读回一致 → stop；
2. 故障复位收口（DONE 与 TIMEOUT 两条路径）；
3. 总线错误注入（`Master_BusHealth` 报错 → HAL 的 cycle_error）；
4. 主轴定向（sim 的 `SIM_ORIENT_DELAY` 那条）。

**保留不动**：RT 的离线配置自检（45 项）+ 5 个冒烟场景 + 主轴回归 —— 它们验的是业务链，仍归 RT。

**新增一条守卫（防"发布件带仿真"）**：两侧各留一个**只属于自己的标记符号**，用 nm 断言互斥：

```bash
# sim 库里有标记；真库里必须没有
nm -g --defined-only lib/cnc_hal_sim.a        | grep -q hal_sim_backend_marker
! nm -g --defined-only lib/cnc_hal/libcnc_hal.a | grep -q hal_sim_backend_marker
```

## 5. 红线

**实时构建物理上不能含仿真。**

- 现在是编译期保证（那个文件不进该档）；搬完之后必须由**库的选择**保证（实时档只链 `libcnc_hal.a`）。
- **不许**加"环境变量切仿真"的口子。
- 发布件要有自动检查盯着（§4.6 的 nm 断言）。

## 6. 阶段与验收

| 阶段 | 做什么 | 验收 |
|---|---|---|
| 0 | HAL 推到 gitea（ABI 7.0 是破坏性的，先跟组里打招呼） | 别人 clone 能拉到 |
| 1 | sim 源码搬进 HAL；编出两个库；补 HAL 自测用例 | HAL ctest 全绿（4 + 新增）；**RT 一行未改**，现有离线回归照旧全绿 |
| 2 | RT 切链 sim 库；删桩与 `internal`；改注入开关名 | 两档都能编；离线自检 45/45；5 冒烟 + 主轴全 PASS |
| 3 | 收尾：`sync.sh` / `SYNCED.txt` 清理、文档更新 | 三仓干净；文档与新事实一致 |

每阶段**独立提交、可单独回滚**；阶段之间不合并提交。

## 7. 风险与对策

| 风险 | 对策 |
|---|---|
| 实时档误链 sim 库 | §4.6 的 nm 断言 + `.pro` review；发布前跑一次 |
| 两库符号冲突（链接顺序问题） | 与现在同机制，已验证；把"链哪个库"写进构建脚本，不靠链接顺序 |
| 桩的行为与真驱动器漂移 | sim 的 DS402 应答用 HAL 用例锁住；真机验证后把差异回填 |
| HAL 用例变多、维护负担上升 | 只锁"契约 + 全周期 + 故障三条"，不追求覆盖率 |
| 搬运过程中 RT 业务回归失效 | 阶段 1 明确要求"RT 一行未改"，用现有 5 冒烟 + 主轴做对照 |

## 8. 开工前要拍的三件事

1. 驱动层接口放**内部头**（推荐，ABI 不动）还是公共头（那就是做 B）？
2. 产物命名 `libcnc_hal.a` / `libcnc_hal_sim.a` 行不行？
3. 拓扑来源走 S1（配置生成）还是 S2（断言一致）？
