# GreeMaster 驱动层

HAL 内部的主站驱动层。源码原本是封装侧的独立包（`DEMO_LIB_AUTO_0920`），2026-09-21
被吸收进 HAL 工程并改造。**原始副本仍留在 `/home/mxh/CNC/DEMO_LIB_AUTO_0920` 作对照。**

这一层的职责：**把 EtherCAT 总线上的东西，变成 HAL 能消费的设备模型。**

---

## 一、文件地图

| 文件 | 干什么 | 相对原代码 |
|---|---|---|
| `main_demo.c/.h` | 启动序列、握手、字典加载；`StopFlag` / `RequestStop` | 原有，去线程/去信号，加握手与建表 |
| `device.c/.h` | 读 EEPROM 定身份、查字典定类型、按类型装配 Entry 句柄 | 原有，类型识别改查字典、`servo_addr_config` 重写 |
| `slave_list.c/.h` | PDO 映射链表（SM / PDO / Entry 三级） | 分配失败返回错误，支持局部链清理 |
| `device_table.c/.h` | **按槽设备表** —— 本层对 HAL 的取数口 | 新增 |
| `servo_step.c/.h` | DS402 推进一拍 + 预置目标位置 | 新增 |
| `devices.json` | 设备字典数据 | 新增（由硬编码表转来） |
| `export.h` | `MASTER_API` 导出标记 | 新增 |
| ~~`cJSON.c/h`~~ | **已删** —— 3302 行死代码，没有任何 `.c` 调它 | 删除 |
| ~~`master_hal.h`~~ | **已删** —— 它描述的 `Master_Start`/`Master_Get_Device_Info`/`Master_Close` 三个函数在源码里根本不存在 | 删除 |

### 依赖方向：本层只依赖 `src/common/`

```
              src/common/            ← 契约：纯定义 + 纯函数，无 IO，谁都不依赖
              ├── Ds402.h            DS402 编码 + 推进规则（纯函数，可离线穷举测试）
              └── devdict.c/.h       设备字典读取器（自包含 C99，无 cJSON）
                    ↑                          ↑
            src/Greemaster/            src/internal/ + src/Dio/
            （本层）                    （HAL 内核）
```

**本层不 include HAL 内核的任何东西**——两边都只指向 `common/`，谁都不指向谁。

这样这一层可以**单独交付**：把 `Greemaster/` + `common/` 打成一个包，谁都能用，
不需要 HAL 存在。（`Ds402.h` 和 `devdict` 本来一度放在 HAL 内核那边，是搬过来的——
那样会让驱动层反过来依赖 HAL，方向错了。）

---

## 二、启动流程（`ethercat_init()` 实测顺序）

```
GM_Resource_Allocation            申请主站资源
GM_Get_Version                    取主站版本
Err_Fun_Register(err_call_back)   注册报错回调（出错时置 stop 标志）
GM_Master_Init                    主站初始化
[GM_Master_Param_Change]          改主站参数        ← PARAM_CHANG 宏，默认关
GM_Master_Start                   主站开始
GM_Slave_Num_Get                  取从站数

┌─ ★ DevDict_Load(DEVICES_JSON_PATH)   加载设备字典
│  ★ get_device_info_from_eeprom()      逐台读 EEPROM：解析身份 → 查字典定类型
│                                        → 存进 g_slave_identity[]（后面要用）
│  GM_Calculate_Config_Info()           算 PDO 映射 → slave_list
│  ★ device_match()                    按类型装配，填 g_device_data[槽]
└─ ★ DeviceTable_Build(slave_num)      建按槽设备表

GM_Master_Set_Cycle(cycle_us)     设通讯周期        ← MasterConfig
[GM_DC_Enable]                    使能 DC 信号      ← MasterConfig.dc_enable
GM_Reg_Resource_Init              寄存器资源        ← 驱动内部申请
GM_Sdo_Datagram_Enable            SDO 使能          ← 驱动内部申请
GM_Config_Download_And_Active     FMMU/PDO/SM 下发并激活
GM_Master_Wait_OP(remaining_s)    等主站进入 OP     ← 启动预算剩余秒数（向上取整）

┌─ ★ master_handshake()               握手前收一帧 → IO 资源扫描 → 握手前发一帧
└─ ★ GM_Master_OP_Valid()             ★ 握手。成功后主站进入周期收发模式
```

`ethercat_init()` 返回时，**「周期收发模式可用」**——不是「主站到 OP 为止」。

周期循环由实时端创建（HAL 不创建线程）：每拍调用 Master_WaitCycle()，
推进各轴 Master_ServoStep()、读写数据，再调用 Master_CommitCycle()。

---

## 三、数据模型：三个层次，别搞混

这是这一层最容易读错的地方。

### 1. 从站（slave）—— 总线上的物理设备

`slave_list` 是它的 PDO 映射链表（SM → PDO → Entry 三级）。一台从站**可以含多根轴**。

### 2. 槽（slot）—— `g_device_data[slot]`，**一根轴或一台词设备**

一台从站在数组里占几个槽，由 `device_slot_count()` 决定：

```
SERVO_TYPE  → 1 槽        GREE_AXIS6_TYPE → 6 槽
GREE_AXIS4_TYPE → 4 槽    CONTROL_PANEL / IO_MODEL → 1 槽
IO_EXPANSION / UNKNOWN → 0 槽（不装配）
```

**槽号 ≠ 从站号。** 这个规则在 `COERequestResult()`（推进）、`device_match()`（推进）、
`DeviceTable_Build()`（推进）**三处共用同一个函数**——各写一遍就会整体错位，而且不报错。
这是这一层以前踩过的坑。

每个槽里是一个 union，按设备类型取用：

```c
typedef union {
    slave_addr   slave;     /* 伺服：11 个 DS402 Entry 句柄 */
    control_addr control;   /* 面板：output_addr[64] / input_addr[64] + out_count/in_count */
    io_addr      io;        /* IO：  io_output_addr[64] / io_input_addr[64] + io_out_count/io_in_count */
} device_data_t;
```

容量上限为 30 台从站、合计 30 个设备槽；多轴设备必须完整容纳，超限直接报错。
IO/面板按方向遍历所有启用 SM 和 PDO，每方向最多 64 个 Entry，按出现顺序绑定。
PDO 解析上限仍为每方向 10 个 PDO、每 PDO 20 个 Entry，超限拒绝，不截断。

**光看字节分不出是哪种**——所以需要下面的设备表。

### 3. 设备表（`DeviceSlot`）—— 按槽索引，带 tag

这是**对 HAL 的取数口**。每个槽一条，带：设备类型（tag）、从站号、EEPROM 身份、
字典里的名称、Entry 句柄快照。HAL 拿它建自己的设备对象。

```c
typedef struct {
    DEVICE_TYPE   type;        /* tag —— union 缺的就是这个 */
    int           slave_pos;
    uint32_t      vendor_id, product_code, revision, serial;
    char          name[64];
    device_data_t entries;     /* 句柄快照 */
} DeviceSlot;
```

---

## 四、设备类型从哪来：设备字典

`devices.json` + `devdict.c`，按 `(vendor_id, product_code, revision)` 匹配。

```json
{ "vendor_id": 477, "product_code": 271601776, "revision": "*",
  "type": "servo", "profile": "ds402", "name": "Delta Servo Driver" }
```

- **`type`**：`servo` / `gree6` / `gree4` / `panel` / `io` ——决定装配走哪条路
- **`profile`**：对象映射策略
  - `ds402` —— 用内置标准对象号（`kDs402Std`，10 个）
  - `custom` —— 用本条 `objects` 段（语义角色 → 实际对象号）
  - `none` —— 本字典不提供对象映射（面板/IO 按位置建句柄，非标多轴在代码里）

**加一台新设备只改 JSON，不改代码**——这是换字典的目的。
格力 axis6/axis4 是例外：多轴的对象号按轴偏移（`0x6041 + 2048*i`、PDO `0x1A00 + 16*i`），
现有字典格式表达不了，所以留在 `servo_addr_axis6_config` / `axis4_config` 里。

字典路径由 `DEVICES_JSON_PATH` 宏指定（默认指向源码树那份，**部署时要覆盖**）。
**加载失败不致命**——清空旧字典、打警告、继续跑、所有设备判 UNKNOWN_TYPE（装配会全部落空）。
字典要求 version=1；拒绝非法数值、重复匹配键、未知字段与尾随垃圾。
custom 伺服必须提供实际模式 mode_display，对象位宽及收发方向须匹配语义角色。

---

## 五、对 HAL 的接口（15 个）

`export.h` 的 `MASTER_API` + 编译时 `-fvisibility=hidden`，把其余 70 多个内部符号全挡住。

### 启动配置：`MasterConfig`

机床/现场相关的参数**不再写死在编译期**，由 `ethercat_init()` 的入参传进来：

```c
typedef struct {
    uint32_t cycle_us;        /* 通讯周期（us），下限 250 */
    uint32_t start_timeout_ms; /* 整个启动过程的总预算 */
    uint32_t cycle_timeout_ms; /* 周期等待超时，转为 SDK 秒数 */
    int      dc_enable;       /* 是否使能 DC 信号（分布式时钟） */
} MasterConfig;

// 不知道填什么：MasterConfig cfg = MASTER_CONFIG_DEFAULT;
```

周期非法（< 250us）或 cfg 为空时，在任何 GM 调用之前返回失败。
EEPROM 解析、句柄绑定或握手失败会回滚资源并清空设备表；运行中再次 init 返回失败。
close 幂等，保留首个清理错误；下次 init 重置停止标志、句柄、健康快照及模式切换策略。
生命周期调用须串行，close 前由调用方退出全部周期调用。

```c
// —— 生命周期 ——
int  ethercat_init(const MasterConfig* cfg);  // 启动到「周期收发模式可用」
int  ethercat_close(void);

// —— 设备模型 ——
int  DeviceTable_Get(const DeviceSlot** out); // ★ 设备表：HAL 建对象的原料

// —— 周期原语（只在实时线程调）——
int  Master_WaitCycle(void);                  // 等节拍 + 收一帧（阻塞点）
int  Master_CommitCycle(void);                // 下发本帧

// —— 伺服：DS402 推进（含模式保证）——
// req 是每拍传进来的，本层不持有：「要不要使能」是上层策略，不是硬件事实
//   DS402_REQ_ENABLE / _DISABLE / _DROP_VOLTAGE / _FAULT_RESET / _NONE
// 期望模式也对不上时会自动切——调用方每拍只说「我要哪个模式」
int  Master_ServoStep(int slot, Ds402Request req, uint16_t op_mode, uint16_t* sw);

// 切模式策略（设备属性，默认 DISABLE_FIRST = 先下使能，规范明确支持）
int  Master_ServoSetModeSwitch(int slot, Ds402ModeSwitch how);
int  Master_ServoGetModeSwitch(int slot, Ds402ModeSwitch* out);

// —— 伺服：按语义角色读写（进出的是硬件原始值，换算归上层）——
int  Master_ServoRead (int slot, DevDictRole role, uint32_t* out);
int  Master_ServoWrite(int slot, DevDictRole role, uint32_t value);

// —— 词设备（面板 / IO 模块）：按 Entry 序号读写 ——
// Entry 位长 1~32，值右对齐；读要把位长带出来（HAL 打包字节图要用）
int  Master_IoReadEntry (int slot, int index, uint32_t* value, int* bit_length);
int  Master_IoWriteEntry(int slot, int index, uint32_t value);

// —— 状态、健康与打断 ——
int  Master_StopFlag(void);                   // 主站出错/已停机？
int  Master_BusHealth(MasterBusHealth* out);  // 最近一次收帧的警告与计数
void Master_RequestStop(void);                // 原子停止通知，不释放、不保证唤醒 SDK
```

### 关于 `Master_ServoStep`

读状态字 → 按 `Ds402_NextStepReq()`（纯函数，在 `src/common/Ds402.h`）算这一拍该做什么
→ 写控制字 / 运行模式 / 目标位置。**不含任何策略**——往哪推由 `req` 传进来。

**请求是每拍传的，本层不持有。** 「要不要使能」是上层策略，不是硬件事实；HAL 那边
本来就要实现 `enable(bool)` 的幂等语义，状态存在那里就够了。同一个请求要一直传，
直到状态字显示已达成为止（那时返回 `STEP_NONE`，什么都不写）。

梯子只有一份，**不分进给轴和主轴**；刚性攻丝就是换个模式参数：

```c
Master_ServoStep(feedSlot,    DS402_REQ_ENABLE, DS402_MODE_CSP, &sw);   // 进给轴
Master_ServoStep(spindleSlot, DS402_REQ_ENABLE, DS402_MODE_CSV, &sw);   // 主轴常态
Master_ServoStep(spindleSlot, DS402_REQ_ENABLE, DS402_MODE_CSP, &sw);   // 主轴攻丝

Master_ServoStep(slot, DS402_REQ_DISABLE,      0, &sw);   // 去使能（停在 SwitchedOn）
Master_ServoStep(slot, DS402_REQ_DROP_VOLTAGE, 0, &sw);   // 断电
Master_ServoStep(slot, DS402_REQ_FAULT_RESET,  0, &sw);   // 清错（清完记得改回去）
```

### 模式切换也由梯子保证

`Master_ServoStep` **不只推使能，还保证模式**：`0x6061`（实际模式）与期望不符时自动切，
使能请求必须成功读取实际模式，缺句柄或读取失败直接报错。
任何 PDO 读写失败都会停止后续写入；需要预置时，先读取实际位置、写入目标位置，再改模式或控制字。
去使能、断电和清错不依赖模式反馈可用。

怎么切由 `Master_ServoSetModeSwitch()` 选：

| 策略 | 路径 | 代价 |
|---|---|---|
| `DS402_MODESW_DISABLE_FIRST`（**默认**） | `0x237` → 写 `0x07` 退回 SwitchedOn → `0x23` 改 `0x6060` → 再上使能 | 切换期间**短暂失力矩** |
| `DS402_MODESW_IN_PLACE` | `0x237` 直接写 `0x6060` | 省一进一出，但**能否生效取决于驱动器** |

**默认选前者**，因为 CiA 402 明确支持「下使能状态下改 `0x6060`」，不依赖任何未验证的假设；
而且切模式那一刻主轴本来就该是停的（刚性攻丝的时序就是：主轴停 → 切位置模式 → 上使能 → 与 Z 同步）。

`IN_PLACE` 要**上机确认驱动器真的支持**再打开——有的驱动会等到下次进 OperationEnabled 才真正切换。

**切模式时自动预置目标位置**（`0x607A` ← `0x6064`）。从速度模式切进位置模式时，驱动会朝
上一次的目标位置冲过去——那是飞车的经典路径，漏一次就是事故。

### 关于 `Master_BusHealth`

返回最近一次收帧时主站报的警告与计数（PDO/DC 警告、CRC 与帧超时累计、期望工作计数器）。
在握手那一帧和每次 `Master_WaitCycle()` 时更新。

**只记录不判断** —— 这里不判严不严重、不报警，报警策略归上层。
也**刻意没暴露 GM 的 `PdoWarn` 类型**，那会把厂商 SDK 拖进本层接口。

没进这个结构的是 `GM_Config_Download_And_Active` 报出的 **op 前**错误计数——那是启动期
一次性的诊断，现在只在 `ethercat_init()` 里打印，混进来会让「最近一次收帧」的语义变糊。

---

## 六、验证状态（2026-09-22）

离线回归入口：

~~~bash
python3 test/run_driver_tests.py
~~~

测试只使用 SDK 头文件和模拟 SDK 实现，不连接主站卡。覆盖：
- 启动阶段逐项注入正/负错误码，失败回滚后重新启动；
- EEPROM 实际字节数、截断数据、PDO/Entry/槽位容量边界；
- 伺服模式读取、目标位置预置、模式/控制字写入失败及写入顺序；
- IO/面板多个 PDO、非整字节长度、SDK 回退映射及释放；
- 字典非法输入、重载失败清空、内存分配失败；
- DS402 纯函数、DeviceBase 状态迁移。

已通过 ASan、UBSan 和泄漏检查。真实 SDK 的共享库链接检查通过，
以隐藏可见性编译后仍只导出 15 个业务函数（不计 ELF 链接器标记）。

硬件上仍需验证握手、EEPROM 实际内容、Entry 绑定、收发与模式切换效果。
超时公开为毫秒，按 SDK 头声明的整数秒向上取整。启动共用单调时钟截止时间，
不可中断 SDK 调用返回后才能检查是否超时。RequestStop 后 Wait/Commit 返回
MASTER_STOP_REQUESTED；Wait 已在 SDK 内时先等 SDK 返回，不继续收帧。
详见 docs/driver-validation.md。
