# P0：GreeMaster / FPGA 能力核查

> 历史 SDK 能力核查（2026-09-26）。本文 P0/P4 等阶段属于当时的 ESI 研究安排，不是当前 RT 接入任务；当前计划见 `../tmp/RT直接接入HAL实施计划书.md`。

- 日期：2026-09-26
- 结论状态：P0 方案核查完成；目标硬件实测纳入 P4
- 本机 SDK：`/opt/GreeMaster/lib/libGREEMASTER.so.1.1.8`
- 头文件版本：`_FULL_VERSION_ = "1.1.8"`，`_API_VERSION_ = 8`，`_BUILD_COUNT_ = 93`，构建日期 2026-09-24（见 `/opt/GreeMaster/include/libGREEMASTER/version.h`）
- 固件版本：未知。本机未发现可访问的 FPGA 设备节点，未调用 `GM_Get_Version()`。
- 证据来源：已安装 SDK 头文件和手册、SDK 示例、当前项目代码、共享库导出符号、本机设备检查，以及 2026-09-26 SDK 作者的说明。SDK 作者的明确说明标为“作者确认”；下表“有接口”仅代表静态接口存在，不代表已在目标固件上验证。

## 1. 能力矩阵

| 能力 / 问题 | 静态证据与判断 | 状态 | 后续验证 |
| --- | --- | --- | --- |
| 启动后扫描数量 | `GM_Master_Start()` 文档称等待所有从站进入 PREOP；`GM_Slave_Num_Get()` 返回扫描数。当前 `main_demo.c` 按此顺序调用 | 有接口 | 用实际网络核对从站数、顺序及缺站行为 |
| 身份信息 | `GM_EEPROM_Get(pos, ...)` 可按位置读 EEPROM；项目从偏移 16/20/24/28 解出 Vendor/Product/Revision/Serial | 有接口 | 与设备标签、ESI 和在线身份核对；验证数据不一致时的处理 |
| 物理端口拓扑 | 已检查公开 `GreeMasterAPI.h`；未发现端口连接图读取接口。数量与按位置的身份不足以还原分支端口关系 | 未发现接口 | 向 SDK 维护方确认或实机检测；第一版将“拓扑”定义为有序从站清单 |
| 取得 PDO 信息 | 当前 1.1.8 头文件和手册声明 `GM_PDO_Map_Get(pos, timeout, flag)` 按从站位置返回 `Slave_info*`；`GM_PDO_Map_Print(slave_list, num)` 打印已有链表。共享库静态检查显示 Get 向 FPGA 发请求、经 bypass 读取缓冲区并解析为 `Slave_info*`；FPGA 数据的上游来源未知 | 接口职责与主机侧读取路径已核对；FPGA 上游来源未知 | 核对 FPGA 实现或实机比较 EEPROM 默认映射与配置后结果；不能假定它是配置后生效映射 |
| 生成并下发配置 | SDK 作者确认：在 `Slave_info` 中新增 PDO 条目并下发配置，SDK / FPGA 会配置从站，使该条目进入周期数据；具体对象仍须从站允许映射 | 作者确认，待实机验证 | 用可配置设备验证新增条目、固定映射设备的拒绝行为；记录具体写入和限制 |
| 配置后实际映射验证 | `GM_PDO_Map_Print()` 只打印传入的 `Slave_info`，不能证实从站实际布局；`GM_PDO_Map_Get()` 返回 FPGA 提供的信息，FPGA 上游来源尚未确认；配置成功与进入 OP 是整体启动结果 | 缺少已确认的逐项读回接口 | 在可用时机独立读取映射对象或通过已知设备与过程数据验证；结果按证据等级标记 |
| SDO 请求接口 | `GM_Sdo_Datagram_Enable()`、`GM_Sdo_Request_Send()`、`GM_Sdo_Receive()` 已导出；`SdoRequests` 有状态和警告字段，`data` 注释称最大 4 字节 | 有接口 | 实机核对读写尺寸、错误码、超时及串行化要求；不能假设可完整上传对象字典 |
| 配置前在线审核对象 | SDK 作者确认普通 SDO 需先执行 `GM_Config_Download_And_Active()`；当前 SDK 不支持在线查询对象类型、权限及 PDO 可映射属性 | 不支持作为启动前审核依据 | 功能支持性由已匹配 ESI 与设备规则在下发前判断；在线信息不作为第一版前置条件 |
| 握手前 SDO | `GM_Wait_Pilot_Data_Response()` 文档明确为握手前 REG/SDO 回复；现有代码还未使用 | 部分支持 | 实机确认请求发送和响应顺序、超时与并发限制 |
| 运行期 SDO | SDK 示例称 SDO/REG 线程须在 PDO 周期线程启动后执行，当前项目有简单读写函数 | 部分支持 | 确认线程同步和失败处理；实时 API 不直接等待 SDO |
| 配置 / 从站错误诊断 | `Err_Fun_Register()` 与错误码可报告整体失败；SDK 作者确认当前不会直接返回拒绝的具体对象及 CoE 原因 | 粒度不足 | 应用层在下发前完成审查，并保存配置计划；实机记录返回码与 AL 状态 |
| DC 配置 | `GM_DC_Enable()` 可开同步帧；`Slave_info` 含 sync 参数；项目固定 `sync_assign_activate=0x300` | 有接口，设备约束未确认 | 分别验证支持/不支持 DC 的从站及所需周期、shift、模式 |
| 容量 | SDK 公开配置 RAM 36864 B、SM RAM 4096 B、拓扑 RAM 8192 B 等；项目 `MAX_DEVICE_NUM=30`，单 PDO 本地解析限 20 条 | 仅静态上限线索 | SDK 确认实际总从站、SM、PDO、Entry、FMMU 和过程映像限制；生成器作保守校验 |
| SDK / 固件兼容性 | `GM_Get_Version()` 可读取主站版本；本机仅知 SDK 1.1.8 | 固件未知 | 实机记录 FPGA 主/次版本与 SDK 版本，保存到测试记录 |

`nm -D` 已确认矩阵中关键的扫描、EEPROM、PDO、SDO、配置与握手函数存在于当前共享库。没有据此推断其实际运行效果。

## 2. 当前代码和 SDK 允许的启动时序

```text
设备字典加载
GM_Resource_Allocation → GM_Get_Version → GM_Master_Init
GM_Master_Start                 # 文档：从站进入 PREOP 后返回
GM_Slave_Num_Get → GM_EEPROM_Get / GM_PDO_Map_Get
生成 Slave_info → GM_Calculate_Config_Info
GM_Master_Set_Cycle → GM_DC_Enable（可选）
GM_Reg_Resource_Init / GM_Sdo_Datagram_Enable
GM_Config_Download_And_Active  # SDK：下发并激活
GM_Master_Wait_OP
GM_Master_Receive → IO_Resource_Allocation_Scanf → GM_Master_Send → GM_Master_OP_Valid
周期线程收发
```

项目当前在 `GM_Calculate_Config_Info` 之后、配置激活之前取得 Entry 句柄并建设备表。这是从本地计划的 `Slave_info` 建句柄；未来应等整体配置成功、进入 OP 与必要的设备检查完成后再向 HAL 发布能力状态。SDK 作者确认配置前不能用普通 SDO 审核对象，因此静态能力判定必须在配置下发前由 ESI 与设备规则完成。

## 3. P4 实机验证用例与记录格式

测试环境应记录设备型号、Vendor ID、Product Code、Revision、Serial、ESI 文件及摘要、SDK 与 FPGA 版本、从站顺序、供电与运动安全状态。只在隔离测试台上尝试故意拒绝的配置，保持驱动未使能、机械负载安全。

| 编号 | 操作 | 需要记录的证据与判定 |
| --- | --- | --- |
| H1 | 启动并扫描一个已知从站 | `GM_Slave_Num_Get` 数量、EEPROM 身份、从站位置、`GM_PDO_Map_Get` 内容；与铭牌/ESI 核对 |
| H2 | 使用已知可用的默认配置下发 | 配置返回码、错误回调、OP 状态、实际循环数据；比对期望映射 |
| H3 | 在配置完成后、握手前与周期线程运行后尝试只读 SDO | 核实具体可用窗口、返回码、状态、超时和警告；PREOP 不作为设计依赖 |
| H4 | 配置成功后读取可访问的 `0x1C12/0x1C13`、`0x16xx/0x1Axx` | 判明是否有独立读回途径；`GM_PDO_Map_Print()` 不作生效证明，Get 用途待确认 |
| H5 | 在隔离测试设备上提交一份预期会被拒绝的配置 | 记录整体失败、错误回调与 AL 状态；随后关闭并重新初始化主站，不在原启动实例内重试 |
| H6 | 支持和不支持 DC 的从站分别启动 | DC 配置、错误、同步状态，核对 `sync_assign_activate` 取值 |

H1–H6 均待实机执行。本 WSL 环境只有 `eth0`、`lo`，未发现 FPGA 设备节点；不调用会启动主站或下发配置的接口。用例 H5 会主动改变从站配置，只能在确认测试设备与现场隔离后执行。

## 4. 对后续阶段的约束与当前发现

1. P1 实现能力模型、显式功能禁用和离线测试。静态“支持”由 ESI 与设备功能定义判定；运行“已就绪”还需整体配置成功与进入 OP。主站不做额外的对象字典验证。
2. P2 从 ESI 和经验证的设备规则获取对象与映射约束。SDK 作者确认 PREOP 普通 SDO 不可用，也没有对象元数据查询接口；首次启动不能依赖在线查词典。
3. P3 依据 ESI / 设备规则规划并下发配置，以整体下发成功和 OP 作为运行门槛。`GM_PDO_Map_Print()` 不验证实际映射，`GM_PDO_Map_Get()` 来源未确认前也不作为生效证据；逐项读回作为后续工程诊断，不阻断 P1/P2。
4. 当前 `read_sdo_index()` / `write_sdo_index()` 对发送返回值、错误状态和超时的处理较弱，且注释中的普通请求最大数据 4 字节；P1/P3 若使用 SDO，应建立串行请求与明确的失败返回。
5. 当前 `COERequestResult()` 对普通伺服无条件补四个对象，`servo_addr_config()` 要求整套对象，`sample_axis()` 固定读取五类反馈；P1 需同步调整，单独放宽其中一处会引入运行期错误。
6. 当前 SDK API 只见有序从站数量与位置读取路径。若业务要求实际端口连接图，应获得 SDK 新接口或 FPGA 支持后再纳入验收。`GM_Config_Download_And_Active()` 失败后，SDK 作者确认主站停滞，必须关闭并重新初始化；同一次启动中不能去掉可选项后重试。

## 5. 现有报警路径与配置策略

当前 `err_call_back()` 可接收主站错误模块 / 状态机 / 错误号 / 从站位置；PDO 模块错误映射为 `ECAT_PDO_ERROR`，并设置停止标志。`GM_Config_Download_And_Active()` 失败会终止启动。`device_match()` 找不到预定义 PDO 条目时返回 `-1`，但不携带缺少的索引、子索引和功能名；`CHECK_RC` 宏仅跳转清理，传入的文字不会打印；`hal_context_start()` 通常只向上层返回 `HAL_ERROR_BUS` 和“主站启动失败”。现有警报能停机，尚不能完整回答“哪台从站缺少哪个 PDO，导致哪个功能禁用”。

规划原则：保留默认及强制 PDO。可选功能在配置下发前依据 ESI、默认映射与经验证的设备规则筛选；不支持或证据不足则禁用并报告应用级诊断，不向主站提交试探性映射。若已批准的配置仍被从站拒绝，整次启动失败，保留主站错误和配置计划上下文，关闭并重新初始化。

## 6. 本次核查结果

- 完成 SDK 1.1.8 头文件、手册、示例、当前项目启动路径的静态对照，并检查共享库导出符号和 `GM_PDO_Map_Get()` 的主机侧调用路径。
- 完成环境检查：WSL2，未发现可用于实机试验的 FPGA 设备节点。
- SDK 作者确认配置前普通 SDO 不可用、不提供对象元数据查询、配置失败须重新初始化，且不能直接报告拒绝的具体对象与 CoE 原因。当前头文件表明 `GM_PDO_Map_Get()` 返回 PDO 信息链表，`GM_PDO_Map_Print()` 才打印现有链表；Get 在主机侧读取 FPGA 缓冲区，FPGA 内部从哪里取得映射仍待核实。修改 `Slave_info` 可触发从站 PDO 映射配置；具体对象能否成功由从站能力决定。
- 方案确认无需主站额外在线字典验证。固件版本读取、配置后读回、固定 / 可变映射及拒绝路径实机试验纳入 P4；相关矩阵保持待验证。