# ESI 离线导入（首版）

> 2026-09-27 复核：本工具链目前用于开发诊断，尚未进入启动授权。已确认审核漏查主轴目标速度、类型与重复映射，并未使用解析合成来源标记；`static_eligible` 不能作为准许运行的凭据。已知问题、实际进度和修复顺序见 [项目复核](review-2026-09-27.md)，下一步先修正数据证据契约。

构建解析工具：

```sh
bash tools/build_esi_extract.sh
```

将各厂商 ESI XML 放进本地目录，建立身份索引：

```sh
python3 tools/esi_catalog.py build /path/to/esi build/esi_catalog.json
python3 tools/esi_catalog.py lookup build/esi_catalog.json 0x1dd 0x10305070 0x2040608
```

`lookup` 的三个参数依次为主站扫描得到的 Vendor ID、Product Code、Revision，可用十进制或 `0x` 十六进制。索引按三元组精确匹配，不自动选用相邻修订版。同一身份的已提取字段有差异时导入失败；相同描述可保留多个来源。未提取的限制差异尚不能检测。索引记录 ESI 路径和 SHA-256，重新导入时重新解析；查询不会检查原文件是否变更，目前也不保存解析器版本。XML 留在本地，不由运行中的主站下载。

提取结果 `schema_version=1`，每个设备包含身份、SM、CoE 标志、Rx/Tx PDO 候选、对象和子索引、访问位、位宽、数据类型、DC 模式。`objects[].entries[].access` 沿用 KickCAT 的位定义：读 PreOP/SafeOP/OP 为 `1/2/4`，写 PreOP/SafeOP/OP 为 `8/16/32`，RxPDO/TxPDO 标志为 `64/128`。`coe=null` 表示 ESI 未声明 CoE 邮箱。

**限制：**KickCAT 会从 PDO 声明补出对象字典条目、类型及访问位，还可能修正映射对象，故 `objects_may_be_synthesized=true`。目前没有逐字段来源，审核器也未检查该标记，不能将这些位直接当作明确的读写或重映射许可。PDO 列表是 ESI 候选配置；DC/SM/PDO 等约束尚未完整输出。功能绑定需要本项目的设备规则、完整计划和下发结果；当前导入工具尚未改变启动行为。

离线回归：

```sh
python3 test/test_esi_import.py build/esi_extract
```

历史公开样本解析冒烟记录（2026-09-26；对象数量包含解析器合成结果，不等于厂商显式对象数量）：

| 厂商与型号 | 来源 | 解析结果 |
| --- | --- | --- |
| 汇川 SV660N | [汇川韩国官网 ESI 下载](https://www.inovance.co.kr/products/%EC%82%B0%EC%97%85%EC%9E%90%EB%8F%99%ED%99%94/acservo/servo-drives670) | 1 个设备版本，105 个字典对象，6 个 RxPDO、5 个 TxPDO |
| 安川 Sigma-X SGDXW | [安川官网 ESI 下载](https://www.yaskawa.com/downloads/search-index/details?docnum=Yaskawa_Sigma-X_CoE_ESI_Files&showType=details) | 8 个设备版本，每版均含对象及 Rx/Tx PDO |
| 台达 ASDA2-E | [公开 GitHub ESI 样本](https://github.com/nicola-sysdesign/asda-test/blob/main/Delta_ASDA2-E_rev4-00_XML_TSE_20160620.xml) | 1 个设备版本，580 个字典对象，4 个 RxPDO、4 个 TxPDO |

这些厂商文件只在临时目录用于验证，未纳入仓库。三菱的 [MR-J5 ESI 官方下载页](https://www.mitsubishielectric.co.jp/fa/download/software/detailsearch.page?infostatus=5_1_2&kisyu=%2Fservo&lang=2&mode=software&select=0&shiryoid=0000000040&softid=3&viewradio=0) 已定位，尚未取得文件验证。

## 无从站时的拓扑审核

用与 EEPROM 扫描相同的有序身份数据制作拓扑快照。可先用 `test/fixtures/topology_delta_example.json` 演练；它只模拟一台台达伺服，不是实机扫描结果。将上面的台达 ESI 放入导入目录后运行：

```sh
python3 tools/esi_topology_audit.py build/esi_catalog.json test/fixtures/topology_delta_example.json
python3 test/test_esi_topology_audit.py
```

拓扑快照每个从站必须给出 `slave_pos`、`vendor_id`、`product_code`、`revision`；伺服用途 `use` 可为 `position`、`spindle` 或尚未分配的 `unassigned`。若取得**当前实际启用**的 PDO 条目，再给出 `active_pdo_entries`，其中 `rx` 和 `tx` 分别是 `{index, subindex, bits}` 数组。不能把 ESI 候选 PDO 列表填进这个字段。缺省时审核结果是 `pending/pdo_unverified`，不会宣告功能已就绪。

有实机时，在启动进程设置 `CNC_HAL_TOPOLOGY_SNAPSHOT=/path/to/topology.json`，驱动在读取 EEPROM 后、下发配置前写出相同结构。它带有 `pre_download_pdo_entries`（含 PDO 索引及条目），来源是 EEPROM 解析结果，缺少 PDO 类别时可能来自 `GM_PDO_Map_Get()`；**它不是已启用映射的读回**。驱动此时不知道机床用途，`use` 为 `unassigned`，离线审核仍会保持 `pending`。可在快照副本中依据机床配置补上 `use=position` 或 `spindle` 后审核；不要把 `pre_download_pdo_entries` 改名为 `active_pdo_entries`。

设置快照路径后，启动流程仍会继续下发和握手；该功能不是只扫描模式。写文件失败只输出错误并继续启动；扫描失败也可能留下上次文件。现阶段文件只用于诊断，不能作为自动启动授权缓存。

审核读取 `src/Greemaster/devices.json`，但 Python 的语法支持和角色表尚未与 C 读取器完全一致。当前普通 DS402 基础依赖相同；Python 对主轴只追加 `actual_speed`，而 C 实际要求 `actual_speed` 与 `target_speed`，此处为待修缺陷。结果区分 `object_missing`、`subindex_missing`、`bits_mismatch`、`esi_mapping_unknown`、`esi_access_unknown`、`not_in_current_pdo`、`pdo_unverified`、`pdo_pre_download` 和 `not_in_pre_download_pdo`。`object_missing` 仅说明提取结果未列出对象，不能证明设备无该对象。可选角色缺失不阻止报告中其他角色，但报告不会实际禁用运行函数；`alarm_control` 固定为 `undefined_rule`。格力多轴和 IO 当前为 `not_audited`。

该工具只输出离线审核报告，尚未接入 `ethercat_init()`。现行 pending/active 快照判定是原型行为，不是目标启动契约。后续静态审核应对本次计划 PDO 作判断，下发成功、OP 和握手后再发布运行能力；不要求主站额外在线查字典或独立读回全部映射。
