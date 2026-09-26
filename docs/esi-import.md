# ESI 离线导入（v2）

> 2026-09-27：N1/N2 已完成离线验收，v2 证据已接入角色依赖审核。`requirements_satisfied` 只表示静态依赖满足，不授权配置或运行。见 [字段规则](esi-schema-v2.md) 和 [审核契约](esi-audit-v2.md)。

构建解析工具：

```sh
bash tools/build_esi_extract.sh
bash tools/build_esi_rules.sh
```

将各厂商 ESI XML 放进本地目录，建立身份索引：

```sh
python3 tools/esi_catalog.py build /path/to/esi build/esi_catalog.json
python3 tools/esi_catalog.py lookup build/esi_catalog.json 0x1dd 0x10305070 0x2040608
```

`lookup` 的三个参数依次为主站扫描得到的 Vendor ID、Product Code、Revision，可用十进制或 `0x` 十六进制。索引按三元组精确匹配，不自动选用相邻修订版。同身份的描述或证据（包括 Device/Vendor 原始树）不同会拒绝合并；相同描述保留多个来源。索引记录 ESI 路径、SHA-256、设备序号、解析器及适配器版本。重新导入时重新解析；查询不检查原文件是否变更。XML 留在本地，不由运行中的主站下载。v1 必须从原 XML 重新构建。

索引 `schema_version=2`，每个设备包含身份、SM、CoE 标志、Rx/Tx PDO 候选、对象、DC 模式及 `evidence`。旧投影 `objects[].entries[].access` 沿用 KickCAT 位定义：读 PreOP/SafeOP/OP 为 `1/2/4`，写为 `8/16/32`，RxPDO/TxPDO 为 `64/128`；它仅作解析诊断。`coe=null` 表示未声明 CoE 邮箱。字段声明及权限来源应查 `evidence`。

**限制：**KickCAT 会补出字典条目、默认权限并修正映射。v2 保存相应来源、警告与差异；缺少声明的权限为 null，不升级成设备许可。Device 内 DC/SM/PDO 等声明树已保存，但完整约束语义、数组及模块组合审核仍待实现。PDO 列表是候选配置。功能绑定还需要设备规则、计划和下发结果；当前导入工具未改变启动行为。

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

## 无从站时的拓扑审核（N2）

先构建 C 字典适配器，再审核 v2 索引与本次计划：

```sh
bash tools/build_esi_rules.sh
python3 -B tools/esi_topology_audit.py build/esi_catalog.json /path/to/topology-plan.json
python3 -B test/test_esi_topology_audit.py
```

完整输入、状态与限制见 [N2 审核契约](esi-audit-v2.md)。每台从站的 `planned_pdo_entries` 是本次明确选择的计划，包含 rx/tx 数组及对象索引、子索引、位宽。缺少计划时保持 pending；不要求先取得实际启用 PDO 快照。现有 `topology_delta_example.json` 只有模拟身份及用途，因此仍会 pending。

审核复用生产 C 字典读取器，并与 C 绑定共享角色表；主轴要求实际速度与目标速度。检查位宽、类型、有符号性、重复映射及证据来源。必需角色缺证据会 blocked，可选功能静态不可用。缺少 ESI 声明仅意味着证据不足。所有结果均不授权配置或运行；多轴/IO 为 not_audited，alarm_control 为 undefined_rule。

有实机时，`CNC_HAL_TOPOLOGY_SNAPSHOT=/path/to/topology.json` 仍会在 EEPROM 读取后输出诊断快照。其 `pre_download_pdo_entries` 可能来自 EEPROM 解析或 `GM_PDO_Map_Get()` 回退，不是实际启用映射读回，也不会自动成为计划。驱动仍会继续下发及握手，快照功能不是只扫描模式；写文件失败会报告并继续，扫描失败可能留下旧文件。

当前审核没有接入 `ethercat_init()`。下一步 N3 拆分扫描、预检、下发边界，后续 N4 完整审核默认/强制映射、容量及 DC 等约束，N5 在整体下发、OP、握手成功后发布运行能力。
