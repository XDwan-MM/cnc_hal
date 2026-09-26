# ESI v2 证据契约

日期：2026-09-27。N1 已实现；N2 已接入离线功能依赖审核，见 [审核契约](esi-audit-v2.md)。配置授权与运行能力发布待后续阶段。

## 数据层次

1. `esi_extract` 是内部提取器：输出 KickCAT 合成后模型 `objects`、关闭合成后的 `declared_dictionary`、分阶段警告和解析器版本。**关闭合成仍会应用默认权限、解析类型和展开数组，不是原始声明。**
2. `esi_catalog.py build` 调用 `esi_evidence.py`，基于同一份冻结 XML 字节提取模型、计算 SHA-256、建立字段证据。完整的 v2 索引由此命令生成，不能用提取器输出代替。
3. `device.evidence` 保存声明、来源、约束、转换和诊断；`device.objects` 等旧投影仅用于诊断。`normalized_model_authoritative=false`、`configuration_authorization=not_evaluated` 明确表示没有授权运行。

## 字段规则

`evidence.entries` 保留条目顺序和重复项；`index/subindex` 定位解析后的条目，`fields` 描述证据。每个字段有 `value`、`origin`、`raw`、`references`、`parser_value`。

| origin | 含义 |
| --- | --- |
| `xml` | 对象或数据类型中的直接声明；值保留 XML 文本或明确数字 |
| `pdo_declaration` | PDO Entry 的直接声明，只说明该 PDO 候选条目 |
| `scalar_convention` | 标量采用子索引 0 的结构约定 |
| `unknown` | 未声明、空值或未能对应；`value=null` |
| `inferred` | 解析器从 PDO 补出的属性；`value=null`，推导值仅在 `parser_value` |
| `parser_modified` | 合成前后发生变化；`value=null`，保留 `declared_value`、`parser_before`、`parser_value` |
| `conflict` | 重复对象或子索引导致无法唯一对应；`value=null` |

条目级来源还有 `dictionary`、`dictionary_subitem`、`pdo_derived`、`parser_generated`、`unresolved`、`conflict`。缺少 Flags 时即使解析器默认 access=7，证据权限仍是 unknown。PDO 补出的对象即使 access 含写位，也不构成 SDO 写入或动态重映射许可。`type.value` 是 XML 类型名，`type.parser_value` 是 KickCAT 类型编号，不能直接比较两者。

`constraints` 保留常用 CoE、SM、PDO 属性；未声明为 null，显式 `0` 为字符串 `"0"`，不会因解析器默认 false 混在一起。完整 Device 与 Vendor 元素树另存于 `declarations`，含属性、非空正文、子元素顺序及定位路径，因此 DC、初始化命令、排除项等未投影字段仍可查阅。它不是逐字节 XML 备份（不保留注释、排版、tail），也不包含 Device 外部的 Groups/Modules 定义；原 XML 应保留，SHA-256 标识其完整字节。

`references` 的 `/Device/...` 路径相对于本设备；来源记录中的 `device_ordinal` 是原文件 Device 的零基序号。`transformations` 按对象索引保存合成、删除或重写的前后模型；它不是每一步内部算法的轨迹。`diagnostics` 保存两个阶段的解析警告和适配诊断。

## 保守边界

- 字典完整性始终 `unknown`；没有对象声明不证明设备不支持。
- 数组展开、复杂类型间接引用、多个 Profile、模块组合尚未完成语义审核。不能唯一对应的字段保留 unknown/unresolved；原声明可供后续审核。
- DataType/SubItem 声明的默认值与解析器实际采用值可能不同，不能直接作为配置数据。
- `unresolved_entries` 只统计条目级 unresolved/conflict，不代表所有未知字段数量，也不是可配置判据。
- 同身份描述含原始树或证据差异时拒绝合并；这是保守冲突检查，尚未做语义等价归一化。
- 无 XSD 全量验证；不支持根元素命名空间变体。解析失败、身份缺失或越界拒绝导入；显式 revision=0 合法。

## 版本及升级

索引包含 `schema_version=2`、KickCAT/tinyxml2 修订号、本地 parser patch、adapter_version，以及每个来源的 SHA-256。修改解析/证据语义时应更新版本；SHA-256 不是签名，也不表示来源可信。

v1 缺少原始证据，必须从 XML 重新构建，不能只改版本号。查询不检查原文件是否更新。写索引采用临时文件替换，任何导入冲突不会覆盖已有索引。

`esi_topology_audit.py` 现只接受 v2，使用字段来源与原始 PDO 声明审核本次计划。v1 索引必须重建。报告中的 `requirements_satisfied` 仅表示角色静态依赖满足，不能用于启动授权。

## 本次验证

- `bash tools/build_esi_extract.sh /tmp/cnc-hal-esi-n1`
- `python3 -B test/test_esi_import.py /tmp/cnc-hal-esi-n1`：显式/缺省权限、PDO 推导、缺失身份与 revision=0、缺失/显式 false 约束、映射覆盖和重定向、重复 PDO、来源摘要、v1 拒绝、原始属性冲突与失败不覆盖索引。
- `python3 -B test/run_driver_tests.py`：context 254、driver 1702、dictionary 63、DS402、DeviceBase、7 项拓扑审核回归及 ASan/UBSan/泄漏检查通过。
- 临时目录内汇川/安川/台达 3 份历史样本成功导入 10 个设备版本；厂商文件未入库。这仅验证导入兼容性，没有从站实测。
