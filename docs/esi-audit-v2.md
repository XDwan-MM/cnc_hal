# N2：离线功能依赖审核

2026-09-27。本阶段完成角色依赖审核，不执行主站配置。无从站即可复现。

## 统一规则

`src/common/servo_roles.def` 是角色名、标准索引、位宽、方向、预期类型和基础/速度依赖的共同来源。C 的标准绑定表、字典校验、运行层基础/速度句柄检查，以及离线 C 适配器均使用它。原多轴标准表顺序保持不变，标准 DS402 仍为原来的 10 个角色；`target_torque` 仅在 custom 配置中启用，未扩大标准绑定范围。

离线工具通过 `esi_rules` 调用生产 `DevDict_Load/Lookup`，不再用 Python 重写字典语法。保留 C 的注释、整数、缺省 revision 通配、精确优先、字段合法性和 custom 必填规则。字典字节在一次审核中冻结并记录摘要。已有 custom servo 仍要求九个角色（含 error_code 和两种速度）写入字典；这与运行时某用途的必需 PDO 清单是两个层次。

基础位置功能需要状态字、控制字、实际位置、目标位置、实际模式、目标模式。主轴另要求实际速度与目标速度同时存在。位置轴缺速度会使速度功能静态不可用，基础位置功能仍可满足；每个从站独立报告。

## 输入

```sh
bash tools/build_esi_extract.sh
bash tools/build_esi_rules.sh
python3 -B tools/esi_catalog.py build /path/to/esi build/esi_catalog.json
python3 -B tools/esi_topology_audit.py build/esi_catalog.json /path/to/topology-plan.json
```

可用 `--rules-helper` 或 `CNC_HAL_ESI_RULES` 指定 C 适配器；角色契约更新后必须重新构建适配器和 HAL。

拓扑仍为 schema v1，包含连续的从站位置、完整身份和 `use=position/spindle/unassigned`。每台从站的 `planned_pdo_entries` 必须明确提供本次待下发计划，包含 rx/tx 数组；每个条目有 `index/subindex/bits`，可带 `pdo_index` 限定候选 PDO。重复条目不会被去重。示意：

```json
{"planned_pdo_entries":{"rx":[{"index":24640,"subindex":0,"bits":16,"pdo_index":5632}],"tx":[]}}
```

此例故意不完整，会因必需角色缺项而阻止需求审核通过。N2 不自动生成计划；不得将 ESI 所有互斥候选拼成计划。`active_pdo_entries` 和 `pre_download_pdo_entries` 仅诊断，均不能替代计划或把运行状态提升为就绪。

## 证据判定

- 位宽、类型、有符号性必须符合角色要求；REAL32 不能代替 DINT。
- 使用明确对象声明及 PdoMapping，或该方向的原始 PDO Entry 声明。SDO 权限未知不会否定一个已明确声明的固定 PDO；PDO 声明也不会赋予 SDO 写入许可。
- 不用 `parser_value` 或合成对象权限补齐缺失字段。缺失对象、类型或映射证据记为 `evidence_insufficient`，不宣称设备不支持。
- 计划同方向内同一索引/子索引重复，无论位宽相同与否，记为 `duplicate_mapping`。重复对象证据、明确声明矛盾或字段被解析器修正不能通过。
- 当前任意解析警告/不支持结构诊断会使该设备所需角色保持未通过，属于保守策略；间接类型、复杂数组的类型语义尚未自动解析，不按位宽猜测。
- 标准类型别名仅限代码中明确列出的等价项。SM 容量、固定/强制映射、排除项、CoE 配置能力、DC、设备初始化命令和完整布局待 N4；N2 的 static_ok 不代表这些约束已通过。

## 报告

报告 schema v2，逐从站/角色给出定位引用、状态、必需性；功能层给出 position/speed/error_code 的依赖。`alarm_control=undefined_rule`，尚未给 0x6FFF 定义通用功能。

| decision | 含义 |
| --- | --- |
| requirements_satisfied | 当前用途的角色静态依赖满足 |
| blocked | 已指定用途/计划，但必需角色缺失、不符或证据不足；仍只是离线结果 |
| pending | 未指定用途或未提供计划 |
| not_audited | 格力多轴、IO 等尚无审核规则 |

所有结果均带 `runtime_ready=false`、`configuration_authorization=not_evaluated`，角色和功能的 `enabled=false`。可选功能证据不足会静态不可用；真正对运行函数禁用、整体下发成功后发布能力、重启失效等行为待 N3/N5。旧 `static_eligible` 已移除，v1 ESI 索引必须从原 XML 重建。

## 验证与下一步

20 项审核回归覆盖两种速度依赖、相同位宽不同类型、重复计划、缺失和合成证据、解析修正、PDO 与对象冲突、纯 PDO 声明、独立从站结果、身份/use/拓扑校验以及 C 字典语义。另有完整 XML → KickCAT → v2 证据 → 审核集成用例，确认纯 PDO 声明可静态通过，REAL 替代 DINT 被拒绝。

现有 context 254、driver 1702、dictionary 63、DS402、DeviceBase、公开 ABI 检查和 ASan/UBSan/泄漏检查通过。实机未验证。下一步 N3 建立下发前边界与预检失败零下发测试，完整计划约束未审核时不能仅凭本报告放行。
