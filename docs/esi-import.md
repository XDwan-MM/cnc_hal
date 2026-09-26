# ESI 离线导入（首版）

构建解析工具：

```sh
bash tools/build_esi_extract.sh
```

将各厂商 ESI XML 放进本地目录，建立身份索引：

```sh
python3 tools/esi_catalog.py build /path/to/esi build/esi_catalog.json
python3 tools/esi_catalog.py lookup build/esi_catalog.json 0x1dd 0x10305070 0x2040608
```

`lookup` 的三个参数依次为主站扫描得到的 Vendor ID、Product Code、Revision，可用十进制或 `0x` 十六进制。索引按三元组精确匹配，不自动选用相邻修订版。同一身份有不同描述时导入失败；相同描述可保留多个来源。索引记录 ESI 路径和 SHA-256，重新导入时重新解析。XML 留在本地，不由运行中的主站下载。

提取结果 `schema_version=1`，每个设备包含身份、SM、CoE 标志、Rx/Tx PDO 候选、对象和子索引、访问位、位宽、数据类型、DC 模式。`objects[].entries[].access` 沿用 KickCAT 的位定义：读 PreOP/SafeOP/OP 为 `1/2/4`，写 PreOP/SafeOP/OP 为 `8/16/32`，RxPDO/TxPDO 标志为 `64/128`。`coe=null` 表示 ESI 未声明 CoE 邮箱。

**限制：**KickCAT 会从 PDO 声明补出对象字典条目及访问位，因此 `objects_may_be_synthesized=true`。这些位只供静态筛查，不能证明实际从站允许在线读写或重新映射。PDO 列表是 ESI 候选配置，不能证明当前 EEPROM/主站启用哪个 PDO。`pdo_config=true` 也不等于任意对象都能改映射。功能绑定还需要本项目的设备规则、默认映射和下发结果；当前导入工具尚未改变启动行为。

离线回归：

```sh
python3 test/test_esi_import.py build/esi_extract
```

已用公开样本做过离线验证：

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

拓扑快照每个从站必须给出 `slave_pos`、`vendor_id`、`product_code`、`revision`；伺服还需给出 `use`（`position` 或 `spindle`）。若取得**当前实际启用**的 PDO 条目，再给出 `active_pdo_entries`，其中 `rx` 和 `tx` 分别是 `{index, subindex, bits}` 数组。不能把 ESI 候选 PDO 列表填进这个字段。缺省时审核结果是 `pending/pdo_unverified`，不会宣告功能已就绪。

审核使用现有 `src/Greemaster/devices.json` 定义的 DS402 或自定义角色。普通伺服的必需角色与当前驱动一致；主轴另需 `actual_speed`。结果区分 `object_missing`、`subindex_missing`、`bits_mismatch`、`esi_mapping_unknown`、`esi_access_unknown`、`not_in_current_pdo` 和 `pdo_unverified`。可选角色缺失不会阻止其他角色；`alarm_control` 因尚无设备功能定义，固定为 `undefined_rule`，不会仅因出现 `0x6FFF` 就启用。格力多轴和 IO 当前为 `not_audited`，需要各自的设备规则。

该工具只输出离线审核报告，尚未接入 `ethercat_init()`；`static_eligible` 也仅表示所给快照与 ESI 一致，仍需主站下发成功及实机验证后才能发布运行能力。
