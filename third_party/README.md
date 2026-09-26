# ESI 解析依赖

- `kickcat_esi/`：从 [KickCAT](https://github.com/leducp/KickCAT) 提取 ESI 解析所需源码及头文件，提交 `089f8cad4e852ea3b9e1d0b407425abe7440e3f3`，许可证见 `kickcat_esi/LICENSE`（CeCILL-C）。本地补丁 `cnc-hal-evidence-1`（2026-09-27）修改 `Device.h`、`Parser.h`、`Parser.cc`：增加可关闭对象合成的解析模式，并将四处解析警告保存为设备诊断；默认仍启用合成。测试样本 `test/fixtures/kickcat_multi_device.xml` 同源；`esi_evidence.xml` 为本项目编写。
- `tinyxml2/`：来自 [TinyXML2](https://github.com/leethomason/tinyxml2)，提交 `8224e427b655b83dae5e2298f1e6919523a78737`，许可证见 `tinyxml2/LICENSE.txt`（zlib）。本项目未修改这些源码。

上述代码仅用于离线 ESI 导入工具，不链接到实时 HAL 库。

升级上游时必须复核补丁并递增补丁版本，运行 ESI 导入回归。关闭合成仍包含解析器缺省与类型展开，不能把该模型称为原始 XML。证据格式见 [ESI v2 契约](../docs/esi-schema-v2.md)。
