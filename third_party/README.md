# ESI 解析依赖

- `kickcat_esi/`：从 [KickCAT](https://github.com/leducp/KickCAT) 提取 ESI 解析所需源码及头文件，提交 `089f8cad4e852ea3b9e1d0b407425abe7440e3f3`，许可证见 `kickcat_esi/LICENSE`（CeCILL-C）。本项目未修改这些源码。测试样本 `test/fixtures/kickcat_multi_device.xml` 同源。
- `tinyxml2/`：来自 [TinyXML2](https://github.com/leethomason/tinyxml2)，提交 `8224e427b655b83dae5e2298f1e6919523a78737`，许可证见 `tinyxml2/LICENSE.txt`（zlib）。本项目未修改这些源码。

上述代码仅用于离线 ESI 导入工具，不链接到实时 HAL 库。
