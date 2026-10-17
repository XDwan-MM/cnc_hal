# CNC HAL

EtherCAT 硬件抽象层，对外提供 C API，当前后端为 GreeMaster。

## 目录

| 目录 | 职责 |
|---|---|
| include | 对外接口与数据类型；调用方只依赖这一层 |
| src/core | 配置校验、context 生命周期和周期控制 |
| src/backend/greemaster | 主站、设备识别、PDO、DS402 与设备字典解析 |
| config | devices.json 设备字典 |
| examples | 可运行的台架程序及其配置 |
| test | 无硬件回归与 RT 候选包工具 |
| docs | API、调用说明及上机用法 |
| cmake | 导出符号配置 |
| build | 构建产物，不提交 |

## 编译与离线验证

在已安装 GreeMaster SDK、libxml2、CMake、C 编译器和 Python 3 的 Linux 环境执行：

```bash
bash build_all.sh
ctest --test-dir build/all --output-on-failure
```

常用入口为 build_all.sh，生成静态库、共享库和测试程序；不会启动机床。
按需构建可用 build_static.sh（仅静态库）、test/build.sh（离线测试）、
examples/build.sh（台架程序）。这些脚本均使用根目录 CMake 配置。

设备字典仍安装到 CMake 配置的 share/cnc_hal/devices.json。
从源码树运行时，将 CNC_HAL_DEVICES_JSON 设置为 config/devices.json 的绝对路径。
公共 API 和运行时安装路径不因源码目录整理而改变。

## 文档

- [API 参考](docs/api-reference.md)
- [不定向 CSP 准备](docs/CSP_PREPARE.md)
- [台架用例用法](docs/测试用例用法.md)
- [上机检查单](docs/上机检查单.md)
- [GreeMaster 后端](docs/greemaster.md)

## 同步 RT

先运行 build_all.sh，再从 port_work 执行 bash lib/cnc_hal/sync.sh。
同步脚本须匹配本目录布局；RT 生产代码只用公共头，模拟后端使用 internal/backend/greemaster。
同步后重编 RT 的 Debug 与 Release。静态库更新不会自动替换已编译的 RT 程序。
