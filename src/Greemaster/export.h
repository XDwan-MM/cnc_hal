#pragma once

/* 导出标记：只有标了本宏的符号对外可见（本层 -fvisibility=hidden 编译）。
 * HAL 自身库目标不加该 flag，内部符号由 cmake/hal.exports 的 version script 挡。
 * version script 只对动态库生效，静态归档里的通用符号会进宿主全局符号空间。 */
#if defined(_WIN32)
#  define MASTER_API __declspec(dllexport)
#else
#  define MASTER_API __attribute__((visibility("default")))
#endif
