#pragma once

/* 导出标记。本层**单独交付**时用 -fvisibility=hidden 编译，只有标了本宏的符号
 * 对外可见——否则 position / slave_num 这类通用名字会和同进程里别的库静默互相顶替。
 *
 * 注意：HAL 自身的库目标（CMakeLists.txt）**不加** -fvisibility=hidden，内部符号
 * 由 cmake/hal.exports 的 version script 挡住。若给该目标补上这个 flag，`hal_*`
 * 公共 ABI 会一起消失（version script 不覆盖 hidden 可见性），消费者链接时报未定义。 */
#if defined(_WIN32)
#  define MASTER_API __declspec(dllexport)
#else
#  define MASTER_API __attribute__((visibility("default")))
#endif
