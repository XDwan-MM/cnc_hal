#pragma once

/* 导出标记。本层**单独交付**时用 -fvisibility=hidden 编译，只有标了本宏的符号
 * 对外可见——否则 position / slave_num 这类通用名字会和同进程里别的库静默互相顶替。
 *
 * 注意：HAL 自身的库目标（CMakeLists.txt）**不加** -fvisibility=hidden，内部符号
 * 由 cmake/hal.exports 的 version script 挡住。若给该目标补上这个 flag，`hal_*`
 * 公共 ABI 会一起消失（version script 不覆盖 hidden 可见性），消费者链接时报未定义。
 *
 * 另外：version script 只对**动态库**生效。静态归档（libcnc_hal.a）没有这一层，
 * 本层的通用符号（err_call_back、device_match、slave_list、g_device_data 等）
 * 会直接进宿主的全局符号空间。现在与 cnc_rt 没有重名，但将来别的静态库定义了
 * 同名符号就会**静默冲突**。 */
#if defined(_WIN32)
#  define MASTER_API __declspec(dllexport)
#else
#  define MASTER_API __attribute__((visibility("default")))
#endif
