#pragma once

/* 导出标记。本层用 -fvisibility=hidden 编译，只有标了本宏的符号对外可见——
 * 否则 position / slave_num 这类通用名字会和同进程里别的库静默互相顶替。 */
#if defined(_WIN32)
#  define MASTER_API __declspec(dllexport)
#else
#  define MASTER_API __attribute__((visibility("default")))
#endif
