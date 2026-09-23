#ifndef _ETHERCAT_H
#define _ETHERCAT_H
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>
#include "/opt/GreeMaster/include/libGREEMASTER/GreeMasterAPI.h"
#include "/opt/GreeMaster/include/libGREEMASTER/gm_errcode.h"
#include "slave_list.h"
#include "export.h"

/* =============== 主站启动配置 =============== */

#define GM_MIN_CYCLE_US 250     /* GM_Master_Set_Cycle 支持的最小周期 */

/** 主站启动配置 —— ethercat_init() 的入参。机床/现场相关的参数都从这儿走。 */
typedef struct {
    uint32_t cycle_us;        /* 通讯周期（us），下限 GM_MIN_CYCLE_US */
    uint32_t start_timeout_ms; /* 启动总预算；SDK 秒粒度等待向上取整 */
    uint32_t cycle_timeout_ms; /* 毫秒；SDK 秒粒度等待向上取整 */
    int      dc_enable;       /* 是否使能 DC 信号（分布式时钟） */
} MasterConfig;

/* 不知道填什么就用它：MasterConfig cfg = MASTER_CONFIG_DEFAULT; */
#define MASTER_CONFIG_DEFAULT                                       \
    { .cycle_us = 1000u, .start_timeout_ms = 120000u, .cycle_timeout_ms = 5000u, \
      .dc_enable = 1 }

#define MASTER_STOP_REQUESTED (-28673)
#define MASTER_START_TIMEOUT  (-28674)

/* 设备字典路径。**开发期默认值**指向源码树那份，部署时必须 -D 覆盖。
 * 加载失败不致命——设备全判 UNKNOWN_TYPE，装配落空。 */
#ifndef DEVICES_JSON_PATH
#define DEVICES_JSON_PATH "/home/mxh/CNC/HAL/src/Greemaster/devices.json"
#endif

/* =============== 开发期开关，上机前保持默认 =============== */

#define PARAM_CHANG 0          /* 是否修改主站参数 */
#define MAN_FMMU    0          /* 是否手动输入 fmmu */

/* 握手前的 REG/SDO 交互窗口（GM_Wait_Pilot_Data_Response）——主站 OP 之后、
 * 握手之前。设备参数（SDO）下发应落在这里。尚未实现。 */
#define OP_PRE_REG_EN 0

/* =============== 错误上下文 =============== */

typedef struct {
    int Model;      /* 报错模块 */
    int Fms;        /* 报错状态机 */
    int Code;       /* 报错索引码 */
    int errslave;
    int ErrorID;    /* 预定义的枚举 ID */
} ErrorContext;

void error_module_init(void);
void error_module_destroy(void);

typedef enum _EcatMasterError
{
    ECAT_INIT_ERROR = 0x1001,                   /* 主站初始化模块错误 */
    ECAT_CONTROL_PANEL_CONNECT_LOST = 0x1002,   /* 控制面板断连 */
    ECAT_IO_CONNECT_LOST = 0x1003,              /* IO 模块断连 */
    ECAT_SLAVE3_CONNECT_LOST = 0x1004,          /* 从站三断连 */
    ECAT_SLAVE4_CONNECT_LOST = 0x1005,          /* 从站四断连 */
    ECAT_SLAVE5_CONNECT_LOST = 0x1006,          /* 从站五断连 */
    ECAT_SLAVE6_CONNECT_LOST = 0x1007,          /* 从站六断连 */
    ECAT_TOPOLOGY_ERROR = 0x1008,               /* 主站拓扑模块错误 */
    ECAT_ACTIVE_ERROR = 0x1009,                 /* 主站激活模块错误 */
    ECAT_DC_ERROR = 0x100A,                     /* 主站 DC 模块错误 */
    ECAT_PDO_ERROR = 0x100B,                    /* 主站 PDO 模块错误 */
    ECAT_RT_ERROR = 0x100C,                     /* 主站 RT 模块错误 */
    UNKOWN_ERROR = 0xFFFF                       /* 未知错误 */
} EcatMasterError;

extern int slave_num;

/* =============== 对外接口 =============== */

/** @brief 启动主站到「周期收发模式可用」。
 *  @param cfg  主站配置（周期、毫秒超时、DC）；资源申请由驱动决定
 *  @return 0 成功；< 0 失败 */
MASTER_API int ethercat_init(const MasterConfig* cfg);

/** @brief 关闭主站，释放资源；幂等。调用方须先退出周期调用。
 *  @return 0 成功；< 0 失败（错误码查主站手册） */
MASTER_API int ethercat_close(void);

/** @brief 等本周期节拍并收一帧——周期里唯一的阻塞点。
 *  @note  只在实时线程调用。超时用 MasterConfig.cycle_timeout_ms。
 *  @return 0 成功；< 0 = 等节拍超时或收帧失败 */
MASTER_API int Master_WaitCycle(void);

/** @brief 下发本帧输出。只把已写入的 PDO 发出去，不做任何计算。
 *  @note  只在实时线程调用。标准周期：WaitCycle → 读写 → CommitCycle。
 *  @return 0 成功；< 0 失败 */
MASTER_API int Master_CommitCycle(void);

/** @brief 主站是否出过错或已停机。
 *  @return 0 = 正常；非 0 = 出错或已停机 */
MASTER_API int Master_StopFlag(void);

/** @brief 非阻塞停止通知；等待中的周期调用在 SDK 返回后报告 MASTER_STOP_REQUESTED。
 *  @note  不释放资源、不承诺唤醒 SDK；与周期调用可并发，生命周期调用串行。 */
MASTER_API void Master_RequestStop(void);

/** 总线健康快照 —— 主站自己累计的警告与计数，掉电清零。 */
typedef struct {
    int      pdo_warn;             /* 非零 = 有 PDO 警告 */
    int      pdo_warn_code;        /* PDO 警告号 */
    unsigned pdo_warn_para;        /* PDO 警告参数 */
    int      dc_warn;              /* 非零 = 有 DC（分布式时钟）警告 */
    int      dc_warn_code;
    unsigned dc_warn_para;
    unsigned crc_err_count;        /* CRC 错误累计次数 */
    unsigned frame_timeout_count;  /* 帧超时累计次数 */
    unsigned expect_wkc_tx;        /* 期望工作计数器（Tx 侧） */
    unsigned expect_wkc_rx;        /* 期望工作计数器（Rx 侧） */
} MasterBusHealth;

/** @brief 取最近一次收帧的总线健康快照。只记录不判断，报警策略归上层。
 *  @param out  带出快照；还没收过帧时全零
 *  @return 0 成功；< 0 = out 为空 */
MASTER_API int Master_BusHealth(MasterBusHealth* out);

#endif
