#pragma once

/* DS402 状态机 —— 与总线/驱动无关的纯编码 + 推进规则。
 * header-only、无依赖，可离线穷举测试。
 * 只回答「这一拍写什么」，不回答「什么时候开始使能」——后者归上层。 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 控制字 0x6040。使能：撤电压 → 关断 → 接通 → 运行使能，逐级往上。 */
#define DS402_CW_DISABLE_VOLTAGE 0x0000u  /* 撤电压：彻底断电，最低态（急停的最彻底做法） */
#define DS402_CW_SHUTDOWN        0x0006u  /* 关断：进 Ready to switch on（带电、未接通） */
#define DS402_CW_SWITCH_ON       0x0007u  /* 接通：进 Switched on（可收指令、不带载力矩） */
#define DS402_CW_ENABLE_OP       0x000Fu  /* 运行使能：进 Operation enabled，此时才出力 */
#define DS402_CW_FAULT_RESET     0x0080u  /* bit7。**上升沿**清错，只在 Fault 时写一次，不保持 */

/* 状态字 0x6041 的 Fault 位（bit3） */
#define DS402_SW_BIT_FAULT       0x0008u  /* 置位 = 驱动器处于 Fault，任何使能推进都不会生效 */

/* 运行模式 0x6060 */
#define DS402_MODE_CSP      0x08u  /* 循环同步位置。进给轴用它（HAL 强制），主轴写位置/保位也用它 */
#define DS402_MODE_CSV      0x09u  /* 循环同步速度。主轴默认（M03/M04 走这条） */

/* 状态字 0x6041 的判定掩码。四种状态各一对「掩码 + 值」：
 * 先把状态字按掩码留下关心的位，再与值比较，相等即处于该状态。
 * 这两列**必须成对使用**，不能只比值——0x27 与 0x23 的差别就在被掩码挡掉的那几位之外。
 *
 * bit4 电压有效 / bit9 远程不参与状态编码。
 * bit5 必须参与使能态判定，避免把 QuickStopActive 当成 OperationEnabled。 */
#define DS402_SW_MASK_SWITCH_ON_DISABLED 0x004Fu  /* 留 bit0-3、bit6；bit5 不参与 */
#define DS402_SW_VAL_SWITCH_ON_DISABLED  0x0040u  /* (状态字 & 掩码) == 本值 → Switch on disabled */
#define DS402_SW_MASK_READY_TO_SWITCH_ON 0x006Fu  /* 比上面多含 bit5 */
#define DS402_SW_VAL_READY_TO_SWITCH_ON  0x0021u  /* → Ready to switch on */
#define DS402_SW_MASK_SWITCHED_ON        0x006Fu  /* 同上 */
#define DS402_SW_VAL_SWITCHED_ON         0x0023u  /* → Switched on */
#define DS402_SW_MASK_OP_ENABLED         0x006Fu  /* 同上 */
#define DS402_SW_VAL_OP_ENABLED          0x0027u  /* → Operation enabled（唯一真正出力的态） */

typedef enum {
    DS402_STEP_NONE = 0,      /* 不动 */
    DS402_STEP_SHUTDOWN,      /* → 写 0x06，并设运行模式 */
    DS402_STEP_SWITCH_ON,     /* → 写 0x07，并用实际位置预置目标位置 */
    DS402_STEP_ENABLE_OP,     /* → 写 0x0F */
    DS402_STEP_DONE,          /* 已在 OperationEnabled */
    DS402_STEP_DISABLE_OP,    /* → 写 0x07，从 OperationEnabled 退回 SwitchedOn */
    DS402_STEP_DROP_VOLTAGE,  /* → 写 0x00，断电 */
    DS402_STEP_FAULT_RESET,   /* → 写 0x80，清错 */
    DS402_STEP_SWITCH_MODE,   /* → 设 0x6060，并用实际位置预置目标位置；不写控制字 */
} Ds402StepKind;

/* 推进方向。ENABLE == 0 是刻意的：零初始化的请求表默认就是它。 */
typedef enum {
    DS402_REQ_ENABLE = 0,     /* 朝 OperationEnabled 推（默认） */
    DS402_REQ_DISABLE,        /* 停在 SwitchedOn（可收指令、不带载） */
    DS402_REQ_DROP_VOLTAGE,   /* 断电 */
    DS402_REQ_FAULT_RESET,    /* 清错。只在 Fault 时写一次 0x80 */
    DS402_REQ_NONE,           /* 什么都不做 */
} Ds402Request;

/* 已使能时发现模式不对，怎么切。DISABLE_FIRST == 0 同样刻意。 */
typedef enum {
    /* 默认。退回 SwitchedOn 再改 0x6060——规范明确支持，代价是切换期间短暂失力矩。 */
    DS402_MODESW_DISABLE_FIRST = 0,
    /* 直接在使能中写 0x6060。省一进一出，但能否生效取决于驱动器，上机确认后再用。 */
    DS402_MODESW_IN_PLACE,
} Ds402ModeSwitch;

typedef struct {
    Ds402StepKind kind;
    uint16_t      control_word;   /* kind 对应要写的控制字 */
    uint16_t      op_mode;        /* 要设的运行模式；0xFFFF = 不设 */
    int           preset_target;  /* 1 = 用当前实际位置预置目标位置（防飞车） */
} Ds402Step;

/* 朝请求推进一步。
 * 请求到达后返回 NONE 或 DONE（「已达成」）。 */
static inline Ds402Step Ds402_NextStepReq(uint16_t status_word, uint16_t current_mode,
                                          Ds402Request req, uint16_t desired_op_mode,
                                          Ds402ModeSwitch msw) {
    Ds402Step s;
    s.kind          = DS402_STEP_NONE;
    s.control_word  = 0;
    s.op_mode       = 0xFFFFu;
    s.preset_target = 0;

    switch (req) {
    /* 什么都不做*/
    case DS402_REQ_NONE:
        return s;
    /* 使能 */
    case DS402_REQ_ENABLE:
        /*  当前状态是switch_on_disabled */
        if ((status_word & DS402_SW_MASK_SWITCH_ON_DISABLED) == DS402_SW_VAL_SWITCH_ON_DISABLED) {
            s.kind         = DS402_STEP_SHUTDOWN;
            s.control_word = DS402_CW_SHUTDOWN;
            s.op_mode      = desired_op_mode;
        }
        /*  当前状态是ready_to_switch_on */
        else if ((status_word & DS402_SW_MASK_READY_TO_SWITCH_ON) == DS402_SW_VAL_READY_TO_SWITCH_ON) {
            s.kind          = DS402_STEP_SWITCH_ON;
            s.control_word  = DS402_CW_SWITCH_ON;
            s.preset_target = 1;
        }
        /*  当前状态是switched_on */
        else if ((status_word & DS402_SW_MASK_SWITCHED_ON) == DS402_SW_VAL_SWITCHED_ON) {
            /*  如果模式不对，不是期望的CSP或CSV，则切模式 */
            if (current_mode != desired_op_mode) {
                s.kind          = DS402_STEP_SWITCH_MODE;
                s.op_mode       = desired_op_mode;
                s.preset_target = 1;
            }
            /*  如果模式对了，就上使能 */
            else {
                s.kind         = DS402_STEP_ENABLE_OP;
                s.control_word = DS402_CW_ENABLE_OP;
                /* 去使能保留了驱动器旧目标；重新上使能前必须重建保持位置。 */
                s.preset_target = 1;
            }
        }
        /*  当前状态是operation_enabled */
        else if ((status_word & DS402_SW_MASK_OP_ENABLED) == DS402_SW_VAL_OP_ENABLED) {
            /*  如果模式不对，不是期望的CSP或CSV，则切模式 */
            if (current_mode != desired_op_mode) {
                /* 直接在使能中写 0x6060——能否成功取决于驱动器，上机确认后再用 */
                if (msw == DS402_MODESW_IN_PLACE) {
                    s.kind          = DS402_STEP_SWITCH_MODE;
                    s.op_mode       = desired_op_mode;
                    s.preset_target = 1;
                }
                /* 默认：先退回 SwitchedOn，下一拍在 0x23 改模式、再上使能 */
                else {    
                    s.kind         = DS402_STEP_DISABLE_OP;
                    s.control_word = DS402_CW_SWITCH_ON;
                }
            }
            /*  如果模式对了，就完成 */
            else {
                s.kind = DS402_STEP_DONE;
            }
        }
        return s;
    
    /* 停止使能 */
    case DS402_REQ_DISABLE:
        /* 目标：停在 SwitchedOn。只有还在 OperationEnabled 时才需要动作。 */
        if ((status_word & DS402_SW_MASK_OP_ENABLED) == DS402_SW_VAL_OP_ENABLED) {
            s.kind         = DS402_STEP_DISABLE_OP;
            s.control_word = DS402_CW_SWITCH_ON;
        }
        return s;
    /* 断电 */
    case DS402_REQ_DROP_VOLTAGE:
        /* 目标断电：（SwitchOnDisabled）。到了就不动。 */
        if ((status_word & DS402_SW_MASK_SWITCH_ON_DISABLED) != DS402_SW_VAL_SWITCH_ON_DISABLED) {
            s.kind         = DS402_STEP_DROP_VOLTAGE;
            s.control_word = DS402_CW_DISABLE_VOLTAGE;
        }
        return s;
    
    /* 故障复位 */
    case DS402_REQ_FAULT_RESET:
        /* 单次高电平动作。HAL 运行层负责低/高交替、释放及超时，
         * 直接使用本纯函数的调用方也必须产生完整的 bit7 上升沿。 */
        if (status_word & DS402_SW_BIT_FAULT) {
            s.kind         = DS402_STEP_FAULT_RESET;
            s.control_word = DS402_CW_FAULT_RESET;
        }
        return s;
    }
    return s;
}


#ifdef __cplusplus
}
#endif
