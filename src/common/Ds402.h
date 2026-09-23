#pragma once

/* DS402 状态机 —— 与总线/驱动无关的纯编码 + 推进规则。
 * header-only、无依赖，可离线穷举测试。
 * 只回答「这一拍写什么」，不回答「什么时候开始使能」——后者归上层。 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 控制字 0x6040 */
#define DS402_CW_DISABLE_VOLTAGE 0x0000u
#define DS402_CW_SHUTDOWN        0x0006u
#define DS402_CW_SWITCH_ON       0x0007u
#define DS402_CW_ENABLE_OP       0x000Fu
#define DS402_CW_FAULT_RESET     0x0080u

/* 状态字 0x6041 的 Fault 位（bit3） */
#define DS402_SW_BIT_FAULT       0x0008u

/* 运行模式 0x6060 */
#define DS402_MODE_CSP      0x08u
#define DS402_MODE_CSV      0x09u

/* 状态字 0x6041 的判定掩码。
 * 对可选位（bit4 电压有效 / bit5 快停 / bit9 远程）保持宽松——部分固件不置这些位。 */
#define DS402_SW_MASK_SWITCH_ON_DISABLED 0x004Fu
#define DS402_SW_VAL_SWITCH_ON_DISABLED  0x0040u
#define DS402_SW_MASK_READY_TO_SWITCH_ON 0x006Fu
#define DS402_SW_VAL_READY_TO_SWITCH_ON  0x0021u
#define DS402_SW_MASK_SWITCHED_ON        0x006Fu
#define DS402_SW_VAL_SWITCHED_ON         0x0023u
#define DS402_SW_MASK_OP_ENABLED         0x027Fu
#define DS402_SW_VAL_OP_ENABLED          0x0237u

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
 *
 * preset_target 是关键：从 ReadyToSwitchOn 进 SwitchedOn 那一拍，必须先把「目标
 * 位置」设成「当前实际位置」，否则驱动会以为要走到零点，直接飞车。
 *
 * 运行模式由这里保证：current_mode != desired_op_mode 时自动切，而且**模式不对就
 * 不上使能**——调用方每拍说清「我要哪个模式」即可。
 *
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
    case DS402_REQ_NONE:
        return s;

    case DS402_REQ_ENABLE:
        if ((status_word & DS402_SW_MASK_SWITCH_ON_DISABLED) == DS402_SW_VAL_SWITCH_ON_DISABLED) {
            s.kind         = DS402_STEP_SHUTDOWN;
            s.control_word = DS402_CW_SHUTDOWN;
            s.op_mode      = desired_op_mode;
        }
        else if ((status_word & DS402_SW_MASK_READY_TO_SWITCH_ON) == DS402_SW_VAL_READY_TO_SWITCH_ON) {
            s.kind          = DS402_STEP_SWITCH_ON;
            s.control_word  = DS402_CW_SWITCH_ON;
            s.preset_target = 1;
        }
        else if ((status_word & DS402_SW_MASK_SWITCHED_ON) == DS402_SW_VAL_SWITCHED_ON) {
            /* 下使能状态下改模式——规范明确支持。
             * 也正是不变量该在的地方：「模式对了才上使能」。 */
            if (current_mode != desired_op_mode) {
                s.kind          = DS402_STEP_SWITCH_MODE;
                s.op_mode       = desired_op_mode;
                s.preset_target = 1;
            }
            else {
                s.kind         = DS402_STEP_ENABLE_OP;
                s.control_word = DS402_CW_ENABLE_OP;
            }
        }
        else if ((status_word & DS402_SW_MASK_OP_ENABLED) == DS402_SW_VAL_OP_ENABLED) {
            if (current_mode != desired_op_mode) {
                if (msw == DS402_MODESW_IN_PLACE) {
                    /* 直接在使能中写 0x6060——能否成功取决于驱动器，上机确认后再用 */
                    s.kind          = DS402_STEP_SWITCH_MODE;
                    s.op_mode       = desired_op_mode;
                    s.preset_target = 1;
                }
                else {
                    /* 默认：先退回 SwitchedOn，下一拍在 0x23 改模式、再上使能 */
                    s.kind         = DS402_STEP_DISABLE_OP;
                    s.control_word = DS402_CW_SWITCH_ON;
                }
            }
            else {
                s.kind = DS402_STEP_DONE;
            }
        }
        return s;

    case DS402_REQ_DISABLE:
        /* 目标：停在 SwitchedOn。只有还在 OperationEnabled 时才需要动作。 */
        if ((status_word & DS402_SW_MASK_OP_ENABLED) == DS402_SW_VAL_OP_ENABLED) {
            s.kind         = DS402_STEP_DISABLE_OP;
            s.control_word = DS402_CW_SWITCH_ON;
        }
        return s;

    case DS402_REQ_DROP_VOLTAGE:
        /* 目标：断电（SwitchOnDisabled）。到了就不动。 */
        if ((status_word & DS402_SW_MASK_SWITCH_ON_DISABLED) != DS402_SW_VAL_SWITCH_ON_DISABLED) {
            s.kind         = DS402_STEP_DROP_VOLTAGE;
            s.control_word = DS402_CW_DISABLE_VOLTAGE;
        }
        return s;

    case DS402_REQ_FAULT_RESET:
        /* 清错要上升沿：只在 Fault 时写一次 0x80。
         * 写完调用方应把请求改回 ENABLE/DISABLE，否则每拍都会重复写。 */
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
