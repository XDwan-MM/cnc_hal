/* DS402 推进规则的离线穷举验证。
 * 纯函数，不需要任何硬件——这是它被抽出来的原因。 */
#include "common/Ds402.h"
#include <stdio.h>

static int fails = 0;
static void chk(int c, const char* w) {
    printf("  %-48s %s\n", w, c ? "OK" : "<-- FAIL");
    if (!c) fails++;
}

/* 「模式已一致时的使能梯子」——用来测梯子本身，不掺模式切换。
 * 等价于改制前那个 Ds402_NextStep()。 */
static Ds402Step step_enable(uint16_t sw, uint16_t mode) {
    return Ds402_NextStepReq(sw, mode, DS402_REQ_ENABLE, mode, DS402_MODESW_DISABLE_FIRST);
}

int main(void) {
    Ds402Step s;

    printf("四个稳定状态字（照 cnc_rt Master_Run 的掩码）：\n");
    s = step_enable(0x0040, DS402_MODE_CSP);
    chk(s.kind == DS402_STEP_SHUTDOWN,   "0x40  SwitchOnDisabled → SHUTDOWN");
    chk(s.control_word == 0x0006,        "      控制字 = 0x06");
    chk(s.op_mode == DS402_MODE_CSP,     "      运行模式 = CSP(0x08)");
    chk(s.preset_target == 0,            "      不预置目标位置");

    s = step_enable(0x0021, DS402_MODE_CSP);
    chk(s.kind == DS402_STEP_SWITCH_ON,  "0x21  ReadyToSwitchOn  → SWITCH_ON");
    chk(s.control_word == 0x0007,        "      控制字 = 0x07");
    chk(s.preset_target == 1,            "      预置目标位置（防飞车）");
    chk(s.op_mode == 0xFFFF,             "      不改运行模式");

    s = step_enable(0x0023, DS402_MODE_CSP);
    chk(s.kind == DS402_STEP_ENABLE_OP,  "0x23  SwitchedOn       → ENABLE_OP");
    chk(s.control_word == 0x000F,        "      控制字 = 0x0F");
    chk(s.preset_target == 0,            "      不预置");

    s = step_enable(0x0237, DS402_MODE_CSP);
    chk(s.kind == DS402_STEP_DONE,       "0x237 OperationEnabled → DONE");
    chk(s.preset_target == 0,            "      不预置");

    printf("\n掩码之外的位置不影响判定（宽松）：\n");
    chk(step_enable(0x0040 | 0x0010, DS402_MODE_CSP).kind == DS402_STEP_SHUTDOWN,
        "0x40 | bit4(电压有效) 仍识别");
    chk(step_enable(0x0040 | 0x0020, DS402_MODE_CSP).kind == DS402_STEP_SHUTDOWN,
        "0x40 | bit5(快停)     仍识别");
    chk(step_enable(0x0237 | 0x0080, DS402_MODE_CSP).kind == DS402_STEP_DONE,
        "0x237 | bit7          仍识别");
    chk(step_enable(0x0237 | 0x0100, DS402_MODE_CSP).kind == DS402_STEP_DONE,
        "0x237 | bit8          仍识别");

    printf("\n不认识的状态字 → 不动：\n");
    chk(step_enable(0x0000, DS402_MODE_CSP).kind == DS402_STEP_NONE, "0x00 初始态 → NONE");
    chk(step_enable(0x0008, DS402_MODE_CSP).kind == DS402_STEP_NONE, "0x08 Fault  → NONE");
    chk(step_enable(0x0007, DS402_MODE_CSP).kind == DS402_STEP_NONE, "0x07 QuickStop → NONE");

    printf("\n完整序列要 4 拍到 DONE：\n");
    {
        const uint16_t seq[]  = {0x0040, 0x0021, 0x0023, 0x0237};
        const Ds402StepKind w[] = {DS402_STEP_SHUTDOWN, DS402_STEP_SWITCH_ON,
                                   DS402_STEP_ENABLE_OP, DS402_STEP_DONE};
        int ok = 1;
        for (int i = 0; i < 4; ++i)
            if (step_enable(seq[i], DS402_MODE_CSP).kind != w[i]) ok = 0;
        chk(ok, "0x40 → 0x21 → 0x23 → 0x237 每拍动作正确");
    }

    printf("\n主轴用 CSV 时模式跟着变：\n");
    chk(step_enable(0x0040, DS402_MODE_CSV).op_mode == DS402_MODE_CSV,
        "0x40 + 期望 CSV → op_mode = 0x09");

    /* ---------- 请求机制（新增） ---------- */
    printf("\nREQ_NONE —— 任何状态字都不动：\n");
    chk(Ds402_NextStepReq(0x0040, DS402_MODE_CSP, DS402_REQ_NONE, DS402_MODE_CSP, DS402_MODESW_DISABLE_FIRST).kind == DS402_STEP_NONE,
        "0x40  不动");
    chk(Ds402_NextStepReq(0x0237, DS402_MODE_CSP, DS402_REQ_NONE, DS402_MODE_CSP, DS402_MODESW_DISABLE_FIRST).kind == DS402_STEP_NONE,
        "0x237 不动");
    chk(Ds402_NextStepReq(0x0008, DS402_MODE_CSP, DS402_REQ_NONE, DS402_MODE_CSP, DS402_MODESW_DISABLE_FIRST).kind == DS402_STEP_NONE,
        "Fault 不动");

    printf("\nREQ_DISABLE —— 停在 SwitchedOn：\n");
    {
        const Ds402Step s = Ds402_NextStepReq(0x0237, DS402_MODE_CSP, DS402_REQ_DISABLE, DS402_MODE_CSP, DS402_MODESW_DISABLE_FIRST);
        chk(s.kind == DS402_STEP_DISABLE_OP && s.control_word == 0x0007,
            "OperationEnabled → 写 0x07 退回去");
    }
    chk(Ds402_NextStepReq(0x0023, DS402_MODE_CSP, DS402_REQ_DISABLE, DS402_MODE_CSP, DS402_MODESW_DISABLE_FIRST).kind == DS402_STEP_NONE,
        "已在 SwitchedOn → 不动（已达成）");

    printf("\nREQ_DROP_VOLTAGE —— 断电：\n");
    {
        const Ds402Step s = Ds402_NextStepReq(0x0237, DS402_MODE_CSP, DS402_REQ_DROP_VOLTAGE, DS402_MODE_CSP, DS402_MODESW_DISABLE_FIRST);
        chk(s.kind == DS402_STEP_DROP_VOLTAGE && s.control_word == 0x0000,
            "OperationEnabled → 写 0x00 断电");
    }
    chk(Ds402_NextStepReq(0x0040, DS402_MODE_CSP, DS402_REQ_DROP_VOLTAGE, DS402_MODE_CSP, DS402_MODESW_DISABLE_FIRST).kind == DS402_STEP_NONE,
        "已断电（0x40）→ 不动");

    printf("\nREQ_FAULT_RESET —— 只在 Fault 时清：\n");
    {
        const Ds402Step s = Ds402_NextStepReq(0x0008, DS402_MODE_CSP, DS402_REQ_FAULT_RESET, DS402_MODE_CSP, DS402_MODESW_DISABLE_FIRST);
        chk(s.kind == DS402_STEP_FAULT_RESET && s.control_word == 0x0080,
            "Fault(bit3) → 写 0x80");
    }
    chk(Ds402_NextStepReq(0x0237, DS402_MODE_CSP, DS402_REQ_FAULT_RESET, DS402_MODE_CSP, DS402_MODESW_DISABLE_FIRST).kind == DS402_STEP_NONE,
        "非 Fault → 不动（不会误清）");
    chk(Ds402_NextStepReq(0x0040, DS402_MODE_CSP, DS402_REQ_FAULT_RESET, DS402_MODE_CSP, DS402_MODESW_DISABLE_FIRST).kind == DS402_STEP_NONE,
        "SwitchOnDisabled → 不动");

    /* ---------- 运行模式切换（新增） ---------- */
    printf("\n模式切换 · 默认策略 = 先下使能（DISABLE_FIRST）：\n");
    {
        const Ds402Step s = Ds402_NextStepReq(0x0237, DS402_MODE_CSV, DS402_REQ_ENABLE,
                                              DS402_MODE_CSP, DS402_MODESW_DISABLE_FIRST);
        chk(s.kind == DS402_STEP_DISABLE_OP && s.control_word == 0x0007,
            "0x237 + 模式不对 → 先写 0x07 退回 SwitchedOn");
    }
    {
        const Ds402Step s = Ds402_NextStepReq(0x0023, DS402_MODE_CSV, DS402_REQ_ENABLE,
                                              DS402_MODE_CSP, DS402_MODESW_DISABLE_FIRST);
        chk(s.kind == DS402_STEP_SWITCH_MODE, "0x23  + 模式不对 → 写 0x6060");
        chk(s.op_mode == DS402_MODE_CSP,      "      要写 CSP(0x08)");
        chk(s.preset_target == 1,             "      预置目标位置（防飞车）");
        chk(s.control_word == 0,              "      不写控制字");
    }
    {
        const Ds402Step s = Ds402_NextStepReq(0x0023, DS402_MODE_CSP, DS402_REQ_ENABLE,
                                              DS402_MODE_CSP, DS402_MODESW_DISABLE_FIRST);
        chk(s.kind == DS402_STEP_ENABLE_OP && s.control_word == 0x000F,
            "0x23  + 模式对了 → 才写 0x0F 上使能");
    }

    printf("\n模式切换 · 使能中直接切（IN_PLACE）：\n");
    {
        const Ds402Step s = Ds402_NextStepReq(0x0237, DS402_MODE_CSV, DS402_REQ_ENABLE,
                                              DS402_MODE_CSP, DS402_MODESW_IN_PLACE);
        chk(s.kind == DS402_STEP_SWITCH_MODE, "0x237 + 模式不对 → 直接写 0x6060");
        chk(s.op_mode == DS402_MODE_CSP,      "      要写 CSP(0x08)");
        chk(s.preset_target == 1,             "      预置目标位置");
        chk(s.control_word == 0,              "      不写控制字");
    }

    printf("\n模式一致时两种策略都不切：\n");
    chk(Ds402_NextStepReq(0x0237, DS402_MODE_CSP, DS402_REQ_ENABLE, DS402_MODE_CSP,
                          DS402_MODESW_DISABLE_FIRST).kind == DS402_STEP_DONE,
        "DISABLE_FIRST + 模式一致 → DONE");
    chk(Ds402_NextStepReq(0x0237, DS402_MODE_CSP, DS402_REQ_ENABLE, DS402_MODE_CSP,
                          DS402_MODESW_IN_PLACE).kind == DS402_STEP_DONE,
        "IN_PLACE      + 模式一致 → DONE");

    printf("\n模式检查只在 REQ_ENABLE 时做：\n");
    chk(Ds402_NextStepReq(0x0237, DS402_MODE_CSV, DS402_REQ_DISABLE, DS402_MODE_CSP,
                          DS402_MODESW_DISABLE_FIRST).kind == DS402_STEP_DISABLE_OP,
        "REQ_DISABLE 时不切模式");
    chk(Ds402_NextStepReq(0x0237, DS402_MODE_CSV, DS402_REQ_NONE, DS402_MODE_CSP,
                          DS402_MODESW_DISABLE_FIRST).kind == DS402_STEP_NONE,
        "REQ_NONE 时什么都不做");

    printf("\n★ 上电路径不受影响（模式在 0x40 就设好了）：\n");
    {
        const uint16_t seq[] = {0x0040, 0x0021, 0x0023, 0x0237};
        const Ds402StepKind w[] = {DS402_STEP_SHUTDOWN, DS402_STEP_SWITCH_ON,
                                   DS402_STEP_ENABLE_OP, DS402_STEP_DONE};
        int ok = 1;
        for (int i = 0; i < 4; ++i)
            if (Ds402_NextStepReq(seq[i], DS402_MODE_CSP, DS402_REQ_ENABLE, DS402_MODE_CSP,
                                  DS402_MODESW_DISABLE_FIRST).kind != w[i]) ok = 0;
        chk(ok, "模式一路正确 → 还是 4 拍到 DONE");
    }

    printf("\n完整的「往上 → 往回」往返：\n");
    {
        int ok = 1;
        /* 使能：0x40 → 0x21 → 0x23 → 0x237 */
        if (Ds402_NextStepReq(0x0040, DS402_MODE_CSP, DS402_REQ_ENABLE, DS402_MODE_CSP, DS402_MODESW_DISABLE_FIRST).kind != DS402_STEP_SHUTDOWN) ok = 0;
        if (Ds402_NextStepReq(0x0237, DS402_MODE_CSP, DS402_REQ_ENABLE, DS402_MODE_CSP, DS402_MODESW_DISABLE_FIRST).kind != DS402_STEP_DONE)     ok = 0;
        /* 去使能：0x237 → 0x23 */
        if (Ds402_NextStepReq(0x0237, DS402_MODE_CSP, DS402_REQ_DISABLE, DS402_MODE_CSP, DS402_MODESW_DISABLE_FIRST).kind != DS402_STEP_DISABLE_OP) ok = 0;
        if (Ds402_NextStepReq(0x0023, DS402_MODE_CSP, DS402_REQ_DISABLE, DS402_MODE_CSP, DS402_MODESW_DISABLE_FIRST).kind != DS402_STEP_NONE)       ok = 0;
        /* 断电：0x23 → 0x40 */
        if (Ds402_NextStepReq(0x0023, DS402_MODE_CSP, DS402_REQ_DROP_VOLTAGE, DS402_MODE_CSP, DS402_MODESW_DISABLE_FIRST).control_word != 0x0000)   ok = 0;
        if (Ds402_NextStepReq(0x0040, DS402_MODE_CSP, DS402_REQ_DROP_VOLTAGE, DS402_MODE_CSP, DS402_MODESW_DISABLE_FIRST).kind != DS402_STEP_NONE)  ok = 0;
        chk(ok, "使能 → 去使能 → 断电，每步都对");
    }

    printf("\n%s（%d 处失败）\n", fails ? "有问题" : "全部通过", fails);
    return fails ? 1 : 0;
}
