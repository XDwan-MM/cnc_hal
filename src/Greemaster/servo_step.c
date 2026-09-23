#include "servo_step.h"
#include "device.h"
#include "device_table.h"

static Ds402ModeSwitch g_msw[MAX_DEVICE_NUM];

void Master_ServoReset(void) {
    memset(g_msw, 0, sizeof(g_msw));
}

MASTER_API int Master_ServoSetModeSwitch(int slot, Ds402ModeSwitch how) {
    if (!DeviceTable_IsServo(slot) ||
        (how != DS402_MODESW_DISABLE_FIRST && how != DS402_MODESW_IN_PLACE)) return -1;
    g_msw[slot] = how;
    return 0;
}

MASTER_API int Master_ServoGetModeSwitch(int slot, Ds402ModeSwitch* out) {
    if (!DeviceTable_IsServo(slot) || !out) return -1;
    *out = g_msw[slot];
    return 0;
}

MASTER_API int Master_ServoStep(int slot, Ds402Request req,
                                uint16_t desired_op_mode, uint16_t* sw_out) {
    if (!DeviceTable_IsServo(slot) || req < DS402_REQ_ENABLE || req > DS402_REQ_NONE)
        return -1;
    if (req == DS402_REQ_ENABLE &&
        desired_op_mode != DS402_MODE_CSP && desired_op_mode != DS402_MODE_CSV)
        return -1;

    const slave_addr* axis = &g_device_data[slot].slave;
    uint16_t sw = 0;
    if (axis->statusWord.bit_length != 16 ||
        GM_TxPdoEntry_Read(axis->statusWord, &sw, sizeof(sw)) != 0) return -1;
    if (sw_out) *sw_out = sw;

    uint8_t cur_mode = 0;
    /* 去使能、断电和清错不能依赖模式反馈可用。 */
    if (req == DS402_REQ_ENABLE &&
        (axis->act_mode.bit_length != 8 ||
         GM_TxPdoEntry_Read(axis->act_mode, &cur_mode, sizeof(cur_mode)) != 0))
        return -1;

    const Ds402Step step = Ds402_NextStepReq(sw, cur_mode, req, desired_op_mode, g_msw[slot]);
    const int write_control = step.kind != DS402_STEP_NONE &&
                              step.kind != DS402_STEP_DONE &&
                              step.kind != DS402_STEP_SWITCH_MODE;
    if ((write_control && axis->Control_word.bit_length != 16) ||
        (step.op_mode != 0xFFFFu && axis->Modes_of_operation.bit_length != 8) ||
        (step.preset_target &&
         (axis->act_pos.bit_length != 32 || axis->target_pos.bit_length != 32)))
        return -1;

    /* 预置成功后才改模式；任何失败都不能继续写后续控制量。 */
    if (step.preset_target) {
        uint32_t actual = 0;
        if (GM_TxPdoEntry_Read(axis->act_pos, &actual, sizeof(actual)) != 0 ||
            GM_RxPdoEntry_Write(axis->target_pos, actual) != 0) return -1;
    }
    if (step.op_mode != 0xFFFFu &&
        GM_RxPdoEntry_Write(axis->Modes_of_operation, step.op_mode) != 0) return -1;
    if (write_control && GM_RxPdoEntry_Write(axis->Control_word, step.control_word) != 0)
        return -1;
    return 0;
}
