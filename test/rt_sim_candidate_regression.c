/* Candidate-only test; compile with -DRT_SIM_SOURCE pointing to the copied RT backend.
 * Including the simulator permits injecting a second fault while bit7 remains high.
 * No real SDK library is linked. */
#ifndef RT_SIM_SOURCE
#error "Define RT_SIM_SOURCE to the candidate RT simulation source"
#endif
#include RT_SIM_SOURCE
#include <assert.h>

static unsigned checks;
#define CHECK(expr) do { ++checks; assert(expr); } while (0)

int main(void) {
    MasterConfig cfg = {1000, 10000, 1000, 0};
    CHECK(Master_StopFlag());
    CHECK(Master_WaitCycle() != 0 && Master_CommitCycle() != 0);
    CHECK(ethercat_init(&cfg) == 0);
    CHECK(!Master_StopFlag());
    const DEVICE_BASIC_INFO* id = device_identity_get(SIM_SLAVE_X);
    CHECK(id->ID == s_slots[SIM_SLAVE_X].vendor_id);
    CHECK(id->CODE == s_slots[SIM_SLAVE_X].product_code);
    CHECK(id->type == SERVO_TYPE);
    CHECK(device_identity_get(-1)->ID == 0);
    CHECK(device_identity_get(SIM_SLOT_COUNT)->ID == 0);
    CHECK(Master_WaitCycle() == 0);
    s_servo[SIM_SLAVE_SPINDLE].fault = 1;
    CHECK(Master_ServoWrite(SIM_SLAVE_SPINDLE, DEV_DICT_ROLE_CONTROL_WORD, 0x80) == 0);
    CHECK(Master_CommitCycle() == 0 && !s_servo[SIM_SLAVE_SPINDLE].fault);
    CHECK(s_servo[SIM_SLAVE_SPINDLE].control_word == 0x80);
    s_servo[SIM_SLAVE_SPINDLE].fault = 1;
    CHECK(Master_CommitCycle() == 0 && s_servo[SIM_SLAVE_SPINDLE].fault);
    CHECK(Master_ServoWrite(SIM_SLAVE_SPINDLE, DEV_DICT_ROLE_CONTROL_WORD, 0) == 0);
    CHECK(Master_CommitCycle() == 0 && s_servo[SIM_SLAVE_SPINDLE].fault);
    CHECK(Master_ServoWrite(SIM_SLAVE_SPINDLE, DEV_DICT_ROLE_CONTROL_WORD, 0x80) == 0);
    CHECK(Master_CommitCycle() == 0 && !s_servo[SIM_SLAVE_SPINDLE].fault);
    Master_RequestStop();
    CHECK(Master_StopFlag());
    CHECK(Master_WaitCycle() == MASTER_STOP_REQUESTED);
    CHECK(Master_CommitCycle() == MASTER_STOP_REQUESTED);
    CHECK(ethercat_close() == 0 && Master_StopFlag());
    CHECK(device_identity_get(SIM_SLAVE_X)->ID == 0);
    CHECK(ethercat_init(&cfg) == 0 && !Master_StopFlag());
    CHECK(Master_WaitCycle() == 0 && Master_CommitCycle() == 0);
    CHECK(ethercat_close() == 0);
    printf("RT simulation candidate: %u checks passed\n", checks);
}
