#pragma once
#include "device.h"

/* Shared dependencies; these checks only inspect handles, never authorize startup. */
static inline int ServoRoles_BaseReady(const slave_addr* h) {
#define SERVO_ROLE(name, id, index, bits, tx, type, base, speed, custom, standard, member) \
    if (base && h->member.bit_length != bits) return 0;
#include "../common/servo_roles.def"
#undef SERVO_ROLE
    return 1;
}
static inline int ServoRoles_SpeedReady(const slave_addr* h) {
#define SERVO_ROLE(name, id, index, bits, tx, type, base, speed, custom, standard, member) \
    if (speed && h->member.bit_length != bits) return 0;
#include "../common/servo_roles.def"
#undef SERVO_ROLE
    return 1;
}
