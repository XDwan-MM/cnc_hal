#pragma once
#include "devdict.h"

typedef struct {
    const char* name;
    DevDictRole role;
    uint16_t index;
    uint8_t bits;
    int is_tx;
    const char* type;
    int base, speed, custom_required, standard;
} ServoRoleSpec;

static inline const ServoRoleSpec* ServoRole_Get(DevDictRole role) {
    static const ServoRoleSpec specs[DEV_DICT_ROLE_COUNT] = {
#define SERVO_ROLE(name, id, index, bits, tx, type, base, speed, custom, standard, member) \
        [DEV_DICT_ROLE_##id] = {#name, DEV_DICT_ROLE_##id, index, bits, tx, #type, base, speed, custom, standard},
#include "servo_roles.def"
#undef SERVO_ROLE
    };
    return role >= 0 && role < DEV_DICT_ROLE_COUNT ? &specs[role] : 0;
}
