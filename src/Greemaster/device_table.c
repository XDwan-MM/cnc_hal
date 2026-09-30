#include "device_table.h"
#include "common/devdict.h"
#include <string.h>

static DeviceSlot g_slots[MAX_DEVICE_NUM];
static int        g_slot_count = 0;

/* 名称来自设备字典；字典未命中的留空串（设备仍会出现在表里）。 */
static void fill_name(DeviceSlot* s) {
    DevDictEntry dict;
    s->name[0] = '\0';
    if (DevDict_Lookup(s->vendor_id, s->product_code, s->revision, &dict))
        snprintf(s->name, sizeof(s->name), "%s", dict.name);
}

void DeviceTable_Reset(void) {
    memset(g_slots, 0, sizeof(g_slots));
    g_slot_count = 0;
}

int DeviceTable_IsServo(int slot) {
    if (slot < 0 || slot >= g_slot_count) return 0;
    const DEVICE_TYPE type = g_slots[slot].type;
    return type == SERVO_TYPE || type == GREE_AXIS6_TYPE || type == GREE_AXIS4_TYPE;
}

int DeviceTable_Build(int slave_num) {
    DeviceTable_Reset();
    if (slave_num < 0 || slave_num > MAX_DEVICE_NUM) return -1;
    int total = 0;
    for (int i = 0; i < slave_num; ++i) {
        const int n = device_slot_count((DEVICE_TYPE)device_identity_get(i)->type);
        if (n > MAX_DEVICE_NUM - total) return -1;
        total += n;
    }
    int slot = 0;
    for (int i = 0; i < slave_num; ++i) {
        const DEVICE_BASIC_INFO* id = device_identity_get(i);
        const int n = device_slot_count((DEVICE_TYPE)id->type);
        for (int k = 0; k < n; ++k, ++slot) {
            DeviceSlot* s = &g_slots[slot];
            s->type = (DEVICE_TYPE)id->type;
            s->slave_pos = i;
            s->axis_index = (s->type == SERVO_TYPE || s->type == GREE_AXIS6_TYPE ||
                             s->type == GREE_AXIS4_TYPE) ? k : -1;
            s->vendor_id = id->ID;
            s->product_code = id->CODE;
            s->revision = id->Revision;
            s->serial = id->Serial;
            s->entries = g_device_data[slot];
            fill_name(s);
        }
    }
    g_slot_count = slot;
    return slot;
}

int DeviceTable_Get(const DeviceSlot** out) {
    if (out) *out = (g_slot_count > 0) ? g_slots : NULL;
    return g_slot_count;
}
