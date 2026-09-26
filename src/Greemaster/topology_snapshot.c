#include "topology_snapshot.h"

static const Slave_info* find_slave(const Slave_info* list, int pos) {
    const Slave_info* found = NULL;
    for (; list; list = list->next) {
        if (list->slave_pos == (uint32_t)pos) {
            if (found) return NULL;
            found = list;
        }
    }
    return found;
}

static int write_entries(FILE* out, const Slave_info* slave, uint32_t direction) {
    int first = 1;
    if (fputc('[', out) == EOF) return -1;
    for (const sm_info* sm = slave->sm_list; sm; sm = sm->next) {
        if (!sm->sm_Enable || sm->sm_TR != direction) continue;
        for (const pdo_info* pdo = sm->pdo_list; pdo; pdo = pdo->next) {
            for (const pdo_entry_info* entry = pdo->entry_list; entry; entry = entry->next) {
                if (!first && fputc(',', out) == EOF) return -1;
                first = 0;
                if (fprintf(out, "{\"pdo_index\":%u,\"index\":%u,"
                         "\"subindex\":%u,\"bits\":%u}",
                         pdo->pdo_index, entry->pdo_entry_index,
                         entry->pdo_entry_subindex, entry->pdo_entry_bit_length) < 0) return -1;
            }
        }
    }
    return fputc(']', out) == EOF ? -1 : 0;
}

int TopologySnapshot_Write(FILE* out, int slave_count, const Slave_info* list) {
    if (!out || slave_count < 0 || slave_count > MAX_DEVICE_NUM ||
        (slave_count > 0 && !list)) return -1;
    if (fputs("{\"schema_version\":1,\"slaves\":[", out) == EOF) return -1;
    for (int pos = 0; pos < slave_count; ++pos) {
        const Slave_info* slave = find_slave(list, pos);
        const DEVICE_BASIC_INFO* id = device_identity_get(pos);
        if (!slave || id->slave_pos != pos || id->slave_total != slave_count) return -1;
        if (pos && fputc(',', out) == EOF) return -1;
        if (fprintf(out, "{\"slave_pos\":%d,\"vendor_id\":%u,"
                    "\"product_code\":%u,\"revision\":%u,\"serial\":%u,"
                    "\"use\":\"unassigned\",\"pre_download_pdo_entries\":{\"rx\":",
                    pos, id->ID, id->CODE, id->Revision, id->Serial) < 0) return -1;
        if (write_entries(out, slave, Rx) != 0 || fputs(",\"tx\":", out) == EOF ||
            write_entries(out, slave, Tx) != 0 || fputs("}}", out) == EOF) return -1;
    }
    if (fputs("]}\n", out) == EOF) return -1;
    return ferror(out) ? -1 : 0;
}
