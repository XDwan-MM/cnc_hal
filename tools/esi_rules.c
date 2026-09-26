/* Offline adapter for the production dictionary parser; no SDK dependency. */
#include "common/devdict.h"
#include "common/servo_roles.h"
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>

static void object(const DevDictObject* o) {
    const ServoRoleSpec* r = ServoRole_Get(o->role);
    printf("\"%s\":{\"index\":%u,\"subindex\":%u,\"bits\":%u,"
           "\"direction\":\"%s\",\"type\":\"%s\",\"base\":%d,\"speed\":%d}",
           r->name, o->index, o->sub, o->bits, o->is_tx ? "tx" : "rx", r->type, r->base, r->speed);
}
static int number(const char* text, uint32_t* out) {
    char* end;
    errno = 0;
    unsigned long long n = strtoull(text, &end, 10);
    if (!*text || *text == '-' || *end || errno || n > UINT32_MAX) return 0;
    *out = (uint32_t)n;
    return 1;
}
int main(int argc, char** argv) {
    char error[256];
    if (argc < 2 || (argc - 2) % 3) return 2;
    if (DevDict_Load(argv[1], error, sizeof(error)) != 0) {
        fprintf(stderr, "%s\n", error);
        return 1;
    }
    printf("{\"contract_version\":1,\"roles\":{");
    for (int i = 0; i < DEV_DICT_ROLE_COUNT; ++i) {
        const ServoRoleSpec* r = ServoRole_Get((DevDictRole)i);
        DevDictObject o = {r->role, r->index, 0, r->bits, r->is_tx};
        if (i) putchar(',');
        object(&o);
    }
    printf("},\"matches\":[");
    for (int a = 2; a < argc; a += 3) {
        uint32_t v, p, rev;
        if (!number(argv[a], &v) || !number(argv[a+1], &p) || !number(argv[a+2], &rev)) return 2;
        if (a != 2) putchar(',');
        DevDictEntry e;
        if (!DevDict_Lookup(v, p, rev, &e)) { printf("null"); continue; }
        /* type/profile are validated enums; no free text is interpolated. */
        printf("{\"type\":\"%s\",\"profile\":\"%s\",\"objects\":{", e.type, e.profile);
        int first = 1;
        if (strcmp(e.profile, "ds402") == 0) {
            for (int i = 0; i < DEV_DICT_ROLE_COUNT; ++i) {
                const ServoRoleSpec* r = ServoRole_Get((DevDictRole)i);
                if (!r->standard) continue;
                DevDictObject o = {r->role, r->index, 0, r->bits, r->is_tx};
                if (!first) putchar(',');
                first = 0;
                object(&o);
            }
        } else for (int i = 0; i < e.object_count; ++i) {
            if (!first) putchar(',');
            first = 0;
            object(&e.objects[i]);
        }
        printf("}}");
    }
    printf("]}\n");
    DevDict_Unload();
    return ferror(stdout) ? 1 : 0;
}
