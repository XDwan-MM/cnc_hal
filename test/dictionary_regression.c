#include "common/devdict.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "line %d: %s\n", __LINE__, #x); return 1; } } while (0)
static int fail_at, malloc_calls;
void* __real_malloc(size_t size);
void* __wrap_malloc(size_t size) {
    if (fail_at && ++malloc_calls == fail_at) return NULL;
    return __real_malloc(size);
}

int main(int argc, char** argv) {
    char error[256];
    CHECK(argc > 1);
    for (int i = 2; i < argc; ++i) {
        CHECK(DevDict_Load(argv[1], error, sizeof(error)) == 0);
        CHECK(DevDict_EntryCount() == 11);
        const int expected_ok = argv[i][0] == '+';
        const int rc = DevDict_Load(argv[i]+1, error, sizeof(error));
        if ((rc == 0) != expected_ok) {
            fprintf(stderr, "case %s rc=%d error=%s\n", argv[i], rc, error);
            return 1;
        }
        if (!expected_ok) CHECK(DevDict_EntryCount() == 0);
        if (expected_ok) {
            /* 2026-10-04：合法用例原先只断言 rc==0，"收下了"不等于"解析对了"。
             * 用例文件名由 run_driver_tests.py 生成，这里按名字分支核对解析结果。 */
            const char* path = argv[i] + 1;
            DevDictEntry e;
            DevDictObject o;
            if (strstr(path, "custom")) {
                CHECK(DevDict_Lookup(1, 2, 0, &e) == 1);
                CHECK(strcmp(e.profile, "custom") == 0 && e.object_count == 9);
                CHECK(DevDict_FindObject(&e, DEV_DICT_ROLE_STATUS_WORD, &o) == 1 &&
                      o.index == 0x6041 && o.bits == 16 && o.is_tx == 1);
                CHECK(DevDict_FindObject(&e, DEV_DICT_ROLE_TARGET_POS, &o) == 1 &&
                      o.index == 0x607A && o.bits == 32 && o.is_tx == 0);
                CHECK(DevDict_FindObject(&e, DEV_DICT_ROLE_OP_MODE, &o) == 1 &&
                      o.index == 0x6060 && o.bits == 8 && o.is_tx == 0);
            } else if (strstr(path, "hex")) {
                /* "0xFFFFFFFF" 必须按十六进制解析 */
                CHECK(DevDict_Lookup(0xFFFFFFFFu, 2, 0, &e) == 1);
            } else if (strstr(path, "decimal_string")) {
                /* "010" 是**十进制字符串**：必须解析成 10（按八进制会给 8） */
                CHECK(DevDict_Lookup(10, 2, 0, &e) == 1);
                CHECK(DevDict_Lookup(8, 2, 0, &e) == 0);
            } else if (strstr(path, "unicode")) {
                CHECK(DevDict_Lookup(1, 2, 0, &e) == 1);
                CHECK(strcmp(e.name, "中文😀") == 0);
            } else if (strstr(path, "valid")) {
                CHECK(DevDict_Lookup(1, 2, 12345, &e) == 1);   /* revision="*" 通配 */
                CHECK(strcmp(e.type, "servo") == 0 && strcmp(e.profile, "ds402") == 0);
                CHECK(strcmp(e.name, "Servo") == 0);
            }
        }
    }
    CHECK(DevDict_Load(argv[1], error, sizeof(error)) == 0);
    CHECK(DevDict_Load(NULL, error, sizeof(error)) < 0 && DevDict_EntryCount() == 0);
    int succeeded = 0;
    for (int i = 1; i < 2000; ++i) {
        fail_at = i; malloc_calls = 0;
        int rc = DevDict_Load(argv[1], error, sizeof(error));
        fail_at = 0;
        if (rc == 0) { succeeded = 1; break; }
        CHECK(DevDict_EntryCount() == 0);
    }
    CHECK(succeeded);
    DevDict_Unload();
    printf("dictionary_regression: %d cases passed\n", argc-2);
    return 0;
}
