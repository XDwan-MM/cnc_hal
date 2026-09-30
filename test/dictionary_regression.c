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
