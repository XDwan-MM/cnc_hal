// devdict.c — 设备字典 JSON 读取器（自包含，严格 C99，无 cJSON 依赖）
// 内置一个仅覆盖所需语法的最小 JSON 解析器 + schema 走查 + 两级匹配。
// 全静态存储、启动时一次加载、单线程使用——见 devdict.h 顶部说明。
#include "devdict.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <limits.h>

// ============================================================
// 最小 JSON 解析器（DOM）
// ============================================================

enum JType { JV_OBJ, JV_ARR, JV_STR, JV_NUM, JV_LIT };   // LIT: true/false/null

typedef struct JNode {
    enum JType type;
    char* key;            // 所在父对象里的键（数组元素为 NULL）
    char* str;            // JV_STR
    long long num;        // JV_NUM / JV_LIT(true=1, false/null=0)
    struct JNode* child;  // OBJ/ARR 第一个子节点
    struct JNode* next;   // 兄弟节点
} JNode;

typedef struct {
    const char* s;
    size_t pos;
    size_t len;
    char err[96];
    int depth;
} JP;

#define JD_MAX_DEPTH 16

/**
 * @brief 记录第一条解析错误（含字符偏移），后续错误不覆盖。
 * @note 只记第一条——JSON 语法错往往连环报，第一条最有参考价值。
 */
static void jfail(JP* p, const char* msg) {
    if (p->err[0] == '\0')
        snprintf(p->err, sizeof(p->err), "%s (偏移 %d)", msg, (int)p->pos);
}

/**
 * @brief 跳过空白与注释。
 * @note 支持 `//` 行注释与块注释——这是**对 JSON 的扩展**，为的是让
 *       手编的字典文件能写说明。标准 JSON 不支持注释，换 cJSON 实现时
 *       需要把这个扩展一起带过去，否则原有字典文件会解析失败。
 */
static void jskip(JP* p) {
    while (p->pos < p->len) {
        const char c = p->s[p->pos];
        if (c == ' ' || c == '\t' || c == '\r' || c == '\n') { ++p->pos; continue; }
        // 注释扩展：// 行注释 与块注释（便于手编文件）
        if (c == '/' && p->pos + 1 < p->len && p->s[p->pos + 1] == '/') {
            while (p->pos < p->len && p->s[p->pos] != '\n') ++p->pos;
            continue;
        }
        if (c == '/' && p->pos + 1 < p->len && p->s[p->pos + 1] == '*') {
            p->pos += 2;
            while (p->pos + 1 < p->len &&
                   !(p->s[p->pos] == '*' && p->s[p->pos + 1] == '/')) ++p->pos;
            if (p->pos + 1 >= p->len) {
                jfail(p, "注释未闭合");
                p->pos = p->len;
                return;
            }
            p->pos += 2;
            continue;
        }
        break;
    }
}

/**
 * @brief 看下一个有效字符（先跳过空白/注释），不消费。
 * @return 字符；已到结尾返回 -1。
 */
static int jpeek(JP* p) {
    jskip(p);
    return p->pos < p->len ? (unsigned char)p->s[p->pos] : -1;
}

/**
 * @brief 若下一个有效字符是 c 则消费并返回 1，否则返回 0（不消费）。
 */
static int jmatch(JP* p, char c) {
    if (jpeek(p) == (unsigned char)c) { ++p->pos; return 1; }
    return 0;
}

static int hex4(const char* text, unsigned* value) {
    *value = 0;
    for (int i = 0; i < 4; ++i) {
        unsigned char c = (unsigned char)text[i];
        unsigned digit;
        if (c >= '0' && c <= '9') digit = c - '0';
        else if (c >= 'a' && c <= 'f') digit = c - 'a' + 10;
        else if (c >= 'A' && c <= 'F') digit = c - 'A' + 10;
        else return 0;
        *value = (*value << 4) | digit;
    }
    return 1;
}

static char* jparse_string_raw(JP* p) {
    if (!jmatch(p, '"')) { jfail(p, "应为字符串"); return NULL; }
    const size_t start = p->pos;
    while (p->pos < p->len && p->s[p->pos] != '"') {
        if (p->s[p->pos] == '\\' && p->pos + 1 < p->len) ++p->pos;
        ++p->pos;
    }
    if (p->pos >= p->len) { jfail(p, "字符串未闭合"); return NULL; }
    const size_t raw_len = p->pos - start;
    char* out = malloc(raw_len + 1);
    if (!out) { jfail(p, "内存不足"); return NULL; }
    size_t n = 0;
    for (size_t i = 0; i < raw_len; ++i) {
        unsigned char c = (unsigned char)p->s[start + i];
        if (c < 0x20) goto invalid;
        if (c != '\\') { out[n++] = (char)c; continue; }
        if (++i >= raw_len) goto invalid;
        c = (unsigned char)p->s[start + i];
        switch (c) {
        case '"': case '\\': case '/': out[n++] = (char)c; break;
        case 'b': out[n++] = '\b'; break;
        case 'f': out[n++] = '\f'; break;
        case 'n': out[n++] = '\n'; break;
        case 'r': out[n++] = '\r'; break;
        case 't': out[n++] = '\t'; break;
        case 'u': {
            unsigned cp;
            if (raw_len - i <= 4 || !hex4(p->s + start + i + 1, &cp)) goto invalid;
            i += 4;
            if (cp >= 0xD800 && cp <= 0xDBFF) {
                unsigned low;
                if (raw_len - i <= 6 || p->s[start+i+1] != '\\' ||
                    p->s[start+i+2] != 'u' || !hex4(p->s+start+i+3, &low) ||
                    low < 0xDC00 || low > 0xDFFF) goto invalid;
                cp = 0x10000 + ((cp - 0xD800) << 10) + low - 0xDC00;
                i += 6;
            } else if (cp >= 0xDC00 && cp <= 0xDFFF) goto invalid;
            if (cp == 0) goto invalid; /* C 字符串不能保存内嵌 NUL。 */
            if (cp < 0x80) out[n++] = (char)cp;
            else if (cp < 0x800) {
                out[n++] = (char)(0xC0 | (cp >> 6));
                out[n++] = (char)(0x80 | (cp & 63));
            } else if (cp < 0x10000) {
                out[n++] = (char)(0xE0 | (cp >> 12));
                out[n++] = (char)(0x80 | ((cp >> 6) & 63));
                out[n++] = (char)(0x80 | (cp & 63));
            } else {
                out[n++] = (char)(0xF0 | (cp >> 18));
                out[n++] = (char)(0x80 | ((cp >> 12) & 63));
                out[n++] = (char)(0x80 | ((cp >> 6) & 63));
                out[n++] = (char)(0x80 | (cp & 63));
            }
            break;
        }
        default: goto invalid;
        }
    }
    out[n] = 0;
    ++p->pos;
    return out;
invalid:
    jfail(p, "字符串含非法字符或转义");
    free(out);
    return NULL;
}

/**
 * @brief 分配一个 AST 节点并初始化各字段为 NULL/0。
 * @return 节点；内存不足返回 NULL。
 * @note 字典只在启动期 Load 一次，这里的 malloc 不在周期路径上。
 */
static JNode* jnew(enum JType t) {
    JNode* n = (JNode*)malloc(sizeof(JNode));
    if (!n) return NULL;
    n->type = t; n->key = NULL; n->str = NULL;
    n->num = 0; n->child = NULL; n->next = NULL;
    return n;
}

/**
 * @brief 递归释放以 n 为头的一条兄弟链（含各自子树）。
 * @note 先取 next 再递归 child，避免释放后访问；入参可为 NULL。
 */
static void jfree(JNode* n) {
    while (n) {
        JNode* nx = n->next;
        jfree(n->child);
        free(n->key);
        free(n->str);
        free(n);
        n = nx;
    }
}

static JNode* jparse_value(JP* p, char* key);

/**
 * @brief 解析对象或数组（两者共用同一套节点结构：child 串成链表）。
 * @param p     [in,out] 解析器状态。
 * @param isObj 1=对象 `{...}`（元素带 key），0=数组 `[...]`（key 为 NULL）。
 * @param key   本节点自身的键名（由调用方传入并接管所有权）；根节点传 NULL。
 * @return 节点；语法错或内存不足返回 NULL（错误已记进 p->err）。
 * @note 深度上限 JD_MAX_DEPTH，超限即失败——防畸形文件把栈打爆。
 */
static JNode* jparse_obj_or_arr(JP* p, int isObj, char* key) {
    JNode* n = jnew(isObj ? JV_OBJ : JV_ARR);
    if (!n) { jfail(p, "内存不足"); free(key); return NULL; }
    n->key = key;
    ++p->pos;   // { 或 [
    if (++p->depth > JD_MAX_DEPTH) { jfail(p, "嵌套过深"); jfree(n); return NULL; }

    JNode** tail = &n->child;
    if (jmatch(p, isObj ? '}' : ']')) { --p->depth; return n; }
    for (;;) {
        char* k = NULL;
        if (isObj) {
            jskip(p);
            k = jparse_string_raw(p);
            if (!k) { jfree(n); return NULL; }
            for (JNode* old = n->child; old; old = old->next) {
                if (strcmp(old->key, k) == 0) {
                    jfail(p, "对象键重复");
                    free(k);
                    jfree(n);
                    return NULL;
                }
            }
            jskip(p);
            if (!jmatch(p, ':')) {
                jfail(p, "对象键后应为 ':'");
                free(k);
                jfree(n);
                return NULL;
            }
        }
        JNode* v = jparse_value(p, k);
        if (!v) { jfree(n); return NULL; }
        *tail = v;
        tail = &v->next;
        if (jmatch(p, ',')) continue;
        if (jmatch(p, isObj ? '}' : ']')) { --p->depth; return n; }
        jfail(p, isObj ? "对象应为 ',' 或 '}'" : "数组应为 ',' 或 ']'");
        jfree(n);
        return NULL;
    }
}

/**
 * @brief 按当前字符分派到 对象/数组/字符串/数字/字面量 的解析。
 * @param key 本节点的键名（非对象成员传 NULL）。
 * @return 节点；语法错返回 NULL。
 */
static JNode* jparse_value(JP* p, char* key) {
    const int c = jpeek(p);
    if (c == '{') return jparse_obj_or_arr(p, 1, key);
    if (c == '[') return jparse_obj_or_arr(p, 0, key);
    if (c == '"') {
        JNode* n = jnew(JV_STR);
        if (!n) { jfail(p, "内存不足"); free(key); return NULL; }
        n->key = key;
        n->str = jparse_string_raw(p);
        if (!n->str) { jfree(n); return NULL; }
        return n;
    }
    if (c == '-' || (c >= '0' && c <= '9')) {
        JNode* n = jnew(JV_NUM);
        if (!n) { jfail(p, "内存不足"); free(key); return NULL; }
        n->key = key;
        const size_t start = p->pos;
        if (p->s[p->pos] == '-') ++p->pos;
        const size_t digits = p->pos;
        while (p->pos < p->len && p->s[p->pos] >= '0' && p->s[p->pos] <= '9')
            ++p->pos;
        char* end = NULL;
        errno = 0;
        n->num = strtoll(p->s + start, &end, 10);
        if (p->pos == digits || (p->pos - digits > 1 && p->s[digits] == '0') ||
            errno == ERANGE || end != p->s + p->pos ||
            (p->pos < p->len && (p->s[p->pos] == '.' ||
                                p->s[p->pos] == 'e' || p->s[p->pos] == 'E'))) {
            jfail(p, "数值必须为有效整数");
            jfree(n);
            return NULL;
        }
        return n;
    }
    if (c == 't' || c == 'f' || c == 'n') {
        JNode* n = jnew(JV_LIT);
        if (!n) { jfail(p, "内存不足"); free(key); return NULL; }
        n->key = key;
        if (p->pos + 4 <= p->len && memcmp(p->s + p->pos, "true", 4) == 0) {
            n->num = 1; p->pos += 4;
        } else if (p->pos + 5 <= p->len && memcmp(p->s + p->pos, "false", 5) == 0) {
            p->pos += 5;
        } else if (p->pos + 4 <= p->len && memcmp(p->s + p->pos, "null", 4) == 0) {
            p->pos += 4;
        } else {
            jfail(p, "非法字面量");
            jfree(n);
            return NULL;
        }
        return n;
    }
    jfail(p, "无法识别的值");
    free(key);
    return NULL;
}

// ---------- DOM 查询辅助 ----------

static JNode* jobj_get(JNode* obj, const char* key) {
    if (!obj || obj->type != JV_OBJ) return NULL;
    {
        JNode* c;
        for (c = obj->child; c; c = c->next)
            if (c->key && strcmp(c->key, key) == 0) return c;
    }
    return NULL;
}

/**
 * @brief 数组元素个数（遍历 child 链）。
 * @return 元素数；非数组或 NULL 返回 0。
 */
static int jarr_size(JNode* arr) {
    int n = 0;
    JNode* c;
    if (!arr || arr->type != JV_ARR) return 0;
    for (c = arr->child; c; c = c->next) ++n;
    return n;
}

static int node_u32(const JNode* node, uint32_t* out) {
    if (!node) return 0;
    if (node->type == JV_NUM) {
        if (node->num < 0 || (unsigned long long)node->num > UINT32_MAX) return 0;
        *out = (uint32_t)node->num;
        return 1;
    }
    if (node->type != JV_STR || node->str[0] < '0' || node->str[0] > '9') return 0;
    char* end;
    errno = 0;
    int base = node->str[0] == '0' && (node->str[1] == 'x' || node->str[1] == 'X') ? 16 : 10;
    unsigned long long value = strtoull(node->str, &end, base);
    if (errno == ERANGE || end == node->str || *end || value > UINT32_MAX) return 0;
    *out = (uint32_t)value;
    return 1;
}

static int jget_u32(JNode* obj, const char* key, uint32_t* out) {
    return node_u32(jobj_get(obj, key), out);
}

static int known_keys(const JNode* obj, const char* const* keys, size_t count) {
    if (!obj || obj->type != JV_OBJ) return 0;
    for (const JNode* field = obj->child; field; field = field->next) {
        size_t i;
        for (i = 0; i < count; ++i)
            if (strcmp(field->key, keys[i]) == 0) break;
        if (i == count) return 0;
    }
    return 1;
}

// ============================================================
// 设备字典：schema 走查 + API
// ============================================================

#define DEV_DICT_MAX_ENTRIES 64

typedef struct {
    uint32_t vendor, product;
    long long revision;
    int revAny;
} DictKey;

typedef struct {
    int count;
    DevDictEntry entries[DEV_DICT_MAX_ENTRIES];
    DictKey keys[DEV_DICT_MAX_ENTRIES];
} DictStore;

static DictStore g_store;

static const struct { const char* name; DevDictRole role; } kRoleTable[] = {
    { "status_word",   DEV_DICT_ROLE_STATUS_WORD },
    { "control_word",  DEV_DICT_ROLE_CONTROL_WORD },
    { "target_pos",    DEV_DICT_ROLE_TARGET_POS },
    { "actual_pos",    DEV_DICT_ROLE_ACTUAL_POS },
    { "target_speed",  DEV_DICT_ROLE_TARGET_SPEED },
    { "actual_speed",  DEV_DICT_ROLE_ACTUAL_SPEED },
    { "actual_torque", DEV_DICT_ROLE_ACTUAL_TORQUE },
    { "target_torque", DEV_DICT_ROLE_TARGET_TORQUE },
    { "mode_display",  DEV_DICT_ROLE_MODE_DISPLAY },
    { "error_code",    DEV_DICT_ROLE_ERROR_CODE },
    { "op_mode",       DEV_DICT_ROLE_OP_MODE },
};

/**
 * @brief 角色名字符串 → 枚举（字典文件里 objects 的键）。
 * @param role 如 "status_word" / "target_pos"。
 * @return 匹配的角色；未识别返回 DEV_DICT_ROLE_COUNT（调用方据此判错——
 *         写错角色名会让 Load 失败，这是刻意的防笔误设计）。
 */
DevDictRole DevDict_RoleFromString(const char* role) {
    size_t i;
    if (!role) return DEV_DICT_ROLE_COUNT;
    for (i = 0; i < sizeof(kRoleTable) / sizeof(kRoleTable[0]); ++i)
        if (strcmp(kRoleTable[i].name, role) == 0) return kRoleTable[i].role;
    return DEV_DICT_ROLE_COUNT;
}

// 加载失败统一出口：填错误、释放资源
// （msg 可能与 err 同缓冲——parseObject 已写入 err 时由调用方直接传入，此时跳过拷贝）
static int loadFail(char* err, int errLen, const char* msg, JNode* root, char* text) {
    if (err && errLen > 0 && msg != err) snprintf(err, (size_t)errLen, "%s", msg);
    jfree(root);
    free(text);
    return -1;
}

// 解析单个对象映射 {index, sub, bits, dir}
static int parseObject(JNode* o, DevDictRole role, DevDictObject* out,
                       char* err, int errLen, const char* roleName) {
    static const char* const keys[] = {"index", "sub", "bits", "dir"};
    static const uint8_t widths[DEV_DICT_ROLE_COUNT] = {16,16,32,32,32,32,16,16,8,16,8};
    const int is_tx = role == DEV_DICT_ROLE_STATUS_WORD || role == DEV_DICT_ROLE_ACTUAL_POS ||
                      role == DEV_DICT_ROLE_ACTUAL_SPEED || role == DEV_DICT_ROLE_ACTUAL_TORQUE ||
                      role == DEV_DICT_ROLE_MODE_DISPLAY || role == DEV_DICT_ROLE_ERROR_CODE;
    uint32_t index, sub = 0, bits = widths[role];
    JNode* dir = jobj_get(o, "dir");
    if (!known_keys(o, keys, sizeof(keys)/sizeof(keys[0])) ||
        !jget_u32(o, "index", &index) || index == 0 || index > UINT16_MAX ||
        (jobj_get(o, "sub") && !jget_u32(o, "sub", &sub)) || sub > UINT8_MAX ||
        (jobj_get(o, "bits") && !jget_u32(o, "bits", &bits)) || bits != widths[role] ||
        (dir && (dir->type != JV_STR || strcmp(dir->str, is_tx ? "tx" : "rx") != 0))) {
        if (err && errLen > 0)
            snprintf(err, (size_t)errLen, "objects.%s: 对象号、位宽、方向或字段非法", roleName);
        return 0;
    }
    out->role = role;
    out->index = (uint16_t)index;
    out->sub = (uint8_t)sub;
    out->bits = (uint8_t)bits;
    out->is_tx = is_tx;
    return 1;
}

static int entryHasRole(const DevDictEntry* entry, DevDictRole role) {
    int i;
    for (i = 0; i < entry->object_count; ++i)
        if (entry->objects[i].role == role) return 1;
    return 0;
}

/* custom 伺服必须能支撑当前 GmServoLink 的使能、位置和速度接口。 */
static int validateCustomServo(const DevDictEntry* entry, int entryIndex,
                               char* err, int errLen) {
    static const DevDictRole required[] = {
        DEV_DICT_ROLE_STATUS_WORD, DEV_DICT_ROLE_CONTROL_WORD,
        DEV_DICT_ROLE_TARGET_POS, DEV_DICT_ROLE_ACTUAL_POS,
        DEV_DICT_ROLE_TARGET_SPEED, DEV_DICT_ROLE_ACTUAL_SPEED,
        DEV_DICT_ROLE_ERROR_CODE, DEV_DICT_ROLE_OP_MODE, DEV_DICT_ROLE_MODE_DISPLAY,
    };
    static const char* const names[] = {
        "status_word", "control_word", "target_pos", "actual_pos",
        "target_speed", "actual_speed", "error_code", "op_mode", "mode_display",
    };
    size_t i;
    if (strcmp(entry->type, "servo") != 0) return 1;
    for (i = 0; i < sizeof(required) / sizeof(required[0]); ++i) {
        if (!entryHasRole(entry, required[i])) {
            if (err && errLen > 0)
                snprintf(err, (size_t)errLen,
                         "devices[%d]: custom servo 缺少必填对象 %s", entryIndex, names[i]);
            return 0;
        }
    }
    return 1;
}

/**
 * @brief 加载并解析设备字典文件（启动时一次）。
 *
 * 流程：读文件 → 最小 JSON 解析器建 AST → 校验 version/devices 结构 →
 * 逐条目提取 (vendor_id, product_code, revision, type, name, profile,
 * objects) 填进静态表 → 释放 AST。
 *
 * @param path   [in]  字典文件路径。
 * @param err    [out] 可选，失败原因（含条目序号/字段名，便于排查手编文件）。
 * @param errLen       err 缓冲区字节数。
 * @return 0 成功；-1 失败（文件不存在 / JSON 语法错 / 字段校验错 / 超容量）。
 *
 * @note **重复调用会先释放上次结果**，可以安全地重载。
 * @note profile "custom" 必须写 objects；"ds402" 用内置标准对象，不可写。
 * @note 全静态存储、无堆常驻（AST 解析完即释放），单线程使用。
 */
int DevDict_Load(const char* path, char* err, int errLen) {
    FILE* f = NULL;
    long sz = 0;
    char* text = NULL;
    JP jp;
    JNode* root = NULL;
    JNode* e = NULL;
    DictStore st;
    int idx = 0;

    DevDict_Unload();
    if (err && errLen > 0) err[0] = '\0';
    if (!path) {
        if (err && errLen > 0) snprintf(err, (size_t)errLen, "path 为空");
        return -1;
    }

    // 读整个文件（上限 1MB——字典是手编小文件）
    f = fopen(path, "rb");
    if (!f) {
        if (err && errLen > 0) snprintf(err, (size_t)errLen, "无法打开 %s", path);
        return -1;
    }
    fseek(f, 0, SEEK_END);
    sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz <= 0 || sz > (1 << 20)) {
        fclose(f);
        if (err && errLen > 0) snprintf(err, (size_t)errLen, "文件大小异常（%ld 字节）", sz);
        return -1;
    }
    text = (char*)malloc((size_t)sz + 1);
    if (!text) { fclose(f); return -1; }
    if (fread(text, 1, (size_t)sz, f) != (size_t)sz) {
        fclose(f);
        free(text);
        if (err && errLen > 0) snprintf(err, (size_t)errLen, "文件读取失败");
        return -1;
    }
    fclose(f);
    text[sz] = '\0';

    // 解析
    jp.s = text; jp.pos = 0; jp.len = (size_t)sz; jp.err[0] = '\0'; jp.depth = 0;
    root = jparse_value(&jp, NULL);
    if (!root) {
        char msg[128];
        snprintf(msg, sizeof(msg), "JSON 语法错误: %s", jp.err);
        return loadFail(err, errLen, msg, root, text);
    }

    jskip(&jp);
    if (jp.err[0] || jp.pos != jp.len)
        return loadFail(err, errLen, "JSON 尾部含非法内容", root, text);

    // ---- schema 走查 ----
    memset(&st, 0, sizeof(st));
    if (root->type != JV_OBJ)
        return loadFail(err, errLen, "顶层应为对象", root, text);
    {
        static const char* const keys[] = {"version", "devices"};
        uint32_t version;
        if (!known_keys(root, keys, 2) || !jget_u32(root, "version", &version) || version != 1)
            return loadFail(err, errLen, "字典 version 必须为 1，顶层只接受 version/devices", root, text);
        JNode* devices = jobj_get(root, "devices");
        if (!devices || devices->type != JV_ARR)
            return loadFail(err, errLen, "缺少 devices 数组", root, text);
        if (jarr_size(devices) > DEV_DICT_MAX_ENTRIES)
            return loadFail(err, errLen, "设备条目数超出上限 64", root, text);

        for (e = devices->child; e; e = e->next, ++idx) {
            DevDictEntry* dst = &st.entries[st.count];
            JNode* v = NULL;
            JNode* objs = NULL;

            if (e->type != JV_OBJ) {
                char msg[96];
                snprintf(msg, sizeof(msg), "devices[%d]: 应为对象", idx);
                return loadFail(err, errLen, msg, root, text);
            }
            static const char* const keys[] = {
                "vendor_id", "product_code", "revision", "type", "name", "profile", "objects"
            };
            if (!known_keys(e, keys, sizeof(keys)/sizeof(keys[0])))
                return loadFail(err, errLen, "设备条目含未知字段", root, text);
            memset(dst, 0, sizeof(*dst));

            v = jobj_get(e, "type");
            if (!v || v->type != JV_STR || v->str[0] == '\0') {
                char msg[96];
                snprintf(msg, sizeof(msg), "devices[%d]: 缺少 type", idx);
                return loadFail(err, errLen, msg, root, text);
            }
            if (strlen(v->str) >= sizeof(dst->type))
                return loadFail(err, errLen, "type 超长", root, text);
            snprintf(dst->type, sizeof(dst->type), "%s", v->str);

            v = jobj_get(e, "name");
            if (!v || v->type != JV_STR) {
                char msg[96];
                snprintf(msg, sizeof(msg), "devices[%d]: 缺少 name", idx);
                return loadFail(err, errLen, msg, root, text);
            }
            if (strlen(v->str) >= sizeof(dst->name))
                return loadFail(err, errLen, "name 超长", root, text);
            snprintf(dst->name, sizeof(dst->name), "%s", v->str);

            v = jobj_get(e, "profile");
            if (!v || v->type != JV_STR || v->str[0] == '\0') {
                char msg[96];
                snprintf(msg, sizeof(msg), "devices[%d]: 缺少 profile", idx);
                return loadFail(err, errLen, msg, root, text);
            }
            if (strlen(v->str) >= sizeof(dst->profile))
                return loadFail(err, errLen, "profile 超长", root, text);
            snprintf(dst->profile, sizeof(dst->profile), "%s", v->str);
            if (strcmp(dst->profile, "ds402") != 0 &&
                strcmp(dst->profile, "custom") != 0 &&
                strcmp(dst->profile, "none") != 0) {
                char msg[96];
                snprintf(msg, sizeof(msg), "devices[%d]: profile 应为 ds402/custom/none", idx);
                return loadFail(err, errLen, msg, root, text);
            }

            if (!jget_u32(e, "vendor_id", &st.keys[st.count].vendor)) {
                char msg[96];
                snprintf(msg, sizeof(msg), "devices[%d]: 缺少 vendor_id", idx);
                return loadFail(err, errLen, msg, root, text);
            }
            if (!jget_u32(e, "product_code", &st.keys[st.count].product)) {
                char msg[96];
                snprintf(msg, sizeof(msg), "devices[%d]: 缺少 product_code", idx);
                return loadFail(err, errLen, msg, root, text);
            }

            st.keys[st.count].revAny = 1;
            st.keys[st.count].revision = 0;
            v = jobj_get(e, "revision");
            if (v && !(v->type == JV_STR && strcmp(v->str, "*") == 0)) {
                uint32_t revision;
                if (!node_u32(v, &revision))
                    return loadFail(err, errLen, "revision 应为 uint32 或 *", root, text);
                st.keys[st.count].revAny = 0;
                st.keys[st.count].revision = revision;
            }
            for (int i = 0; i < st.count; ++i) {
                const DictKey* old = &st.keys[i];
                const DictKey* key = &st.keys[st.count];
                if (old->vendor == key->vendor && old->product == key->product &&
                    old->revAny == key->revAny && old->revision == key->revision)
                    return loadFail(err, errLen, "设备匹配键重复", root, text);
            }
            const int servo = strcmp(dst->type, "servo") == 0;
            if ((!servo && strcmp(dst->type, "gree6") != 0 && strcmp(dst->type, "gree4") != 0 &&
                 strcmp(dst->type, "panel") != 0 && strcmp(dst->type, "io") != 0 &&
                 strcmp(dst->type, "io_expansion") != 0) ||
                (servo == (strcmp(dst->profile, "none") == 0)))
                return loadFail(err, errLen, "设备 type/profile 不匹配", root, text);

            // objects：只有 custom 能写（且必写）。
            //   ds402 —— 用内置标准 DS402 对象号
            //   none  —— 本字典不提供对象映射；面板/IO 按位置建句柄，非标多轴在代码里
            dst->object_count = 0;
            objs = jobj_get(e, "objects");
            if (strcmp(dst->profile, "custom") == 0 && !objs) {
                char msg[96];
                snprintf(msg, sizeof(msg), "devices[%d]: custom profile 缺少 objects", idx);
                return loadFail(err, errLen, msg, root, text);
            }
            if (strcmp(dst->profile, "custom") != 0 && objs) {
                char msg[96];
                snprintf(msg, sizeof(msg), "devices[%d]: 只有 custom profile 能写 objects", idx);
                return loadFail(err, errLen, msg, root, text);
            }
            if (objs) {
                JNode* o = NULL;
                if (objs->type != JV_OBJ)
                    return loadFail(err, errLen, "objects 应为对象", root, text);
                for (o = objs->child; o; o = o->next) {
                    const DevDictRole r = DevDict_RoleFromString(o->key);
                    if (r == DEV_DICT_ROLE_COUNT) {
                        char msg[96];
                        snprintf(msg, sizeof(msg), "devices[%d]: 未知角色 '%s'", idx, o->key);
                        return loadFail(err, errLen, msg, root, text);
                    }
                    if (dst->object_count >= DEV_DICT_MAX_OBJECTS)
                        return loadFail(err, errLen, "objects 条目超上限 11", root, text);
                    {
                        int j;
                        for (j = 0; j < dst->object_count; ++j) {
                            if (dst->objects[j].role == r) {
                                char msg[96];
                                snprintf(msg, sizeof(msg), "devices[%d]: 角色 '%s' 重复", idx, o->key);
                                return loadFail(err, errLen, msg, root, text);
                            }
                        }
                    }
                    if (!parseObject(o, r, &dst->objects[dst->object_count],
                                     err, errLen, o->key))
                        return loadFail(err, errLen, err ? err : "objects 条目非法", root, text);
                    for (int j = 0; j < dst->object_count; ++j) {
                        const DevDictObject* old = &dst->objects[j];
                        const DevDictObject* cur = &dst->objects[dst->object_count];
                        if (old->index == cur->index && old->sub == cur->sub && old->is_tx == cur->is_tx)
                            return loadFail(err, errLen, "多个角色绑定同一对象", root, text);
                    }
                    ++dst->object_count;
                }
            }
            if (strcmp(dst->profile, "custom") == 0 &&
                !validateCustomServo(dst, idx, err, errLen))
                return loadFail(err, errLen, err ? err : "custom servo 对象不完整", root, text);
            ++st.count;
        }
    }

    // ---- 提交（覆盖上次加载）----
    g_store = st;
    jfree(root);
    free(text);
    return 0;
}

int DevDict_Lookup(uint32_t vendor, uint32_t product, uint32_t revision,
                   DevDictEntry* out) {
    int pass = 0, i = 0;
    if (!out) return 0;
    // 两轮匹配：先精确 revision，再通配
    for (pass = 0; pass < 2; ++pass) {
        for (i = 0; i < g_store.count; ++i) {
            const DictKey* k = &g_store.keys[i];
            if (k->vendor != vendor || k->product != product) continue;
            if (pass == 0 && (k->revAny || k->revision != (long long)revision)) continue;
            if (pass == 1 && !k->revAny) continue;
            *out = g_store.entries[i];
            return 1;
        }
    }
    return 0;
}

int DevDict_FindObject(const DevDictEntry* entry, DevDictRole role,
                       DevDictObject* out) {
    int i;
    if (!entry || !out || role < 0 || role >= DEV_DICT_ROLE_COUNT) return 0;
    for (i = 0; i < entry->object_count; ++i) {
        if (entry->objects[i].role == role) {
            *out = entry->objects[i];
            return 1;
        }
    }
    return 0;
}

/**
 * @brief 已加载的条目数（诊断用）。
 * @return 条目数；未加载或加载失败为 0。
 */
int DevDict_EntryCount(void) {
    return g_store.count;
}

/**
 * @brief 释放内部数据（退出时调用）。
 * @note 不调用也没有泄漏危害——数据是静态存储的。提供它主要是为了
 *       支持"重载"语义与干净退出。
 */
void DevDict_Unload(void) {
    g_store.count = 0;
}

// ============================================================
// 说明：本实现刻意不用第三方 JSON 库——自包含可测、无版本耦合；
// 若封装侧希望统一用其 cJSON，DevDict_* 接口保持不变、换实现即可。
// ============================================================
