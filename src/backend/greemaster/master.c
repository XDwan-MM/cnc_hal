#include "master.h"
#include "device.h"
#include "backend/greemaster/devdict.h"
#include "device_table.h"
#include "servo_step.h"
#include <stdatomic.h>
#include <time.h>
#ifdef __DeveloperMode__
#define debug printf
#else
#define debug(...)
#endif

// ==============全局变量定义===============
static atomic_int   InterruptFlag = 0;  // 打断标志（传给 GM 的阻塞调用）
static atomic_int   exit_flag      = 1;  // 主站出错/已停机，错误回调或关闭时置 1
static atomic_int stop_requested = 0;
static uint64_t start_deadline_ms;

static uint64_t monotonic_ms(void) {
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return UINT64_MAX;
    return (uint64_t)ts.tv_sec * 1000u + (uint64_t)ts.tv_nsec / 1000000u;
}

static uint32_t seconds_from_ms(uint64_t ms) {
    return (uint32_t)(ms / 1000u + (ms % 1000u != 0));
}

static int startup_expired(void) {
    return monotonic_ms() >= start_deadline_ms;
}

static uint32_t startup_seconds_left(void) {
    const uint64_t now = monotonic_ms();
    return now >= start_deadline_ms ? 0 : seconds_from_ms(start_deadline_ms - now);
}

/* 每个阶段共用一个截止时间；不可中断的 SDK 调用只能返回后检测超时。 */
#define CHECK_RC(value, message, label) do { \
    if ((value) != 0) { snprintf(g_start_error, sizeof(g_start_error), "%s (rc=%d)", message, (int)(value)); goto label; } \
    if (startup_expired()) { rc = MASTER_START_TIMEOUT; snprintf(g_start_error, sizeof(g_start_error), "启动总预算已耗尽"); goto label; } \
} while (0)
int slave_num = 0;
static char g_start_error[192];
const char* Master_StartupError(void) { return g_start_error; }

/* ethercat_init() 存下的配置：周期原语无入参，需要这里的 cycle_timeout_ms。 */
static MasterConfig g_cfg;
static int g_resources, g_master_initialized, g_io_resources, g_ready;
int Master_SlaveCount(void) { return g_ready ? slave_num : 0; }

/* 传给 GM 阻塞调用的打断标志地址。SDK 只读它，HAL 侧一律 atomic 访问；
 * 单独给取址函数是为了 device.c 的 EEPROM / PDO 映射读取也能被停止。
 * _Atomic int 与 int 布局一致，这里取地址转换。 */
int* Master_InterruptFlag(void) { return (int*)&InterruptFlag; }
/* 是否已被要求打断（错误回调或 Master_RequestStop 置位）。 */
int Master_Interrupted(void) { return atomic_load(&InterruptFlag) != 0; }
device_data_t g_device_data[MAX_DEVICE_NUM] = { 0 };  // 按【槽】索引

/* 全局错误上下文：**当前只写不读**（错误回调填，无读取者）。 */
static ErrorContext g_current_error_ctx;

static uint32_t get_unique_error_id(uint32_t model, uint32_t fms, uint32_t code, uint32_t Errslave) {
    if (model == 1) {
        return ECAT_INIT_ERROR;
    }
    else if (model == 2) {
        return ECAT_TOPOLOGY_ERROR;
    }
    else if (model == 3) {
        if (fms == 1 && code == 2 && Errslave == 1) {
            return ECAT_IO_CONNECT_LOST;
        }
        else if (fms == 1 && code == 2 && Errslave == 2) {
            return ECAT_SLAVE3_CONNECT_LOST;
        }
        else if (fms == 1 && code == 2 && Errslave == 3) {
            return ECAT_SLAVE4_CONNECT_LOST;
        }
        else if (fms == 1 && code == 2 && Errslave == 4) {
            return ECAT_SLAVE5_CONNECT_LOST;
        }
        else if (fms == 1 && code == 2 && Errslave == 5) {
            return ECAT_SLAVE6_CONNECT_LOST;
        }
    }
    else if (model == 4) {
        return ECAT_DC_ERROR;
    }
    else if (model == 5) {
        return ECAT_PDO_ERROR;
    }
    else if (model == 8) {
        return ECAT_RT_ERROR;
    }
    else if (model == 9) {
        return ECAT_CONTROL_PANEL_CONNECT_LOST;
    }
    return UNKOWN_ERROR;
}

void error_module_init(void) {
    memset(&g_current_error_ctx, 0, sizeof(g_current_error_ctx));
    //atomic_store(&g_has_new_error, false);
}

void err_call_back() {
    Err_info errinfo;
    /* 先清零再取：失败时 SDK 不保证出参被填，读未初始化的栈会把
     * 模块/状态机/错误号报成垃圾，把现场排错引向错误方向。 */
    memset(&errinfo, 0, sizeof(errinfo));
    if (Err_Code_Get(&errinfo) != 0)
        printf("错误回调：Err_Code_Get 失败，错误详情不可用\n");
    printf("错误模块：%d, 错误状态机：%d, 错误号：%d\n", errinfo.Model,
        errinfo.Fms, errinfo.Code);
    g_current_error_ctx.Model = errinfo.Model;
    g_current_error_ctx.Fms = errinfo.Fms;
    g_current_error_ctx.Code = errinfo.Code;
    g_current_error_ctx.errslave = errinfo.Errslave;
    g_current_error_ctx.ErrorID = get_unique_error_id(
        errinfo.Model, errinfo.Fms, errinfo.Code, errinfo.Errslave
    );

    //atomic_store(&g_has_new_error, true);
    Err_Info_Get(errinfo, NULL);
    exit_flag = 1;
    atomic_store(&InterruptFlag, 1);
}

// ================ 新增：初始化函数（供外部调用）=================
/* 主站进 OP 之后、进周期收发模式之前的一次握手，顺序（不可换）：
 *   握手前收一帧 → IO 端口资源分配 → 握手前发一帧 → GM_Master_OP_Valid()
 * @return 0 成功；非 0 为 GM 错误码。 */

/* 最近一次收帧的警告，Master_BusHealth() 读它。 */
static PdoWarn g_last_warn;

/* 只缓存不打印：本函数在 Master_WaitCycle() 的实时路径上，而 PdoWar_Info_Get()
 * 是 SDK 的打印型接口（OutFile=NULL 打到 stdout）。详情改在 Master_BusHealth() 取。 */
static void record_warn(const PdoWarn* w) {
    if (!w) return;
    g_last_warn = *w;
}

static int master_handshake(void) {
    PdoWarn warn;
    int rc;

    memset(&warn, 0, sizeof(warn));

    printf("========== 握手前收一帧:GM_Master_Receive ============\n");
    rc = GM_Master_Receive(1, &warn);
    if (rc != 0) return rc < 0 ? rc : -1;
    if (startup_expired()) return MASTER_START_TIMEOUT;
    record_warn(&warn);
    if (warn.FlameTimeOutCount || warn.CRCErrCount)
        printf("警告：握手前 CRC 错误 %u 次，帧超时 %u 次\n",
               (unsigned)warn.CRCErrCount, (unsigned)warn.FlameTimeOutCount);

    /* IO 端口资源分配（GM 原语）。 */
    g_io_resources = 1;
    rc = IO_Resource_Allocation_Scanf();
    if (rc != 0) return -1;
    if (startup_expired()) return MASTER_START_TIMEOUT;

    printf("========== 握手前发一帧:GM_Master_Send ============\n");
    rc = GM_Master_Send(1);
    if (rc != 0) return rc < 0 ? rc : -1;
    if (startup_expired()) return MASTER_START_TIMEOUT;

    printf("========== 与主站握手:GM_Master_OP_Valid ============\n");
    rc = GM_Master_OP_Valid();
    if (rc != 0) return rc < 0 ? rc : -1;

    printf("握手完成，主站已进入周期收发模式\n");
    return 0;
}

MASTER_API int ethercat_init(const MasterConfig* cfg) {
    int rc = 0;
    uint32_t CRCCount = 0;
    uint32_t TimeOutCount = 0;
    DEVICE_TYPE types[MAX_DEVICE_NUM] = {0};
    /* 先清上次失败原因（下面有提前 return，避免 StartupError 报旧原因）。 */
    g_start_error[0] = '\0';
    if (g_resources || g_ready) return -1;

    /* 配置非法就早失败，放在任何 GM 调用之前，不留半开的主站。 */
    if (!cfg) {
        printf("错误：ethercat_init() 的 cfg 为空\n");
        return -1;
    }
    if (cfg->cycle_us < GM_MIN_CYCLE_US || cfg->cycle_us > UINT32_MAX / 1000u ||
        !cfg->start_timeout_ms ||
        !cfg->cycle_timeout_ms || (cfg->dc_enable != 0 && cfg->dc_enable != 1)) {
        printf("错误：主站配置无效（周期 %u us，允许 %u..%u us；超时须非零，DC 须为 0/1）\n",
               (unsigned)cfg->cycle_us, (unsigned)GM_MIN_CYCLE_US,
               (unsigned)(UINT32_MAX / 1000u));
        return -1;
    }
    g_cfg = *cfg;
    const uint64_t now = monotonic_ms();
    if (now == UINT64_MAX) return -1;
    start_deadline_ms = now + cfg->start_timeout_ms;
    atomic_store(&InterruptFlag, 0);
    exit_flag = 0;
    stop_requested = 0;
    device_reset();
    DeviceTable_Reset();
    Master_ServoReset();
    memset(&g_last_warn, 0, sizeof(g_last_warn));

    error_module_init();

    /* 在申请主站资源前确认字典可用，避免路径错误时仍把总线拉到 OP。 */
    const char* dict_path = getenv("CNC_HAL_DEVICES_JSON");
    if (!dict_path) dict_path = DEVICES_JSON_PATH;
    if (!dict_path || !*dict_path) {
        printf("错误：设备字典路径未配置；请设置 CNC_HAL_DEVICES_JSON\n");
        exit_flag = 1;
        return MASTER_DEVICE_DICTIONARY_ERROR;
    }
    char dict_error[192] = {0};
    if (DevDict_Load(dict_path, dict_error, sizeof(dict_error)) != 0) {
        printf("错误：设备字典加载失败（%s）：%s\n", dict_path, dict_error);
        exit_flag = 1;
        return MASTER_DEVICE_DICTIONARY_ERROR;
    }
    if (startup_expired()) {
        DevDict_Unload();
        exit_flag = 1;
        return MASTER_START_TIMEOUT;
    }
    printf("设备字典已加载：%s（%d 条）\n", dict_path, DevDict_EntryCount());

    printf("================ 申请主站资源:GM_Resource_Allocation ==================\n");
    g_resources = 1;
    rc = GM_Resource_Allocation();
    CHECK_RC(rc, "申请主站资源失败", err_close);

    printf("================ 获取主站版本号:GM_Get_Version ==================\n");
    _Version_ MasterVersionCode = GM_Get_Version();
    printf("主站版本号：%d.%d.%d\n", MasterVersionCode.FPGA_Version_Main, MasterVersionCode.FPGA_Version_Sec, MasterVersionCode.API_Version);

    printf("================ 申请报错回调函数:Err_Fun_Register ==================\n");
    rc = Err_Fun_Register(err_call_back);
    CHECK_RC(rc, "申请报错回调函数失败", err_close);

    printf("==================== 主站初始化:GM_Master_Init ========================\n");
    g_master_initialized = 1;
    rc = GM_Master_Init();
    CHECK_RC(rc, "主站初始化失败", err_close);

    if (PARAM_CHANG) {
        _MASTER_PARAM_SRTUCT_.P_AL_STATE_CHANGE_TIMEOUT = 10000;
        printf("==================== 主站参数修改:GM_Master_Param_Change ========================\n");
        rc = GM_Master_Param_Change();
        CHECK_RC(rc, "主站参数修改", err_close);
    }

    printf("===================== 主站开始:GM_Master_Start ========================\n");
    uint32_t seconds_left = startup_seconds_left();
    if (!seconds_left) { rc = MASTER_START_TIMEOUT; goto err_close; }
    rc = GM_Master_Start(seconds_left, Master_InterruptFlag());
    CHECK_RC(rc, "主站开始失败", err_close);

    slave_num = GM_Slave_Num_Get();
    if (slave_num < 0 || slave_num > MAX_DEVICE_NUM) {
        rc = slave_num < 0 ? slave_num : -1;
        printf("slave num get error\n");
        goto err_close;
    }
    else {
        printf("slave num is %d\n", slave_num);
    }
    if (startup_expired()) { rc = MASTER_START_TIMEOUT; goto err_close; }
    rc = get_device_info_from_eeprom(slave_num, types, cfg->cycle_us, cfg->dc_enable);
    CHECK_RC(rc, "读取或解析 EEPROM 失败", err_close);

    printf("=============== 计算PDO映射:GM_Calculate_Config_Info ==================\n");
    rc = GM_Calculate_Config_Info(&slave_list);
    CHECK_RC(rc, "计算PDO映射失败", err_close);

    GM_PDO_Map_Print(slave_list, 0);

    rc = device_match(slave_list, types, slave_num);
    if (rc != 0 && device_bind_error()[0]) {
        snprintf(g_start_error, sizeof(g_start_error), "%s", device_bind_error());
        goto err_close;
    }
    CHECK_RC(rc, "设备句柄装配失败", err_close);

    /* 装配完成后建按槽设备表（本层对 HAL 的取数口）。 */
    rc = DeviceTable_Build(slave_num);
    if (rc < 0) goto err_close;

    printf("=============== 设置主站通讯周期:GM_Master_Set_Cycle (%u us) ==========\n",
           (unsigned)cfg->cycle_us);
    rc = GM_Master_Set_Cycle(cfg->cycle_us);
    CHECK_RC(rc, "设置主站通讯周期失败", err_close);

    if (cfg->dc_enable) {
        printf("=============== 使能DC信号:GM_DC_Enable ==================\n");
        rc = GM_DC_Enable();
        CHECK_RC(rc, "使能DC信号失败", err_close);
    }

    { /* 驱动内部的状态查询资源（reg_num=2，mem_size={6,6}），不是公共配置开关。 */
        printf("=============== 正在申请寄存器资源 ==================\n");
        int reg_num = 2;
        uint32_t mem_size[2] = { 6, 6 };
        rc = GM_Reg_Resource_Init(reg_num, mem_size);
        CHECK_RC(rc, "申请寄存器资源失败", err_close);
    }

    { /* 驱动内部的设备参数通道（SDO）。 */
        printf("=============== 正在申请SDO资源 ==================\n");
        rc = GM_Sdo_Datagram_Enable();
        CHECK_RC(rc, "SDO功能使能失败", err_close);
    }

    printf("========== 配置信息下发并激活:GM_Config_Download_And_Active ============\n");
    seconds_left = startup_seconds_left();
    if (!seconds_left) { rc = MASTER_START_TIMEOUT; goto err_close; }
    rc = GM_Config_Download_And_Active(slave_list, &CRCCount, &TimeOutCount, seconds_left, Master_InterruptFlag(), NULL);   /* 手动 FMMU 表已删除：见提交说明 */
    CHECK_RC(rc, "配置信息下发失败", err_close);

    if (CRCCount || TimeOutCount) {
        printf("op前数据帧出现错误: CRC错误次数:%d, 超时错误次数:%d\n", CRCCount, TimeOutCount);
    }

    printf("========== 等待主站进入OP:GM_Master_Wait_OP ============\n");
    seconds_left = startup_seconds_left();
    if (!seconds_left) { rc = MASTER_START_TIMEOUT; goto err_close; }
    rc = GM_Master_Wait_OP(seconds_left, Master_InterruptFlag());
    CHECK_RC(rc, "等待主站进入OP", err_close);

    /* 进 OP 只是第一步：还要握手才进周期收发模式。 */
    rc = master_handshake();
    CHECK_RC(rc, "与主站握手", err_close);

    if (exit_flag) { rc = -1; goto err_close; }
    g_ready = 1;
    return 0;

err_close:
    ethercat_close();
    return rc < 0 ? rc : -1;
}

MASTER_API int ethercat_close(void) {
    int first_error = 0, rc;
    exit_flag = 1;
    atomic_store(&InterruptFlag, 1);
    g_ready = 0;
    /* 调用方须先退出周期调用。先释放 PDO 映射链表，再关主站。 */
    if (slave_list) {
        rc = GM_Free_PDO_Map(slave_list);
        if (rc != 0 && first_error == 0) first_error = rc < 0 ? rc : -1;
        slave_list = NULL;
    }
    if (g_master_initialized) {
        rc = GM_Master_Close();
        if (rc != 0 && first_error == 0) first_error = rc < 0 ? rc : -1;
        g_master_initialized = 0;
    }
    if (g_io_resources) {
        rc = IO_Resource_Release();
        if (rc != 0 && first_error == 0) first_error = rc < 0 ? rc : -1;
        g_io_resources = 0;
    }
    if (g_resources) {
        rc = GM_Resource_Release();
        if (rc != 0 && first_error == 0) first_error = rc < 0 ? rc : -1;
        g_resources = 0;
    }
    DevDict_Unload();
    DeviceTable_Reset();
    device_reset();
    Master_ServoReset();
    slave_num = 0;
    memset(&g_last_warn, 0, sizeof(g_last_warn));
    memset(&g_cfg, 0, sizeof(g_cfg));
    return first_error;
}

MASTER_API int Master_WaitCycle(void) {
    if (!g_ready) return -1;
    if (stop_requested) return MASTER_STOP_REQUESTED;
    if (exit_flag) return -1;
    int rc = GM_Wait_Master_Sync(seconds_from_ms(g_cfg.cycle_timeout_ms));
    if (stop_requested) return MASTER_STOP_REQUESTED;
    if (rc != 0) return rc < 0 ? rc : -1;

    PdoWarn warn;
    memset(&warn, 0, sizeof(warn));
    rc = GM_Master_Receive(1, &warn);
    if (stop_requested) return MASTER_STOP_REQUESTED;
    if (rc != 0) return rc < 0 ? rc : -1;

    /* 警告不致命，报警策略归上层；记下来供 Master_BusHealth() 查。 */
    record_warn(&warn);
    return 0;
}

MASTER_API int Master_BusHealth(MasterBusHealth* out) {
    if (!out) return -1;
    /* 非 RT 查询路径：有警告标志时才向 SDK 取详情（该接口会打印）。 */
    if (g_last_warn.PDOWarnFlag || g_last_warn.DCWarnFlag)
        PdoWar_Info_Get(NULL, &g_last_warn);
    out->pdo_warn             = (int)g_last_warn.PDOWarnFlag;
    out->pdo_warn_code        = (int)g_last_warn.PDOWarn;
    out->pdo_warn_para        = g_last_warn.PDOWarnPara;
    out->dc_warn              = (int)g_last_warn.DCWarnFlag;
    out->dc_warn_code         = (int)g_last_warn.DCWarn;
    out->dc_warn_para         = g_last_warn.DCWarnPara;
    out->crc_err_count        = g_last_warn.CRCErrCount;
    out->frame_timeout_count  = g_last_warn.FlameTimeOutCount;
    out->expect_wkc_tx        = g_last_warn.EXTxWKC;
    out->expect_wkc_rx        = g_last_warn.EXRxWKC;
    return 0;
}

MASTER_API int Master_CommitCycle(void) {
    if (!g_ready) return -1;
    if (stop_requested) return MASTER_STOP_REQUESTED;
    if (exit_flag) return -1;
    const int rc = GM_Master_Send(1);
    return rc == 0 ? 0 : (rc < 0 ? rc : -1);
}

/* 主站是否出过错或已停机（err_call_back / ethercat_close() 置位）；给 HAL 只读查询。 */
MASTER_API int Master_StopFlag(void) {
    return exit_flag || stop_requested;
}

/* 两个标志都用 atomic：会被停止请求线程或 SDK 回调线程写，阻塞调用在本线程轮询。 */
MASTER_API void Master_RequestStop(void) {
    stop_requested = 1;
    atomic_store(&InterruptFlag, 1);   /* 让启动期的阻塞调用也能被停掉 */
}

int Master_SdoTransfer(int slave_pos, uint16_t index, uint8_t subindex,
                       void* data, uint16_t size, int write, uint32_t timeout_ms) {
    if (!g_ready || slave_pos < 0 || slave_pos >= slave_num || !data ||
        (size != 1 && size != 2 && size != 4) || !timeout_ms) return -1;
    if (stop_requested || exit_flag) return -4;
    const uint64_t started = monotonic_ms();
    if (started == UINT64_MAX) return -1;
    uint8_t buffer[8] = {0};
    if (write) memcpy(buffer, data, size);
    SdoRequests req = {
        .WR = write ? 1 : 0, .SlavePos = (uint16_t)slave_pos,
        .Index = index, .SubIndex = subindex, .Size = size,
        .data = buffer, .status = P_SDO_IDLE
    };
    if (GM_Sdo_Request_Send(req) != 0) return -1;
    for (;;) {
        if (stop_requested || exit_flag) return -4;
        const uint64_t now = monotonic_ms();
        if (now == UINT64_MAX) return -1;
        if (now - started >= timeout_ms) return -2;
        /* SDK 接口按整秒等待；每次只等 1 秒，再检查总截止时间。 */
        if (GM_Sdo_Receive(&req, 1) != 0) return -1;
        if (req.status == P_SDO_SUCCESS) {
            if (req.Size != size) return -3;
            if (!write) memcpy(data, buffer, size);
            return 0;
        }
        if (req.status == P_SDO_ERROR) return -3;
    }
}
