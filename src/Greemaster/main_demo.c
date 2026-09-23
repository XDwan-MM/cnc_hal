#include "main_demo.h"
#include "device.h"
#include "common/devdict.h"
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
static int          InterruptFlag = 0;  // 打断标志（传给 GM 的阻塞调用）
static atomic_int   exit_flag      = 1;  // 主站出错/已停机：错误回调或关闭时置 1
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
#undef CHECK_RC
#define CHECK_RC(value, message, label) do { \
    if ((value) != 0) goto label; \
    if (startup_expired()) { rc = MASTER_START_TIMEOUT; goto label; } \
} while (0)
int slave_num = 0;

/* ethercat_init() 存下的配置。周期原语（Master_WaitCycle）要用 cycle_timeout_ms，
 * 而那些函数没有入参——配置在启动时定一次，之后不变。 */
static MasterConfig g_cfg;
static int g_resources, g_master_initialized, g_io_resources, g_ready;
device_data_t g_device_data[MAX_DEVICE_NUM] = { 0 };  // 假设最多 30 个设备
// ================打断函数=================
// 原 sigint_handler() 已删除：库不该装 SIGINT 处理器，会覆盖宿主程序自己的。
// Ctrl-C、退出流程归调用方。HAL 只保留 exit_flag / InterruptFlag 供其查询与打断。

// 实际分配内存的全局错误上下文
static ErrorContext g_current_error_ctx;

// ==========================================
// 3. 辅助函数
// ==========================================
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

// ==========================================
// 4. 公共接口实现
// ==========================================

void error_module_init(void) {
    memset(&g_current_error_ctx, 0, sizeof(g_current_error_ctx));
    //atomic_store(&g_has_new_error, false);
}

void error_module_destroy(void) {
    // 1. 强制清除当前错误
    //atomic_store(&g_has_new_error, false);
    memset(&g_current_error_ctx, 0, sizeof(g_current_error_ctx));

    // 2. 如果有其他资源（如日志文件句柄、子线程ID等），在这里关闭

    printf("[SYS] Error module destroyed/cleaned up.\n");
}
// ================错误处理函数=================
void err_call_back() {
    Err_info errinfo;
    Err_Code_Get(&errinfo);
    printf("错误模块：%d, 错误状态机：%d, 错误号：%d\n", errinfo.Model,
        errinfo.Fms, errinfo.Code);
    // 1. 提取关键字段并查表
    g_current_error_ctx.Model = errinfo.Model;
    g_current_error_ctx.Fms = errinfo.Fms;
    g_current_error_ctx.Code = errinfo.Code;
    g_current_error_ctx.errslave = errinfo.Errslave;
    g_current_error_ctx.ErrorID = get_unique_error_id(
        errinfo.Model, errinfo.Fms, errinfo.Code, errinfo.Errslave
    );

    // 2. 原子设置标志位：有新错误
    //atomic_store(&g_has_new_error, true);
    Err_Info_Get(errinfo, NULL);
    exit_flag = 1;
    InterruptFlag = 1;
}


// 提前声明
Fmmu_Manul* FMMU_Manul_Set_Func();
// ================ 新增：初始化函数（供外部调用）=================
/* 主站进 OP 之后、进周期收发模式之前的一次握手。
 *
 * 顺序照 cnc_rt/src/master/master.c 的 Master_Receive() 转写——那是跑过的实现：
 *   握手前收一帧 → IO 端口资源分配 → 握手前发一帧 → GM_Master_OP_Valid()
 * 握手成功后主站才进周期性收发数据帧模式。
 *
 * @return 0 成功；非 0 为 GM 错误码。 */

/* 最近一次收帧的警告。Master_BusHealth() 读它。 */
static PdoWarn g_last_warn;

/* 收帧后统一记录；有标志就顺便取详情。 */
static void record_warn(const PdoWarn* w) {
    if (!w) return;
    g_last_warn = *w;
    if (w->PDOWarnFlag || w->DCWarnFlag)
        PdoWar_Info_Get(NULL, &g_last_warn);
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

    /* IO 端口资源分配。GM 原语，归本层（原先在封装侧）。 */
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
    Fmmu_Manul* fmmu_manul_list = NULL;
    if (g_resources || g_ready) return -1;

    /* 配置非法就早失败——放在任何 GM 调用之前，不留下半开的主站。 */
    if (!cfg) {
        printf("错误：ethercat_init() 的 cfg 为空\n");
        return -1;
    }
    if (cfg->cycle_us < GM_MIN_CYCLE_US || !cfg->start_timeout_ms ||
        !cfg->cycle_timeout_ms || (cfg->dc_enable != 0 && cfg->dc_enable != 1)) {
        printf("错误：通讯周期 %u us 小于主站下限 %u us\n",
               (unsigned)cfg->cycle_us, (unsigned)GM_MIN_CYCLE_US);
        return -1;
    }
    g_cfg = *cfg;
    const uint64_t now = monotonic_ms();
    if (now == UINT64_MAX) return -1;
    start_deadline_ms = now + cfg->start_timeout_ms;
    InterruptFlag = 0;
    exit_flag = 0;
    stop_requested = 0;
    device_reset();
    DeviceTable_Reset();
    Master_ServoReset();
    memset(&g_last_warn, 0, sizeof(g_last_warn));

    error_module_init();

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
    rc = GM_Master_Start(seconds_left, &InterruptFlag);
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
    /* 设备字典：按 EEPROM 里的 厂商/产品/版本 定设备类型。
     * 加载失败不致命——未命中的设备一律判 UNKNOWN_TYPE，仍会上报。 */
    {
        char dictErr[192] = {0};
        if (DevDict_Load(DEVICES_JSON_PATH, dictErr, sizeof(dictErr)) != 0) {
            printf("警告：设备字典加载失败（%s）—— 所有设备将判为 UNKNOWN_TYPE\n", dictErr);
        }
        else {
            printf("设备字典已加载：%s（%d 条）\n", DEVICES_JSON_PATH, DevDict_EntryCount());
        }
    }

    if (startup_expired()) { rc = MASTER_START_TIMEOUT; goto err_close; }
    rc = get_device_info_from_eeprom(slave_num, types);
    CHECK_RC(rc, "读取或解析 EEPROM 失败", err_close);

    // 计算PDO映射
    printf("=============== 计算PDO映射:GM_Calculate_Config_Info ==================\n");
    rc = GM_Calculate_Config_Info(&slave_list);
    CHECK_RC(rc, "计算PDO映射失败", err_close);

    GM_PDO_Map_Print(slave_list, 0);

    rc = device_match(slave_list, types, slave_num);
    CHECK_RC(rc, "设备句柄装配失败", err_close);

    /* 装配完成后建按槽设备表——这是本层对 HAL 的取数口。 */
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

    { /* 驱动内部的状态查询资源，不作为公共配置开关。 */
        printf("=============== 正在申请寄存器资源 ==================\n");
        int reg_num = 2;
        uint32_t mem_size[2] = { 6, 6 };
        rc = GM_Reg_Resource_Init(reg_num, mem_size);
        CHECK_RC(rc, "申请寄存器资源失败", err_close);
    }

    { /* 驱动内部的设备参数通道。 */
        printf("=============== 正在申请SDO资源 ==================\n");
        rc = GM_Sdo_Datagram_Enable();
        CHECK_RC(rc, "SDO功能使能失败", err_close);
    }

    if (MAN_FMMU) {
        fmmu_manul_list = FMMU_Manul_Set_Func();
        if (!fmmu_manul_list) { rc = -1; goto err_close; }
    }

    printf("========== 配置信息下发并激活:GM_Config_Download_And_Active ============\n");
    seconds_left = startup_seconds_left();
    if (!seconds_left) { rc = MASTER_START_TIMEOUT; goto err_close; }
    rc = GM_Config_Download_And_Active(slave_list, &CRCCount, &TimeOutCount, seconds_left, &InterruptFlag, fmmu_manul_list);
    CHECK_RC(rc, "配置信息下发失败", err_close);

    if (CRCCount || TimeOutCount) {
        printf("op前数据帧出现错误: CRC错误次数:%d, 超时错误次数:%d\n", CRCCount, TimeOutCount);
    }

    if (fmmu_manul_list) {
        Fmmu_Manul_Free(fmmu_manul_list);
        fmmu_manul_list = NULL;
    }

    printf("========== 等待主站进入OP:GM_Master_Wait_OP ============\n");
    seconds_left = startup_seconds_left();
    if (!seconds_left) { rc = MASTER_START_TIMEOUT; goto err_close; }
    rc = GM_Master_Wait_OP(seconds_left, &InterruptFlag);
    CHECK_RC(rc, "等待主站进入OP", err_close);

    /* 主站到 OP 只是第一步——还要握手才进周期收发模式，从站才会走 OP。 */
    rc = master_handshake();
    CHECK_RC(rc, "与主站握手", err_close);

    if (exit_flag) { rc = -1; goto err_close; }
    g_ready = 1;
    return 0;

err_close:
    if (fmmu_manul_list) Fmmu_Manul_Free(fmmu_manul_list);
    ethercat_close();
    return rc < 0 ? rc : -1;
}

// ================ 新增：关闭函数 =================
MASTER_API int ethercat_close(void) {
    int first_error = 0, rc;
    exit_flag = 1;
    InterruptFlag = 1;
    g_ready = 0;
    /* 调用方须先退出周期调用，再释放主站与句柄。 */
    if (g_master_initialized) {
        rc = GM_Master_Close();
        if (rc != 0 && first_error == 0) first_error = rc < 0 ? rc : -1;
        g_master_initialized = 0;
    }
    if (slave_list) {
        rc = GM_Free_PDO_Map(slave_list);
        if (rc != 0 && first_error == 0) first_error = rc < 0 ? rc : -1;
        slave_list = NULL;
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

/* ==================== 周期原语（只在实时线程调用） ==================== */

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

    /* 警告不致命，也不在这里报警——报警策略归上层。记下来供 Master_BusHealth() 查。 */
    record_warn(&warn);
    return 0;
}

MASTER_API int Master_BusHealth(MasterBusHealth* out) {
    if (!out) return -1;
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

/* 主站是否出过错或已停机。错误回调（err_call_back）和 ethercat_close() 会置位。
 * 给 HAL 查询用——它只读，不通过这个去推状态。 */
MASTER_API int Master_StopFlag(void) {
    return exit_flag || stop_requested;
}

/* 不跨线程写入 SDK 接收的普通 int，避免与 SDK 的读取竞争。 */
MASTER_API void Master_RequestStop(void) {
    stop_requested = 1;
}

/* TODO(总线状态) 原 reg_thread_func 的职责：查从站的 AL 状态（寄存器 0x130），
 * 判断有没有掉出 OP。它走 mailbox，是慢通道，**不能放周期里**；将来由 HAL 提供
 * 非 RT 的查询接口，调用节奏由实时端定。原实现只是每秒 printf 一次。 */

int write_sdo_index(int slave_pos, uint16_t index, uint8_t subindex, void* data, size_t data_size, int max_retry) {
    int cnt = 0;
    
    // 严格按结构体定义初始化
    SdoRequests sdo_request = {
        .WR = 1,                  // 0 = 写操作
        .SlavePos = slave_pos,            // 从站位置（0表示当前从站）
        .Index = index,           // 索引号
        .SubIndex = subindex,     // 子索引号
        .Size = (uint16_t)data_size, // 传输大小（转换为uint16_t）
        .data = (uint8_t*)data,   // 数据指针（转换为uint8_t*）
        .status = 0,              // 初始状态
        .Warn = 0,
        .WarnPra1 = 0,
        .WarnPra2 = 0
    };

    // 发送初始请求
    GM_Sdo_Request_Send(sdo_request);
    //rc = GM_Sdo_Receive(&sdo_request, 5);
    while(sdo_request.status != P_SDO_SUCCESS){
        if(cnt > max_retry){
            return -1;
        }
        GM_Sdo_Receive(&sdo_request, 5);
        if(sdo_request.status == P_SDO_ERROR){
            printf("sdo send error %d\n",sdo_request.status);
            GM_Sdo_Request_Send(sdo_request);
        }
        printf("sdo send status is %d\n",sdo_request.status);
        cnt++;
    }
    return 0;
}
// 通用SDO读取函数（严格匹配结构体定义）
int read_sdo_index(int slave_pos, uint16_t index, uint8_t subindex, void* data, size_t data_size, int max_retry) {
    int cnt = 0;
    
    // 严格按结构体定义初始化
    SdoRequests sdo_request = {
        .WR = 0,                  // 0 = 读操作
        .SlavePos = slave_pos,            // 从站位置（0表示当前从站）
        .Index = index,           // 索引号
        .SubIndex = subindex,     // 子索引号
        .Size = (uint16_t)data_size, // 传输大小（转换为uint16_t）
        .data = (uint8_t*)data,   // 数据指针（转换为uint8_t*）
        .status = 0,              // 初始状态
        .Warn = 0,
        .WarnPra1 = 0,
        .WarnPra2 = 0
    };

    // 发送初始请求
    // 发送初始请求
    GM_Sdo_Request_Send(sdo_request);
    //rc = GM_Sdo_Receive(&sdo_request, 5);
    while(sdo_request.status != P_SDO_SUCCESS){
        if(cnt > max_retry){
            return -1;
        }
        GM_Sdo_Receive(&sdo_request, 5);
        if(sdo_request.status == P_SDO_ERROR){
            printf("sdo send error %d\n",sdo_request.status);
            GM_Sdo_Request_Send(sdo_request);
        }
        printf("sdo send status is %d\n",sdo_request.status);
        cnt++;
    }
    //printf("SDO warn is%d, %d, %d\n", sdo_request.Warn, sdo_request.WarnPra1, sdo_request.WarnPra2);
    //SdoWar_Info_Get(NULL, &sdo_request);        // 检测是否存在警告信息
    
    return 0;
}


// ====================================================手动设置FMMU函数============================================
Fmmu_Manul* FMMU_Manul_Set_Func(){
    // 示例：初始化2个从设备，每个从设备的FMMU数量分别为3和2
    uint32_t num_slaves = 7;                    // 从站数量
    uint32_t every_slave_fmmu_num[] = {2,2,2,2,2,2,2};   // 每个从站所需要的fmmu

    // 调用初始化函数
    Fmmu_Manul* fmmu_manul = Fmmu_Manul_Init(num_slaves, every_slave_fmmu_num);
    if (!fmmu_manul) {
        printf("初始化失败！\n");
        return NULL;
    }

    // 示例：访问并打印FMMU数据
    printf("总从设备数量: %d\n", fmmu_manul->total_slave_number);
    fmmu_manul->total_Rxpdo_addr = 0;       // 收fmmu起始地址
    fmmu_manul->total_Rxpdo_size = 20;       // 收fmmu总长度
    fmmu_manul->total_Txpdo_addr = 20;       // 发fmmu起始地址
    fmmu_manul->total_Txpdo_size = 20;       // 发fmmu总长度

    uint32_t fmmu_log_start_addr[7][2] = {{0x0000, 0x0014}, 
                                          {0x0000, 0x0014},
                                          {0x0010, 0x0014},
                                          {0x0010, 0x0024},
                                          {0x0010, 0x0024},
                                          {0x0010, 0x0026},
                                          {0x0012, 0x0026},};
    uint32_t fmmu_data_size[7][2]      = {{0x0000, 0x0000}, 
                                          {0x0010, 0x0000},
                                          {0x0000, 0x0010},
                                          {0x0000, 0x0000},
                                          {0x0000, 0x0002},
                                          {0x0002, 0x0000},
                                          {0x0002, 0x0002},};
    uint32_t fmmu_log_start_bit[7][2]  = {{0x0000, 0x0000}, 
                                          {0x0000, 0x0000},
                                          {0x0000, 0x0000},
                                          {0x0000, 0x0000},
                                          {0x0000, 0x0000},
                                          {0x0000, 0x0000},
                                          {0x0000, 0x0000},};
    uint32_t fmmu_log_end_bit[7][2]    = {{0x0007, 0x0007}, 
                                          {0x0007, 0x0007},
                                          {0x0007, 0x0007},
                                          {0x0007, 0x0007},
                                          {0x0007, 0x0007},
                                          {0x0007, 0x0007},
                                          {0x0007, 0x0007},};
    uint32_t fmmu_phy_start_addr[7][2] = {{0x0000, 0x0000}, 
                                          {0x0000, 0x0000},
                                          {0x0000, 0x0000},
                                          {0x1700, 0x1C00},
                                          {0x1100, 0x1400},
                                          {0x1100, 0x1400},
                                          {0x1100, 0x1400},};
    uint32_t fmmu_start_bit[7][2]      = {{0x0000, 0x0000}, 
                                          {0x0000, 0x0000},
                                          {0x0000, 0x0000},
                                          {0x0000, 0x0000},
                                          {0x0000, 0x0000},
                                          {0x0000, 0x0000},
                                          {0x0000, 0x0000},};                                          
    uint32_t fmmu_dir[7][2]            = {{0x0002, 0x0001}, 
                                          {0x0002, 0x0001},
                                          {0x0002, 0x0001},
                                          {0x0002, 0x0001},
                                          {0x0002, 0x0001},
                                          {0x0002, 0x0001},
                                          {0x0002, 0x0001},};           
    uint32_t fmmu_enable[7][2]         = {{0x0000, 0x0000}, 
                                          {0x0001, 0x0000},
                                          {0x0000, 0x0001},
                                          {0x0000, 0x0000},
                                          {0x0000, 0x0001},
                                          {0x0001, 0x0000},
                                          {0x0001, 0x0001},};           
    for (size_t i = 0; i < 7; i++)
    {
        for (size_t j = 0; j < 2; j++)
        {
            fmmu_manul->Fmmu_manul_slave_list[i].Fmmu_manul_unit_list[j].fmmu_num = j;
            fmmu_manul->Fmmu_manul_slave_list[i].Fmmu_manul_unit_list[j].fmmu_log_start_addr = fmmu_log_start_addr[i][j];
            fmmu_manul->Fmmu_manul_slave_list[i].Fmmu_manul_unit_list[j].fmmu_data_size = fmmu_data_size[i][j];
            fmmu_manul->Fmmu_manul_slave_list[i].Fmmu_manul_unit_list[j].fmmu_log_start_bit = fmmu_log_start_bit[i][j];
            fmmu_manul->Fmmu_manul_slave_list[i].Fmmu_manul_unit_list[j].fmmu_log_end_bit = fmmu_log_end_bit[i][j];
            fmmu_manul->Fmmu_manul_slave_list[i].Fmmu_manul_unit_list[j].fmmu_phy_start_addr = fmmu_phy_start_addr[i][j];
            fmmu_manul->Fmmu_manul_slave_list[i].Fmmu_manul_unit_list[j].fmmu_start_bit = fmmu_start_bit[i][j];
            fmmu_manul->Fmmu_manul_slave_list[i].Fmmu_manul_unit_list[j].fmmu_dir = fmmu_dir[i][j];
            fmmu_manul->Fmmu_manul_slave_list[i].Fmmu_manul_unit_list[j].fmmu_enable = fmmu_enable[i][j];
        }
        
    }
    
    // 遍历每个从设备
    for (uint32_t i = 0; i < num_slaves; i++) {
        Fmmu_Manul_Slave* slave = &fmmu_manul->Fmmu_manul_slave_list[i];
        printf("从设备 %d:\n", i);
        printf("  FMMU总数量: %d\n", slave->fmmu_total_quantity);

        // 遍历该从设备的每个FMMU单元
        for (uint32_t j = 0; j < slave->fmmu_total_quantity; j++) {
            Fmmu_Manul_Unit* unit = &slave->Fmmu_manul_unit_list[j];
            printf("    FMMU单元 %d:\n", j);
            printf("      编号: %d\n", unit->fmmu_num);
            printf("      逻辑起始地址: 0x%x\n", unit->fmmu_log_start_addr);
            printf("      数据长度: %d\n", unit->fmmu_data_size);
            printf("      逻辑起始位: %d\n", unit->fmmu_log_start_bit);
            printf("      逻辑结束位: %d\n", unit->fmmu_log_end_bit);
            printf("      物理起始地址: 0x%x\n", unit->fmmu_phy_start_addr);
            printf("      起始位: %d\n", unit->fmmu_start_bit);
            printf("      类型: %d\n", unit->fmmu_dir);
            printf("      使能: %d\n", unit->fmmu_enable);
            printf("      保留字段: %d, %d, %d\n", unit->fmmu_reserved1, unit->fmmu_reserved2, unit->fmmu_reserved3);
        }
    }

    return fmmu_manul;
}
