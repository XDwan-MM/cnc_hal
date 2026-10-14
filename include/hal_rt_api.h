#pragma once
#include "hal_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 周期面：单个 RT 线程串行调用，一个周期固定三拍
 *
 *     hal_rt_wait_cycle()  ->  hal_rt_begin_cycle()  ->  hal_rt_commit_cycle()
 *
 * wait 是唯一阻塞点。begin 与 commit 之间可调 write_* / read_*：read_* 读本拍 begin
 * 的快照（begin 之前没有快照），write_* 只暂存、commit 才下发总线，故同拍先写后读
 * 读不到自己刚写的值。上一拍 commit 之后才能进下一次 wait。
 *
 * 公共返回码（下面不再逐个重复）：
 *   HAL_ERROR_ARGUMENT    参数为空、轴号不存在、越界或非有限值
 *   HAL_ERROR_NOT_RUNNING 未 start，或所需运行状态未达成
 *   HAL_ERROR_STOPPED     已 request_stop，或等待被停止打断
 *   HAL_ERROR_BUS         总线失败；首个错误码闭锁到 stop/start。request_stop 只通知，
 *                         资源仍须 stop/destroy。
 *
 * 参数约定：c = 已 start 的 context；id = 逻辑轴号。 */

/**
 * @brief 周期第一拍：等主站的下一个周期
 * @return 0 = 成功；HAL_ERROR_STATE = 上一拍未走完；其余见文件头
 */
int32_t hal_rt_wait_cycle(HalContext* c);

/**
 * @brief 周期第二拍：采轴实际值、各推一拍 DS402 状态机、读 IO 输入，并补发待清零的速度目标
 * @return 0 = 成功；成功后 read_* 才有本拍快照。HAL_ERROR_STATE = 未先 wait；
 *         HAL_ERROR_ARGUMENT = 非同一坐标系的预置值超出 int32（本拍未写控制 PDO）
 */
int32_t hal_rt_begin_cycle(HalContext* c);

/**
 * @brief 周期第三拍：下发本拍暂存的轴指令与 IO 输出，提交主站，回空闲相
 * @return 0 = 成功；HAL_ERROR_STATE = 未先 begin；其余见文件头
 */
int32_t hal_rt_commit_cycle(HalContext* c);

/**
 * @brief 使能 / 去使能一根进给轴
 * 去使能停在 SwitchedOn（可收指令、不带载）；断电用 hal_rt_axis_estop。
 * 已有撤销电压覆盖时 on=0 保持断电；显式 on=1 或受理故障复位可解除覆盖，同拍停止优先。
 * 请求变化会丢弃该轴未提交的运动指令并取消定向；恢复运动须重新下发目标。
 * @param on 1 = 使能，0 = 去使能；其它值拒绝
 * @return 0 = 成功；其余见文件头
 */
int32_t hal_rt_axis_enable(HalContext* c, HalAxisId id, int32_t on);

/**
 * @brief 在 CSP 模式下下发目标位置（用户单位）
 * 受理条件：本拍已 begin 未 commit、轴已使能、状态机到位、实际模式为 CSP。
 * 坐标相对 hal_rt_axis_set_pos 建立的基准。
 * @param pos 目标位置（用户单位），须有限
 * @return 0 = 成功；HAL_ERROR_NOT_RUNNING = 上述条件不满足；
 *         HAL_ERROR_ARGUMENT = pos 非有限或换算后超出 int32；其余见文件头
 */
int32_t hal_rt_axis_write_pos(HalContext* c, HalAxisId id, double pos);

/**
 * @brief 读最近一次采样的实际位置（用户单位）；只读快照，不访问总线
 * @param pos 带出实际位置
 * @return 0 = 成功；其余见文件头
 */
int32_t hal_rt_axis_read_pos(HalContext* c, HalAxisId id, double* pos);

/**
 * @brief 请求清除驱动器故障（DS402 bit7 上升沿）
 * 非阻塞：只置意图，边沿由之后的 begin 产生（先 0x0000，再 0x0080/0x0000 交替），
 * Fault 消失或超时后释放 bit7。超时自首次高电平拍起算，fault_reset_timeout_ms 按
 * cycle_us 向上取整。
 * 终态一律未使能（成功、超时、被急停打断都不恢复使能），再运动须显式 enable。
 * 重复调用会重开低电平准备阶段并重置超时预算；不要每拍重发（永不超时），查进展用
 * hal_rt_axis_fault_reset_state()。急停/去使能优先：同拍有停止请求返回 HAL_ERROR_STATE；
 * 复位期间停止会取消复位（回 HAL_RESET_NONE），并清除 stop_override；持续急停须每拍
 * 请求停止。PENDING 时 enable 返回 HAL_ERROR_NOT_RUNNING。
 * @return 0 = 成功；HAL_ERROR_STATE = 不在 begin 与 commit 之间；其余见文件头
 */
int32_t hal_rt_axis_fault_reset(HalContext* c, HalAxisId id);

/**
 * @brief 查故障复位进展
 * @param out 带出 HAL_RESET_* 之一（见 hal_config_api.h）
 * @return 0 = 成功；其余见文件头
 */
int32_t hal_rt_axis_fault_reset_state(const HalContext* c, HalAxisId id, int32_t* out);

/**
 * @brief 读最近一次采样的轴状态
 * @param out 状态快照；使能位取自状态字的 DS402 判定
 * @return 0 = 成功；HAL_ERROR_NOT_RUNNING = 还没跑过 begin；其余见文件头
 */
int32_t hal_rt_axis_read_status(HalContext* c, HalAxisId id, HalCAxisStatus* out);

/**
 * @brief 把该轴当前位置重新定义为 pos（用户单位）
 * 只平移命令侧与反馈侧坐标基准，不写驱动器、不改已暂存目标，故本身不会让轴动；使能中可调。
 * @param pos 新当前位置（用户单位），须有限
 * @return 0 = 成功；HAL_ERROR_NOT_RUNNING = 还没跑过 begin；其余见文件头
 */
int32_t hal_rt_axis_set_pos(HalContext* c, HalAxisId id, double pos);

/**
 * @brief 急停一根进给轴
 * 取消未提交的运动、使能与模式请求，按 estop_action 覆盖本拍控制字（0x07 或 0x00）；
 * begin 之后到达时 commit 覆盖 begin 写下的控制字与速度目标。重复调用幂等；
 * 恢复须显式 enable 并再经一次 begin。
 * @note 总线故障闭锁时仍受理并记录意图，但闭锁下不发 PDO，故「受理成功」不等于硬件已执行。
 * @return 0 = 成功；其余见文件头
 */
int32_t hal_rt_axis_estop(HalContext* c, HalAxisId id);

/**
 * @brief 使能 / 去使能一根主轴
 * @param id 逻辑轴号，且必须是主轴
 * @param on 1 = 使能，0 = 去使能；其它值拒绝
 * @return 0 = 成功；HAL_ERROR_ARGUMENT = 不是主轴；其余见文件头
 */
int32_t hal_rt_spindle_enable(HalContext* c, HalAxisId id, int32_t on);

/**
 * @brief 在 CSV 模式下下发转速指令
 * 受理条件：本拍已 begin 未 commit、轴已使能、状态机到位、实际模式为 CSV。
 * 转向由 dir 决定，rpm 不接受负值。
 * @param id  逻辑轴号，且必须是主轴
 * @param rpm 转速大小（rpm），须有限且 0 <= rpm <= 配置 max_speed
 * @param dir 1 = 正转（M03），-1 = 反转（M04），0 = 停
 * @return 0 = 成功；HAL_ERROR_NOT_RUNNING = 运行状态不满足；其余见文件头
 */
int32_t hal_rt_spindle_write_speed(HalContext* c, HalAxisId id, double rpm, int32_t dir);

/**
 * @brief 读最近一次采样的实际转速
 * @param id  逻辑轴号，且必须是主轴
 * @param rpm 带出实际转速（rpm，正 = 正转）
 * @return 0 = 成功；其余见文件头
 */
int32_t hal_rt_spindle_read_speed(HalContext* c, HalAxisId id, double* rpm);

/**
 * @brief 读最近一次采样的主轴状态
 * @param id  逻辑轴号，且必须是主轴
 * @param out 转速侧快照
 * @return 0 = 成功；HAL_ERROR_NOT_RUNNING = 还没跑过 begin；其余见文件头
 */
int32_t hal_rt_spindle_read_status(HalContext* c, HalAxisId id, HalCSpindleStatus* out);

/* 主轴定向：厂商专有控制字，整字覆盖 0x6040
 *
 * 定向字与完成位由配置给（HalCAxisCfg 的 vendor_orient_control_word /
 * vendor_orient_done_mask）；定向期间整字覆盖普通 DS402 控制字。
 * 优先级：急停 > 定向字 > 普通 DS402 推进。
 *
 * 约束：
 * 1. 定向期间控制字与状态字语义由驱动器决定，HAL 不保证 DS402 合规，也不解读它。
 * 2. 释放时 HAL 撤销厂商字并按当前使能意图补写普通控制字，不假定驱动器已去使能。
 * 3. 调用方必须保证「请求 -> 等 DONE -> 释放」，异常路径也要释放：不释放则驱动器
 *    停在厂商状态，普通控制字无法接管。
 * 4. 两个配置都为 0 表示该轴不支持定向，request_orient 返回 HAL_ERROR_UNSUPPORTED。 */

/**
 * @brief 请求主轴定向（厂商专有控制字）
 * 受理后每拍下发 vendor_orient_control_word，普通 DS402 推进暂停，直到 release_orient。
 * 完成判据：先采到 done mask 全低，再采到高并保持 vendor_orient_delay_ms；请求前的低电平、
 * 一直为高的 CSV 到速位不算本轮。
 * @param id 逻辑轴号，且必须是主轴
 * @return 0 = 已受理；HAL_ERROR_UNSUPPORTED = 未配定向控制字；
 *         HAL_ERROR_NOT_RUNNING = 不在 begin 与 commit 之间；
 *         HAL_ERROR_NOT_ENABLED = 未到 OperationEnabled；其余见文件头
 */
int32_t hal_rt_spindle_request_orient(HalContext* c, HalAxisId id);

/**
 * @brief 定向是否已完成（状态字的 done 位）；只读
 * @param id 逻辑轴号，且必须是主轴
 * @param out 带出 HAL_ORIENT_*
 * @return 0 = 成功；其余见文件头
 */
int32_t hal_rt_spindle_orient_state(HalContext* c, HalAxisId id, int32_t* out);

/**
 * @brief 释放定向，交回普通 DS402 状态机
 * 清定向状态，并在 commit 时按最终使能意图补写普通控制字（避免厂商字残留在 PDO）；
 * 急停与故障复位优先。
 * 不改变使能意图：释放后是否带力矩由 hal_rt_axis_enable() 决定。
 * @param id 逻辑轴号，且必须是主轴
 * @return 0 = 成功；其余见文件头
 */
int32_t hal_rt_spindle_release_orient(HalContext* c, HalAxisId id);

/**
 * @brief 读一根轴还差什么才能受理运动指令；只读
 * 位含义（=1 表示该项不满足）：bit0 PHASE 不在 begin 与 commit 之间；
 * bit1 NOT_ENABLED 未到 OperationEnabled；bit2 NOT_READY 状态机或采样未到位；
 * bit4 MODE_ACTUAL 实际模式（0x6061）未跟上 HAL 要求的模式。全 0 = 就绪。
 * 模式切换追赶期间会短暂非零，持续非零才需处置。
 * @note 本查询只比「实际 vs 要求」，要求由 hal_rt_spindle_request_mode() 设定。
 *       write_pos 要 CSP、write_speed 要 CSV，故顺序为：设期望模式 -> 等全 0 -> 下发。
 * @param out 带出位掩码
 * @return 0 = 成功；HAL_ERROR_ARGUMENT = 轴号不存在或 out 为空；其余见文件头
 */
int32_t hal_rt_axis_readiness(HalContext* c, HalAxisId id, uint32_t* out);

/**
 * @brief 急停一根主轴；语义同 hal_rt_axis_estop，断电/去使能由 estop_action 决定
 * @param id 逻辑轴号，且必须是主轴
 * @return 0 = 成功；其余见文件头
 */
int32_t hal_rt_spindle_estop(HalContext* c, HalAxisId id);

/**
 * @brief 请求切换主轴运行模式（CSP / CSV）
 * 只改期望模式，不动使能状态；实际切换在之后的 begin 完成（默认先退回 SwitchedOn 再写 0x6060）。
 * 期望模式变化时才丢弃未提交的运动指令并取消定向；重复调用无副作用。
 * @param id   逻辑轴号，且必须是主轴
 * @param mode HAL_SPINDLE_CSP 或 HAL_SPINDLE_CSV
 * @return 0 = 成功；其余见文件头
 */
int32_t hal_rt_spindle_request_mode(HalContext* c, HalAxisId id, HalSpindleMode mode);

/**
 * @brief 在 CSP 模式下下发相对既定零点的主轴单圈刻度（度）
 * 零点由 hal_rt_axis_set_pos(c, id, 0) 建立；命令侧基准在首次采样与使能那一拍被拉平到实际
 * 位置（hal_context.c 的 sample/enable），所以还没写过位置时基准就是当前位置。
 * 范围 [-360, 360]：正/负表示方向，0 取最近零刻度；方向由上一条已受理目标计算。
 * 重复同一普通目标不再转一圈（设零、通用累计位置写入、取消运动都会重置该判定）；
 * +360/-360 是整圈命令，重复调用会重复转圈。不生成轨迹，速度/加速度由上层负责；
 * 受理条件同 write_pos。
 * @param id  逻辑轴号，且必须是主轴
 * @param deg 相对既定零点的刻度（度），范围 [-360, 360]
 * @return 0 = 成功；其余见文件头
 */
int32_t hal_rt_spindle_write_pos(HalContext* c, HalAxisId id, double deg);

/**
 * @brief 把本拍采到的输入映像拷到调用方 PLC X 区
 * 普通 IO 与面板共用一块映像，各段按配置的 X 起始地址落位：段内覆盖、段外补 0，
 * 调用方不必先清缓冲。
 * @param x_image 目标缓冲
 * @param x_len   缓冲长度，须 >= 映像长度（start 时按配置算出的 x_size）
 * @return 0 = 成功；HAL_ERROR_ARGUMENT = 缓冲不够长；HAL_ERROR_NOT_RUNNING = 还没跑过 begin；其余见文件头
 */
int32_t hal_rt_io_snapshot_inputs(HalContext* c, uint8_t* x_image, uint32_t x_len);

/**
 * @brief 把调用方 PLC Y 区拆成输出 Entry，暂存待 commit 下发
 * 只在 begin 与 commit 之间调用；本拍不调用则本拍不写输出（旧值保留）。
 * 总线按整 Entry 下发，只改其中几位需调用方自己读改写。
 * @param y_image 源缓冲
 * @param y_len   缓冲长度，须 >= 映像长度（start 时按配置算出的 y_size）
 * @return 0 = 成功；HAL_ERROR_STATE = 不在 begin 与 commit 之间；
 *         HAL_ERROR_ARGUMENT = 缓冲不够长；其余见文件头
 */
int32_t hal_rt_io_flush_outputs(HalContext* c, const uint8_t* y_image, uint32_t y_len);

#ifdef __cplusplus
}
#endif
