#pragma once
#include "hal_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 周期面 —— 由单个 RT 线程串行调用，一个周期固定三拍：
 *
 *     hal_rt_wait_cycle()  →  hal_rt_begin_cycle()  →  hal_rt_commit_cycle()
 *
 * wait 是唯一的阻塞点。begin 与 commit 之间可以调 write_* / read_*；read_* 读的是
 * 本次 begin 采到的快照，begin 之前没有快照。write_* 一律只暂存，到 commit 才真正
 * 下发总线——所以同一拍里先写后读，读不到自己刚写的值。上一拍 commit 之后才能进
 * 下一次 wait。
 *
 * 公共返回码（每个函数都可能返回，下面不再逐个重复）：
 *   HAL_ERROR_ARGUMENT    context 为空、轴号不存在、入参越界或非有限值
 *   HAL_ERROR_NOT_RUNNING 未 start，或该操作要求的运行状态尚未达成
 *   HAL_ERROR_STOPPED     已 request_stop，或阻塞等待被主动停止打断
 *   HAL_ERROR_BUS         总线失败。首个错误码被闭锁，此后所有周期调用持续返回它，
 *                         直到 stop/start 清掉。request_stop 只写通知、不释放资源，
 *                         仍须走 stop/destroy。
 *
 * 详细契约见 docs/api-reference.md 第 4 章「周期接口与线程约束」。 */

/**
 * @brief 周期第一拍：等主站的下一个周期
 *
 * @param c 已 start 的 context
 * @return int32_t 0 = 成功；HAL_ERROR_STATE = 上一拍未走完（phase != 0）；其余见文件头
 */
int32_t hal_rt_wait_cycle(HalContext* c);

/**
 * @brief 周期第二拍：采所有轴的实际值、各推一拍 DS402 状态机、读 IO 输入
 *
 * 返回成功后 read_* 才拿得到本拍快照。使能、取消运动或急停留下的「速度目标清零」
 * 也在这一拍补发。
 *
 * @param c 已 start 的 context
 * @return int32_t 0 = 成功；HAL_ERROR_STATE = 未先 wait；
 *         HAL_ERROR_ARGUMENT = 非同一计数坐标系的预置值超出 int32；
 *         此时未写本拍控制 PDO，调用方应停止周期并处置配置。
 */
int32_t hal_rt_begin_cycle(HalContext* c);

/**
 * @brief 周期第三拍：下发本拍暂存的轴指令与 IO 输出，提交主站，回到空闲相
 *
 * @param c 已 start 的 context
 * @return int32_t 0 = 成功；HAL_ERROR_STATE = 未先 begin；其余见文件头
 */
int32_t hal_rt_commit_cycle(HalContext* c);

/**
 * @brief 使能 / 去使能一根进给轴
 *
 * 去使能停在 SwitchedOn（可收指令、不带载），不是断电——要断电用 hal_rt_axis_estop。
 * 已有撤销电压急停覆盖时，on=0 保持断电，不降成 0x07；显式 on=1 或后续拍
 * 受理的故障复位可解除历史覆盖，同拍停止仍优先。
 * 请求发生变化会丢弃该轴尚未提交的运动指令，恢复运动须重新下发目标位置。
 *
 * @param c  已 start 的 context
 * @param id 逻辑轴号
 * @param on 1 = 使能，0 = 去使能；其它值拒绝
 * @return int32_t 0 = 成功；其余见文件头
 */
int32_t hal_rt_axis_enable(HalContext* c, HalAxisId id, int32_t on);

/**
 * @brief 在 CSP 模式下下发目标位置（用户单位）
 *
 * 只在「本拍已 begin 且未 commit」+ 该轴已使能 + 状态机已到位 + 运行模式确为 CSP 时
 * 受理。坐标相对 hal_rt_axis_set_pos 建立的基准。
 *
 * @param c   已 start 的 context
 * @param id  逻辑轴号
 * @param pos 目标位置（用户单位），须为有限值
 * @return int32_t 0 = 成功；HAL_ERROR_NOT_RUNNING = 上面任一条件不满足；
 *                 HAL_ERROR_ARGUMENT = pos 非有限值或换算后超出 int32 计数；其余见文件头
 */
int32_t hal_rt_axis_write_pos(HalContext* c, HalAxisId id, double pos);

/**
 * @brief 读最近一次采样的实际位置（用户单位）
 *
 * 只读快照，不触发总线访问。
 *
 * @param c   已 start 的 context
 * @param id  逻辑轴号
 * @param pos 带出实际位置
 * @return int32_t 0 = 成功；其余见文件头
 */
int32_t hal_rt_axis_read_pos(HalContext* c, HalAxisId id, double* pos);

/**
 * @brief 请求清除驱动器的故障（DS402 Fault Reset）
 *
 * 非阻塞：只置意图，真正的控制字边沿（0x0080）由之后的 hal_rt_begin_cycle 产生。
 * 先发送一拍 0x0000，再交替发送 0x0080/0x0000，形成完整 bit7 上升沿。
 * Fault 消失或超时后发送 0x0000 释放 bit7；超时从首次高电平拍起算，
 * fault_reset_timeout_ms 按 cycle_us 向上取整，热路径不取时钟。
 *
 * **终态一律是未使能**——不论成功、超时还是中途被急停打断，轴都不会自动恢复使能。
 * 要重新运动必须显式 hal_rt_axis_enable()，且要再经一次 begin 确认状态机到位。
 * 这是有意的：驱动器刚从故障恢复时自动上使能是危险的。
 *
 * 重复调用刷新请求：从低电平准备阶段重新开始，首次高电平拍重新计超时；
 * 刷新请求不能代替查询 hal_rt_axis_fault_reset_state()。
 * 急停/去使能优先：同拍已有停止请求时返回 HAL_ERROR_STATE；复位期间的停止
 * 取消复位，状态回到 HAL_RESET_NONE。后续拍可显式请求复位，仍不会自动使能。
 * 后续拍受理复位会清除此前急停/去使能的停止动作覆盖（stop_override）。
 * 一次停止请求不是永久闩锁；持续急停须每拍请求停止，条件解除前禁止发起复位。
 * 复位 PENDING 时使能返回 HAL_ERROR_NOT_RUNNING；停止通知/总线闭锁按公共码拒绝。
 *
 * @param c  已 start 的 context
 * @param id 逻辑轴号
 * @return int32_t 0 = 成功；HAL_ERROR_STATE = 不在 begin 与 commit 之间；其余见文件头
 */
int32_t hal_rt_axis_fault_reset(HalContext* c, HalAxisId id);

/**
 * @brief 查故障复位的进展
 *
 * @param c   已 start 的 context
 * @param id  逻辑轴号
 * @param out 带出 HAL_RESET_* 之一（见 hal_config_api.h）
 * @return int32_t 0 = 成功；其余见文件头
 */
int32_t hal_rt_axis_fault_reset_state(const HalContext* c, HalAxisId id, int32_t* out);

/**
 * @brief 读最近一次采样的轴状态
 *
 * @param c   已 start 的 context
 * @param id  逻辑轴号
 * @param out 带出状态快照；使能位取自状态字的 DS402 判定，不是 HAL 自己的推断
 * @return int32_t 0 = 成功；HAL_ERROR_NOT_RUNNING = 还没跑过 begin，没有快照；其余见文件头
 */
int32_t hal_rt_axis_read_status(HalContext* c, HalAxisId id, HalCAxisStatus* out);

/**
 * @brief 把该轴的当前位置重新定义为 pos（用户单位）
 *
 * 回零、对刀之后设坐标用的。只平移命令侧与反馈侧的坐标基准，不写驱动器，也不改动
 * 已暂存的运动目标，所以调用它本身不会让轴动。使能中也可以调。
 *
 * @param c   已 start 的 context
 * @param id  逻辑轴号
 * @param pos 新的当前位置（用户单位），须为有限值
 * @return int32_t 0 = 成功；HAL_ERROR_NOT_RUNNING = 还没跑过 begin；其余见文件头
 */
int32_t hal_rt_axis_set_pos(HalContext* c, HalAxisId id, double pos);

/**
 * @brief 急停一根进给轴
 *
 * 取消尚未提交的运动与使能、模式请求，按配置的 estop_action 把本拍控制字覆盖成
 * 去使能（0x07）或断电（0x00）。若在 begin 之后才到达，commit 会用急停值覆盖
 * begin 写下的旧控制字和速度目标，防止本拍仍把旧的使能动作发出去。重复调用幂等，
 * 恢复须显式 enable 并再次经 begin 确认后才可运动。
 *
 * @note 有意不因总线故障闭锁而拒绝：此时仍记录停止意图，但闭锁下不再发 PDO，
 *       所以「受理成功」不等于失联硬件真的执行了。
 *
 * @param c  已 start 的 context
 * @param id 逻辑轴号
 * @return int32_t 0 = 成功；其余见文件头
 */
int32_t hal_rt_axis_estop(HalContext* c, HalAxisId id);

/**
 * @brief 使能 / 去使能一根主轴
 *
 * @param c  已 start 的 context
 * @param id 逻辑轴号，且必须是主轴
 * @param on 1 = 使能，0 = 去使能；其它值拒绝
 * @return int32_t 0 = 成功；HAL_ERROR_ARGUMENT = 该轴号不是主轴；其余见文件头
 */
int32_t hal_rt_spindle_enable(HalContext* c, HalAxisId id, int32_t on);

/**
 * @brief 在 CSV 模式下下发转速指令
 *
 * 只在「本拍已 begin 且未 commit」+ 该轴已使能 + 状态机已到位 + 运行模式确为 CSV 时
 * 受理。转向由 dir 决定，所以 rpm 本身不接受负值
 * 本拍带出的 at_speed 先按指令值算，下一拍采样时再按实测刷新。
 *
 * @param c   已 start 的 context
 * @param id  逻辑轴号，且必须是主轴
 * @param rpm 转速大小（rpm），须有限且 0 <= rpm <= 配置的 max_speed
 * @param dir 1 = 正转（M03），-1 = 反转（M04），0 = 停
 * @return int32_t 0 = 成功；HAL_ERROR_NOT_RUNNING = 运行状态不满足；其余见文件头
 */
int32_t hal_rt_spindle_write_speed(HalContext* c, HalAxisId id, double rpm, int32_t dir);

/**
 * @brief 读最近一次采样的实际转速
 *
 * @param c   已 start 的 context
 * @param id  逻辑轴号，且必须是主轴
 * @param rpm 带出实际转速（rpm，带符号：正 = 正转）
 * @return int32_t 0 = 成功；其余见文件头
 */
int32_t hal_rt_spindle_read_speed(HalContext* c, HalAxisId id, double* rpm);

/**
 * @brief 读最近一次采样的主轴状态
 *
 * @param c   已 start 的 context
 * @param id  逻辑轴号，且必须是主轴
 * @param out 带出转速侧快照，含 at_speed（|指令 - 实际| 是否已进入转速窗口）
 * @return int32_t 0 = 成功；HAL_ERROR_NOT_RUNNING = 还没跑过 begin，没有快照；其余见文件头
 */
int32_t hal_rt_spindle_read_status(HalContext* c, HalAxisId id, HalCSpindleStatus* out);

/**
 * @brief 急停一根主轴
 *
 * 语义同 hal_rt_axis_estop，断电还是去使能由该主轴的 estop_action 配置决定。
 *
 * @param c  已 start 的 context
 * @param id 逻辑轴号，且必须是主轴
 * @return int32_t 0 = 成功；其余见文件头
 */
int32_t hal_rt_spindle_estop(HalContext* c, HalAxisId id);

/**
 * @brief 请求切换主轴的运行模式（CSP / CSV）
 *
 * 只改期望模式，不改变使能状态——急停中的轴不会因为切模式被重新使能。实际切换发生在
 * 之后的 begin 里，由 DS402 状态机保证「模式不对就不上使能」，默认先退回 SwitchedOn
 * 再改 0x6060。期望模式真的变了才丢弃尚未提交的运动指令，重复调用无副作用。
 *
 * @param c    已 start 的 context
 * @param id   逻辑轴号，且必须是主轴
 * @param mode HAL_SPINDLE_CSP 或 HAL_SPINDLE_CSV
 * @return int32_t 0 = 成功；其余见文件头
 */
int32_t hal_rt_spindle_request_mode(HalContext* c, HalAxisId id, HalSpindleMode mode);

/**
 * @brief 在 CSP 模式下下发相对既定零点的主轴单圈刻度（度）
 *
 * 对“上电位置为 0°”的主轴，调用方应在启动后首次采样成功且尚未运动时，
 * 调用 hal_rt_axis_set_pos(c, id, 0) 建立本次运行的零点；不依赖驱动器
 * 是否已经把原始位置 PDO 清零。
 * 普通目标位于 [-360, 360]：正数沿正向、负数沿反向到达该刻度；0
 * 取最近的零刻度。方向从上一条已受理目标计算，切入 CSP 时该目标预置为
 * 当前位置；取消运动后再次就绪时也从当时反馈重建基准。连续重复普通目标
 * 不会再转一圈（设零或通用累计位置写入会重置重复判定）；+360/-360 是分别正转/反转
 * 完整一圈的命令，重复调用会重复转圈。
 * 此接口不生成运动轨迹；速度和加速度控制由上层负责。受理条件同
 * hal_rt_axis_write_pos。
 *
 * @param c   已 start 的 context
 * @param id  逻辑轴号，且必须是主轴
 * @param deg 相对既定零点的刻度（度），范围 [-360, 360]；符号指定方向
 * @return int32_t 0 = 成功；其余见文件头
 */
int32_t hal_rt_spindle_write_pos(HalContext* c, HalAxisId id, double deg);

/**
 * @brief 把本拍采到的输入映像拷到调用方的 PLC X 区
 *
 * 普通 IO 与面板共用同一块映像，各段按配置的 X 起始地址落位：段内覆盖，段外补 0，
 * 所以调用方不必先清缓冲。
 *
 * @param c       已 start 的 context
 * @param x_image 目标缓冲
 * @param x_len   缓冲长度，必须 >= 映像长度（start 时按配置算出的 x_size）
 * @return int32_t 0 = 成功；HAL_ERROR_ARGUMENT = 缓冲不够长，说明调用方与 HAL 对地址域
 *                 的理解不一致；HAL_ERROR_NOT_RUNNING = 还没跑过 begin；其余见文件头
 */
int32_t hal_rt_io_snapshot_inputs(HalContext* c, uint8_t* x_image, uint32_t x_len);

/**
 * @brief 把调用方的 PLC Y 区拆成各输出 Entry，暂存待 commit 下发
 *
 * 只在 begin 与 commit 之间调用；本拍没调用则本拍不写任何输出，旧值原样保留。
 * 总线按整 Entry 下发，要只改其中几位的话由调用方自己做读改写。
 *
 * @param c       已 start 的 context
 * @param y_image 源缓冲
 * @param y_len   缓冲长度，必须 >= 映像长度（start 时按配置算出的 y_size）
 * @return int32_t 0 = 成功；HAL_ERROR_STATE = 不在 begin 与 commit 之间；
 *                 HAL_ERROR_ARGUMENT = 缓冲不够长；其余见文件头
 */
int32_t hal_rt_io_flush_outputs(HalContext* c, const uint8_t* y_image, uint32_t y_len);

#ifdef __cplusplus
}
#endif
