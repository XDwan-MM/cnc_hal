# 不定向进入 CSP

接口位于 include/hal_rt_api.h：

```c
int32_t hal_rt_spindle_prepare_csp(HalContext *c, HalAxisId id, uint32_t timeout_ms);
int32_t hal_rt_spindle_prepare_csp_state(HalContext *c, HalAxisId id,
                                       int32_t *state, int32_t *result);
```

这不是 M 代码解析器。它只负责停转、切 CSP、使能与位置保持；不发送定向控制字，
不处理 PLC FIN，也不自动清零坐标。不修改原 M19 或 M29 的调用路径。

## 调用顺序

由拥有 HAL context 的同一个周期线程操作。不要从 MDI/UI 线程或另一个进程直接调用。

1. 正常运行 wait -> begin -> commit；主轴实际模式必须已是 CSV 或 CSP。
2. 在 begin 与 commit 之间，对一次新请求调用 prepare_csp(c, id, 10000)，
   其中 10000 是整个操作的超时毫秒数，按机床允许停车时间确定。
3. 返回 HAL_OK 只表示受理；继续正常周期，后续每拍查询 prepare_csp_state。
   不要每拍重发 prepare。重复请求返回 HAL_ERROR_BUSY，不重置超时。
4. 查询本身返回 HAL_OK 后，再看 state/result：
   STOPPING 或 SWITCHING 表示进行中；DONE 且 result == HAL_OK 表示完成；
   ERROR 或 CANCELLED 表示本次终止，不允许继续发送运动目标。
5. DONE 后核对实际 CSP、使能与 readiness，再交给唯一的位置插补输出者。
   后续运动必须经过上层轨迹规划。准备完成不等于 PLC FIN 完成。

调用方式示意（嵌入已有周期控制，不是独立可执行程序）：

```c
/* After a successful begin, once for a new request: */
int32_t rc = hal_rt_spindle_prepare_csp(c, spindle_id, 10000);
/* rc != HAL_OK: report rejection; do not advance the machining sequence. */

/* On subsequent successful begin cycles: */
int32_t state, result;
rc = hal_rt_spindle_prepare_csp_state(c, spindle_id, &state, &result);
/* Check rc first; only DONE + HAL_OK permits handing over to position control.
 * Always retain the application's normal commit/error/stop handling. */
```

## 行为与限制

- CSV 下写零速度，驱动器负责实际减速。HAL 不做软件斜坡；上机前必须验证驱动停车参数。
- 实际速度绝对值 <= 1 rpm 连续至少 10 ms 后才切 CSP；切换后再次确认至少 10 ms
  的低速、使能及位置保持。位置保持容限取命令/反馈一个计数对应用户单位的较大值。
- 切换复用 DS402 的实际位置预置，命令/反馈当量不同也分别换算。
- 已经在 CSP 运动时拒绝受理，不抢走正在运行的插补。定向未释放、复位或模式切换中也拒绝。
- 准备期间位置、速度、模式、定向与设零请求返回 BUSY；调用者应暂停原有输出者。
  不能让 RT 旧的逐拍位置/模式请求与该准备操作争用。
- 去使能、急停、故障复位取消本次准备。故障或总超时转 ERROR 并撤销使能意图。
  总线错误时查询返回总线错误，不能把旧 DONE 当成当前安全保证。
- 保留累计坐标：原位置 720 度，完成后仍是原坐标附近，不变成 0 度。
  若实时端确需设零，须在完成后显式调用 hal_rt_axis_set_pos，并同步其累计目标基准。
- hal_rt_spindle_write_pos 是单圈刻度接口；hal_rt_axis_write_pos 是累计位置接口。
  两者都不生成轨迹，不应把大角度终点直接作为单拍 CSP 阶跃目标。

## 验证状态

离线验证包含冷启动、运动中等停稳、未提交旧速度覆盖、保留非零坐标、
不同命令/反馈当量、重复请求、互斥请求、超时、故障、急停、去使能及复位取消。
这些用例并入 HAL 现有 context_regression.c，没有新增独立测试脚本。
未完成实机验证；驱动减速、机械惯性和 PLC 握手仍由现场验收确认。
