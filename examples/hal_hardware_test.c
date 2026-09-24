/* 真实主站/从站联调程序；所有设备参数来自 hal_hardware_test_config.c。
 * 本程序不是轨迹规划器，运动目标只能使用现场审核后的短距离目标。
 */
#define _POSIX_C_SOURCE 200809L
#include "hal_hardware_test_config.h"
#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

typedef struct {
    HalContext* ctx;
    const HalCConfig* cfg;
    const HalHardwareTestSettings* settings;
    unsigned cycle;
} Runner;

static int call(const char* name, int32_t rc) {
    if (rc != HAL_OK) {
        char message[96];
        hal_error_text(rc, message, sizeof(message));
        fprintf(stderr, "FAIL %s rc=%d (%s)\n", name, rc, message);
    }
    return rc == HAL_OK ? 0 : -1;
}

static int prompt(const char* message) {
    char line[32];
    printf("\n%s 输入 YES 后继续: ", message);
    fflush(stdout);
    return fgets(line, sizeof(line), stdin) && strcmp(line, "YES\n") == 0 ? 0 : -1;
}

static int begin_tick(Runner* r) {
    if (call("hal_rt_wait_cycle", hal_rt_wait_cycle(r->ctx))) return -1;
    if (call("hal_rt_begin_cycle", hal_rt_begin_cycle(r->ctx))) return -1;
    ++r->cycle;
    return 0;
}

static int end_tick(Runner* r) {
    return call("hal_rt_commit_cycle", hal_rt_commit_cycle(r->ctx));
}

static int tick(Runner* r) {
    if (begin_tick(r)) return -1;
    return end_tick(r);
}

static void stop_intent(Runner* r) {
    for (int i = 0; i < r->cfg->axis_count; ++i) {
        int logical = r->cfg->axes[i].logical_axis;
        if (logical >= 0) (void)hal_rt_axis_estop(r->ctx, (HalAxisId)logical);
    }
    for (int i = 0; i < r->cfg->spindle_count; ++i) {
        int logical = r->cfg->spindles[i].axis.logical_axis;
        (void)hal_rt_spindle_estop(r->ctx, (HalAxisId)logical);
    }
}

static int static_cases(const HalCConfig* config) {
    HalCConfig bad;
    char err[256];
    if (call("hal_config_validate(valid)", hal_config_validate(config, err, sizeof(err))))
        return -1;
#define NEGATIVE(field, value, wanted) do {                              \
    bad = *config; bad.field = value;                                     \
    int32_t rc = hal_config_validate(&bad, err, sizeof(err));             \
    if (rc != wanted) {                                                    \
        fprintf(stderr, "FAIL %-35s expected=%d actual=%d err=%s\n",      \
                #field, wanted, rc, err);                                  \
        return -1;                                                         \
    }                                                                      \
    printf("EXPECTED_REJECT %-27s rc=%d err=%s\n", #field, rc, err);      \
} while (0)
    NEGATIVE(abi_minor, (uint16_t)(HAL_C_ABI_MINOR + 1), HAL_ERROR_ABI);
    NEGATIVE(struct_size, 0, HAL_ERROR_ABI);
    NEGATIVE(reserved, 1, HAL_ERROR_ABI);
    NEGATIVE(topology_fingerprint, 1, HAL_ERROR_CONFIG);
    NEGATIVE(cycle_us, 0, HAL_ERROR_CONFIG);
    if (config->axis_count > 0) {
        NEGATIVE(axes[0].estop_action, 0, HAL_ERROR_CONFIG);
        NEGATIVE(axes[0].enc_off, NAN, HAL_ERROR_CONFIG);
        NEGATIVE(axes[0].command_units_per_count, 0, HAL_ERROR_CONFIG);
    }
#undef NEGATIVE
    printf("PASS static config cases\n");
    return 0;
}

/* 不访问主站的内置配置回归，可在现场参数未知时先运行。 */
static int selftest(void) {
    HalCConfig config = {0};
    config.abi_major = HAL_C_ABI_MAJOR;
    config.abi_minor = HAL_C_ABI_MINOR;
    config.struct_size = sizeof(config);
    config.cycle_us = 1000;
    config.start_timeout_ms = 10000;
    config.cycle_timeout_ms = 2000;
    config.axis_count = 3;
    config.spindle_count = 1;
    config.io_count = 1;
    config.panel_count = 1;
    for (int i = 0; i < 4; ++i) {
        HalCAxisCfg* a = &config.axes[i];
        if (i == 3) a = &config.spindles[0].axis;
        a->slave_pos = i == 3 ? 2 : i + 3;
        a->axis_index = 0;
        a->logical_axis = i == 3 ? 0 : i + 1;
        a->estop_action = HAL_ESTOP_DISABLE_VOLTAGE;
        a->work_mode = i == 3 ? HAL_WORK_VELOCITY : HAL_WORK_POSITION;
        a->encoder_type = HAL_ENC_INCREMENTAL_Z;
        a->feedback_pulses_per_rev = 1;
        a->feedback_wrap = HAL_WRAP_LINEAR;
        a->command_units_per_count = 1;
        a->feedback_units_per_count = 1;
    }
    config.spindles[0].max_speed = 1000;
    config.ios[0] = (HalCIoCfg){.slave_pos = 1, .x_start = 256, .y_start = 256};
    config.panels[0] = (HalCPanelCfg){.slave_pos = 0, .x_start = 0, .y_start = 0};
    return static_cases(&config);
}

static int identities(Runner* r) {
    int expected = 0;
    for (int i = 0; i < r->cfg->axis_count; ++i)
        expected += r->cfg->axes[i].logical_axis >= 0;
    expected += r->cfg->spindle_count;
    int count = hal_axis_count(r->ctx);
    printf("AXIS count=%d expected=%d\n", count, expected);
    if (count != expected) return -1;
    for (int i = 0; i < r->cfg->axis_count + r->cfg->spindle_count; ++i) {
        int logical = i < r->cfg->axis_count ? r->cfg->axes[i].logical_axis :
            r->cfg->spindles[i - r->cfg->axis_count].axis.logical_axis;
        if (logical < 0) continue;
        HalAxisId id = HAL_INVALID_ID;
        HalCIdentity identity;
        if (call("hal_axis_resolve", hal_axis_resolve(r->ctx, logical, &id)) ||
            id != (HalAxisId)logical ||
            call("hal_device_identity", hal_device_identity(r->ctx, id, &identity)))
            return -1;
        printf("IDENTITY logical=%d type=%d vendor=%u product=%u revision=%u "
               "serial=%u family=%d device_id=%llu name=%s\n",
               logical, identity.type, identity.vendor_id, identity.product_code,
               identity.revision, identity.serial, identity.family_index,
               (unsigned long long)identity.device_id, identity.name);
    }
    return 0;
}

static int print_snapshot(Runner* r) {
    for (int i = 0; i < r->cfg->axis_count + r->cfg->spindle_count; ++i) {
        int spindle = i >= r->cfg->axis_count;
        int logical = spindle ? r->cfg->spindles[i - r->cfg->axis_count].axis.logical_axis :
            r->cfg->axes[i].logical_axis;
        if (logical < 0) continue;
        HalCAxisStatus a;
        if (call("hal_rt_axis_read_status",
                 hal_rt_axis_read_status(r->ctx, (HalAxisId)logical, &a))) return -1;
        printf("CYCLE %u AXIS %d enabled=%d valid=%d pos=%.9f command=%.9f sw=0x%04x err=0x%04x\n",
               r->cycle, logical, a.enabled, a.position_valid, a.actual_pos,
               a.command_pos, a.raw_status, a.error_code);
        if (spindle) {
            HalCSpindleStatus s;
            if (call("hal_rt_spindle_read_status",
                     hal_rt_spindle_read_status(r->ctx, (HalAxisId)logical, &s))) return -1;
            printf("CYCLE %u SPINDLE %d mode=%d actual_rpm=%.6f command_rpm=%.6f at_speed=%d angle=%.6f\n",
                   r->cycle, logical, s.mode, s.actual_speed, s.command_speed,
                   s.at_speed, s.position_deg);
        }
    }
    uint32_t len = r->settings->x_len;
    if (len) {
        uint8_t* image = calloc(len, 1);
        if (!image) return -1;
        int rc = hal_rt_io_snapshot_inputs(r->ctx, image, len);
        if (call("hal_rt_io_snapshot_inputs", rc)) { free(image); return -1; }
        printf("CYCLE %u X[%u]=", r->cycle, len);
        for (uint32_t i = 0; i < len; ++i) printf("%02x", image[i]);
        putchar('\n');
        free(image);
    }
    return 0;
}

static int observe(Runner* r) {
    if (r->settings->observe_cycles == 0) {
        fprintf(stderr, "observe_cycles 必须大于 0\n");
        return -1;
    }
    for (unsigned i = 0; i < r->settings->observe_cycles; ++i) {
        if (begin_tick(r)) return -1;
        int result = print_snapshot(r);
        int commit = end_tick(r);
        if (result || commit) return -1;
    }
    return 0;
}

static int protocol_case(Runner* r) {
    HalAxisId id = (HalAxisId)r->cfg->axes[0].logical_axis;
    HalCAxisStatus status;
    int32_t rc = hal_rt_axis_read_status(r->ctx, id, &status);
    if (rc != HAL_ERROR_NOT_RUNNING) {
        fprintf(stderr, "首拍前读取状态应为 NOT_RUNNING，实际 %d\n", rc);
        return -1;
    }
    rc = hal_rt_begin_cycle(r->ctx);
    if (rc != HAL_ERROR_STATE) return -1;
    if (begin_tick(r)) return -1;
    rc = hal_rt_begin_cycle(r->ctx);
    if (rc != HAL_ERROR_STATE) return -1;
    rc = hal_rt_axis_write_pos(r->ctx, id, 0);
    if (rc != HAL_ERROR_NOT_RUNNING) return -1;
    if (end_tick(r)) return -1;
    rc = hal_rt_commit_cycle(r->ctx);
    if (rc != HAL_ERROR_STATE) return -1;
    if (tick(r)) return -1;
    printf("PASS protocol: 无快照读取、跳拍、重复 begin/commit、未使能运动门控\n");
    return 0;
}

static int read_x(Runner* r, uint8_t* image, uint32_t len) {
    if (begin_tick(r)) return -1;
    int rc = hal_rt_io_snapshot_inputs(r->ctx, image, len);
    int committed = end_tick(r);
    return call("hal_rt_io_snapshot_inputs", rc) || committed ? -1 : 0;
}

static int inputs_case(Runner* r) {
    uint32_t len = r->settings->x_len;
    if (!len || len > HAL_TEST_IMAGE_MAX) return -1;
    uint8_t* before = calloc(len, 1);
    uint8_t* after = calloc(len, 1);
    if (!before || !after) { free(before); free(after); return -1; }
    int result = -1;
    if (prompt("将面板/IO 输入保持基线状态")) goto done;
    if (read_x(r, before, len)) goto done;
    if (prompt("只改变一个待测面板按钮或 IO 输入点，并保持状态")) goto done;
    if (read_x(r, after, len)) goto done;
    int changed = 0;
    for (uint32_t i = 0; i < len; ++i) {
        uint8_t delta = before[i] ^ after[i];
        if (delta) {
            printf("INPUT_CHANGED X byte=%u before=0x%02x after=0x%02x xor=0x%02x\n",
                   i, before[i], after[i], delta);
            ++changed;
        }
    }
    if (!changed) fprintf(stderr, "输入未变化；检查接线、位映射和操作\n");
    result = changed ? 0 : -1;
done:
    free(before);
    free(after);
    return result;
}

static int write_image(Runner* r, const uint8_t* image) {
    if (begin_tick(r)) return -1;
    int rc = hal_rt_io_flush_outputs(r->ctx, image, r->settings->y_len);
    int committed = end_tick(r);
    return call("hal_rt_io_flush_outputs", rc) || committed ? -1 : 0;
}

static int io_case(Runner* r) {
    if (r->cfg->io_count + r->cfg->panel_count == 0 ||
        !r->settings->io_images_confirmed ||
        r->settings->y_len == 0 || r->settings->y_len > HAL_TEST_IMAGE_MAX ||
        r->settings->x_len == 0 || r->settings->x_len > HAL_TEST_IMAGE_MAX) {
        fprintf(stderr, "IO 用例需要已审核的 X/Y 长度和映像\n");
        return -1;
    }
    if (prompt("确认 Y 安全映像及测试映像已逐位审核，危险负载已隔离")) return -1;
    if (write_image(r, r->settings->y_safe)) return -1;
    if (tick(r)) return -1;
    if (prompt("检查当前端子为安全状态，准备切到测试映像")) return -1;
    if (write_image(r, r->settings->y_test)) goto restore;
    if (begin_tick(r)) goto restore;
    int sampled = print_snapshot(r);
    int committed = end_tick(r);
    if (sampled || committed) goto restore;
    if (prompt("确认实物输出仅在批准点位变化；继续会恢复安全映像")) goto restore;
    if (write_image(r, r->settings->y_safe)) return -1;
    return tick(r);
restore:
    (void)write_image(r, r->settings->y_safe);
    return -1;
}

static int axis_case(Runner* r, int feed_index) {
    const HalHardwareTestSettings* s = r->settings;
    const HalFeedHardwareCase* f = &s->feed[feed_index];
    if (!f->scaling_confirmed || !isfinite(f->target) ||
        !isfinite(f->max_step) || !isfinite(f->tolerance) ||
        f->max_step <= 0 || f->tolerance < 0 || !s->settle_cycles) return -1;
    HalAxisId id = (HalAxisId)r->cfg->axes[feed_index].logical_axis;
    HalCAxisStatus a;
    if (begin_tick(r)) return -1;
    int rc = hal_rt_axis_read_status(r->ctx, id, &a);
    int commit = end_tick(r);
    if (call("hal_rt_axis_read_status", rc) || commit) return -1;
    printf("AXIS %u start=%.9f target=%.9f max_step=%.9f\n",
           id, a.actual_pos, f->target, f->max_step);
    if (fabs(f->target - a.actual_pos) > f->max_step) {
        fprintf(stderr, "目标超出已批准的单次位移上限\n");
        return -1;
    }
    if (prompt("确认该轴安全行程、方向、速度限制和实体停机手段")) return -1;
    if (call("hal_rt_axis_enable(1)", hal_rt_axis_enable(r->ctx, id, 1))) return -1;
    int ready = 0;
    for (unsigned i = 0; i < s->settle_cycles; ++i) {
        if (begin_tick(r)) goto failed;
        rc = hal_rt_axis_read_status(r->ctx, id, &a);
        commit = end_tick(r);
        if (call("axis status", rc) || commit) goto failed;
        if (a.error_code) goto failed;
        if (a.enabled) { ready = 1; break; }
    }
    if (!ready) { fprintf(stderr, "轴使能超时\n"); goto failed; }
    if (fabs(f->target - a.actual_pos) > f->max_step) {
        fprintf(stderr, "使能后目标超出已批准的单次位移上限\n");
        goto failed;
    }
    if (prompt("轴已使能且仍静止；输入 YES 下发现场批准的目标位置")) goto failed;
    if (begin_tick(r)) goto failed;
    rc = hal_rt_axis_write_pos(r->ctx, id, f->target);
    if (rc) (void)hal_rt_axis_estop(r->ctx, id);
    commit = end_tick(r);
    if (call("hal_rt_axis_write_pos", rc) || commit) goto failed;
    for (unsigned i = 0; i < s->settle_cycles; ++i) {
        if (begin_tick(r)) goto failed;
        rc = hal_rt_axis_read_status(r->ctx, id, &a);
        commit = end_tick(r);
        if (call("axis status", rc) || commit) goto failed;
        printf("AXIS %u pos=%.9f target=%.9f sw=0x%04x err=0x%04x\n",
               id, a.actual_pos, f->target, a.raw_status, a.error_code);
        if (a.error_code) goto failed;
        if (fabs(a.actual_pos - f->target) <= f->tolerance) {
            (void)hal_rt_axis_estop(r->ctx, id);
            if (tick(r)) return -1;
            return prompt("确认电机实际转向和转角/位移与记录一致，且已经停稳");
        }
    }
    fprintf(stderr, "轴未在规定周期内到达目标\n");
failed:
    (void)hal_rt_axis_estop(r->ctx, id);
    (void)tick(r);
    return -1;
}

static int calibrate_case(Runner* r, int feed_index) {
    const HalHardwareTestSettings* s = r->settings;
    const HalFeedHardwareCase* f = &s->feed[feed_index];
    if (!f->scaling_confirmed || !isfinite(f->calibrated_position)) return -1;
    if (prompt("确认轴静止且 calibrated_position 已由独立机械基准测得")) return -1;
    HalAxisId id = (HalAxisId)r->cfg->axes[feed_index].logical_axis;
    if (begin_tick(r)) return -1;
    HalCAxisStatus before, after;
    int rc = hal_rt_axis_read_status(r->ctx, id, &before);
    if (rc == HAL_OK) rc = hal_rt_axis_set_pos(r->ctx, id, f->calibrated_position);
    if (rc == HAL_OK) rc = hal_rt_axis_read_status(r->ctx, id, &after);
    int commit = end_tick(r);
    if (call("set_pos/read_status", rc) || commit) return -1;
    printf("CALIBRATE before=%.9f after=%.9f command=%.9f\n",
           before.actual_pos, after.actual_pos, after.command_pos);
    if (fabs(after.actual_pos - f->calibrated_position) > 1e-9) return -1;
    if (begin_tick(r)) return -1;
    rc = hal_rt_axis_read_status(r->ctx, id, &after);
    commit = end_tick(r);
    return call("read calibrated position", rc) || commit ||
        fabs(after.actual_pos - f->calibrated_position) > 1e-6 ? -1 : 0;
}

static int estop_case(Runner* r, int feed_index) {
    const HalHardwareTestSettings* s = r->settings;
    if (!s->feed[feed_index].scaling_confirmed || !s->settle_cycles) return -1;
    HalAxisId id = (HalAxisId)r->cfg->axes[feed_index].logical_axis;
    if (prompt("确认测试轴静止、驱动可使能、实体停机手段可用")) return -1;
    if (call("axis enable", hal_rt_axis_enable(r->ctx, id, 1))) return -1;
    HalCAxisStatus status;
    int ready = 0;
    for (unsigned i = 0; i < s->settle_cycles; ++i) {
        if (begin_tick(r)) goto failed;
        int rc = hal_rt_axis_read_status(r->ctx, id, &status);
        int committed = end_tick(r);
        if (call("axis status", rc) || committed) goto failed;
        if (status.error_code) goto failed;
        if (status.enabled) { ready = 1; break; }
    }
    if (!ready) goto failed;
    if (prompt("已使能且静止；输入 YES 发出配置的 DS402 急停动作")) goto failed;
    if (call("hal_rt_axis_estop", hal_rt_axis_estop(r->ctx, id))) goto failed;
    if (tick(r)) goto failed;
    for (unsigned i = 0; i < s->settle_cycles; ++i) {
        if (begin_tick(r)) goto failed;
        int rc = hal_rt_axis_read_status(r->ctx, id, &status);
        int committed = end_tick(r);
        if (call("axis status", rc) || committed) goto failed;
        printf("ESTOP axis=%u enabled=%d sw=0x%04x err=0x%04x\n",
               id, status.enabled, status.raw_status, status.error_code);
        if (!status.enabled) return 0;
    }
    fprintf(stderr, "急停动作未在规定周期内反映到状态字\n");
failed:
    (void)hal_rt_axis_estop(r->ctx, id);
    (void)tick(r);
    return -1;
}

static int spindle_snapshot(Runner* r, HalCSpindleStatus* state) {
    if (begin_tick(r)) return -1;
    int rc = hal_rt_spindle_read_status(r->ctx, 0, state);
    int committed = end_tick(r);
    return call("spindle status", rc) || committed ? -1 : 0;
}

static int spindle_write_speed_tick(Runner* r, double rpm, int dir) {
    if (begin_tick(r)) return -1;
    int rc = hal_rt_spindle_write_speed(r->ctx, 0, rpm, dir);
    if (rc) (void)hal_rt_spindle_estop(r->ctx, 0);
    int committed = end_tick(r);
    return call("hal_rt_spindle_write_speed", rc) || committed ? -1 : 0;
}

static int spindle_speed_case(Runner* r, int dir) {
    const HalSpindleHardwareCase* s = &r->settings->spindle;
    if (!s->scaling_confirmed || !s->speed_pdo_units_confirmed ||
        !isfinite(s->rpm) || s->rpm <= 0 ||
        s->rpm > r->cfg->spindles[0].max_speed ||
        !isfinite(s->speed_tolerance) || s->speed_tolerance < 0 ||
        !r->settings->settle_cycles) return -1;
    if (prompt(dir > 0 ?
               "确认主轴正转方向、低速值、外部测速与实体停机手段" :
               "确认主轴反转方向、低速值、外部测速与实体停机手段")) return -1;
    if (call("spindle mode CSV",
             hal_rt_spindle_request_mode(r->ctx, 0, HAL_SPINDLE_CSV)) ||
        call("spindle enable", hal_rt_spindle_enable(r->ctx, 0, 1))) return -1;
    HalCSpindleStatus state;
    int ready = 0;
    for (unsigned i = 0; i < r->settings->settle_cycles; ++i) {
        if (spindle_snapshot(r, &state)) goto failed;
        if (state.mode == HAL_SPINDLE_CSV && state.enabled) { ready = 1; break; }
    }
    if (!ready) goto failed;
    if (spindle_write_speed_tick(r, 0, 0)) goto failed;
    ready = 0;
    for (unsigned i = 0; i < r->settings->settle_cycles; ++i) {
        if (spindle_snapshot(r, &state)) goto failed;
        if (fabs(state.actual_speed) <= s->speed_tolerance) { ready = 1; break; }
    }
    if (!ready) { fprintf(stderr, "主轴初始零速未确认\n"); goto failed; }
    if (prompt("主轴零速已确认；输入 YES 开始低速旋转")) goto failed;
    if (spindle_write_speed_tick(r, s->rpm, dir)) goto failed;
    ready = 0;
    for (unsigned i = 0; i < r->settings->settle_cycles; ++i) {
        if (spindle_snapshot(r, &state)) goto failed;
        printf("SPINDLE rpm=%.6f expected=%.6f at_speed=%d mode=%d sw=0x%04x\n",
               state.actual_speed, dir * s->rpm, state.at_speed,
               state.mode, state.raw_status);
        if (fabs(state.actual_speed - dir * s->rpm) <= s->speed_tolerance) {
            ready = 1; break;
        }
    }
    if (!ready) { fprintf(stderr, "主轴转速未到位\n"); goto failed; }
    if (spindle_write_speed_tick(r, 0, 0)) goto failed;
    ready = 0;
    for (unsigned i = 0; i < r->settings->settle_cycles; ++i) {
        if (spindle_snapshot(r, &state)) goto failed;
        printf("SPINDLE stopping rpm=%.6f\n", state.actual_speed);
        if (fabs(state.actual_speed) <= s->speed_tolerance) { ready = 1; break; }
    }
    if (!ready) { fprintf(stderr, "主轴未在规定周期内停转\n"); goto failed; }
    (void)hal_rt_spindle_estop(r->ctx, 0);
    if (tick(r)) return -1;
    return prompt("确认外部测速、实际转向与记录一致，且主轴已停稳");
failed:
    (void)hal_rt_spindle_estop(r->ctx, 0);
    (void)tick(r);
    return -1;
}

static int spindle_angle_case(Runner* r) {
    const HalSpindleHardwareCase* s = &r->settings->spindle;
    if (!s->scaling_confirmed || !isfinite(s->angle_target) ||
        !isfinite(s->max_angle_step) || s->max_angle_step <= 0 ||
        !isfinite(s->angle_tolerance) || s->angle_tolerance < 0 ||
        !r->settings->settle_cycles) {
        fprintf(stderr, "spindle-angle 配置未就绪：scaling_confirmed=%d "
                "angle_target=%.6f max_angle_step=%.6f angle_tolerance=%.6f "
                "settle_cycles=%u\n",
                s->scaling_confirmed, s->angle_target, s->max_angle_step,
                s->angle_tolerance, r->settings->settle_cycles);
        return -1;
    }
    HalCSpindleStatus state;
    if (spindle_snapshot(r, &state)) return -1;
    printf("SPINDLE raw startup position=%.6f deg; test target=%.6f deg; max_step=%.6f deg\n",
           state.position_deg, s->angle_target, s->max_angle_step);
    if (fabs(s->angle_target) > s->max_angle_step) {
        fprintf(stderr, "角度目标相对测试零点超出 max_angle_step\n");
        return -1;
    }
    if (prompt("确认主轴静止；将当前位置临时标为本次测试的 0 度，随后测试 CSP 小角度目标")) return -1;
    if (call("hal_rt_axis_set_pos(spindle, 0 deg)",
             hal_rt_axis_set_pos(r->ctx, 0, 0.0))) return -1;
    if (spindle_snapshot(r, &state)) return -1;
    printf("SPINDLE test zero position=%.6f deg\n", state.position_deg);
    if (fabs(state.position_deg) > s->angle_tolerance) {
        fprintf(stderr, "重新设坐标后仍未处于测试零点\n");
        return -1;
    }
    if (call("spindle mode CSP",
             hal_rt_spindle_request_mode(r->ctx, 0, HAL_SPINDLE_CSP)) ||
        call("spindle enable", hal_rt_spindle_enable(r->ctx, 0, 1))) return -1;
    int ready = 0;
    for (unsigned i = 0; i < r->settings->settle_cycles; ++i) {
        if (spindle_snapshot(r, &state)) goto failed;
        if (state.mode == HAL_SPINDLE_CSP && state.enabled) { ready = 1; break; }
    }
    if (!ready) {
        fprintf(stderr, "主轴未在规定周期内进入 CSP 使能状态\n");
        goto failed;
    }
    if (fabs(s->angle_target - state.position_deg) > s->max_angle_step) {
        fprintf(stderr, "使能后角度目标相对当前位置超出 max_angle_step: "
                "current=%.6f target=%.6f limit=%.6f\n",
                state.position_deg, s->angle_target, s->max_angle_step);
        goto failed;
    }
    if (prompt("主轴已在 CSP 使能且静止；输入 YES 下发目标角度")) goto failed;
    if (begin_tick(r)) goto failed;
    int rc = hal_rt_spindle_write_pos(r->ctx, 0, s->angle_target);
    if (rc) (void)hal_rt_spindle_estop(r->ctx, 0);
    int committed = end_tick(r);
    if (call("hal_rt_spindle_write_pos", rc) || committed) goto failed;
    for (unsigned i = 0; i < r->settings->settle_cycles; ++i) {
        if (spindle_snapshot(r, &state)) goto failed;
        printf("SPINDLE angle=%.6f target=%.6f mode=%d sw=0x%04x\n",
               state.position_deg, s->angle_target, state.mode, state.raw_status);
        if (fabs(state.position_deg - s->angle_target) <= s->angle_tolerance) {
            (void)hal_rt_spindle_estop(r->ctx, 0);
            if (tick(r)) return -1;
            return prompt("确认主轴实际转角与记录一致，且已经停稳");
        }
    }
failed:
    (void)hal_rt_spindle_estop(r->ctx, 0);
    (void)tick(r);
    return -1;
}

static int spindle_estop_case(Runner* r) {
    if (!r->settings->spindle.scaling_confirmed ||
        !r->settings->settle_cycles) return -1;
    if (prompt("确认主轴静止、允许使能且实体停机手段可用")) return -1;
    if (call("spindle enable", hal_rt_spindle_enable(r->ctx, 0, 1))) return -1;
    HalCSpindleStatus state;
    int ready = 0;
    for (unsigned i = 0; i < r->settings->settle_cycles; ++i) {
        if (spindle_snapshot(r, &state)) goto failed;
        if (state.enabled) { ready = 1; break; }
    }
    if (!ready) goto failed;
    if (prompt("主轴已使能且静止；输入 YES 执行软件急停")) goto failed;
    if (call("hal_rt_spindle_estop", hal_rt_spindle_estop(r->ctx, 0))) goto failed;
    if (tick(r)) goto failed;
    for (unsigned i = 0; i < r->settings->settle_cycles; ++i) {
        if (spindle_snapshot(r, &state)) goto failed;
        printf("SPINDLE ESTOP enabled=%d sw=0x%04x rpm=%.6f\n",
               state.enabled, state.raw_status, state.actual_speed);
        if (!state.enabled) return 0;
    }
failed:
    (void)hal_rt_spindle_estop(r->ctx, 0);
    (void)tick(r);
    return -1;
}

/* 两次启动保持机械位置不变，仅改变配置偏置，检查坐标差的符号。 */
static int offset_sample(const HalCConfig* config, int32_t logical, double* pos) {
    char err[256] = {0};
    HalContext* ctx = NULL;
    int rc = hal_context_create(config, &ctx, err, sizeof(err));
    if (rc) { fprintf(stderr, "offset create: %s\n", err); return -1; }
    rc = hal_context_start(ctx, err, sizeof(err));
    if (rc) {
        fprintf(stderr, "offset start: %s\n", err);
        hal_context_destroy(ctx);
        return -1;
    }
    int result = call("offset wait", hal_rt_wait_cycle(ctx));
    if (!result) result = call("offset begin", hal_rt_begin_cycle(ctx));
    if (!result) result = call("offset read", hal_rt_axis_read_pos(ctx, (HalAxisId)logical, pos));
    if (!result) result = call("offset commit", hal_rt_commit_cycle(ctx));
    (void)hal_context_request_stop(ctx);
    if (call("offset stop", hal_context_stop(ctx))) result = -1;
    hal_context_destroy(ctx);
    return result;
}

static int offset_case(const HalCConfig* config, const HalHardwareTestSettings* settings,
                       int feed_index) {
    const HalFeedHardwareCase* f = &settings->feed[feed_index];
    int logical = config->axes[feed_index].logical_axis;
    if (!f->scaling_confirmed || !isfinite(f->offset_test_delta) ||
        f->offset_test_delta == 0 || !isfinite(f->tolerance) ||
        f->tolerance < 0) return -1;
    if (prompt("确认这根电机静止；两次启动之间禁止改变机械位置")) return -1;
    double base = 0, changed = 0;
    if (offset_sample(config, logical, &base)) return -1;
    HalCConfig changed_config = *config;
    changed_config.axes[feed_index].enc_off += f->offset_test_delta;
    if (!isfinite(changed_config.axes[feed_index].enc_off)) return -1;
    if (offset_sample(&changed_config, logical, &changed)) return -1;
    double observed = changed - base;
    printf("OFFSET base=%.9f changed=%.9f actual_delta=%.9f expected_delta=%.9f\n",
           base, changed, observed, -f->offset_test_delta);
    return fabs(observed + f->offset_test_delta) <= f->tolerance ? 0 : -1;
}

typedef struct { Runner* runner; int rc; } ThreadRun;
static void* stop_worker(void* arg) {
    ThreadRun* state = arg;
    for (;;) {
        int rc = hal_rt_wait_cycle(state->runner->ctx);
        if (rc == HAL_ERROR_STOPPED) { state->rc = 0; return NULL; }
        if (rc) break;
        rc = hal_rt_begin_cycle(state->runner->ctx);
        if (rc == HAL_ERROR_STOPPED) { state->rc = 0; return NULL; }
        if (rc) break;
        rc = hal_rt_commit_cycle(state->runner->ctx);
        if (rc == HAL_ERROR_STOPPED) { state->rc = 0; return NULL; }
        if (rc) break;
    }
    state->rc = -1;
    return NULL;
}

static int concurrent_stop(Runner* r) {
    ThreadRun state = {r, -1};
    pthread_t thread;
    if (pthread_create(&thread, NULL, stop_worker, &state)) return -1;
    struct timespec delay = {0, 100000000};
    nanosleep(&delay, NULL);
    int requested = hal_context_request_stop(r->ctx);
    int joined = pthread_join(thread, NULL);
    printf("STOP request=%d worker=%d join=%d\n", requested, state.rc, joined);
    return requested || joined || state.rc ? -1 : 0;
}

static void usage(const char* program) {
    fprintf(stderr,
            "用法: %s selftest|static|observe|protocol|inputs|io|stop\n"
            "      %s axis|estop|calibrate|offset 1|2|3\n"
            "      %s spindle-speed|spindle-reverse|spindle-angle|spindle-estop\n"
            "从站 2 是主轴（逻辑 0）；从站 3～5 是进给轴（逻辑 1～3）。\n"
            "先编辑 examples/hal_hardware_test_config.c。写输出或运动的用例需现场输入 YES。\n",
            program, program, program);
}

int main(int argc, char** argv) {
    if (argc < 2 || argc > 3) { usage(argv[0]); return 2; }
    if (argc == 2 && strcmp(argv[1], "selftest") == 0)
        return selftest() ? 1 : 0;
    int needs_feed = !strcmp(argv[1], "axis") || !strcmp(argv[1], "estop") ||
        !strcmp(argv[1], "calibrate") || !strcmp(argv[1], "offset");
    if ((needs_feed && argc != 3) || (!needs_feed && argc != 2)) {
        usage(argv[0]); return 2;
    }
    int feed_index = -1;
    if (needs_feed) {
        char* end = NULL;
        long value = strtol(argv[2], &end, 10);
        if (!end || *end || value < 1 || value > 3) {
            usage(argv[0]); return 2;
        }
        feed_index = (int)value - 1;
    }
    HalCConfig config;
    HalHardwareTestSettings settings;
    if (hal_hardware_test_config(&config, &settings)) {
        fprintf(stderr, "现场配置尚未填写审核：请编辑 hal_hardware_test_config.c\n");
        return 2;
    }
    if (config.axis_count != 3 || config.spindle_count != 1 ||
        config.io_count != 1 || config.panel_count != 1) {
        fprintf(stderr, "本程序要求测试台拓扑：1 台主轴、3 台进给轴、1 个 IO、1 个面板\n");
        return 2;
    }
    if (config.panels[0].slave_pos != 0 || config.ios[0].slave_pos != 1 ||
        config.spindles[0].axis.slave_pos != 2 ||
        config.spindles[0].axis.logical_axis != 0 ||
        config.spindles[0].axis.work_mode != HAL_WORK_VELOCITY) {
        fprintf(stderr, "面板/IO/主轴从站或主轴逻辑号与台架约定不符\n");
        return 2;
    }
    for (int i = 0; i < 3; ++i) {
        if (config.axes[i].slave_pos != i + 3 ||
            config.axes[i].logical_axis != i + 1) {
            fprintf(stderr, "进给轴 %d 的从站或逻辑轴号与台架约定不符\n", i + 1);
            return 2;
        }
    }
    if (static_cases(&config)) return 1;
    if (strcmp(argv[1], "static") == 0) return 0;
    if (strcmp(argv[1], "offset") == 0)
        return offset_case(&config, &settings, feed_index) ? 1 : 0;
    const char* mode = argv[1];
    if (strcmp(mode, "observe") && strcmp(mode, "protocol") &&
        strcmp(mode, "inputs") && strcmp(mode, "io") &&
        strcmp(mode, "axis") && strcmp(mode, "estop") && strcmp(mode, "calibrate") &&
        strcmp(mode, "spindle-speed") && strcmp(mode, "spindle-reverse") &&
        strcmp(mode, "spindle-angle") && strcmp(mode, "spindle-estop") &&
        strcmp(mode, "stop")) {
        usage(argv[0]); return 2;
    }
    char err[256] = {0};
    HalContext* ctx = NULL;
    int rc = hal_context_create(&config, &ctx, err, sizeof(err));
    if (rc) { fprintf(stderr, "create rc=%d %s\n", rc, err); return 1; }
    rc = hal_context_start(ctx, err, sizeof(err));
    if (rc) {
        fprintf(stderr, "start rc=%d %s\n", rc, err);
        hal_context_destroy(ctx);
        return 1;
    }
    Runner runner = {ctx, &config, &settings, 0};
    int result = identities(&runner);
    if (!result && strcmp(mode, "observe") == 0) result = observe(&runner);
    if (!result && strcmp(mode, "protocol") == 0) result = protocol_case(&runner);
    if (!result && strcmp(mode, "inputs") == 0) result = inputs_case(&runner);
    if (!result && strcmp(mode, "io") == 0) result = io_case(&runner);
    if (!result && strcmp(mode, "axis") == 0) result = axis_case(&runner, feed_index);
    if (!result && strcmp(mode, "estop") == 0) result = estop_case(&runner, feed_index);
    if (!result && strcmp(mode, "calibrate") == 0) result = calibrate_case(&runner, feed_index);
    if (!result && strcmp(mode, "spindle-speed") == 0)
        result = spindle_speed_case(&runner, 1);
    if (!result && strcmp(mode, "spindle-reverse") == 0)
        result = spindle_speed_case(&runner, -1);
    if (!result && strcmp(mode, "spindle-angle") == 0)
        result = spindle_angle_case(&runner);
    if (!result && strcmp(mode, "spindle-estop") == 0)
        result = spindle_estop_case(&runner);
    if (!result && strcmp(mode, "stop") == 0) result = concurrent_stop(&runner);
    if (result && strcmp(mode, "stop") != 0) {
        stop_intent(&runner);
        (void)tick(&runner);
    }
    if (strcmp(mode, "stop") != 0) (void)hal_context_request_stop(ctx);
    if (call("hal_context_stop", hal_context_stop(ctx))) result = -1;
    hal_context_destroy(ctx);
    printf("RESULT %s %s\n", mode, result ? "FAIL" : "PASS");
    return result ? 1 : 0;
}
