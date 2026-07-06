#include "grab_mission.h"
#include "servo_grip.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>

static const char *TAG = "grab";

/* ── 任务参数（调参入口，联调后按实测回填）──
 * 工作高度按 ~1.7m 巡航设计（用户 2026-07-05 确认）：下降行程 ~1.5m，
 * 0.3 m/s 斜坡本身就要 5s，超时都留了余量。 */
#define ALIGN_TOL_MIN_M    0.08f  /* 对准容差下限（低空/视觉地板处生效） */
#define ALIGN_TOL_K        0.15f  /* 容差随高度放宽: tol = max(0.08, 0.15*h)。
                                   * 1.7m 时 ≈0.26m —— 光流定位噪声随高度线性
                                   * 变大（1 count @1.7m ≈ 0.2 m/s），高空只做
                                   * 粗对准，精对准留给低空台阶 */
#define ALIGN_OK_N         3      /* 连续 N 条容差内测量才进入下降 */
#define ALIGN_CORR_GAP_S   1.0f   /* 两次视觉修正最小间隔（等位置环稳定，边降边修节奏） */
#define ALIGN_TIMEOUT_S    20.0f  /* 单级高度上的对准超时（每级台阶完成后重置） */
#define MISSION_TIMEOUT_S  120.0f /* 任务总超时兜底（1.7m 分段全流程 ~30-60s） */
#define VISION_FLOOR_M     0.35f  /* TOF 低于此不再对准（相机近距盲区），转末段开环 */
#define DESCEND_STEP_M     0.25f  /* ALIGN 阶段每级台阶下降量（1.7m→0.35m 约 6 级） */
#define STEP_SETTLE_TOL_M  0.08f  /* 台阶到位判据 */
#define DESCEND_TIMEOUT_S  20.0f  /* 末段/测试直降：1.7m→0.1m 斜坡 5.3s + PID 滞后余量 */
#define GRASP_SETTLE_S     0.3f   /* 爪到位后再夹稳一会 */
#define GRASP_TIMEOUT_S    3.0f   /* 爪 0.75s 应到位；超时按已完成处理 */
#define ASCEND_TOL_M       0.10f
#define VZ_STEADY_MS       0.15f  /* 垂直稳定判据 m/s */
#define ASCEND_TIMEOUT_S   15.0f  /* 回升 1.5m 斜坡 5s + 余量 */
#define START_MARGIN_M     0.10f  /* 启动时须高于触发高度至少这么多 */
#define GRAB_TOF_MIN_M     0.10f  /* 触发高度可调范围（TOF 落地读数≈0.20） */
#define GRAB_TOF_MAX_M     0.50f
#define DROP_TOF_MIN_M     0.15f  /* 投放高度可调范围（筐沿高度 + 落差） */
#define DROP_TOF_MAX_M     0.60f
#define GOTO_TOL_M         0.25f  /* 返航到标记点的到达容差（配大开口筐） */
#define GOTO_CORR_GAP_S    1.0f   /* 返航 move_to 重发最小间隔（自愈用户干预） */
#define GOTO_TIMEOUT_S     30.0f

static bool tof_ok(uint16_t mm) { return mm >= 40 && mm <= 4000; }

static float since_s(int64_t t_us)
{
    return (float)(esp_timer_get_time() - t_us) * 1e-6f;
}

static void enter(grab_mission_t *gm, grab_state_t s)
{
    gm->state = s;
    gm->state_since_us = esp_timer_get_time();
}

static void mission_abort_internal(grab_mission_t *gm, altitude_ctrl_t *alt,
                                   uint16_t tof_mm, const char *reason)
{
    ESP_LOGE(TAG, "任务中止 (%s), 状态=%d — 回起始高度 %.2fm, 爪保持当前角",
             reason, gm->state, gm->start_alt_m);
    if (tof_ok(tof_mm))
        altitude_set_target(alt, gm->start_alt_m, tof_mm * 0.001f);
    gm->result = -1;
    enter(gm, GRAB_IDLE);
}

void grab_mission_init(grab_mission_t *gm)
{
    gm->state = GRAB_IDLE;
    gm->mission = GRAB_MISSION_GRAB;
    gm->test_mode = false;
    gm->grab_tof_m = 0.20f;
    gm->drop_tof_m = 0.30f;
    gm->mark_x = gm->mark_y = 0.0f;
    gm->mark_valid = false;
    gm->start_alt_m = 0.0f;
    gm->state_since_us = 0;
    gm->mission_since_us = 0;
    gm->last_corr_us = 0;
    gm->grasp_done_us = 0;
    gm->align_ok_cnt = 0;
    gm->align_stepping = false;
    gm->result = 0;
}

bool grab_mission_active(const grab_mission_t *gm)
{
    return gm->state != GRAB_IDLE;
}

int grab_mission_start(grab_mission_t *gm, bool test_mode, bool p4_alive,
                       const setpoint_t *sp, altitude_ctrl_t *alt,
                       uint16_t tof_mm)
{
    if (gm->state != GRAB_IDLE) {
        ESP_LOGE(TAG, "启动拒绝: 任务进行中 (状态=%d)", gm->state);
        return -1;
    }
    if (sp->mode != MODE_ALT_HOLD && sp->mode != MODE_POS_HOLD) {
        ESP_LOGE(TAG, "启动拒绝: 需要定高/定点模式 (当前 %s)",
                 commander_mode_name(sp->mode));
        return -1;
    }
    if (!tof_ok(tof_mm)) {
        ESP_LOGE(TAG, "启动拒绝: TOF 无效 (%u mm)", tof_mm);
        return -1;
    }

    float tof_m = tof_mm * 0.001f;
    float grab_tof = sp->grab_tof_m;
    if (grab_tof < GRAB_TOF_MIN_M) grab_tof = GRAB_TOF_MIN_M;
    if (grab_tof > GRAB_TOF_MAX_M) grab_tof = GRAB_TOF_MAX_M;

    if (tof_m < grab_tof + START_MARGIN_M) {
        ESP_LOGE(TAG, "启动拒绝: 高度不足 (TOF %.2fm, 需 > %.2fm)",
                 tof_m, grab_tof + START_MARGIN_M);
        return -1;
    }
    if (servo_grip_get_angle() > SERVO_GRIP_OPEN_DEG + 15.0f) {
        ESP_LOGE(TAG, "启动拒绝: 机械爪未张开 (%.0f°) — 先张开再触发",
                 servo_grip_get_angle());
        return -1;
    }
    if (!test_mode && !p4_alive) {
        ESP_LOGE(TAG, "启动拒绝: P4 链路离线 (正式流程需 P4; 无 P4 用测试模式)");
        return -1;
    }

    gm->mission = GRAB_MISSION_GRAB;
    gm->test_mode = test_mode;
    gm->grab_tof_m = grab_tof;
    gm->start_alt_m = alt->target_valid ? alt->target_final_m : tof_m;
    gm->result = 0;
    gm->align_ok_cnt = 0;
    gm->align_stepping = false;
    gm->last_corr_us = 0;
    gm->grasp_done_us = 0;
    gm->mission_since_us = esp_timer_get_time();

    if (test_mode) {
        /* 假设当前位置精确：直接末段下降。斜坡终点压到触发高度以下 5cm，
         * 保证 TOF 读数一定穿越触发线（PID 稳态误差不会卡在线上方） */
        float final = grab_tof - 0.05f;
        if (final < 0.05f) final = 0.05f;
        altitude_set_target(alt, final, tof_m);
        enter(gm, GRAB_DESCEND);
        ESP_LOGW(TAG, "测试抓取启动: %.2fm 下降 -> 触发 %.2fm (完成后回 %.2fm)",
                 tof_m, grab_tof, gm->start_alt_m);
    } else {
        enter(gm, GRAB_ALIGN);
        ESP_LOGW(TAG, "抓取任务启动: P4 对准阶段 (当前 %.2fm, 触发 %.2fm)",
                 tof_m, grab_tof);
    }
    return 0;
}

int grab_mission_start_drop(grab_mission_t *gm, bool use_goto,
                            const setpoint_t *sp, altitude_ctrl_t *alt,
                            const flow_hold_t *fh, uint16_t tof_mm)
{
    if (gm->state != GRAB_IDLE) {
        ESP_LOGE(TAG, "投放拒绝: 任务进行中 (状态=%d)", gm->state);
        return -1;
    }
    if (sp->mode != MODE_ALT_HOLD && sp->mode != MODE_POS_HOLD) {
        ESP_LOGE(TAG, "投放拒绝: 需要定高/定点模式 (当前 %s)",
                 commander_mode_name(sp->mode));
        return -1;
    }
    if (!tof_ok(tof_mm)) {
        ESP_LOGE(TAG, "投放拒绝: TOF 无效 (%u mm)", tof_mm);
        return -1;
    }

    float tof_m = tof_mm * 0.001f;
    float drop_tof = sp->drop_tof_m;
    if (drop_tof < DROP_TOF_MIN_M) drop_tof = DROP_TOF_MIN_M;
    if (drop_tof > DROP_TOF_MAX_M) drop_tof = DROP_TOF_MAX_M;

    if (tof_m < drop_tof + START_MARGIN_M) {
        ESP_LOGE(TAG, "投放拒绝: 高度不足 (TOF %.2fm, 需 > %.2fm)",
                 tof_m, drop_tof + START_MARGIN_M);
        return -1;
    }
    if (use_goto) {
        if (!gm->mark_valid) {
            ESP_LOGE(TAG, "返航投放拒绝: 投放点未标记 (先悬停筐上方按【标记投放点】)");
            return -1;
        }
        if (fh->quality_gain < 0.05f) {
            ESP_LOGE(TAG, "返航投放拒绝: 光流不可信 (qg=%.2f) — 航位导航不可用",
                     fh->quality_gain);
            return -1;
        }
    }

    gm->mission = GRAB_MISSION_DROP;
    gm->test_mode = false;
    gm->drop_tof_m = drop_tof;
    gm->start_alt_m = alt->target_valid ? alt->target_final_m : tof_m;
    gm->result = 0;
    gm->last_corr_us = 0;
    gm->grasp_done_us = 0;
    gm->mission_since_us = esp_timer_get_time();

    if (use_goto) {
        enter(gm, GRAB_GOTO);
        ESP_LOGW(TAG, "返航投放: (%.2f, %.2f)m -> 标记点 (%.2f, %.2f)m, 投放高度 %.2fm",
                 fh->pos_x_m, fh->pos_y_m, gm->mark_x, gm->mark_y, drop_tof);
    } else {
        float final = drop_tof - 0.05f;
        if (final < 0.05f) final = 0.05f;
        altitude_set_target(alt, final, tof_m);
        enter(gm, GRAB_DESCEND);
        ESP_LOGW(TAG, "就地投放: %.2fm 下降 -> 张爪高度 %.2fm (完成后回 %.2fm)",
                 tof_m, drop_tof, gm->start_alt_m);
    }
    return 0;
}

int grab_mission_mark_drop(grab_mission_t *gm, const setpoint_t *sp,
                           const flow_hold_t *fh)
{
    if (sp->mode != MODE_ALT_HOLD && sp->mode != MODE_POS_HOLD) {
        ESP_LOGE(TAG, "标记拒绝: 需悬停在定高/定点模式 (当前 %s)",
                 commander_mode_name(sp->mode));
        return -1;
    }
    if (fh->quality_gain < 0.05f) {
        ESP_LOGE(TAG, "标记拒绝: 光流不可信 (qg=%.2f), 坐标不可靠", fh->quality_gain);
        return -1;
    }
    gm->mark_x = fh->pos_x_m;
    gm->mark_y = fh->pos_y_m;
    gm->mark_valid = true;
    ESP_LOGW(TAG, "投放点已标记: (%.2f, %.2f)m — 仅本次解锁周期有效",
             gm->mark_x, gm->mark_y);
    return 0;
}

void grab_mission_clear_mark(grab_mission_t *gm)
{
    if (gm->mark_valid) {
        gm->mark_valid = false;
        ESP_LOGW(TAG, "投放点标记失效 (航位坐标系已复位)");
    }
}

void grab_mission_abort(grab_mission_t *gm, altitude_ctrl_t *alt,
                        uint16_t tof_mm, const char *reason)
{
    if (gm->state == GRAB_IDLE) return;
    mission_abort_internal(gm, alt, tof_mm, reason);
}

void grab_mission_update(grab_mission_t *gm, const setpoint_t *sp,
                         altitude_ctrl_t *alt, position_ctrl_t *pos,
                         const flow_hold_t *fh, const grab_meas_t *meas,
                         uint16_t tof_mm, float dt)
{
    (void)dt;
    if (gm->state == GRAB_IDLE) return;

    /* ── 全局中止条件 ── */
    bool alt_mode = (sp->mode == MODE_ALT_HOLD || sp->mode == MODE_POS_HOLD);
    if (!alt_mode || sp->throttle < 0.05f) {
        mission_abort_internal(gm, alt, tof_mm, "模式退出/油门切断");
        return;
    }
    if (!tof_ok(tof_mm)) {
        /* TOF 驱动 500ms 无新样本才报 0——到这里已经确认失效，任务瞎飞没有意义 */
        mission_abort_internal(gm, alt, tof_mm, "TOF 失效");
        return;
    }
    if (since_s(gm->mission_since_us) > MISSION_TIMEOUT_S) {
        mission_abort_internal(gm, alt, tof_mm, "任务总超时");
        return;
    }
    float tof_m = tof_mm * 0.001f;

    switch (gm->state) {

    case GRAB_ALIGN:
        if (since_s(gm->state_since_us) > ALIGN_TIMEOUT_S) {
            mission_abort_internal(gm, alt, tof_mm, "对准超时");
            return;
        }
        if (gm->align_stepping) {
            /* 等本级台阶下降到位且垂直稳定，再回到测量 */
            if (fabsf(tof_m - alt->target_final_m) < STEP_SETTLE_TOL_M
                && fabsf(alt->vz) < VZ_STEADY_MS) {
                gm->align_stepping = false;
                gm->align_ok_cnt = 0;   /* 新高度重新确认对准 */
                gm->state_since_us = esp_timer_get_time();  /* 每级台阶重置
                                        * 对准计时——1.7m 分段下来总时长远超
                                        * 单级 20s，超时按"本级高度"计 */
            }
            break;
        }
        if (meas->valid) {
            float err = fmaxf(fabsf(meas->dx_m), fabsf(meas->dy_m));
            /* 容差随高度放宽：高空光流定位噪声大，只做粗对准 */
            float tol = fmaxf(ALIGN_TOL_MIN_M, ALIGN_TOL_K * tof_m);
            if (err < tol) {
                gm->align_ok_cnt++;
            } else {
                gm->align_ok_cnt = 0;
                /* 修正限速：上一次 move_to 稳定后才接受下一条（look-then-move） */
                if (gm->last_corr_us == 0
                    || since_s(gm->last_corr_us) > ALIGN_CORR_GAP_S) {
                    position_set_target(pos, meas->dx_m, meas->dy_m,
                                        fh->pos_x_m, fh->pos_y_m);
                    gm->last_corr_us = esp_timer_get_time();
                    ESP_LOGW(TAG, "视觉修正: dx=%.2f dy=%.2f (m)",
                             meas->dx_m, meas->dy_m);
                }
            }
        }
        if (gm->align_ok_cnt >= ALIGN_OK_N) {
            if (tof_m > VISION_FLOOR_M + STEP_SETTLE_TOL_M) {
                /* 边降边修：下一级台阶，落稳后回到测量重新对准 */
                float next = tof_m - DESCEND_STEP_M;
                if (next < VISION_FLOOR_M) next = VISION_FLOOR_M;
                altitude_set_target(alt, next, tof_m);
                gm->align_stepping = true;
                ESP_LOGW(TAG, "对准 OK — 下降台阶 %.2fm -> %.2fm", tof_m, next);
            } else {
                /* 已到相机盲区上沿：末段开环下降 */
                float final = gm->grab_tof_m - 0.05f;
                if (final < 0.05f) final = 0.05f;
                altitude_set_target(alt, final, tof_m);
                enter(gm, GRAB_DESCEND);
                ESP_LOGW(TAG, "进入末段开环下降 -> 触发 %.2fm", gm->grab_tof_m);
            }
        }
        break;

    case GRAB_GOTO: {
        /* 返航投放：航位推算 move_to 标记点。到达判据 = 距标记点 < 容差且
         * 位置环已转入 hold；若用户方向键干预打断了 move_to（main 会 reset
         * 后在当前点重锁），距离判据不满足 → 限速重发 move_to 自愈 */
        if (since_s(gm->state_since_us) > GOTO_TIMEOUT_S) {
            mission_abort_internal(gm, alt, tof_mm, "返航超时");
            return;
        }
        float dx = gm->mark_x - fh->pos_x_m;
        float dy = gm->mark_y - fh->pos_y_m;
        float dist = fmaxf(fabsf(dx), fabsf(dy));
        if (dist < GOTO_TOL_M && pos->active && pos->hold) {
            float final = gm->drop_tof_m - 0.05f;
            if (final < 0.05f) final = 0.05f;
            altitude_set_target(alt, final, tof_m);
            enter(gm, GRAB_DESCEND);
            ESP_LOGW(TAG, "到达投放点 (偏差 %.2fm) — 下降到张爪高度 %.2fm",
                     dist, gm->drop_tof_m);
        } else if ((!pos->active || pos->hold)
                   && (gm->last_corr_us == 0
                       || since_s(gm->last_corr_us) > GOTO_CORR_GAP_S)) {
            position_set_target(pos, dx, dy, fh->pos_x_m, fh->pos_y_m);
            gm->last_corr_us = esp_timer_get_time();
            ESP_LOGW(TAG, "返航 move_to: dx=%.2f dy=%.2f (剩余 %.2fm)", dx, dy, dist);
        }
        break; }

    case GRAB_DESCEND: {
        if (since_s(gm->state_since_us) > DESCEND_TIMEOUT_S) {
            mission_abort_internal(gm, alt, tof_mm, "下降超时");
            return;
        }
        float trig = (gm->mission == GRAB_MISSION_DROP) ? gm->drop_tof_m
                                                        : gm->grab_tof_m;
        if (tof_m <= trig) {
            altitude_capture_target(alt, tof_m);   /* 停止下降，定高执行爪动作 */
            gm->grasp_done_us = 0;
            if (gm->mission == GRAB_MISSION_DROP) {
                commander_set_grip(SERVO_GRIP_OPEN_DEG);
                enter(gm, GRAB_RELEASE);
                ESP_LOGW(TAG, "TOF %.2fm 触发 — 张爪投放", tof_m);
            } else {
                commander_set_grip(SERVO_GRIP_CLOSE_DEG);
                enter(gm, GRAB_GRASP);
                ESP_LOGW(TAG, "TOF %.2fm 触发 — 闭爪", tof_m);
            }
        }
        break; }

    case GRAB_GRASP:
        /* 每拍重写闭合角：与 WS 解析整结构体赋值的竞态即使丢写也 10ms 自愈 */
        commander_set_grip(SERVO_GRIP_CLOSE_DEG);
        if (gm->grasp_done_us == 0) {
            /* 到位判据用实际输出角而非 is_moving（本拍 setpoint 可能还没
             * 推到 servo——主循环中任务先于舵机段执行） */
            if (!servo_grip_is_moving()
                && fabsf(servo_grip_get_angle() - SERVO_GRIP_CLOSE_DEG) < 2.0f) {
                gm->grasp_done_us = esp_timer_get_time();
            }
        } else if (since_s(gm->grasp_done_us) > GRASP_SETTLE_S) {
            altitude_set_target(alt, gm->start_alt_m, tof_m);
            enter(gm, GRAB_ASCEND);
            ESP_LOGW(TAG, "闭爪完成 — 上升回 %.2fm", gm->start_alt_m);
            break;
        }
        if (since_s(gm->state_since_us) > GRASP_TIMEOUT_S) {
            altitude_set_target(alt, gm->start_alt_m, tof_m);
            enter(gm, GRAB_ASCEND);
            ESP_LOGW(TAG, "闭爪超时(按已完成处理) — 上升回 %.2fm", gm->start_alt_m);
        }
        break;

    case GRAB_RELEASE:
        /* 投放张爪（GRASP 的镜像）：每拍重写张开角防 setpoint 竞态丢写 */
        commander_set_grip(SERVO_GRIP_OPEN_DEG);
        if (gm->grasp_done_us == 0) {
            if (!servo_grip_is_moving()
                && fabsf(servo_grip_get_angle() - SERVO_GRIP_OPEN_DEG) < 2.0f) {
                gm->grasp_done_us = esp_timer_get_time();
            }
        } else if (since_s(gm->grasp_done_us) > GRASP_SETTLE_S) {
            altitude_set_target(alt, gm->start_alt_m, tof_m);
            enter(gm, GRAB_ASCEND);
            ESP_LOGW(TAG, "投放完成 — 上升回 %.2fm", gm->start_alt_m);
            break;
        }
        if (since_s(gm->state_since_us) > GRASP_TIMEOUT_S) {
            altitude_set_target(alt, gm->start_alt_m, tof_m);
            enter(gm, GRAB_ASCEND);
            ESP_LOGW(TAG, "张爪超时(按已完成处理) — 上升回 %.2fm", gm->start_alt_m);
        }
        break;

    case GRAB_ASCEND:
        if (gm->mission == GRAB_MISSION_GRAB) {
            commander_set_grip(SERVO_GRIP_CLOSE_DEG);   /* 抓取：上升全程保持夹紧 */
        }
        /* 投放：sp 已是张开值，不再干预（爪即起落架，回升后可直接降落） */
        if ((fabsf(tof_m - gm->start_alt_m) < ASCEND_TOL_M
             && fabsf(alt->vz) < VZ_STEADY_MS)
            || since_s(gm->state_since_us) > ASCEND_TIMEOUT_S) {
            gm->result = 1;
            enter(gm, GRAB_IDLE);
            if (gm->mission == GRAB_MISSION_DROP) {
                ESP_LOGW(TAG, "投放任务完成 (%.1fs) — 爪已张开, 起落架就绪可降落",
                         since_s(gm->mission_since_us));
            } else {
                ESP_LOGW(TAG, "抓取任务完成 (%.1fs) — 爪保持闭合, 可返航投放",
                         since_s(gm->mission_since_us));
            }
        }
        break;

    default:
        enter(gm, GRAB_IDLE);
        break;
    }
}
