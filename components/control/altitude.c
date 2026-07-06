#include "altitude.h"
#include "esp_timer.h"
#include <math.h>

/* Altitude P+I gains (output: throttle adjust) */
#define ALT_KP  0.25f   /* lowered: cut bandwidth to stop the mid-period swing */
#define ALT_KI  0.02f   /* slow integral for hover trim (small → less overshoot) */
#define ALT_KD  0.0f    /* PID derivative unused — damping done on vz below */
#define ALT_OUT_LIMIT   0.3f   /* max ±30% throttle adjustment */
#define ALT_INT_LIMIT   0.15f  /* integral windup limit */

/* Vertical-speed damping: throttle per m/s of climb. This is what kills the
 * up/down oscillation. Computed only on fresh TOF samples (~10Hz, stair-step
 * data) so it never sees the differentiation spikes of the cached frames. */
#define ALT_KD_VZ       0.50f  /* damping against the swing */
#define VZ_DT_MIN       0.02f  /* clamp measurement interval (s) */
#define VZ_DT_MAX       0.30f
#define VZ_FUSE_K       0.2f   /* TOF correction gain in the vz complementary filter */

/* Target slew rate (m/s). The (ramped) target climbs toward the final target
 * at this rate so the height error — and thus the throttle push — stays small.
 * This is what prevents the takeoff overshoot/crash from a stepped target. */
#define ALT_RAMP_RATE   0.30f

/* az_up 直流跟踪时间常数：远慢于真实机动（加减速瞬态 <1s），刚好吸收
 * 准静态的零偏/振动整流偏移。持续匀速升降时真实 az_up≈0，不受影响。 */
#define AZ_DC_TAU_S     2.0f

void altitude_init(altitude_ctrl_t *alt)
{
    pid_init(&alt->pid, ALT_KP, ALT_KI, ALT_KD, ALT_OUT_LIMIT, ALT_INT_LIMIT);
    alt->target_m = 0.0f;
    alt->target_final_m = 0.0f;
    alt->target_valid = false;
    alt->prev_m = 0.0f;
    alt->vz = 0.0f;
    alt->last_change_us = 0;
    alt->az_lp = 0.0f;
    alt->az_lp_init = false;
}

void altitude_track_az(altitude_ctrl_t *alt, float az_up, float dt)
{
    if (!alt->az_lp_init) {
        alt->az_lp = az_up;       /* 首样本锁存，跳过从 0 收敛的瞬态 */
        alt->az_lp_init = true;
        return;
    }
    alt->az_lp += (dt / AZ_DC_TAU_S) * (az_up - alt->az_lp);
}

void altitude_set_target(altitude_ctrl_t *alt, float final_m, float current_m)
{
    alt->target_final_m = final_m;
    alt->target_m = current_m;   /* ramp starts at current height */
    alt->target_valid = true;
    pid_reset(&alt->pid);
    alt->prev_m = current_m;
    alt->vz = 0.0f;
    alt->last_change_us = 0;
}

void altitude_capture_target(altitude_ctrl_t *alt, float current_m)
{
    /* Hold in place: final == current, ramp is a no-op. */
    altitude_set_target(alt, current_m, current_m);
}

float altitude_update(altitude_ctrl_t *alt, float current_m, float az_up, float dt)
{
    if (!alt->target_valid) return 0.0f;

    /* Ramp the working target toward the final target (slew-rate limit). */
    float step = ALT_RAMP_RATE * dt;
    if (alt->target_m < alt->target_final_m - step)      alt->target_m += step;
    else if (alt->target_m > alt->target_final_m + step) alt->target_m -= step;
    else                                                 alt->target_m = alt->target_final_m;

    /* Vertical speed via complementary filter:
     *   - predict every loop with IMU world-up accel (high-rate, low-latency,
     *     but drifts due to accel bias);
     *   - correct only on fresh TOF samples with the height difference
     *     (low-rate & noisy, but absolute — pulls the drift back).
     * The TOF noise is attenuated by VZ_FUSE_K instead of differentiated raw,
     * so vz stays smooth without the lag of a heavy low-pass.
     * 只积分高通分量：扣掉 az_lp 直流（残余零偏 + 振动整流），否则
     * 稳态偏移 ≈ 0.5×直流 直接卡死起飞位置锁的 |vz|<0.15 门。 */
    alt->vz += (az_up - alt->az_lp) * dt;
    if (current_m != alt->prev_m) {
        int64_t now = esp_timer_get_time();
        float dt_meas = (alt->last_change_us == 0)
                      ? 0.1f
                      : (float)(now - alt->last_change_us) * 1e-6f;
        if (dt_meas < VZ_DT_MIN) dt_meas = VZ_DT_MIN;
        if (dt_meas > VZ_DT_MAX) dt_meas = VZ_DT_MAX;
        float vz_meas = (current_m - alt->prev_m) / dt_meas;
        alt->vz += VZ_FUSE_K * (vz_meas - alt->vz);
        alt->prev_m = current_m;
        alt->last_change_us = now;
    }

    /* P + I (runs every loop; pid integrates with the main-loop dt). */
    float pi = pid_update(&alt->pid, alt->target_m, current_m, dt);

    /* Damping: climbing (vz>0) → reduce throttle, descending → add throttle. */
    float output = pi - ALT_KD_VZ * alt->vz;
    if (output > ALT_OUT_LIMIT)  output = ALT_OUT_LIMIT;
    if (output < -ALT_OUT_LIMIT) output = -ALT_OUT_LIMIT;
    return output;
}

void altitude_reset(altitude_ctrl_t *alt)
{
    pid_reset(&alt->pid);
    alt->prev_m = 0.0f;
    alt->vz = 0.0f;
    alt->last_change_us = 0;
    /* az_lp 不清零：输入调理滤波器必须连续运行（DISARMED 时本函数每拍
     * 被调，清了跟踪器就永远学不到零偏——flow_hold 同款教训 2026-07-04） */
}
