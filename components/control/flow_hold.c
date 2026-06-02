#include "flow_hold.h"
#include "esp_timer.h"
#include <math.h>

#define FLOW_KP              0.05f
#define FLOW_KI              0.01f
#define FLOW_KD              0.0f
#define FLOW_OUTPUT_LIMIT    5.0f
#define FLOW_INTEGRAL_LIMIT  2.0f
#define FLOW_QUALITY_THRESHOLD 100
#define FLOW_MAX_HEIGHT_M    3.0f
#define FLOW_QUALITY_SMOOTH  0.3f

#define DT_MIN  0.005f
#define DT_MAX  0.200f
#define DECAY   0.95f

void flow_hold_init(flow_hold_t *fh)
{
    pid_init(&fh->pid_vx, FLOW_KP, FLOW_KI, FLOW_KD,
             FLOW_OUTPUT_LIMIT, FLOW_INTEGRAL_LIMIT);
    pid_init(&fh->pid_vy, FLOW_KP, FLOW_KI, FLOW_KD,
             FLOW_OUTPUT_LIMIT, FLOW_INTEGRAL_LIMIT);
    fh->out_pitch_deg = 0.0f;
    fh->out_roll_deg  = 0.0f;
    fh->quality_gain  = 0.0f;
    fh->last_update_us = 0;
    fh->active = false;
}

void flow_hold_update(flow_hold_t *fh, int16_t flow_x, int16_t flow_y,
                      uint8_t qual, float height_m)
{
    bool valid = (qual > FLOW_QUALITY_THRESHOLD)
              && (height_m > 0.04f)
              && (height_m < FLOW_MAX_HEIGHT_M);

    /* EMA smooth of quality signal */
    float q_target = valid ? 1.0f : 0.0f;
    fh->quality_gain += FLOW_QUALITY_SMOOTH * (q_target - fh->quality_gain);

    int64_t now = esp_timer_get_time();

    if (valid) {
        float dt;
        if (fh->last_update_us == 0) {
            dt = 0.01f;  /* first frame, use nominal dt */
        } else {
            dt = (float)(now - fh->last_update_us) * 1e-6f;
        }
        if (dt < DT_MIN) dt = DT_MIN;
        if (dt > DT_MAX) dt = DT_MAX;

        /* Flow is velocity; setpoint = 0 means "resist any motion".
         * Sign: positive flow_x = forward → PID outputs negative → nose-up (backward tilt)
         *       positive flow_y = right   → PID outputs negative → left-roll tilt
         * Negate to get the correction direction the mixer expects. */
        fh->out_pitch_deg = pid_update(&fh->pid_vx, 0.0f, (float)(-flow_x), dt);
        fh->out_roll_deg  = pid_update(&fh->pid_vy, 0.0f, (float)(-flow_y), dt);
    } else {
        /* Fade corrections smoothly when quality drops */
        fh->out_pitch_deg *= DECAY;
        fh->out_roll_deg  *= DECAY;
    }

    fh->last_update_us = now;
    fh->active = (fh->quality_gain > 0.01f);
}

void flow_hold_reset(flow_hold_t *fh)
{
    pid_reset(&fh->pid_vx);
    pid_reset(&fh->pid_vy);
    fh->out_pitch_deg = 0.0f;
    fh->out_roll_deg  = 0.0f;
    fh->quality_gain  = 0.0f;
    fh->last_update_us = 0;
    fh->active = false;
}

bool flow_hold_is_active(const flow_hold_t *fh)
{
    return fh->active;
}
