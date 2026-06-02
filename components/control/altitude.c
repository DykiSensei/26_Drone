#include "altitude.h"
#include <math.h>

/* Conservative altitude PID gains */
#define ALT_KP  0.5f    /* (m error → throttle): 0.2m error → 0.1 throttle */
#define ALT_KI  0.05f   /* slow integral for hover trim */
#define ALT_KD  0.0f    /* no derivative (TOF data is already slow) */
#define ALT_OUT_LIMIT   0.3f   /* max ±30% throttle adjustment */
#define ALT_INT_LIMIT   0.15f  /* integral windup limit */

void altitude_init(altitude_ctrl_t *alt)
{
    pid_init(&alt->pid, ALT_KP, ALT_KI, ALT_KD, ALT_OUT_LIMIT, ALT_INT_LIMIT);
    alt->target_m = 0.0f;
    alt->target_valid = false;
}

void altitude_capture_target(altitude_ctrl_t *alt, float current_m)
{
    alt->target_m = current_m;
    alt->target_valid = true;
    pid_reset(&alt->pid);
}

float altitude_update(altitude_ctrl_t *alt, float current_m, float dt)
{
    if (!alt->target_valid) return 0.0f;
    return pid_update(&alt->pid, alt->target_m, current_m, dt);
}

void altitude_reset(altitude_ctrl_t *alt)
{
    pid_reset(&alt->pid);
}
