#pragma once

#include "pid.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    pid_t   pid_x, pid_y;      /* 位置 → 速度 PID */
    float   target_x, target_y; /* 目标位置 (光流积分单位) */
    float   start_x, start_y;  /* 启动时的光流积分值 */
    float   out_vx, out_vy;    /* 输出的速度指令 */
    bool    active;             /* 是否正在执行位置控制 */
    bool    hold;               /* true=持续位置保持(不因到达退出); false=move_to 一次性 */
    int     reached_count;      /* 连续到达目标周期计数 */
} position_ctrl_t;

void position_init(position_ctrl_t *pos);
void position_set_target(position_ctrl_t *pos, float offset_x, float offset_y,
                         float current_flow_ix, float current_flow_iy);
/* 锁定当前点持续位置保持(对抗漂移)；与 move_to 不同，不因到达而退出 */
void position_hold_start(position_ctrl_t *pos, float current_ix, float current_iy);
void position_update(position_ctrl_t *pos, float flow_ix, float flow_iy, float dt);
void position_reset(position_ctrl_t *pos);
bool position_reached(const position_ctrl_t *pos, float current_ix, float current_iy);

#ifdef __cplusplus
}
#endif
