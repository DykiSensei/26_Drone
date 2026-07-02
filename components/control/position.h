#pragma once

#include "pid.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    pid_t   pid_x, pid_y;      /* 位置 (m) → 速度 (m/s) PID */
    float   target_x, target_y; /* 目标位置 (m，flow_hold 航位推算坐标系) */
    float   start_x, start_y;  /* 启动时的位置 (m) */
    float   out_vx, out_vy;    /* 输出的速度指令 (m/s) */
    bool    active;             /* 是否正在执行位置控制 */
    bool    hold;               /* true=持续位置保持(不因到达退出); false=move_to 一次性 */
    int     reached_count;      /* 连续到达目标周期计数（move_to 防抖） */
} position_ctrl_t;

void position_init(position_ctrl_t *pos);
/* move_to 一次性移动：目标 = 当前位置 + 偏移，偏移单位为米 */
void position_set_target(position_ctrl_t *pos, float offset_x, float offset_y,
                         float current_x_m, float current_y_m);
/* 锁定当前点持续位置保持(对抗漂移)；与 move_to 不同，不因到达而退出 */
void position_hold_start(position_ctrl_t *pos, float current_x_m, float current_y_m);
void position_update(position_ctrl_t *pos, float x_m, float y_m, float dt);
void position_reset(position_ctrl_t *pos);
/* move_to 是否到达（连续 POS_REACHED_CNT 周期在容差内，由 position_update 计数） */
bool position_reached(const position_ctrl_t *pos);

#ifdef __cplusplus
}
#endif
