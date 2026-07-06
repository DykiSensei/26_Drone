#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 机械爪开/合角（2026-07-06 舵机标定模式实测，新爪体）：0° 完全张开，
 * 117° 齿条行程极限（完全闭合）。闭合预设取 110°——留 7° 余量，抓到物体
 * 时舵机不顶极限持续堵转（MG995 堵转 1A+ 会过热）。
 * SERVO_GRIP_MAX_DEG 是驱动层硬限位，clamp 在驱动内部执行，任何上层调用
 * 都不可能越过；超过 117° 舵机空转打滑。
 * 机械爪兼作起落架：地面停放/上电默认必须完全张开（OPEN=0°，init 即输出）。 */
#define SERVO_GRIP_OPEN_DEG    0.0f
#define SERVO_GRIP_CLOSE_DEG   110.0f  /* 抓取闭合角：行程极限 −7° 防堵转 */
#define SERVO_GRIP_MAX_DEG     117.0f  /* 机械硬限位：齿条行程极限，超过即空转 */

/**
 * @brief 初始化机械爪舵机（LEDC TIMER_1 / 通道 4，50Hz，GPIO10）
 *        上电即输出张开角脉冲——首个脉冲舵机以自身全速转到位（上电前
 *        位置未知，无法限速），之后的目标变化才走限速逼近
 * @return 0 成功, -1 失败（调用方按非致命处理：无抓取功能但不影响飞行）
 */
int servo_grip_init(void);

/** @brief 设置目标角度（0–180 钳位），由 servo_grip_update 限速逼近 */
void servo_grip_set_angle(float deg);

/** @brief 张开到 SERVO_GRIP_OPEN_DEG（飞控抓取流程用的语义接口） */
void servo_grip_open(void);

/** @brief 闭合到 SERVO_GRIP_CLOSE_DEG（飞控抓取流程用的语义接口） */
void servo_grip_close(void);

/**
 * @brief 限速逼近目标角，主循环每拍（100Hz）调用，dt 为实测秒
 *        闭爪太快有电流尖峰 + 飞行中的反扭矩，故不直接跳变
 */
void servo_grip_update(float dt);

/**
 * @brief 舵机标定模式：开启后角度限位临时放开到 0–180°（探索新爪体的
 *        开/合角），且转速降为 45°/s（慢速逼近，接近齿条行程极限时来得及
 *        收手——超行程舵机会空转打滑）。关闭后恢复 SERVO_GRIP_MAX_DEG
 *        硬限位，超限目标自动收回。**只允许 DISARMED 下开启**（main.c 门控:
 *        每拍 set_calib_mode(mode==DISARMED && sp->grip_calib)，解锁即自动
 *        退出）。标定出的角度回填本文件三个宏后重新编译。
 */
void servo_grip_set_calib_mode(bool on);

/** @brief 当前（限速后）输出角度，遥测用 */
float servo_grip_get_angle(void);

/** @brief 是否仍在向目标角运动（抓取状态机判断闭爪完成用） */
bool servo_grip_is_moving(void);

#ifdef __cplusplus
}
#endif
