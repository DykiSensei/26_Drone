#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 机械爪开/合预设角（度）。爪体机构行程未实测——先取保守默认，台架上用
 * 前端调试滑条找到实际开/合角后回填这两个宏。
 * 闭合角务必设为"刚好夹紧"而非机构极限：夹住目标后舵机堵转（MG995 堵转
 * 电流 1A+），顶着极限角持续堵转会过热。 */
#define SERVO_GRIP_OPEN_DEG    90.0f
#define SERVO_GRIP_CLOSE_DEG   150.0f

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

/** @brief 当前（限速后）输出角度，遥测用 */
float servo_grip_get_angle(void);

/** @brief 是否仍在向目标角运动（抓取状态机判断闭爪完成用） */
bool servo_grip_is_moving(void);

#ifdef __cplusplus
}
#endif
