#pragma once

/* grab_mission — 抓取任务状态机（全流程控制）
 *
 * 正式流程（需 P4）： IDLE → ALIGN（视觉对准+分段下降，边降边修）
 *                        → DESCEND（末段开环下降，相机盲区）
 *                        → GRASP（TOF 触发闭爪）→ ASCEND（回起始高度）→ IDLE
 * 测试流程（无 P4，前端触发）： IDLE → DESCEND → GRASP → ASCEND → IDLE
 *   假设：触发时位置精确，下降全程不施加视觉修正（光流位置保持照常抗漂移）。
 *
 * TOF 安装位置离地约 20cm（落地=爪触地时 TOF≈0.20m），因此"抓取触发高度"
 * 直接用 TOF 读数（默认 0.20m，前端 grab_cfg 滑条可调）。末段飞到目标正上方
 * 时 TOF 读的是目标顶面——正好是"距目标高度"，直接可用。
 *
 * 爪的归属：任务经 commander_set_grip() 写 setpoint 闭爪（sp->grip_angle 仍是
 * 唯一真源），GRASP/ASCEND 期间每拍重写——与 WS 解析整结构体赋值的竞态即使
 * 丢一次写，下一拍(10ms)自愈。任务结束后 sp 保持闭合值，断连复位也不掉爪。 */

#include "commander.h"
#include "altitude.h"
#include "position.h"
#include "flow_hold.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    GRAB_IDLE    = 0,
    GRAB_ALIGN   = 1,   /* P4 视觉对准 + 分段下降（测试模式跳过） */
    GRAB_DESCEND = 2,   /* 末段下降，等 TOF 到触发高度 */
    GRAB_GRASP   = 3,   /* 定高闭爪 */
    GRAB_ASCEND  = 4,   /* 回起始高度 */
} grab_state_t;

/* 已通过全部安全门（协议四道门+准静态门，main.c 负责）的视觉测量 */
typedef struct {
    float dx_m, dy_m;   /* 机体系目标偏差，前+/右+ */
    bool  valid;        /* 本拍有一条可采信的新测量 */
} grab_meas_t;

typedef struct {
    grab_state_t state;
    bool    test_mode;        /* true=无 P4 测试（跳过 ALIGN，不用视觉修正） */
    float   grab_tof_m;       /* 触发闭爪的 TOF 读数（任务开始时从 sp 拷贝） */
    float   start_alt_m;      /* 任务起始保持高度，抓取后回到这里 */
    int64_t state_since_us;
    int64_t mission_since_us;
    int64_t last_corr_us;     /* 上次应用视觉修正时刻（修正限速） */
    int64_t grasp_done_us;    /* 爪到位时刻（0=未到位） */
    int     align_ok_cnt;     /* 连续容差内测量计数 */
    bool    align_stepping;   /* ALIGN 子状态：正在下降一级台阶 */
    int     result;           /* 上次任务结果: 0=无 1=成功 -1=中止 */
} grab_mission_t;

void grab_mission_init(grab_mission_t *gm);

/**
 * @brief 启动抓取任务（主循环上下文调用）。前置校验：ALT/POS_HOLD 模式、
 *        TOF 有效且高于触发高度+10cm、爪已张开、非测试模式还需 P4 链路存活。
 * @return 0=已启动, -1=前置校验失败（原因见日志）
 */
int grab_mission_start(grab_mission_t *gm, bool test_mode, bool p4_alive,
                       const setpoint_t *sp, altitude_ctrl_t *alt,
                       uint16_t tof_mm);

/**
 * @brief 中止任务：回到起始高度，爪保持当前角（绝不强制张开——防掉落）
 */
void grab_mission_abort(grab_mission_t *gm, altitude_ctrl_t *alt,
                        uint16_t tof_mm, const char *reason);

/**
 * @brief 状态机推进，主循环每拍（100Hz）调用。内部自带全局中止条件：
 *        退出定高模式 / 油门切断 / TOF 失效 → 自动中止。
 */
void grab_mission_update(grab_mission_t *gm, const setpoint_t *sp,
                         altitude_ctrl_t *alt, position_ctrl_t *pos,
                         const flow_hold_t *fh, const grab_meas_t *meas,
                         uint16_t tof_mm, float dt);

bool grab_mission_active(const grab_mission_t *gm);

#ifdef __cplusplus
}
#endif
