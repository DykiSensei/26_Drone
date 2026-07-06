#pragma once

/* p4link_protocol.h — S3↔P4 串口协议定义（协议版本 1）
 *
 * ⚠️ 2026-07-05 理论设计，P4 硬件未就绪，尚未实测联调。
 *
 * 本文件是协议的唯一权威定义：S3 与 P4 两个工程各持一份拷贝，内容必须
 * 逐字节一致。双方都是 ESP32（小端），packed 结构体直接收发，无字节序转换。
 *
 * 角色划分（设计基石）：
 *   P4 = 纯传感器 —— 只报告"看到了什么、多确定"（目标偏差 + 置信度），
 *        永不直接指挥飞行；
 *   S3 = 任务与安全决策全权 —— 抓取状态机、所有门限判断都在飞控侧，
 *        P4 输入一律按不可信传感器处理（限幅/置信度/陈旧度/超时四道门）。
 *
 * 物理层：UART 115200-8N1，3.3V TTL，必须共地
 *   S3: UART2（GPIO 矩阵映射） TX=GPIO17 → P4.RX
 *                              RX=GPIO18 ← P4.TX
 *   （UART0=控制台且 GPIO44 被光流 TX 顶死；UART1 RX 已被光流占用）
 *
 * 帧格式（所有多字节字段小端）：
 *   [0xAA][0x55][type u8][len u8][payload: len 字节][crc_lo][crc_hi]
 *   crc16 = CRC-16/CCITT-FALSE，覆盖 type + len + payload（不含同步字节）
 *   解析：状态机扫描 0xAA 0x55 同步 → 读 type/len → 收 payload+CRC →
 *         CRC 错整帧丢弃并重新扫描同步（帧短、损失小，不做转义/COBS）
 */

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define P4LINK_PROTO_VER    1
#define P4LINK_MAGIC0       0xAA
#define P4LINK_MAGIC1       0x55
#define P4LINK_MAX_PAYLOAD  64
#define P4LINK_BAUD         115200
#define P4LINK_FRAME_OVERHEAD 6   /* magic×2 + type + len + crc×2 */

/* 消息类型分区：0x0x = S3→P4，0x1x = P4→S3（双向都有扩展空间） */
typedef enum {
    P4LINK_MSG_STATE  = 0x01,   /* S3→P4 状态广播，50Hz */
    P4LINK_MSG_TARGET = 0x10,   /* P4→S3 目标偏差；未锁定时也是心跳 */
} p4link_msg_type_t;

/* ─────────────── MSG_STATE：S3→P4，50Hz ───────────────
 * P4 用途：像素偏差 → 地面平面米制投影需要姿态角与高度；
 *          flags/mode 告诉 P4 飞机当前是否稳定（测量窗口是否有效）。 */

/* flags 位定义 */
#define P4LINK_F_ARMED       (1u << 0)  /* mode != DISARMED */
#define P4LINK_F_FLOW_TRUST  (1u << 1)  /* flow_hold.quality_gain >= 0.05 */
#define P4LINK_F_POS_HOLD    (1u << 2)  /* 位置锁激活（悬停稳定，适合测量） */
#define P4LINK_F_TOF_VALID   (1u << 3)  /* height_mm 有效 */

typedef struct __attribute__((packed)) {
    uint32_t ts_ms;       /* S3 毫秒时钟（esp_timer_get_time()/1000 截断 u32） */
    int16_t  roll_cdeg;   /* 姿态角，0.01°（centi-degree），与遥测同号约定 */
    int16_t  pitch_cdeg;
    int16_t  yaw_cdeg;
    uint16_t height_mm;   /* TOF 距正下方表面距离；0=无效。
                           * ⚠️ 末段下降时读的是目标顶面而非地面——P4 投影
                           * 到目标平面时这正是所需的"距目标高度" */
    int16_t  vz_mms;      /* 垂直速度 mm/s，向上为正 */
    uint8_t  flow_qual;   /* 光流质量原始值（参考） */
    uint8_t  mode;        /* flight_mode_t: 0=DISARMED 1=STAB 2=ALT 3=POS */
    uint8_t  grip_deg;    /* 机械爪当前角 0–117（0=完全张开，110=抓取闭合预设，
                           * 117=行程极限；2026-07-06 新爪体标定值） */
    uint8_t  flags;       /* P4LINK_F_* 位组合 */
} p4link_state_t;         /* payload = 18 字节，线上 24 字节 */

/* ─────────────── MSG_TARGET：P4→S3 ───────────────
 * 锁定目标时 20–30Hz；未锁定时也必须 ≥10Hz 发 SEARCHING 帧作为链路心跳
 * （S3 靠它检测 P4 存活——LINK_TIMEOUT 超时即中止抓取回悬停）。 */

typedef enum {
    P4LINK_TRACK_SEARCHING = 0,  /* 未发现目标（心跳帧，dx/dy/conf 无意义） */
    P4LINK_TRACK_LOCKED    = 1,  /* 已锁定，dx/dy 有效 */
    P4LINK_TRACK_LOST      = 2,  /* 刚丢失（曾锁定；S3 可决定悬停等待/重搜） */
} p4link_track_state_t;

typedef struct __attribute__((packed)) {
    uint32_t ts_echo_ms;  /* P4 做投影解算时所用 MSG_STATE 的 ts_ms **原样回传**。
                           * S3 用 (now_ms - ts_echo_ms) 得到测量总陈旧度
                           * （含 P4 处理耗时 + 链路时延）——无需两侧时钟同步。
                           * P4 尚未收到任何 MSG_STATE 时填 0：S3 把该帧仅当
                           * 心跳，不采信测量（S3 保证发出的 ts_ms 恒 ≥1） */
    int16_t  dx_mm;       /* 目标相对机体偏差：机体前方为正（同 move_to x 约定） */
    int16_t  dy_mm;       /* 机体右方为正（同 move_to y 约定） */
    uint8_t  conf;        /* 置信度 0–100 */
    uint8_t  track;       /* p4link_track_state_t */
    uint16_t reserved;    /* 目标类别/ID 预留，v1 填 0 */
} p4link_target_t;        /* payload = 12 字节，线上 18 字节 */

/* ─────────────── S3 侧安全门限（协议约定默认值，实现处可调参） ─────────────── */
#define P4LINK_OFFSET_CLAMP_MM  1000  /* dx/dy 限幅 ±1m（比 move_to ±3m 更紧） */
#define P4LINK_CONF_MIN         50    /* conf 低于此丢弃该测量 */
#define P4LINK_AGE_MAX_MS       200   /* 陈旧度超过此丢弃（now - ts_echo_ms） */
#define P4LINK_LINK_TIMEOUT_MS  500   /* 无有效帧超时→中止抓取序列回位置保持。
                                       * ⚠️ 只中止任务，绝不 DISARM——P4 不是
                                       * 遥控链路，它挂了飞机必须还能飞 */
#define P4LINK_TILT_GATE_CDEG   500   /* |roll|或|pitch| > 5.00° 时不采信测量
                                       * （准静态门：v1 不做严格时戳补偿，只在
                                       * 近悬停状态接受视觉修正，5° @1m ≈ 9cm
                                       * 投影误差上限） */

/* ─────────────── CRC-16/CCITT-FALSE 参考实现 ───────────────
 * poly=0x1021, init=0xFFFF, 不反转, 无输出异或。两侧必须用同一实现
 * （不要换用各自 SDK 的 ROM CRC——多项式/反转约定不一致）。
 * 帧短(≤24B)频率低(≤80Hz)，位运算实现的开销可忽略。 */
static inline uint16_t p4link_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
    }
    return crc;
}

#ifdef __cplusplus
}
#endif
