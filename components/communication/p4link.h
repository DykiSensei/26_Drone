#pragma once

/* p4link — S3 侧串口驱动（协议定义见 p4link_protocol.h / P4LINK_PROTOCOL.md）
 * UART2, TX=GPIO17 → P4.RX, RX=GPIO18 ← P4.TX, 115200-8N1
 * 独立 rx 任务（Core 1）喂解析状态机；发送非阻塞（FIFO 装不下即丢帧）。
 * ⚠️ 2026-07-05: P4 硬件未就绪——驱动可编译可运行，无对端时静默待机
 * （p4link_alive() 恒 false，抓取任务的 P4 路径拒绝启动）。 */

#include "p4link_protocol.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 UART2 + 启动 p4link_rx 解析任务
 * @return 0 成功, -1 失败（调用方按非致命处理：无 P4 链路但不影响飞行）
 */
int p4link_init(void);

/**
 * @brief 发送一帧 MSG_STATE（主循环 50Hz 调用）。
 *        非阻塞：uart_tx_chars 只写硬件 FIFO 能装下的部分——50Hz×24B 远低于
 *        FIFO 排空速度，正常永远整帧写入；极端拥塞下的半帧由 P4 侧 CRC
 *        丢弃并重同步，绝不阻塞控制环。
 */
void p4link_send_state(const p4link_state_t *st);

/**
 * @brief 取走最新一条 MSG_TARGET（每帧只交付一次，take 语义）
 * @param out         输出测量（原始值，安全门控由调用方负责）
 * @param rx_time_us  可为 NULL；输出该帧的本地接收时刻
 * @return true=有新帧并已写入 out；false=自上次取走后无新帧
 */
bool p4link_take_target(p4link_target_t *out, int64_t *rx_time_us);

/**
 * @brief 查看最近一帧 MSG_TARGET（**非消费**：不影响 take 语义的 fresh 标志）。
 *        仅供遥测/联调观察最新原始测量（dx/dy/conf/track），与 p4link_take_target
 *        的门控消费路径互不干扰。
 * @param out         输出最近一帧原始测量（若从未收到则不写）
 * @param rx_time_us  可为 NULL；输出该帧的本地接收时刻
 * @return true=曾收到过至少一帧；false=从未收到（out 未写）
 */
bool p4link_peek_target(p4link_target_t *out, int64_t *rx_time_us);

/**
 * @brief 链路存活：P4LINK_LINK_TIMEOUT_MS 内收到过任何 CRC 正确的帧
 */
bool p4link_alive(void);

/**
 * @brief 接收统计（联调/遥测用）
 */
void p4link_get_stats(uint32_t *rx_ok, uint32_t *crc_err);

#ifdef __cplusplus
}
#endif
