/**
 * @file protocol.h
 * @brief 上下位机共享通信协议定义
 *
 * 本文件同时用于：
 *   - host-rk3588 (上位机, uart_comm.cpp)
 *   - mcu-stm32   (下位机, UART 中断/轮询接收)
 *
 * 协议格式: [0xAA][0x55][LEN][CMD][payload...][CRC8]
 */

#ifndef SHARED_PROTOCOL_H
#define SHARED_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- 帧格式常量 ---------- */
#define FRAME_HEAD0       0xAA
#define FRAME_HEAD1       0x55
#define FRAME_HEAD_LEN    2
#define FRAME_EXTRA_LEN   3   /* LEN + CMD + CRC8 */

/* ---------- 命令字 ---------- */
#define CMD_HEARTBEAT     0x01  /**< 心跳/握手 */
#define CMD_TARGET_POSE   0x10  /**< 上位机 -> 下位机: 目标位姿 */
#define CMD_CURRENT_POSE  0x20  /**< 下位机 -> 上位机: 当前末端位姿 */
#define CMD_ARM_TARGET    0x30  /**< 上位机 -> 下位机: 简化机械臂目标指令 (x,y,z,k1,k2) */

/* ---------- 数据结构 ---------- */

/**
 * @brief 6DOF 位姿: 位置(mm) + 四元数
 *
 * 坐标系: 机械臂基座系
 *   - 原点: J1 关节中心（或下位机定义原点）
 *   - +Z: 竖直向上
 *   - 单位: 位置 mm, 四元数 归一化
 */
typedef struct {
    float x;   /**< 位置 X, mm */
    float y;   /**< 位置 Y, mm */
    float z;   /**< 位置 Z, mm */
    float qx;  /**< 四元数 X */
    float qy;  /**< 四元数 Y */
    float qz;  /**< 四元数 Z */
    float qw;  /**< 四元数 W */
} Pose6D;

/**
 * @brief 简化机械臂目标指令
 *
 * 共 10 字节: [x][y][z][k1][k2]，每个 int16 小端
 *   - x,y,z: 空间坐标, 单位 cm
 *   - k1,k2: 舵机角度, 单位度
 */
typedef struct {
    int16_t x;   /**< cm */
    int16_t y;   /**< cm */
    int16_t z;   /**< cm */
    int16_t k1;  /**< 舵机1角度, 度 */
    int16_t k2;  /**< 舵机2角度, 度 */
} ArmTarget;

/* ---------- 校验 ---------- */
static inline uint8_t protocol_crc8(const uint8_t *data, uint16_t len)
{
    uint8_t c = 0;
    for (uint16_t i = 0; i < len; ++i) {
        c += data[i];
    }
    return c;
}

/* ---------- 帧打包辅助函数 (可在下位机复用) ---------- */

/**
 * @brief 打包一帧数据到缓冲区
 * @param buf    输出缓冲区, 至少 len+5 字节
 * @param cmd    命令字
 * @param payload 载荷数据指针
 * @param len    载荷长度 (<= 250)
 * @return 打包后的总字节数
 */
static inline uint16_t protocol_pack_frame(uint8_t *buf, uint8_t cmd,
                                            const uint8_t *payload, uint8_t len)
{
    buf[0] = FRAME_HEAD0;
    buf[1] = FRAME_HEAD1;
    buf[2] = len;
    buf[3] = cmd;
    if (len > 0 && payload != NULL) {
        for (uint8_t i = 0; i < len; ++i) {
            buf[4 + i] = payload[i];
        }
    }
    buf[4 + len] = protocol_crc8(&buf[2], len + 2);  /* CRC = LEN + CMD + payload */
    return 5 + len;
}

#ifdef __cplusplus
}
#endif

#endif /* SHARED_PROTOCOL_H */
