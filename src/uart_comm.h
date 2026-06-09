#ifndef UART_COMM_H
#define UART_COMM_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 6DOF 位姿: 位置(mm) + 四元数 */
typedef struct {
    float x, y, z;         /* 位置, mm */
    float qx, qy, qz, qw;  /* 姿态, 四元数 (归一化) */
} Pose6D;

/* 收到下位机回传位姿时的回调 */
typedef void (*uart_pose_callback_t)(const Pose6D* pose);

/**
 * 打开并配置串口
 * @param device   设备节点, e.g. "/dev/ttyS9"
 * @param baudrate 波特率, 支持 9600~1500000 (建议 115200 或 921600)
 * @return 0 成功, -1 失败
 */
int  uart_init(const char* device, int baudrate);

/**
 * 关闭串口, 停止接收线程, 释放资源
 */
void uart_cleanup(void);

/**
 * 发送一次心跳/握手帧 (调试用)
 * @return 0 成功, -1 失败
 */
int uart_send_heartbeat(void);

/**
 * 向下位机发送目标位姿
 * @param pose 目标位姿指针
 * @return 0 成功, -1 失败
 */
int uart_send_target_pose(const Pose6D* pose);

/**
 * 向下位机发送机械臂目标指令（二进制 int16 小端格式）
 * 共 10 字节: [x][y][z][k1][k2]，每个 int16 小端
 *   x,y,z: 空间坐标, 单位 cm
 *   k1,k2: 舵机角度, 单位度
 * @return 0 成功, -1 失败
 */
int uart_send_arm_target(float x, float y, float z, float k1, float k2, uint8_t flag);

/**
 * 启动后台接收线程, 循环解析下位机回传帧
 * @param cb 收到 CURRENT_POSE 帧时的回调, 可为 NULL
 * @return 0 成功, -1 失败
 */
int uart_start_receiver(uart_pose_callback_t cb);

/**
 * 停止接收线程
 */
void uart_stop_receiver(void);

/**
 * 原始发送 (调试用)
 */
int uart_send_raw(const uint8_t* data, size_t len);

/**
 * 原始接收, 带超时 (调试用)
 * @return 实际读取字节数, 0 超时, -1 错误
 */
int uart_recv_raw(uint8_t* buf, size_t max_len, int timeout_ms);

/* 运动完成状态: 1=空闲/完成, 0=运动中/等待回执 */
extern volatile int g_uart_move_complete;

/* 下位机初始化成功: 1=已收到 "init success", 0=未收到 */
extern volatile int g_uart_init_success;

/* 下位机归位完成: 1=已收到归位后的首次 "move_success", 0=未收到 */
extern volatile int g_uart_homing_done;

/* 头部静止状态: 1=头部当前静止(wx/wz<3), 0=头部在动 */
extern volatile int g_head_stationary;

/* 机械臂到位状态: 1=头部静止持续400ms且距离上次发令>200ms, 认为机械臂已跟踪到位 */
extern volatile int g_arm_stable;

/* 调用方在确定发送后调用, 将状态置为运动中 */
void uart_set_move_pending(void);

/* 握手延时期间禁止 UART 发送: 1=禁止, 0=允许 */
extern volatile int g_uart_block_tx;

/* 诊断日志: 带运行时长, 同时写文件和终端 */
void diag_log(const char* fmt, ...);

#ifdef __cplusplus
}
#endif

#endif // UART_COMM_H
