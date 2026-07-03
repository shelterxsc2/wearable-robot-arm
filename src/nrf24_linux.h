/**
 * @file    nrf24_linux.h
 * @brief   nRF24L01+ Linux driver for RK3588 (spidev + sysfs GPIO).
 * @note    Thread-safe RX with pthread mutex.
 *          Zero dependency: no libgpiod required.
 */

#ifndef __NRF24_LINUX_H
#define __NRF24_LINUX_H

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== */
/*  Config: adapt these to your wiring                                      */
/* ======================================================================== */

/* SPI device path. RK3588 has /dev/spidev4.0 on the 40-pin header */
#define NRF24_SPIDEV_PATH       "/dev/spidev4.0"

/* Sysfs GPIO numbers for CE and IRQ.
 *  ELF2 P26 header wiring:
 *    CE  -> P26-11 (GPIO2_D0) = 64 + 3*8 + 0 = 88
 *    IRQ -> P26-12 (GPIO2_D2) = 64 + 3*8 + 2 = 90
 *  If you change wiring, recalc: gpio = base + group*8 + pin
 */
#define NRF24_CE_GPIO           88
#define NRF24_IRQ_GPIO          90

/* SPI speed. nRF24L01+ max is 8 MHz. For long wires / breadboard, use 4 MHz. */
#define NRF24_SPI_SPEED_HZ      4000000

/* ======================================================================== */
/*  Constants (same as STM32 driver)                                        */
/* ======================================================================== */

#define NRF24_PAYLOAD_WIDTH     22
#define NRF24_ADDR_WIDTH        5

/* Commands */
#define NRF24_CMD_R_REGISTER    0x00
#define NRF24_CMD_W_REGISTER    0x20
#define NRF24_CMD_R_RX_PAYLOAD  0x61
#define NRF24_CMD_FLUSH_TX      0xE1
#define NRF24_CMD_FLUSH_RX      0xE2
#define NRF24_CMD_NOP           0xFF

/* Registers */
#define NRF24_REG_CONFIG        0x00
#define NRF24_REG_EN_AA         0x01
#define NRF24_REG_EN_RXADDR     0x02
#define NRF24_REG_SETUP_AW      0x03
#define NRF24_REG_SETUP_RETR    0x04
#define NRF24_REG_RF_CH         0x05
#define NRF24_REG_RF_SETUP      0x06
#define NRF24_REG_STATUS        0x07
#define NRF24_REG_RX_ADDR_P0    0x0A
#define NRF24_REG_TX_ADDR       0x10
#define NRF24_REG_RX_PW_P0      0x11
#define NRF24_REG_FEATURE       0x1D
#define NRF24_REG_DYNPD         0x1C

/* Default values (MUST match STM32 TX side exactly) */
#define NRF24_DEFAULT_CONFIG_RX 0x0FU
#define NRF24_DEFAULT_EN_AA     0x01U
#define NRF24_DEFAULT_EN_RXADDR 0x01U
#define NRF24_DEFAULT_SETUP_AW  0x03U
#define NRF24_DEFAULT_SETUP_RETR 0x55U
#define NRF24_DEFAULT_RF_CH     0x53U
#define NRF24_DEFAULT_RF_SETUP  0x47U
#define NRF24_DEFAULT_RX_PW_P0  0x16U
#define NRF24_DEFAULT_FEATURE   0x00U
#define NRF24_DEFAULT_DYNPD     0x00U

/* 角加速度 + wz/wx 历史循环缓冲区大小 */
#define NRF24_AST_BUF_SIZE      10
#define NRF24_WZ_HIST_SIZE      10
#define NRF24_WX_HIST_SIZE      10
#define NRF24_WY_HIST_SIZE      10
/* roll/pitch/yaw 历史循环缓冲区 (degrees) — 供 A-inverse 平均 */
#define NRF24_ANGLE_HIST_SIZE   16

/* ======================================================================== */
/*  Shared state (thread-safe)                                              */
/* ======================================================================== */

typedef struct {
    pthread_mutex_t mutex;
    bool data_ready;                        /* new frame available */
    uint8_t rx_buffer[NRF24_PAYLOAD_WIDTH]; /* last received payload (22B = angle 11B + gyro 11B) */
    uint32_t rx_count;                      /* total good frames */
    uint32_t error_count;                   /* CRC/bad header/etc */
    /* IMU angles (degrees), parsed from 11-byte frame, thread-safe */
    float gy_roll;
    float gy_pitch;
    float gy_yaw;
    /* IMU angular velocity (deg/s), parsed from 11-byte gyro frame */
    float gy_wx;
    float gy_wy;
    float gy_wz;
    /* IMU quaternion, q0/q1/q2/q3 = qw/qx/qy/qz */
    float gy_qw;
    float gy_qx;
    float gy_qy;
    float gy_qz;
    bool  quat_valid;
    bool  imu_valid;                        /* checksum + header passed */
    /* PnP 视觉修正量（度），由视觉线程写入，IMU 控制线程读取 */
    float pnp_yaw_correction;
    float pnp_pitch_correction;
    bool  pnp_valid;
    bool  pnp_correction_ready;             /* 连续 3 帧 PnP 有效，可执行零飘修正 */
    /* 角加速度循环缓冲区 (deg/s^2) */
    float gy_ast_buf[NRF24_AST_BUF_SIZE];   /* 最近 N 次角加速度 */
    int   gy_ast_idx;                       /* 下一次写入位置 */
    int   gy_ast_count;                     /* 当前有效数据量 (0..NRF24_AST_BUF_SIZE) */
    /* wz 历史循环缓冲区 (deg/s) — 供上位机 50ms 控制周期内分析 10ms 分辨率数据 */
    float gy_wz_hist[NRF24_WZ_HIST_SIZE];
    int   gy_wz_idx;
    int   gy_wz_count;
    /* wx 历史循环缓冲区 (deg/s) */
    float gy_wx_hist[NRF24_WX_HIST_SIZE];
    int   gy_wx_idx;
    int   gy_wx_count;
    /* wy 历史循环缓冲区 (deg/s) — 供 pitch 控制链路使用 */
    float gy_wy_hist[NRF24_WY_HIST_SIZE];
    int   gy_wy_idx;
    int   gy_wy_count;
    /* roll/pitch/yaw 历史循环缓冲区 (degrees) — 供 A-inverse 平均 */
    float gy_roll_hist[NRF24_ANGLE_HIST_SIZE];
    float gy_pitch_hist[NRF24_ANGLE_HIST_SIZE];
    float gy_yaw_hist[NRF24_ANGLE_HIST_SIZE];
    int   gy_angle_idx;
    int   gy_angle_count;
} nrf24_shared_state_t;

extern nrf24_shared_state_t g_nrf24_state;
extern volatile int g_nrf24_running;        /* thread control flag */

/* ======================================================================== */
/*  APIs                                                                    */
/* ======================================================================== */

/**
 * @brief  Initialize spidev + GPIOs + nRF24 registers (RX mode).
 * @return 0 on success, -1 on error.
 */
int nrf24_linux_init(void);

/**
 * @brief  Deinit: close spidev, release GPIOs.
 */
void nrf24_linux_deinit(void);

/**
 * @brief  Start the background RX thread.
 * @param tid  output: thread ID.
 * @return 0 on success.
 */
int nrf24_rx_thread_start(pthread_t* tid);

/**
 * @brief  Stop RX thread and wait for it to exit.
 */
void nrf24_rx_thread_stop(void);

/* Low-level SPI + register APIs (exported for debug) */
uint8_t nrf24_read_reg(uint8_t reg);
void    nrf24_write_reg(uint8_t reg, uint8_t value);
void    nrf24_read_rx_payload(uint8_t* buf, uint8_t len);
void    nrf24_flush_rx(void);
void    nrf24_flush_tx(void);

#ifdef __cplusplus
}
#endif

#endif /* __NRF24_LINUX_H */
