/**
 * @file    imu2_i2c.h
 * @brief   On-board IMU (I2C4) driver for RK3588.
 * @note    100 Hz sampling via dedicated pthread; shared state protected by mutex.
 *          SCL.1 / SDA.1 -> /dev/i2c-4, slave addr 0x50 (7-bit).
 */

#ifndef __IMU2_I2C_H
#define __IMU2_I2C_H

#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================================================================== */
/*  Config                                                                  */
/* ======================================================================== */

#define IMU2_I2C_DEV_PATH       "/dev/i2c-4"
#define IMU2_SLAVE_ADDR_7BIT    0x50

/* 寄存器地址 (16-bit signed, little-endian) */
#define IMU2_REG_AX             0x34
#define IMU2_REG_AY             0x35
#define IMU2_REG_AZ             0x36
#define IMU2_REG_GX             0x37
#define IMU2_REG_GY             0x38
#define IMU2_REG_GZ             0x39
#define IMU2_REG_ROLL           0x3D
#define IMU2_REG_PITCH          0x3E
#define IMU2_REG_YAW            0x3F

/* 转换系数 */
#define IMU2_SCALE_ACC          (16.0f * 9.8f / 32768.0f)   /* m/s^2  */
#define IMU2_SCALE_GYRO         (2000.0f / 32768.0f)         /* deg/s  */
#define IMU2_SCALE_ANGLE        (180.0f / 32768.0f)          /* deg    */

/* 采样周期: 100 Hz -> 10 ms */
#define IMU2_SAMPLE_PERIOD_US   10000

/* ======================================================================== */
/*  Shared state (thread-safe)                                              */
/* ======================================================================== */

typedef struct {
    pthread_mutex_t mutex;
    bool data_ready;            /* new sample available */
    uint32_t sample_count;      /* total good samples */
    bool valid;                 /* last read succeeded */

    /* 加速度 (m/s^2) */
    float ax;
    float ay;
    float az;

    /* 角速度 (deg/s) */
    float gx;
    float gy;
    float gz;

    /* 欧拉角 (deg) */
    float roll;
    float pitch;
    float yaw;
} imu2_shared_state_t;

extern imu2_shared_state_t g_imu2_state;
extern volatile int g_imu2_running;

/* ======================================================================== */
/*  APIs                                                                    */
/* ======================================================================== */

/**
 * @brief  Open I2C bus and configure slave address.
 * @return 0 on success, -1 on error.
 */
int imu2_i2c_init(void);

/**
 * @brief  Close I2C fd.
 */
void imu2_i2c_deinit(void);

/**
 * @brief  Start the background sampling thread (100 Hz).
 * @param tid  output: thread ID.
 * @return 0 on success.
 */
int imu2_i2c_thread_start(pthread_t *tid);

/**
 * @brief  Stop sampling thread and wait for it to exit.
 */
void imu2_i2c_thread_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __IMU2_I2C_H */
