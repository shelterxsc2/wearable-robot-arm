/**
 * @file    imu2_i2c.c
 * @brief   On-board IMU (I2C4) driver — 100 Hz pthread sampler.
 * @note    Uses Linux I2C_RDWR ioctl for atomic write-register + read-data.
 */

#include "imu2_i2c.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

/* ======================================================================== */
/*  Static state                                                            */
/* ======================================================================== */

static int g_i2c_fd = -1;
static pthread_t g_imu2_tid = 0;

imu2_shared_state_t g_imu2_state = {
    .mutex        = PTHREAD_MUTEX_INITIALIZER,
    .data_ready   = false,
    .sample_count = 0,
    .valid        = false,
    .ax = 0.0f, .ay = 0.0f, .az = 0.0f,
    .gx = 0.0f, .gy = 0.0f, .gz = 0.0f,
    .roll = 0.0f, .pitch = 0.0f, .yaw = 0.0f,
};

volatile int g_imu2_running = 0;

/* ======================================================================== */
/*  Low-level I2C helpers                                                   */
/* ======================================================================== */

/**
 * @brief  Read N bytes from a specific register using I2C_RDWR.
 * @param  reg   register address to write first
 * @param  buf   output buffer
 * @param  len   bytes to read
 * @return 0 on success, -1 on error.
 */
static int i2c_read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data rdwr;

    msgs[0].addr  = IMU2_SLAVE_ADDR_7BIT;
    msgs[0].flags = 0;              /* write */
    msgs[0].len   = 1;
    msgs[0].buf   = &reg;

    msgs[1].addr  = IMU2_SLAVE_ADDR_7BIT;
    msgs[1].flags = I2C_M_RD;       /* read */
    msgs[1].len   = len;
    msgs[1].buf   = buf;

    rdwr.msgs  = msgs;
    rdwr.nmsgs = 2;

    if (ioctl(g_i2c_fd, I2C_RDWR, &rdwr) < 0) {
        return -1;
    }
    return 0;
}

/**
 * @brief  Parse little-endian int16 from two bytes.
 */
static inline int16_t parse_i16(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/**
 * @brief  Read one complete IMU sample (acc + gyro + angle).
 * @return 0 on success, -1 on error.
 */
static int imu2_i2c_read_once(void)
{
    uint8_t acc_buf[6];
    uint8_t gyro_buf[6];
    uint8_t angle_buf[6];
    int ret;

    /* 加速度: 0x34 ~ 0x36 (AX, AY, AZ) */
    ret = i2c_read_regs(IMU2_REG_AX, acc_buf, 6);
    if (ret < 0) return -1;

    /* 角速度: 0x37 ~ 0x39 (GX, GY, GZ) */
    ret = i2c_read_regs(IMU2_REG_GX, gyro_buf, 6);
    if (ret < 0) return -1;

    /* 欧拉角: 0x3D ~ 0x3F (Roll, Pitch, Yaw) */
    ret = i2c_read_regs(IMU2_REG_ROLL, angle_buf, 6);
    if (ret < 0) return -1;

    /* 解析并转换 */
    float ax = parse_i16(&acc_buf[0])   * IMU2_SCALE_ACC;
    float ay = parse_i16(&acc_buf[2])   * IMU2_SCALE_ACC;
    float az = parse_i16(&acc_buf[4])   * IMU2_SCALE_ACC;

    float gx = parse_i16(&gyro_buf[0])  * IMU2_SCALE_GYRO;
    float gy = parse_i16(&gyro_buf[2])  * IMU2_SCALE_GYRO;
    float gz = parse_i16(&gyro_buf[4])  * IMU2_SCALE_GYRO;

    float roll  = parse_i16(&angle_buf[0]) * IMU2_SCALE_ANGLE;
    float pitch = parse_i16(&angle_buf[2]) * IMU2_SCALE_ANGLE;
    float yaw   = parse_i16(&angle_buf[4]) * IMU2_SCALE_ANGLE;

    /* 更新共享状态 */
    pthread_mutex_lock(&g_imu2_state.mutex);
    g_imu2_state.ax = ax;
    g_imu2_state.ay = ay;
    g_imu2_state.az = az;
    g_imu2_state.gx = gx;
    g_imu2_state.gy = gy;
    g_imu2_state.gz = gz;
    g_imu2_state.roll  = roll;
    g_imu2_state.pitch = pitch;
    g_imu2_state.yaw   = yaw;
    g_imu2_state.valid = true;
    g_imu2_state.data_ready = true;
    g_imu2_state.sample_count++;
    pthread_mutex_unlock(&g_imu2_state.mutex);

    return 0;
}

/* ======================================================================== */
/*  Thread                                                                  */
/* ======================================================================== */

static void *imu2_i2c_thread_func(void *arg)
{
    (void)arg;
    int print_cnt = 0;

    printf("[IMU2-I2C] Sampling thread started (100 Hz)\n");

    while (g_imu2_running) {
        if (imu2_i2c_read_once() < 0) {
            /* 单次读取失败，标记无效但继续 */
            pthread_mutex_lock(&g_imu2_state.mutex);
            g_imu2_state.valid = false;
            pthread_mutex_unlock(&g_imu2_state.mutex);
            usleep(IMU2_SAMPLE_PERIOD_US);
            continue;
        }

        /* IMU2 log disabled — use NRF24 (head IMU) log instead */
        (void)print_cnt;

        usleep(IMU2_SAMPLE_PERIOD_US);
    }

    printf("[IMU2-I2C] Sampling thread exiting\n");
    return NULL;
}

/* ======================================================================== */
/*  Public APIs                                                             */
/* ======================================================================== */

int imu2_i2c_init(void)
{
    g_i2c_fd = open(IMU2_I2C_DEV_PATH, O_RDWR);
    if (g_i2c_fd < 0) {
        fprintf(stderr, "[IMU2-I2C] Failed to open %s: %s\n",
                IMU2_I2C_DEV_PATH, strerror(errno));
        return -1;
    }

    if (ioctl(g_i2c_fd, I2C_SLAVE, IMU2_SLAVE_ADDR_7BIT) < 0) {
        fprintf(stderr, "[IMU2-I2C] Failed to set slave addr 0x%02X: %s\n",
                IMU2_SLAVE_ADDR_7BIT, strerror(errno));
        close(g_i2c_fd);
        g_i2c_fd = -1;
        return -1;
    }

    printf("[IMU2-I2C] Bus %s, slave 0x%02X initialized\n",
           IMU2_I2C_DEV_PATH, IMU2_SLAVE_ADDR_7BIT);
    return 0;
}

void imu2_i2c_deinit(void)
{
    if (g_i2c_fd >= 0) {
        close(g_i2c_fd);
        g_i2c_fd = -1;
        printf("[IMU2-I2C] Bus closed\n");
    }
}

int imu2_i2c_thread_start(pthread_t *tid)
{
    if (g_i2c_fd < 0) {
        fprintf(stderr, "[IMU2-I2C] I2C not initialized\n");
        return -1;
    }

    g_imu2_running = 1;
    if (pthread_create(tid, NULL, imu2_i2c_thread_func, NULL) != 0) {
        g_imu2_running = 0;
        fprintf(stderr, "[IMU2-I2C] Failed to create sampling thread\n");
        return -1;
    }

    g_imu2_tid = *tid;
    return 0;
}

void imu2_i2c_thread_stop(void)
{
    if (g_imu2_running) {
        g_imu2_running = 0;
        if (g_imu2_tid != 0) {
            pthread_join(g_imu2_tid, NULL);
            g_imu2_tid = 0;
        }
    }
}
