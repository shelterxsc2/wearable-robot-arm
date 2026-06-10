/**
 * @file    nrf24_linux.c
 * @brief   nRF24L01+ Linux driver (spidev + sysfs GPIO).
 * @note    Blocking IRQ wait via poll() on sysfs GPIO value file.
 *          RX runs in a dedicated pthread; shared state protected by mutex.
 */

#include "nrf24_linux.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

/* ======================================================================== */
/*  Static state                                                            */
/* ======================================================================== */

static int g_spidev_fd = -1;
static int g_ce_gpio   = NRF24_CE_GPIO;
static int g_irq_gpio  = NRF24_IRQ_GPIO;

nrf24_shared_state_t g_nrf24_state = {
    .mutex      = PTHREAD_MUTEX_INITIALIZER,
    .data_ready = false,
    .rx_count   = 0,
    .error_count= 0,
    .gy_roll    = 0.0f,
    .gy_pitch   = 0.0f,
    .gy_yaw     = 0.0f,
    .gy_wx      = 0.0f,
    .gy_wy      = 0.0f,
    .gy_wz      = 0.0f,
    .imu_valid  = false,
    .pnp_yaw_correction   = 0.0f,
    .pnp_pitch_correction = 0.0f,
    .pnp_valid            = false,
    .pnp_correction_ready = false,
    .gy_ast_idx = 0,
    .gy_ast_count = 0,
    .gy_wz_hist = {0},
    .gy_wz_idx  = 0,
    .gy_wz_count= 0,
    .gy_wx_hist = {0},
    .gy_wx_idx  = 0,
    .gy_wx_count= 0,
};

volatile int g_nrf24_running = 0;

/* A-init 信号与完成标志（定义在 main.cpp / rga_npu.cpp） */
extern volatile int g_wait_a_init;
extern volatile int g_r_init_set;

/* ======================================================================== */
/*  Sysfs GPIO helpers                                                      */
/* ======================================================================== */

static int gpio_export_raw(int gpio)
{
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d", gpio);
    if (access(path, F_OK) == 0) return 0; /* already exported */

    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) return -1; /* permission denied or not exist */

    char buf[8];
    int len = snprintf(buf, sizeof(buf), "%d", gpio);
    int ret = write(fd, buf, len);
    close(fd);
    return (ret < 0 && errno != EBUSY) ? -1 : 0;
}

static int gpio_export_with_fallback(int gpio)
{
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d", gpio);
    if (access(path, F_OK) == 0) return 0; /* already exported */

    /* 1. Try direct export (works if root or udev rule allows) */
    if (gpio_export_raw(gpio) == 0) {
        usleep(50000);
        return 0;
    }

    /* 2. Try sudo -n (non-interactive) export */
    char cmd[128];
    snprintf(cmd, sizeof(cmd),
             "sudo -n sh -c 'echo %d > /sys/class/gpio/export' 2>/dev/null",
             gpio);
    if (system(cmd) == 0) {
        usleep(100000);
        if (access(path, F_OK) == 0) return 0;
    }

    /* 3. Fail */
    return -1;
}

static int gpio_unexport(int gpio)
{
    int fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if (fd < 0) {
        /* try sudo fallback */
        char cmd[128];
        snprintf(cmd, sizeof(cmd),
                 "sudo -n sh -c 'echo %d > /sys/class/gpio/unexport' 2>/dev/null",
                 gpio);
        system(cmd);
        return 0;
    }
    char buf[8];
    int len = snprintf(buf, sizeof(buf), "%d", gpio);
    write(fd, buf, len);
    close(fd);
    return 0;
}

static int gpio_set_direction(int gpio, const char* dir)
{
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", gpio);
    int fd = open(path, O_WRONLY);
    if (fd < 0) { perror(path); return -1; }
    write(fd, dir, strlen(dir));
    close(fd);
    return 0;
}

static int gpio_set_value(int gpio, int val)
{
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio);
    int fd = open(path, O_WRONLY);
    if (fd < 0) return -1;
    dprintf(fd, "%d", val);
    close(fd);
    return 0;
}

static int gpio_set_edge(int gpio, const char* edge)
{
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/edge", gpio);
    int fd = open(path, O_WRONLY);
    if (fd < 0) { perror(path); return -1; }
    write(fd, edge, strlen(edge));
    close(fd);
    return 0;
}

/**
 * @brief  Blocking wait for IRQ falling edge via poll() on sysfs value.
 * @return 1 if IRQ fired, 0 on timeout, -1 on error.
 */
static int gpio_wait_irq(int gpio, int timeout_ms)
{
    char path[64];
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio);
    int fd = open(path, O_RDONLY | O_NONBLOCK);
    if (fd < 0) { perror(path); return -1; }

    /* If IRQ is already low, a frame arrived while we were processing.
       poll() on sysfs GPIO is edge-triggered and would miss this. */
    char val;
    if (read(fd, &val, 1) == 1 && val == '0') {
        close(fd);
        return 1; /* IRQ already pending */
    }

    struct pollfd pfd = {
        .fd = fd,
        .events = POLLPRI,
    };

    int ret = poll(&pfd, 1, timeout_ms);
    close(fd);

    if (ret < 0) { perror("poll"); return -1; }
    if (ret == 0) return 0; /* timeout */
    return (pfd.revents & POLLPRI) ? 1 : 0;
}

/* ======================================================================== */
/*  SPI helpers                                                             */
/* ======================================================================== */

static int spidev_xfer(const uint8_t* tx, uint8_t* rx, size_t len)
{
    if (g_spidev_fd < 0) return -1;

    struct spi_ioc_transfer tr = {
        .tx_buf        = (unsigned long)tx,
        .rx_buf        = (unsigned long)rx,
        .len           = len,
        .speed_hz      = NRF24_SPI_SPEED_HZ,
        .delay_usecs   = 0,
        .bits_per_word = 8,
    };

    int ret = ioctl(g_spidev_fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 0) { perror("SPI_IOC_MESSAGE"); }
    return ret;
}

uint8_t nrf24_read_reg(uint8_t reg)
{
    uint8_t tx[2] = { NRF24_CMD_R_REGISTER | reg, NRF24_CMD_NOP };
    uint8_t rx[2] = {0};
    spidev_xfer(tx, rx, 2);
    return rx[1];
}

void nrf24_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { NRF24_CMD_W_REGISTER | reg, value };
    uint8_t rx[2] = {0};
    spidev_xfer(tx, rx, 2);
}

void nrf24_read_rx_payload(uint8_t* buf, uint8_t len)
{
    if (len > NRF24_PAYLOAD_WIDTH) len = NRF24_PAYLOAD_WIDTH;

    uint8_t tx[1 + NRF24_PAYLOAD_WIDTH];
    uint8_t rx[1 + NRF24_PAYLOAD_WIDTH];
    memset(tx, NRF24_CMD_NOP, sizeof(tx));
    tx[0] = NRF24_CMD_R_RX_PAYLOAD;

    spidev_xfer(tx, rx, 1 + len);
    memcpy(buf, rx + 1, len);
}

void nrf24_flush_rx(void)
{
    uint8_t tx = NRF24_CMD_FLUSH_RX;
    uint8_t rx = 0;
    spidev_xfer(&tx, &rx, 1);
}

void nrf24_flush_tx(void)
{
    uint8_t tx = NRF24_CMD_FLUSH_TX;
    uint8_t rx = 0;
    spidev_xfer(&tx, &rx, 1);
}

/* ======================================================================== */
/*  Address helpers                                                         */
/* ======================================================================== */

static void nrf24_write_addr(uint8_t reg, const uint8_t* addr, uint8_t len)
{
    uint8_t tx[6];
    tx[0] = NRF24_CMD_W_REGISTER | reg;
    memcpy(tx + 1, addr, len);
    uint8_t rx[6] = {0};
    spidev_xfer(tx, rx, 1 + len);
}

/* ======================================================================== */
/*  Init / Deinit                                                           */
/* ======================================================================== */

int nrf24_linux_init(void)
{
    uint8_t mode, bits;
    uint32_t speed;
    uint8_t config;
    const uint8_t addr[NRF24_ADDR_WIDTH] = {0xB3, 0x47, 0xA1, 0x82, 0x69};

    /* 1. Open spidev */
    g_spidev_fd = open(NRF24_SPIDEV_PATH, O_RDWR);
    if (g_spidev_fd < 0) {
        fprintf(stderr, "[NRF24] Failed to open %s: %s\n", NRF24_SPIDEV_PATH, strerror(errno));
        return -1;
    }

    mode = SPI_MODE_0; /* CPOL=0, CPHA=0 */
    bits = 8;
    speed = NRF24_SPI_SPEED_HZ;
    ioctl(g_spidev_fd, SPI_IOC_WR_MODE, &mode);
    ioctl(g_spidev_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(g_spidev_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    /* 2. Export GPIOs (with sudo fallback) */
    if (gpio_export_with_fallback(g_ce_gpio) < 0 || gpio_export_with_fallback(g_irq_gpio) < 0) {
        fprintf(stderr, "[NRF24] Failed to export GPIOs. Try:\n");
        fprintf(stderr, "  sudo sh -c 'echo %d > /sys/class/gpio/export'\n", g_ce_gpio);
        fprintf(stderr, "  sudo sh -c 'echo %d > /sys/class/gpio/export'\n", g_irq_gpio);
        fprintf(stderr, "  Or run this program with sudo.\n");
        close(g_spidev_fd);
        g_spidev_fd = -1;
        return -1;
    }

    gpio_set_direction(g_ce_gpio, "out");
    gpio_set_direction(g_irq_gpio, "in");
    gpio_set_edge(g_irq_gpio, "falling");

    gpio_set_value(g_ce_gpio, 0); /* CE low during config */

    /* 3. nRF24 register init — MUST match STM32 TX side exactly */
    usleep(5000); /* > 100 us power-on */

    nrf24_write_reg(NRF24_REG_CONFIG,      NRF24_DEFAULT_CONFIG_RX);
    nrf24_write_reg(NRF24_REG_EN_AA,       NRF24_DEFAULT_EN_AA);
    nrf24_write_reg(NRF24_REG_EN_RXADDR,   NRF24_DEFAULT_EN_RXADDR);
    nrf24_write_reg(NRF24_REG_SETUP_AW,    NRF24_DEFAULT_SETUP_AW);
    nrf24_write_reg(NRF24_REG_SETUP_RETR,  NRF24_DEFAULT_SETUP_RETR);
    nrf24_write_reg(NRF24_REG_RF_CH,       NRF24_DEFAULT_RF_CH);
    nrf24_write_reg(NRF24_REG_RF_SETUP,    NRF24_DEFAULT_RF_SETUP);
    nrf24_write_reg(NRF24_REG_RX_PW_P0,    NRF24_DEFAULT_RX_PW_P0);
    nrf24_write_reg(NRF24_REG_FEATURE,     NRF24_DEFAULT_FEATURE);
    nrf24_write_reg(NRF24_REG_DYNPD,       NRF24_DEFAULT_DYNPD);

    /* clear IRQ flags */
    nrf24_write_reg(NRF24_REG_STATUS, 0x70);

    nrf24_flush_tx();
    nrf24_flush_rx();

    /* address — must match TX side */
    nrf24_write_addr(NRF24_REG_TX_ADDR,     addr, NRF24_ADDR_WIDTH);
    nrf24_write_addr(NRF24_REG_RX_ADDR_P0,  addr, NRF24_ADDR_WIDTH);

    /* sanity check + full register dump */
    config = nrf24_read_reg(NRF24_REG_CONFIG);
    uint8_t reg_rf_ch   = nrf24_read_reg(NRF24_REG_RF_CH);
    uint8_t reg_rf_setup= nrf24_read_reg(NRF24_REG_RF_SETUP);
    uint8_t reg_rx_pw   = nrf24_read_reg(NRF24_REG_RX_PW_P0);
    uint8_t reg_en_aa   = nrf24_read_reg(NRF24_REG_EN_AA);
    uint8_t reg_setup_retr = nrf24_read_reg(NRF24_REG_SETUP_RETR);
    uint8_t reg_fifo_status = nrf24_read_reg(0x17);

    printf("[NRF24] Init OK. SPI=%s, CE=GPIO%d, IRQ=GPIO%d\n",
           NRF24_SPIDEV_PATH, g_ce_gpio, g_irq_gpio);
    printf("[NRF24] REG DUMP: CONFIG=0x%02X RF_CH=0x%02X RF_SETUP=0x%02X "
           "RX_PW_P0=0x%02X EN_AA=0x%02X SETUP_RETR=0x%02X FIFO=0x%02X\n",
           config, reg_rf_ch, reg_rf_setup, reg_rx_pw, reg_en_aa,
           reg_setup_retr, reg_fifo_status);
    if ((config & 0x02) == 0) {
        fprintf(stderr, "[NRF24] WARNING: PWR_UP bit not set. Wiring issue?\n");
    }

    /* CE high -> enter RX mode */
    gpio_set_value(g_ce_gpio, 1);
    return 0;
}

void nrf24_linux_deinit(void)
{
    g_nrf24_running = 0;

    if (g_spidev_fd >= 0) {
        gpio_set_value(g_ce_gpio, 0);
        close(g_spidev_fd);
        g_spidev_fd = -1;
    }
    gpio_unexport(g_ce_gpio);
    gpio_unexport(g_irq_gpio);
}

/* ======================================================================== */
/*  Frame parser                                                            */
/* ======================================================================== */

/**
 * @brief  Parse 22-byte combined IMU frame. Frame order is NOT fixed.
 * @param  buf   22-byte raw payload.
 * @return true if at least one valid frame (angle or gyro) was parsed.
 *
 * Tries both positions [0..10] and [11..21] for each frame type.
 * Accepts: angle+gyro, gyro+angle, angle+angle, gyro+gyro (any combo).
 */
static bool nrf24_parse_22b(const uint8_t* buf,
                            float* roll, float* pitch, float* yaw,
                            float* wx, float* wy, float* wz)
{
    bool got_angle = false;
    bool got_gyro  = false;

    /* Try position 0..10 */
    if (buf[0] == 0x55 && buf[1] == 0x53) {
        uint8_t sum = 0x55 + 0x53;
        for (int i = 2; i < 10; i++) sum += buf[i];
        if (sum == buf[10]) {
            int16_t r = (int16_t)((buf[3] << 8) | buf[2]);
            int16_t p = (int16_t)((buf[5] << 8) | buf[4]);
            int16_t y = (int16_t)((buf[7] << 8) | buf[6]);
            *roll  = r / 32768.0f * 180.0f;
            *pitch = p / 32768.0f * 180.0f;
            *yaw   = y / 32768.0f * 180.0f;
            got_angle = true;
        }
    } else if (buf[0] == 0x55 && buf[1] == 0x52) {
        uint8_t sum = 0x55 + 0x52;
        for (int i = 2; i < 10; i++) sum += buf[i];
        if (sum == buf[10]) {
            int16_t x = (int16_t)((buf[3] << 8) | buf[2]);
            int16_t y = (int16_t)((buf[5] << 8) | buf[4]);
            int16_t z = (int16_t)((buf[7] << 8) | buf[6]);
            *wx = x / 32768.0f * 2000.0f;
            *wy = y / 32768.0f * 2000.0f;
            *wz = z / 32768.0f * 2000.0f;
            got_gyro = true;
        }
    }

    /* Try position 11..21 */
    if (buf[11] == 0x55 && buf[12] == 0x53) {
        uint8_t sum = 0x55 + 0x53;
        for (int i = 13; i < 21; i++) sum += buf[i];
        if (sum == buf[21]) {
            int16_t r = (int16_t)((buf[14] << 8) | buf[13]);
            int16_t p = (int16_t)((buf[16] << 8) | buf[15]);
            int16_t y = (int16_t)((buf[18] << 8) | buf[17]);
            *roll  = r / 32768.0f * 180.0f;
            *pitch = p / 32768.0f * 180.0f;
            *yaw   = y / 32768.0f * 180.0f;
            got_angle = true;
        }
    } else if (buf[11] == 0x55 && buf[12] == 0x52) {
        uint8_t sum = 0x55 + 0x52;
        for (int i = 13; i < 21; i++) sum += buf[i];
        if (sum == buf[21]) {
            int16_t x = (int16_t)((buf[14] << 8) | buf[13]);
            int16_t y = (int16_t)((buf[16] << 8) | buf[15]);
            int16_t z = (int16_t)((buf[18] << 8) | buf[17]);
            *wx = x / 32768.0f * 2000.0f;
            *wy = y / 32768.0f * 2000.0f;
            *wz = z / 32768.0f * 2000.0f;
            got_gyro = true;
        }
    }

    return got_angle || got_gyro;
}

/* ======================================================================== */
/*  RX Thread                                                               */
/* ======================================================================== */

static void* nrf24_rx_thread_func(void* arg)
{
    (void)arg;
    printf("[NRF24] RX thread started (poll mode, no IRQ)\n");

    /* Rate stats */
    uint32_t loop_frames = 0;
    struct timespec ts_start, ts_now;
    clock_gettime(CLOCK_MONOTONIC, &ts_start);

    /* 角加速度状态: prev_wz + 时间戳用于计算 gy_ast */
    float prev_wz = 0.0f;
    int   have_prev_wz = 0;
    struct timespec ts_prev_ast;
    clock_gettime(CLOCK_MONOTONIC, &ts_prev_ast);

    int diag_cnt = 0;
    int print_cnt = 0;
    while (g_nrf24_running) {
        /* Poll STATUS register every 5 ms */
        uint8_t status = nrf24_read_reg(NRF24_REG_STATUS);
        uint8_t fifo_status = nrf24_read_reg(0x17);
        uint8_t config = nrf24_read_reg(NRF24_REG_CONFIG);

        /* 诊断：每 1 秒打印一次寄存器状态 */
        if (++diag_cnt >= 200) {
            diag_cnt = 0;
            /* printf("[NRF24-DIAG] STATUS=0x%02X FIFO=0x%02X CONFIG=0x%02X "
                   "RX_DR=%d TX_DS=%d MAX_RT=%d RX_EMPTY=%d PWR_UP=%d PRIM_RX=%d "
                   "(total_rx=%u err=%u)\n",
                   status, fifo_status, config,
                   (status >> 6) & 1,
                   (status >> 5) & 1,
                   (status >> 4) & 1,
                   fifo_status & 1,
                   (config >> 1) & 1,
                   config & 1,
                   g_nrf24_state.rx_count, g_nrf24_state.error_count); */
        }

        /* TX_DS / MAX_RT */
        if (status & 0x20) { /* TX_DS */
            nrf24_write_reg(NRF24_REG_STATUS, 0x20);
        }
        if (status & 0x10) { /* MAX_RT */
            nrf24_write_reg(NRF24_REG_STATUS, 0x10);
            nrf24_flush_tx();
        }

        /* RX_DR: drain RX FIFO */
        if (status & 0x40) {
            do {
                uint8_t buf[NRF24_PAYLOAD_WIDTH];
                nrf24_read_rx_payload(buf, NRF24_PAYLOAD_WIDTH);

                float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
                float wx = 0.0f, wy = 0.0f, wz = 0.0f;
                bool valid = nrf24_parse_22b(buf, &roll, &pitch, &yaw, &wx, &wy, &wz);
                loop_frames++;

                if (valid && have_prev_wz) {
                    struct timespec ts_now_ast;
                    clock_gettime(CLOCK_MONOTONIC, &ts_now_ast);
                    float dt = (ts_now_ast.tv_sec - ts_prev_ast.tv_sec)
                             + (ts_now_ast.tv_nsec - ts_prev_ast.tv_nsec) / 1e9f;
                    if (dt <= 0.0f || dt > 0.05f) dt = 0.01f;  /* 异常保护，默认 10ms */

                    float ast = (wz - prev_wz) / dt;  /* 修正单位：deg/s^2 */
                    pthread_mutex_lock(&g_nrf24_state.mutex);
                    g_nrf24_state.gy_ast_buf[g_nrf24_state.gy_ast_idx] = ast;
                    g_nrf24_state.gy_ast_idx = (g_nrf24_state.gy_ast_idx + 1) % NRF24_AST_BUF_SIZE;
                    if (g_nrf24_state.gy_ast_count < NRF24_AST_BUF_SIZE)
                        g_nrf24_state.gy_ast_count++;
                    pthread_mutex_unlock(&g_nrf24_state.mutex);
                }
                if (valid) {
                    prev_wz = wz;
                    have_prev_wz = 1;
                    clock_gettime(CLOCK_MONOTONIC, &ts_prev_ast);
                }

                pthread_mutex_lock(&g_nrf24_state.mutex);
                memcpy(g_nrf24_state.rx_buffer, buf, NRF24_PAYLOAD_WIDTH);
                g_nrf24_state.data_ready = true;
                g_nrf24_state.rx_count++;
                if (valid) {
                    float store_roll = roll, store_pitch = pitch, store_yaw = yaw;

                    /* A-init：握手线程已发信号，收集 5 帧算平均 */
                    if (g_wait_a_init && !g_r_init_set) {
                        static float acc_roll = 0.0f, acc_pitch = 0.0f, acc_yaw = 0.0f;
                        static int acc_count = 0;

                        acc_roll  += roll;
                        acc_pitch += pitch;
                        acc_yaw   += yaw;
                        acc_count++;

                        if (acc_count >= 5) {
                            store_roll  = acc_roll  / 5.0f;
                            store_pitch = acc_pitch / 5.0f;
                            store_yaw   = acc_yaw   / 5.0f;
                            printf("[A-INIT] 5-frame avg captured: roll=%.2f pitch=%.2f yaw=%.2f\n",
                                   store_roll, store_pitch, store_yaw);
                            g_r_init_set = 1;
                            g_wait_a_init = 0;
                            acc_roll = acc_pitch = acc_yaw = 0.0f;
                            acc_count = 0;
                        }
                    }

                    g_nrf24_state.gy_roll  = store_roll;
                    g_nrf24_state.gy_pitch = store_pitch;
                    g_nrf24_state.gy_yaw   = store_yaw;
                    g_nrf24_state.gy_wx    = wx;
                    g_nrf24_state.gy_wy    = wy;
                    g_nrf24_state.gy_wz    = wz;
                    g_nrf24_state.imu_valid = true;
                    /* 记录角度历史 */
                    g_nrf24_state.gy_roll_hist[g_nrf24_state.gy_angle_idx] = store_roll;
                    g_nrf24_state.gy_pitch_hist[g_nrf24_state.gy_angle_idx] = store_pitch;
                    g_nrf24_state.gy_yaw_hist[g_nrf24_state.gy_angle_idx] = store_yaw;
                    g_nrf24_state.gy_angle_idx = (g_nrf24_state.gy_angle_idx + 1) % NRF24_ANGLE_HIST_SIZE;
                    if (g_nrf24_state.gy_angle_count < NRF24_ANGLE_HIST_SIZE)
                        g_nrf24_state.gy_angle_count++;
                    /* 记录 wy 历史 */
                    g_nrf24_state.gy_wy_hist[g_nrf24_state.gy_wy_idx] = wy;
                    g_nrf24_state.gy_wy_idx = (g_nrf24_state.gy_wy_idx + 1) % NRF24_WY_HIST_SIZE;
                    if (g_nrf24_state.gy_wy_count < NRF24_WY_HIST_SIZE)
                        g_nrf24_state.gy_wy_count++;
                    /* 记录 wz 历史 */
                    g_nrf24_state.gy_wz_hist[g_nrf24_state.gy_wz_idx] = wz;
                    g_nrf24_state.gy_wz_idx = (g_nrf24_state.gy_wz_idx + 1) % NRF24_WZ_HIST_SIZE;
                    if (g_nrf24_state.gy_wz_count < NRF24_WZ_HIST_SIZE)
                        g_nrf24_state.gy_wz_count++;
                    /* 记录 wx 历史 */
                    g_nrf24_state.gy_wx_hist[g_nrf24_state.gy_wx_idx] = wx;
                    g_nrf24_state.gy_wx_idx = (g_nrf24_state.gy_wx_idx + 1) % NRF24_WX_HIST_SIZE;
                    if (g_nrf24_state.gy_wx_count < NRF24_WX_HIST_SIZE)
                        g_nrf24_state.gy_wx_count++;

                    static int first_frame = 1;
                    if (first_frame) {
                        first_frame = 0;
                        printf("[NRF24] First valid frame received!\n");
                    }
                    print_cnt++;
                    if (print_cnt >= 10) {
                        print_cnt = 0;
                        printf("[NRF24] GYRO(%.2f,%.2f,%.2f) ANGLE(%.2f,%.2f,%.2f)\n",
                               wx, wy, wz, roll, pitch, yaw);
                    }
                } else {
                    g_nrf24_state.error_count++;
                    g_nrf24_state.imu_valid = false;
                }
                pthread_mutex_unlock(&g_nrf24_state.mutex);

                /* Check RX FIFO empty (FIFO_STATUS bit0 = RX_EMPTY) */
                uint8_t fifo_status = nrf24_read_reg(0x17);
                if (fifo_status & 0x01) break; /* RX_EMPTY = 1, done */
            } while (g_nrf24_running);

            /* Clear RX_DR after draining FIFO */
            nrf24_write_reg(NRF24_REG_STATUS, 0x40);
        }

        /* Rate print once per second */
        clock_gettime(CLOCK_MONOTONIC, &ts_now);
        double elapsed = (ts_now.tv_sec - ts_start.tv_sec)
                       + (ts_now.tv_nsec - ts_start.tv_nsec) / 1e9;
        if (elapsed >= 1.0) {
            /* printf("[NRF24] Rate: %u frames/sec  (valid=%u err=%u)\n",
                   loop_frames,
                   g_nrf24_state.rx_count, g_nrf24_state.error_count); */
            loop_frames = 0;
            ts_start = ts_now;
        }

        usleep(5000); /* 5ms poll interval */
    }

    /* printf("[NRF24] RX thread exiting\n"); */
    return NULL;
}

int nrf24_rx_thread_start(pthread_t* tid)
{
    g_nrf24_running = 1;
    if (pthread_create(tid, NULL, nrf24_rx_thread_func, NULL) != 0) {
        perror("pthread_create");
        g_nrf24_running = 0;
        return -1;
    }
    return 0;
}

void nrf24_rx_thread_stop(void)
{
    g_nrf24_running = 0;
}
