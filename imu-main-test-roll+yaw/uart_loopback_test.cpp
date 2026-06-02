/**
 * uart_loopback_test.cpp - UART 回环测试
 * 用法: 短接 UART 的 TX 和 RX 引脚，然后运行本程序
 *       ./uart_test /dev/ttyS9
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <poll.h>

static int uart_open(const char* device, int baudrate)
{
    int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        fprintf(stderr, "[TEST] open %s failed: %s\n", device, strerror(errno));
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        fprintf(stderr, "[TEST] tcgetattr failed: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    speed_t bd;
    switch (baudrate) {
        case 9600:    bd = B9600;    break;
        case 115200:  bd = B115200;  break;
        case 921600:  bd = B921600;  break;
        default:      bd = B115200;  break;
    }
    cfsetospeed(&tty, bd);
    cfsetispeed(&tty, bd);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "[TEST] tcsetattr failed: %s\n", strerror(errno));
        close(fd);
        return -1;
    }
    tcflush(fd, TCIOFLUSH);
    return fd;
}

static int uart_recv(int fd, char* buf, int max_len, int timeout_ms)
{
    struct pollfd pfd = { fd, POLLIN, 0 };
    int ret = poll(&pfd, 1, timeout_ms);
    if (ret <= 0) return ret;
    ssize_t r = read(fd, buf, max_len);
    return (int)r;
}

int main(int argc, char* argv[])
{
    const char* device = (argc > 1) ? argv[1] : "/dev/ttyS9";
    int baud = (argc > 2) ? atoi(argv[2]) : 115200;

    printf("========================================\n");
    printf("UART Loopback Test\n");
    printf("Device: %s @ %d baud\n", device, baud);
    printf("请先将 %s 的 TX 和 RX 短接！\n", device);
    printf("========================================\n\n");

    int fd = uart_open(device, baud);
    if (fd < 0) return 1;
    printf("[TEST] %s opened OK\n\n", device);

    const char* test_msgs[] = {
        "Hello UART",
        "60,55,10,15,-20",
        "ABC123!@#"
    };
    int pass = 0, fail = 0;

    for (int i = 0; i < 3; i++) {
        const char* msg = test_msgs[i];
        int msg_len = strlen(msg);

        // 先清空接收缓冲区
        char discard[256];
        while (uart_recv(fd, discard, sizeof(discard), 50) > 0);

        // 发送
        ssize_t w = write(fd, msg, msg_len);
        tcdrain(fd);
        usleep(20000); // 20ms 等待传输

        // 接收
        char rx[256];
        int n = uart_recv(fd, rx, sizeof(rx), 200);

        printf("[TEST] Send: '%s' (%d bytes)\n", msg, msg_len);
        if (n == msg_len && memcmp(rx, msg, msg_len) == 0) {
            printf("[TEST] Recv: '%.*s' -> PASS\n", n, rx);
            pass++;
        } else if (n > 0) {
            printf("[TEST] Recv: '%.*s' (%d bytes) -> FAIL (mismatch)\n", n, rx, n);
            fail++;
        } else {
            printf("[TEST] Recv: NOTHING -> FAIL (no data)\n");
            fail++;
        }
        printf("\n");
        usleep(100000);
    }

    printf("========================================\n");
    printf("Result: %d PASS, %d FAIL\n", pass, fail);
    if (fail == 0) {
        printf("UART 回环测试通过，硬件正常！\n");
    } else {
        printf("UART 回环测试失败，请检查:\n");
        printf("  1. TX 和 RX 是否已短接\n");
        printf("  2. 引脚是否正确\n");
        printf("  3. 设备节点是否存在 (%s)\n", device);
    }
    printf("========================================\n");

    close(fd);
    return fail > 0 ? 1 : 0;
}
