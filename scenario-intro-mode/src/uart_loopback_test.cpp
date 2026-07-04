// UART loopback test for /dev/ttyS9 @ 115200
// Build: g++ -std=c++17 -o build/uart_loopback_test src/uart_loopback_test.cpp
//
// Hardware setup: short TXD and RXD pins on the UART header before running.

#include <cstdio>
#include <cstring>
#include <cerrno>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string>
#include <vector>
#include <chrono>

static int uart_open(const char* dev) {
    int fd = open(dev, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("open");
    }
    return fd;
}

static bool uart_setup(int fd, int baud) {
    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        return false;
    }

    speed_t speed;
    switch (baud) {
        case 9600:   speed = B9600;   break;
        case 115200: speed = B115200; break;
        default:     speed = B115200; break;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                         // disable break processing
    tty.c_lflag = 0;                                // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;                            // read doesn't block
    tty.c_cc[VTIME] = 5;                            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);                // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);              // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        return false;
    }
    return true;
}

static bool send_all(int fd, const uint8_t* data, size_t len) {
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = write(fd, data + sent, len - sent);
        if (n < 0) {
            if (errno == EINTR) continue;
            perror("write");
            return false;
        }
        sent += (size_t)n;
    }
    tcdrain(fd);
    return true;
}

static std::vector<uint8_t> recv_all(int fd, size_t expect, int timeout_ms) {
    std::vector<uint8_t> buf;
    buf.reserve(expect);

    auto t0 = std::chrono::steady_clock::now();
    while (buf.size() < expect) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - t0).count();
        if (elapsed > timeout_ms) break;

        uint8_t tmp[256];
        int remaining = timeout_ms - (int)elapsed;
        struct timeval tv;
        tv.tv_sec  = remaining / 1000;
        tv.tv_usec = (remaining % 1000) * 1000;

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);

        int ret = select(fd + 1, &rfds, nullptr, nullptr, &tv);
        if (ret < 0) {
            if (errno == EINTR) continue;
            perror("select");
            break;
        }
        if (ret == 0) break;

        ssize_t n = read(fd, tmp, sizeof(tmp));
        if (n < 0) {
            if (errno == EINTR) continue;
            perror("read");
            break;
        }
        if (n == 0) continue;
        buf.insert(buf.end(), tmp, tmp + n);
    }
    return buf;
}

static void hexdump(const char* label, const std::vector<uint8_t>& data) {
    printf("%s (%zu bytes): ", label, data.size());
    for (auto b : data) {
        printf("%02X ", b);
    }
    printf("\n");
}

int main(int argc, char** argv) {
    const char* dev = (argc > 1) ? argv[1] : "/dev/ttyS9";
    int baud = 115200;

    printf("=== UART Loopback Test ===\n");
    printf("Device : %s\n", dev);
    printf("Baud   : %d\n", baud);
    printf("NOTE   : TXD and RXD must be shorted before running!\n\n");

    int fd = uart_open(dev);
    if (fd < 0) {
        printf("FAIL: cannot open %s\n", dev);
        return 1;
    }

    if (!uart_setup(fd, baud)) {
        printf("FAIL: cannot configure %s\n", dev);
        close(fd);
        return 1;
    }

    // Drain any stale data
    tcflush(fd, TCIOFLUSH);
    usleep(100000); // 100ms settle

    // Test patterns
    std::vector<std::vector<uint8_t>> patterns = {
        {0xAA, 0x55, 0x01, 0x02, 0x03, 0x04, 0x05},
        {0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF},
        {'H', 'e', 'l', 'l', 'o', ' ', 'U', 'A', 'R', 'T', '!', '\n'},
        {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0},
    };

    int pass = 0, fail = 0;

    for (size_t i = 0; i < patterns.size(); ++i) {
        const auto& tx = patterns[i];
        printf("--- Test %zu: send %zu bytes ---\n", i + 1, tx.size());

        if (!send_all(fd, tx.data(), tx.size())) {
            printf("  FAIL: send error\n");
            ++fail;
            continue;
        }

        // Give a little time for loopback
        usleep(50000);

        auto rx = recv_all(fd, tx.size(), 1000);
        hexdump("  TX", tx);
        hexdump("  RX", rx);

        if (rx.size() != tx.size()) {
            printf("  FAIL: length mismatch (expect %zu, got %zu)\n", tx.size(), rx.size());
            ++fail;
            continue;
        }

        if (memcmp(rx.data(), tx.data(), tx.size()) != 0) {
            printf("  FAIL: data mismatch\n");
            ++fail;
            continue;
        }

        printf("  PASS\n");
        ++pass;
    }

    // Stress test: send 1KB random-ish data
    printf("\n--- Stress test: 1024 bytes ---\n");
    std::vector<uint8_t> big(1024);
    for (size_t i = 0; i < big.size(); ++i) {
        big[i] = (uint8_t)(i & 0xFF);
    }
    if (send_all(fd, big.data(), big.size())) {
        usleep(200000);
        auto rx = recv_all(fd, big.size(), 2000);
        if (rx.size() == big.size() && memcmp(rx.data(), big.data(), big.size()) == 0) {
            printf("  PASS (1024/1024 bytes match)\n");
            ++pass;
        } else {
            printf("  FAIL (got %zu bytes, %zu match prefix)\n", rx.size(),
                   std::min(rx.size(), big.size()));
            ++fail;
        }
    } else {
        printf("  FAIL: send error\n");
        ++fail;
    }

    printf("\n=== Result: %d passed, %d failed ===\n", pass, fail);

    close(fd);
    return (fail > 0) ? 1 : 0;
}
