/**
 * ws_client.cpp - 极简 WebSocket 客户端（仅文本发送，基于 socket + 自实现 Base64）
 */
#include "ws_client.h"
#include <cstdio>
#include <cstring>
#include <ctime>
#include <cstdlib>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <errno.h>

/* ---------- 自实现 Base64 ---------- */
static const char BASE64_CHARS[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static void my_base64_encode(const unsigned char *input, int length, char *output) {
    int i = 0, j = 0;
    unsigned char buf[3];
    while (length--) {
        buf[i++] = *input++;
        if (i == 3) {
            output[j++] = BASE64_CHARS[(buf[0] & 0xfc) >> 2];
            output[j++] = BASE64_CHARS[((buf[0] & 0x03) << 4) + ((buf[1] & 0xf0) >> 4)];
            output[j++] = BASE64_CHARS[((buf[1] & 0x0f) << 2) + ((buf[2] & 0xc0) >> 6)];
            output[j++] = BASE64_CHARS[buf[2] & 0x3f];
            i = 0;
        }
    }
    if (i > 0) {
        output[j++] = BASE64_CHARS[(buf[0] & 0xfc) >> 2];
        if (i == 1) {
            output[j++] = BASE64_CHARS[(buf[0] & 0x03) << 4];
            output[j++] = '=';
        } else {
            output[j++] = BASE64_CHARS[((buf[0] & 0x03) << 4) + ((buf[1] & 0xf0) >> 4)];
            output[j++] = BASE64_CHARS[(buf[1] & 0x0f) << 2];
        }
        output[j++] = '=';
    }
    output[j] = '\0';
}

static void generate_ws_key(char *key_out, int key_out_len) {
    unsigned char rand_bytes[16];
    for (int i = 0; i < 16; i++) rand_bytes[i] = (unsigned char)(rand() & 0xFF);
    my_base64_encode(rand_bytes, 16, key_out);
    (void)key_out_len;
}

/* ---------- WebSocket 连接（带详细日志） ---------- */
int ws_connect_url(const char *url) {
    printf("[WS-DEBUG] ws_connect_url: %s\n", url);

    if (strncmp(url, "ws://", 5) != 0) {
        printf("[WS-DEBUG] ERROR: URL must start with ws://\n");
        return -1;
    }
    const char *p = url + 5;
    const char *slash = strchr(p, '/');
    const char *colon = strchr(p, ':');

    char host[128] = "";
    int port = 80;
    char path[256] = "/";

    if (colon && (!slash || colon < slash)) {
        int hlen = (int)(colon - p);
        if (hlen >= (int)sizeof(host)) hlen = (int)sizeof(host) - 1;
        memcpy(host, p, hlen);
        host[hlen] = '\0';
        port = atoi(colon + 1);
        if (slash) {
            int plen = (int)strlen(slash);
            if (plen >= (int)sizeof(path)) plen = (int)sizeof(path) - 1;
            memcpy(path, slash, plen);
            path[plen] = '\0';
        }
    } else {
        if (slash) {
            int hlen = (int)(slash - p);
            if (hlen >= (int)sizeof(host)) hlen = (int)sizeof(host) - 1;
            memcpy(host, p, hlen);
            host[hlen] = '\0';
            int plen = (int)strlen(slash);
            if (plen >= (int)sizeof(path)) plen = (int)sizeof(path) - 1;
            memcpy(path, slash, plen);
            path[plen] = '\0';
        } else {
            strncpy(host, p, sizeof(host) - 1);
        }
    }

    printf("[WS-DEBUG] Parsed: host=%s port=%d path=%s\n", host, port, path);

    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        printf("[WS-DEBUG] ERROR: socket() failed: %s\n", strerror(errno));
        return -1;
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);

    struct hostent *he = gethostbyname(host);
    if (he && he->h_addr_list[0]) {
        memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
        printf("[WS-DEBUG] DNS resolved by gethostbyname\n");
    } else if (inet_pton(AF_INET, host, &addr.sin_addr) <= 0) {
        printf("[WS-DEBUG] ERROR: inet_pton failed for %s: %s\n", host, strerror(errno));
        close(sock);
        return -1;
    } else {
        printf("[WS-DEBUG] IP address parsed by inet_pton\n");
    }

    printf("[WS-DEBUG] Connecting to %s:%d ...\n", inet_ntoa(addr.sin_addr), port);
    if (connect(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        printf("[WS-DEBUG] ERROR: connect() failed: %s\n", strerror(errno));
        close(sock);
        return -1;
    }
    printf("[WS-DEBUG] TCP connected\n");

    char ws_key[64];
    generate_ws_key(ws_key, sizeof(ws_key));

    char request[1024];
    snprintf(request, sizeof(request),
        "GET %s HTTP/1.1\r\n"
        "Host: %s:%d\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Key: %s\r\n"
        "Sec-WebSocket-Version: 13\r\n"
        "\r\n",
        path, host, port, ws_key);

    printf("[WS-DEBUG] Sending HTTP upgrade request (%zu bytes)\n", strlen(request));
    if (send(sock, request, strlen(request), 0) < 0) {
        printf("[WS-DEBUG] ERROR: send() failed: %s\n", strerror(errno));
        close(sock);
        return -1;
    }

    char response[2048];
    int total = 0;
    while (total < (int)sizeof(response) - 1) {
        int n = (int)recv(sock, response + total, sizeof(response) - 1 - total, 0);
        if (n <= 0) {
            printf("[WS-DEBUG] ERROR: recv() returned %d: %s\n", n, strerror(errno));
            close(sock);
            return -1;
        }
        total += n;
        response[total] = '\0';
        if (strstr(response, "\r\n\r\n") != NULL) break;
    }

    printf("[WS-DEBUG] HTTP response (%d bytes):\n%.*s\n", total, total, response);

    if (strstr(response, "101") == NULL && strstr(response, "Switching Protocols") == NULL) {
        printf("[WS-DEBUG] ERROR: Handshake failed (no 101)\n");
        close(sock);
        return -1;
    }

    printf("[WS-DEBUG] WebSocket handshake OK\n");
    return sock;
}

/* ---------- 发送文本帧（带日志） ---------- */
int ws_send_text(int sock, const char *text) {
    if (sock < 0) return -1;
    size_t len = strlen(text);
    if (len > 2048) return -1;

    unsigned char frame[4096];
    size_t pos = 0;
    frame[pos++] = 0x81; // FIN=1, opcode=text

    if (len < 126) {
        frame[pos++] = (unsigned char)(0x80 | len);
    } else {
        frame[pos++] = 0xFE;
        frame[pos++] = (unsigned char)((len >> 8) & 0xFF);
        frame[pos++] = (unsigned char)(len & 0xFF);
    }

    unsigned char mask[4];
    mask[0] = (unsigned char)(rand() & 0xFF);
    mask[1] = (unsigned char)(rand() & 0xFF);
    mask[2] = (unsigned char)(rand() & 0xFF);
    mask[3] = (unsigned char)(rand() & 0xFF);
    memcpy(frame + pos, mask, 4);
    pos += 4;

    for (size_t i = 0; i < len; i++) {
        frame[pos++] = (unsigned char)(text[i] ^ mask[i % 4]);
    }

    ssize_t sent = send(sock, frame, pos, 0);
    if (sent != (ssize_t)pos) {
        printf("[WS-DEBUG] ERROR: send() returned %zd (expected %zu): %s\n", sent, pos, strerror(errno));
        return -1;
    }
    return 0;
}

int ws_recv_text(int sock, char *text, int capacity) {
    static int state_sock = -1;
    static unsigned char pending[8192];
    static size_t used = 0;
    if (sock < 0 || !text || capacity < 2) return -1;
    if (state_sock != sock) { state_sock = sock; used = 0; }

    if (used == sizeof(pending)) {
        used = 0;
        return -1;
    }

    ssize_t n = recv(sock, pending + used, sizeof(pending) - used, MSG_DONTWAIT);
    if (n == 0) return -1;
    if (n > 0) used += (size_t)n;
    else if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) return -1;
    if (used < 2) return 0;

    size_t pos = 2;
    unsigned int opcode = pending[0] & 0x0f;
    int masked = (pending[1] & 0x80) != 0;
    unsigned long long payload_len = pending[1] & 0x7f;
    if (payload_len == 126) {
        if (used < 4) return 0;
        payload_len = ((unsigned long long)pending[2] << 8) | pending[3];
        pos = 4;
    } else if (payload_len == 127) {
        if (used < 10) return 0;
        payload_len = 0;
        for (int i = 0; i < 8; ++i) payload_len = (payload_len << 8) | pending[2 + i];
        pos = 10;
    }
    unsigned char mask[4] = {0, 0, 0, 0};
    if (masked) {
        if (used < pos + 4) return 0;
        memcpy(mask, pending + pos, 4);
        pos += 4;
    }
    if (payload_len > sizeof(pending)) { used = 0; return -1; }
    if (used < pos + payload_len) return 0;
    size_t frame_len = pos + (size_t)payload_len;

    if (opcode == 0x8) { used = 0; return -1; }
    if (opcode != 0x1) {
        memmove(pending, pending + frame_len, used - frame_len);
        used -= frame_len;
        return 0;
    }
    size_t out_len = (size_t)payload_len;
    if (out_len >= (size_t)capacity) out_len = (size_t)capacity - 1;
    for (size_t i = 0; i < out_len; ++i) {
        unsigned char value = pending[pos + i];
        text[i] = (char)(masked ? (value ^ mask[i % 4]) : value);
    }
    text[out_len] = '\0';
    memmove(pending, pending + frame_len, used - frame_len);
    used -= frame_len;
    return (int)out_len;
}

void ws_close(int sock) {
    if (sock >= 0) close(sock);
}
