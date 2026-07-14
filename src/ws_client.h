#ifndef WS_CLIENT_H
#define WS_CLIENT_H

#ifdef __cplusplus
extern "C" {
#endif

/* 连接 WebSocket 服务器，返回 socket fd，失败返回 -1 */
int ws_connect_url(const char *url);

/* 发送文本帧，sock 为 ws_connect_url 返回值 */
int ws_send_text(int sock, const char *text);

/* Non-blocking receive: >0 text length, 0 no complete frame, -1 disconnected. */
int ws_recv_text(int sock, char *text, int capacity);

/* 关闭连接 */
void ws_close(int sock);

#ifdef __cplusplus
}
#endif

#endif // WS_CLIENT_H
