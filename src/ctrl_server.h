#ifndef CTRL_SERVER_H
#define CTRL_SERVER_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 启动端侧 HTTP 控制服务器（在独立线程中运行）
 * @param port 监听端口，如 8080
 * @return 0 成功，-1 失败
 */
int ctrl_server_start(int port);

/**
 * @brief 停止控制服务器
 */
void ctrl_server_stop(void);

#ifdef __cplusplus
}
#endif

#endif // CTRL_SERVER_H
