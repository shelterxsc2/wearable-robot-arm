#ifndef BLUETOOTH_SPP_H
#define BLUETOOTH_SPP_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 初始化蓝牙 BLE 客户端
 *  - 启用 hci0，解除 rfkill
 *  - 连接 D-Bus 系统总线
 * 返回 0 成功，-1 失败
 */
int bluetooth_spp_init(void);

/**
 * 启动蓝牙 BLE GATT 连接线程（阻塞直到首次连接成功）
 * 线程内循环: 连接 BT24 -> 轮询 ReadValue -> 解析指令 -> 调用 set_pose_mode()
 * 返回 0 成功，-1 失败
 */
int bluetooth_spp_start(void);

/**
 * 停止蓝牙监听线程并断开当前连接
 */
void bluetooth_spp_stop(void);

/**
 * 清理蓝牙资源：断开 BLE 连接、关闭 D-Bus、恢复 hci 状态
 */
void bluetooth_spp_cleanup(void);

#ifdef __cplusplus
}
#endif

#endif // BLUETOOTH_SPP_H
