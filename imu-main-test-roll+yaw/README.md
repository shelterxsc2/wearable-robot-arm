# imu-main-test-roll+yaw

**RK3588 上位机代码 — NRF24 IMU 控制链路（滚转驱动俯仰）**

---

## 版本说明

本文件夹保存的是 **2025-05-28 版本** 的可穿戴机械臂 RK3588 上位机源代码。

相比早期视觉闭环版本，本版已将**活跃控制路径完全切换为 NRF24 无线 IMU**，视觉链路仅保留 RTSP 推流和 3D 姿态显示，不再参与发令。

> 偏航（yaw）控制代码已注释保留，当前仅启用滚转（roll）驱动 Z 轴俯仰。

---

## 硬件拓扑

| 组件 | 型号/规格 | 职责 |
|------|----------|------|
| 上位机 | RK3588 (ELF2) | NRF24 IMU 接收 → 状态机/预测 → UART 发令；视觉推理 + RTSP 推流 |
| 下位机 | STM32H723 | UART 接收 → 逆运动学 → S-curve 速度规划 → 电机驱动 |
| 摄像头 | Realtek USB (0bda:5858) | 1920×1080 MJPG 30fps，仅用于推流和显示 |
| 无线 IMU | NRF24 + 陀螺仪 | 头戴/颈挂，发送 roll/pitch/yaw 和角速度 wx/wy/wz |
| 通信 | UART TTL `/dev/ttyS9` | 115200 baud，10 字节裸 int16 (x, y, z, k1, k2) |

---

## 控制策略

### 活跃链路：roll → Z 轴

1. **输入**：IMU `gy_roll`（滚转角）+ `gy_wx`（滚转角速度）
2. **状态机**：8 状态运动 FSM（停止→加速→匀速→减速→停止及过渡态）
3. **终点预测**：`pred_delta = wx * dt * k`，按速度和状态自适应 dt/k
4. **发令条件**：
   - 采样周期 50ms
   - 发令间隔 150ms（关键窗口）/ 200ms（常规）
   - 预测目标变化 > 5° 才发令
5. **输出映射**：
   ```
   delta_roll = target_roll - baseline_roll
   cum_pitch_offset = -delta_roll * π/180   （限幅 ±45°）
   tz = 40 + 15 * sin(cum_pitch_offset)
   ty = 67    （固定，= FACE_Y 10 + TRACK_DIST 57）
   tx = 0     （固定）
   ```
   - 抬头（roll 减小）→ `cum_pitch_offset` 正 → `tz` 增大（机械臂上升）
   - 低头（roll 增大）→ `cum_pitch_offset` 负 → `tz` 减小（机械臂下降）

### 偏航控制（已注释保留）

原 `wz` 驱动的 yaw → XY 平面运动逻辑已整体注释，如需恢复可在 `nrf24_control_update()` 中取消注释 yaw 控制段。

---

## 关键参数

| 参数 | 值 | 说明 |
|------|-----|------|
| `NRF_FACE_X_CM` | 0 | 人脸 X 坐标（固定）|
| `NRF_FACE_Y_CM` | 10 | 人脸 Y 坐标（几何参考）|
| `NRF_FACE_Z_CM` | 40 | 人脸 Z 基准高度 |
| `ty` 固定值 | 67 | `10 + 57`，正对脸时的跟踪距离 |
| Z 轴振幅 | 15 | `tz` 最大偏移量（±45° 时 ±10.6cm）|
| 发令阈值 | 5° | `CMD_ROLL_THRESHOLD_DEG` |
| 状态机窗口 | 5 帧 ≈ 50ms | `wx` 历史分析窗口 |

---

## 编译

```bash
cd /home/elf/work/twice
g++ -std=c++17 -O2 -Isrc \
  src/main.cpp src/rga_npu.cpp src/gst_rtsp.cpp src/gst_rtmp.cpp \
  src/uart_comm.cpp src/wifi.cpp src/nrf24_linux.c /tmp/bt_stub.c \
  -o build/cc \
  $(pkg-config --cflags --libs opencv4 gstreamer-1.0 gstreamer-app-1.0 gstreamer-rtsp-server-1.0) \
  /usr/lib/aarch64-linux-gnu/libwpa_client.a \
  -lrknnrt -lrga -lpthread -lm -ldl
```

> `/tmp/bt_stub.c` 为蓝牙空实现，用于解决蓝牙禁用后的链接错误。

---

## 文件清单

| 文件 | 说明 |
|------|------|
| `main.cpp` | 主入口：CPU 调频、WiFi、NPU/RGA、UART、NRF24、RTSP |
| `rga_npu.cpp/h` | 视觉链路 + NRF24 控制核心（8 状态机、终点预测、PnP）|
| `nrf24_linux.c/h` | NRF24 SPI 驱动 + IMU 数据解析 + 历史环形缓冲 |
| `uart_comm.cpp/h` | UART 驱动，10 字节裸 int16 发送 |
| `gst_rtsp.cpp/h` | GStreamer RTSP 推流 |
| `gst_rtmp.cpp/h` | GStreamer RTMP 推流 |
| `wifi.cpp/h` | WiFi 连接管理 |
| `bluetooth_spp.c/h` | 蓝牙 SPP（已禁用，保留接口）|
| `ws_client.cpp/h` | WebSocket 客户端（保留）|
| `uart_loopback_test.cpp` | UART 回环测试 |

---

## 已知问题

1. **wx 噪声**：静止时 `wx` 仍有尖峰，可能误判运动状态
2. **5° 发令阈值**：小角度动作有初始延迟
3. **STM32 短距减速**：MCU 侧 `<3cm → 15%` 速度，小位移响应慢
4. **纯开环**：无机械臂实际位姿回传
5. **UART 裸协议**：无帧头/CRC，存在错包风险
6. **视觉算力浪费**：每帧仍跑完整 PnP，但结果仅用于显示

---

## 作者

shelterxsc2
