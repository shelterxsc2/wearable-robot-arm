# 可穿戴机械臂 — NRF24 IMU 控制 + RTSP 推流

> **分支**: `imu-rtsp-main`  
> **核心路径**: NRF24 无线 IMU → 8 状态运动状态机 → UART → STM32  
> **视觉链路**: 仅用于 RTSP 推流与画面显示，不参与控制

---

## 硬件组成

| 组件 | 型号/职责 |
|------|----------|
| 上位机 | RK3588 (ELF2) — IMU 接收、运动预测、RTSP 推流 |
| 下位机 | STM32H723 — 逆运动学、S-curve、电机/舵机驱动 |
| 摄像头 | Realtek USB Camera (0bda:5858) — 1920×1080@30fps MJPG |
| 无线 IMU | NRF24 + 陀螺仪 — 头戴/颈挂，发送 roll/yaw/pitch + wx/wy/wz |
| 机械臂 | 可穿戴三轴 + 末端双舵机云台 (J1~J3 + J4/J5) |

---

## 数据流

```
IMU (头部) → NRF24 无线 → RK3588 SPI
                              ↓
                    8 状态运动状态机 + 终点预测器
                              ↓
                    目标位姿 (tx,ty,tz) + 舵机 (k1,k2)
                              ↓
                    UART 115200 @ 5~7Hz → STM32 → 电机

USB Camera → GStreamer → RGA → NPU (face_best.rknn)
                              ↓
                         PnP + 3D 绘制（仅显示）
                              ↓
                         RTSP 推流 (rtsp://<IP>:8554/stream)
```

---

## 当前控制逻辑

### 输入
- `gy_roll` / `gy_yaw`：IMU 欧拉角
- `gy_wx` / `gy_wz`：角速度

### Roll → Z 轴（机械臂升降）
```
tz = 40 + 15 × sin(Δroll)    // 限幅 ±45°
ty = 67  (固定)
tx = 0   (固定)
```

### Yaw → XY 平面（机械臂水平移动）
```
tx = 57 × sin(Δyaw)
ty = 10 + 57 × cos(Δyaw)     // 限幅 ±90°
```

### 8 状态运动状态机 + 终点预测器
- `dt` 自适应：250~500ms（速度越大越短）
- `k` 状态调制：0.00 ~ 0.70（加速→匀速窗口最大）
- 发令频率：主窗口 150ms，常规/停止 200ms
- 阈值：5°

### 当前舵机配置
- `k1 = 50` (J5 水平舵机)
- `k2 = 145` (J4 俯仰舵机)

---

## 编译

```bash
cd /home/elf/work/twice  # 或你的项目路径

g++ -std=c++17 -O2 -Isrc \
  src/main.cpp src/rga_npu.cpp src/gst_rtsp.cpp src/gst_rtmp.cpp \
  src/uart_comm.cpp src/wifi.cpp \
  src/nrf24_linux.c /tmp/bt_stub.c \
  -o build/cc \
  $(pkg-config --cflags --libs opencv4 gstreamer-1.0 gstreamer-app-1.0 gstreamer-rtsp-server-1.0) \
  /usr/lib/aarch64-linux-gnu/libwpa_client.a \
  -lrknnrt -lrga -lpthread -lm -ldl
```

---

## 运行

```bash
sudo ./build/cc
```

启动后等待：
```
[RTSP] Server running on port 8554
[RTSP] Stream URL: rtsp://<IP>:8554/stream
```

然后用 VLC / ffplay 拉流：
```bash
# 强制 TCP（推荐）
ffplay -rtsp_transport tcp rtsp://10.104.247.114:8554/stream
```

---

## 文件结构

```
src/
  main.cpp          # 初始化 WiFi/NPU/RGA/UART/NRF24/RTSP，主循环
  rga_npu.cpp       # 视觉链路(deprecated) + NRF24 控制核心
  nrf24_linux.c/h   # NRF24 SPI 驱动 + IMU 帧解析 + 历史缓冲
  uart_comm.cpp/h   # UART 通信（10 字节 raw int16）
  gst_rtsp.cpp/h    # GStreamer RTSP 推流
  wifi.cpp/h        # WiFi 连接
docs/               # 项目文档（控制设计、标定指南、调试日志）
scripts/            # 标定脚本
calib/              # 相机标定参数
```

---

## 已知问题

1. **wx 噪声**：静止时 wx 仍有尖峰，可能误判运动状态
2. **5° 阈值延迟**：小角度头部动作不触发发令
3. **STM32 短距减速**：小位移响应极慢
4. **纯开环**：上位机不知道机械臂实际位置
5. **视觉算力浪费**：每帧仍跑完整 PnP + 3D 绘制，仅用于显示

---

## 下一步

- J4 舵机标定（方案 A：简单映射）
- 下位机回传关节角 / 末端位姿
- 通信协议升级（帧头 + CRC + 双向）
