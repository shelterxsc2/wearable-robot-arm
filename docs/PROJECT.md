# ELF2 (RK3588) 可穿戴机械臂上位机系统

## 一、项目概述

本项目是一个运行在 **RK3588 (ELF2 开发板)** 上的可穿戴机械臂控制系统。机械臂背在人体背部，末端搭载摄像头。当前**活跃控制路径为 NRF24 无线 IMU**（头戴/颈挂陀螺仪），视觉链路仅用于 RTSP 推流和画面显示。

### 硬件组成
- **上位机**: RK3588 (ELF2)，负责 NRF24 IMU 接收、运动预测、目标位置生成、UART 通信、RTSP 推流
- **无线 IMU**: NRF24 + 陀螺仪，~100Hz 发送 roll/pitch/yaw + wx/wy/wz
- **摄像头**: Realtek USB Camera3 (`/dev/video21`)，MJPG 1920×1080@30fps
- **机械臂**: 可穿戴三轴 + 末端双舵机云台
  - J1: 腰部 Pan（360°）
  - J2: 大臂俯仰
  - J3: 小臂俯仰
  - J4: 俯仰舵机（与摄像头直连）
  - J5: 水平舵机
- **通信**: UART TTL `/dev/ttyS9` @ 115200（10 字节 raw int16）

### 下位机职责
- 逆运动学（`Coordinate_Inverse_Settlement`）
- S-curve 7 相速度规划
- 电机驱动 + 舵机 PWM 控制
- **Servo1 参与逆运动学**：改变前臂有效长度

---

## 二、当前已完成的工作

### 1. NRF24 IMU 控制链路（活跃）
- **NRF24 SPI 驱动**: `nrf24_linux.c`，spidev + sysfs GPIO，4MHz，22B 帧解析
- **IMU 数据解析**: 角度帧（0x55 0x53）+ 陀螺仪帧（0x55 0x52）
- **历史环形缓冲**: `wx_hist` / `wz_hist`，10ms 分辨率，供 50ms 控制周期分析
- **8 状态运动状态机**: `MotionContext` + `next_motion_state()`，完整实现
- **终点预测器**: 自适应 `dt`（250~500ms）+ 状态调制 `k`（0.00~0.70）
- **Roll → Z 轴**: `tz = 40 + 15 * sin(delta_roll * π/180)`，限幅 ±45°
- **Yaw → XY 平面**: `tx = 57*sin(cum_yaw)`, `ty = 10 + 57*cos(cum_yaw)`
- **双轴并存**: Roll 和 Yaw 各跑独立状态机，发令逻辑为"或"合并

### 2. 视觉链路（仅显示）
- **视频采集**: GStreamer `v4l2src` MJPG 1920×1080@30fps
- **格式转换**: `mppjpegdec` 硬件解码 → NV12
- **RTSP 推流**: 1080p30 H.264 硬件编码
- **AI 推理**: RKNN `face_best.rknn`，6 点关键点
- **PnP + 3D 绘制**: `solvePnP` + 立方体框 + 坐标轴（仅显示，不发令）
- **OneEuroFilter**: 已禁用（`PNP_USE_FILTER 0`）

### 3. 通信与控制
- **UART 串口**: `/dev/ttyS9` @ 115200，10 字节 raw int16 (x,y,z,k1,k2)
- **蓝牙 BLE SPP**: 已禁用，`bt_stub.c` 空实现链接通过
- **WiFi 连接**: 自动连接预设 WiFi，DHCP + 静态 IP fallback

---

## 三、文件结构

```
twice/
├── src/
│   ├── main.cpp              # 主程序：初始化 + NRF24 定时器 + RTSP 主循环
│   ├── rga_npu.cpp/h         # 视觉链路(deprecated) + NRF24 控制核心
│   ├── nrf24_linux.c/h       # NRF24 SPI 驱动 + IMU 解析 + 历史缓冲
│   ├── uart_comm.cpp/h       # UART 驱动，10 字节 raw int16
│   ├── gst_rtsp.cpp/h        # GStreamer RTSP 服务器
│   ├── gst_rtmp.cpp/h        # GStreamer RTMP 推流（备用）
│   ├── wifi.cpp/h            # WiFi 连接
│   ├── bluetooth_spp.c/h     # BLE（已禁用）
│   ├── ws_client.cpp/h       # WebSocket 客户端
│   └── uart_loopback_test.cpp
├── models/
│   ├── best.rknn             # 人体姿态模型
│   └── face_best.rknn        # 人脸模型（6 点 PnP）
├── calib/
│   ├── camera_matrix.npy/txt # 相机内参
│   └── dist_coeffs.npy/txt   # 畸变系数
├── docs/
│   ├── PROJECT.md            # 本文件
│   ├── project-summary-for-chatgpt.md
│   ├── motion-control-framework.md
│   ├── servo-control-design.md      # 舵机控制方案 A/B
│   └── servo-calibration-guide.md   # J4 标定手册
└── wearable-robot-arm/       # Git 仓库：上下位机完整项目
    └── imu-main-test-roll+yaw/      # 当前版本存档
```

---

## 四、编译命令（当前版本）

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

## 五、关键坐标系定义

### 人脸坐标系（PnP 3D 模板）
- **原点**: 鼻尖附近
- **+X**: 人脸左侧
- **+Y**: 人脸下方
- **+Z**: 人脸后方（远离相机）

### IMU 坐标系（重要）
- `gy_roll/yaw/pitch` 仅代表 **IMU 自身欧拉角**
- 与视觉坐标系/机械臂坐标系**无直接对应**
- 当前代码中 `roll` 对应头部俯仰效果，`yaw` 对应水平转动效果

### 机械臂基座坐标系
- **原点**: 机械臂 J1 关节中心
- **+Z**: 竖直向上

---

## 六、改动记录

### 2026-05-28 当前版本

**状态**: Roll + Yaw 双轴控制，J4 舵机标定准备中

| 改动 | 说明 |
|------|------|
| Yaw 控制恢复 | 从 `imu-main-test` 恢复完整 yaw 状态机，与 roll 并存 |
| 发令逻辑改"或" | `should_cmd = roll_should_cmd \|\| yaw_should_cmd` |
| ty 固定为 67 | `10 + 57`，几何正对值 |
| roll 极性修正 | `cum_pitch_offset = delta_roll * π/180`（去掉负号） |
| 预测器 k 值调低 | `STATE_ACCEL` / `STATE_STOP_TO_ACCEL` 从 0.70 → **0.35** |
| 蓝牙编译修复 | `/tmp/bt_stub.c` 空实现 |
| GitHub 存档 | `wearable-robot-arm/imu-main-test-roll+yaw/` |
| 舵机设计方案 | `docs/servo-control-design.md`（方案 A/B） |
| 标定手册 | `docs/servo-calibration-guide.md` |

### 2026-05-20~21（历史记录：视觉 PID 调试，已废弃）

> 视觉闭环控制已 deprecated，以下记录保留供参考：

- 偏航 PID 调参（KP=0.9, KI=0.04, 死区 6°）
- 纯积分 + 速度前馈 + 冻结机制
- 俯仰控制完全禁用
- UART 发送周期 3Hz

---

## 七、关键问题与待办

### P0：J4 舵机标定（当前）
- [ ] 上机实测 `baseline_roll`、`baseline_servo1`、`K_SERVO`
- [ ] 验证低头/抬头双向补偿效果
- [ ] 确认 J4 机械限位

### P1：通信升级
- [ ] 下位机回传 `complete` 信号（文本或二进制帧）
- [ ] 可选：下位机回传 J1/J2/J3 目标值，为上位机正运动学做准备
- [ ] UART 协议升级：帧头+长度+CMD+payload+CRC8

### P2：下位机配合
- [ ] 提供 6 个几何常数（L1, L2, L_connect, L_end, Offset_Upper, Offset_Fore）
- [ ] 讨论"小变化不重置 S 曲线"的可行性

### P3：长期
- [ ] 上位机闭环（有反馈后做正运动学校正）
- [ ] 参数热加载
- [ ] CSV 记录与离线回放
- [ ] ROS2 化 / 仿真

---

## 八、外部建议摘要

### ChatGPT 技术评审核心观点

1. **当前架构最不合适的地方**：上位机在做"看起来像闭环、实际像开环"的控制，没有关节角/末端位姿反馈
2. **控制频率分层不对**：NRF24 采样 100Hz → 状态机 20Hz → UART 5~7Hz → 下位机 1ms S 曲线
3. **缺少离线调试闭环**：应先录数据 → 离线 replay → 改参数不编译 → 再上机
4. **运动生硬**：上位机限速 + 下位机轨迹规划，两者都要做
5. **如果末端有双舵机云台**：云台负责快速跟脸，机械臂负责慢速把云台带回中位

---

> **最重要的一句话**：当前最该追求的不是"更复杂的控制算法"，而是**把系统改成可观测、可回放、分频率、分职责、安全限幅的结构**。在那之前，控制算法再精妙也会像在雾里开车。
