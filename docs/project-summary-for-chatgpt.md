# 可穿戴机械臂 NRF24 IMU 控制系统 —— 项目概述

## 一、项目目标

做一个**可穿戴三轴机械臂**，背在人体背部，末端搭载摄像头，通过**头戴/颈挂 IMU + NRF24 无线链路**实时跟踪人头姿态并控制机械臂运动，让摄像头始终正对人脸。

> 视觉闭环（PnP + PID）已 deprecated，当前活跃控制路径为 **NRF24 IMU 控制链路**。

---

## 二、硬件组成

| 组件 | 型号/规格 | 职责 |
|------|----------|------|
| 上位机 | RK3588 (ELF2 开发板) | NRF24 IMU 接收、8状态运动预测、目标位置生成、UART 通信、RTSP 推流 |
| 下位机 | STM32H723 (裸机 HAL) | 逆运动学、S-curve 速度规划、电机驱动、舵机控制 |
| 摄像头 | Realtek USB Camera (0bda:5858) | 1920×1080 MJPG 30fps，仅用于 RTSP 推流和画面显示 |
| 机械臂 | 可穿戴三轴 + 末端双舵机云台 | J1(Pan)、J2(俯仰)、J3(俯仰)、J4(俯仰舵机)、J5(水平舵机) |
| 无线 IMU | NRF24 + 陀螺仪 | 头戴/颈挂，发送 roll/pitch/yaw 和角速度 wx/wy/wz |
| 通信 | UART TTL `/dev/ttyS9` @ 115200 | 10 字节 raw int16 (x,y,z,k1,k2) |

---

## 三、当前软件架构

### 上位机 (C++, 单进程)

```
src/
  main.cpp          # 初始化 WiFi/NPU/RGA/UART/NRF24/RTSP，主循环
  rga_npu.cpp       # 视觉链路(deprecated) + NRF24 控制核心
  nrf24_linux.c/h   # NRF24 SPI 驱动 + IMU 数据解析 + 历史环形缓冲
  uart_comm.cpp/h   # UART 发送目标位置 (x,y,z,k1,k2)
  gst_rtsp.cpp/h    # GStreamer RTSP 推流
  wifi.cpp/h        # WiFi 连接
  bluetooth_spp.c/h # BLE（已禁用，bt_stub.c 空实现链接）
```

**活跃控制流（NRF24 IMU）**：
```
IMU (头部) → NRF24 无线 → RK3588 SPI
                              ↓
                    8状态运动状态机 + 终点预测器
                              ↓
                    目标位置 (tx,ty,tz) + 舵机角度 (k1,k2)
                              ↓
                    UART @ 5~7Hz → STM32 → S-curve → 电机
```

**视觉链路（仅推流/显示）**：
```
USB Camera → GStreamer (MJPG→NV12) → RGA → NPU (face_best.rknn)
                                              ↓
                                         solvePnP + 3D 绘制（仅显示）
                                              ↓
                                         RTSP 推流
```

### 下位机 (C, 裸机 HAL, Keil MDK)

- 接收 10 字节 raw 数据（x,y,z,k1,k2，小端 int16）
- `Coordinate_Inverse_Settlement(X, Y, Z, Servo_Angle, ...)` 做逆运动学
- **舵机角度参与逆运动学**：Servo1 改变前臂有效长度 `L2_v`
- S-curve 7 相速度规划（J1/J2/J3 独立规划）
- PWM 控制舵机 J4/J5
- **单向通信**：不往上位机回传关节角或末端位姿（当前）

---

## 四、当前 NRF24 控制逻辑（详细）

### 输入

- `gy_roll`：IMU 滚转角（对应头部俯仰运动）
- `gy_yaw`：IMU 偏航角（对应头部水平转动）
- `gy_wx`：滚转角速度
- `gy_wz`：偏航角速度

> IMU 的 roll/yaw/pitch 仅代表 IMU 自身欧拉角，与视觉坐标系/机械臂坐标系无关。

### Roll → Z 轴（机械臂升降）

```
baseline_roll = 开机标定值
delta_roll = current_roll - baseline_roll
cum_pitch_offset = delta_roll × π/180   （限幅 ±45°）
tz = 40 + 15 × sin(cum_pitch_offset)
ty = 67    （固定，= FACE_Y 10 + TRACK_DIST 57）
tx = 0     （固定）
```

- 抬头（roll 减小）→ `tz` 增大（机械臂上升）
- 低头（roll 增大）→ `tz` 减小（机械臂下降）

### Yaw → XY 平面（机械臂水平移动）

```
baseline_yaw = 开机标定值
delta_yaw = current_yaw - baseline_yaw
cum_yaw_offset = -delta_yaw × π/180   （限幅 ±90°）
tx = 0 + 57 × sin(cum_yaw_offset)
ty = 10 + 57 × cos(cum_yaw_offset)
```

### 8 状态运动状态机

| 状态 | 含义 |
|------|------|
| STATE_STOP_TO_ACCEL | 静止→加速 |
| STATE_ACCEL | 加速中 |
| STATE_ACCEL_TO_CONST | 加速→匀速（主窗口）|
| STATE_CONST_SPEED | 匀速 |
| STATE_CONST_TO_DECEL | 匀速→减速（修正窗口）|
| STATE_DECEL_TO_STOP | 减速→停止 |
| STATE_STOP | 停止 |
| STATE_DECEL_STOP_TO_ACCEL | 减速→停止→加速 |

### 终点预测器

```
pred_delta = w × dt × k
```

- `dt`：自适应预测窗口（250~500ms，速度越大窗口越短）
- `k`：状态调制系数（0.00~0.70，加速→匀速窗口最大）

### 发令策略

| 条件 | 间隔 |
|------|------|
| 加速→匀速 / 匀速→减速 窗口 | 150ms |
| 常规 / 停止对准 | 200ms |
| 预测目标变化阈值 | 5° |

---

## 五、已做的工作

1. **NRF24 IMU 链路打通**：SPI 驱动、22B 帧解析、角度/角速度提取、wx/wz 历史环形缓冲
2. **8 状态运动状态机**：完整实现 + 终点预测器 + 自适应发令策略
3. **Roll → Z 轴控制**：极性修正（roll 减小 = 抬头 = tz 增大），限幅 ±45°
4. **Yaw → XY 平面控制**：从 `imu-main-test` 恢复完整状态机代码，双轴并存
5. **UART 通信打通**：10 字节 raw int16，链路已通
6. **RTSP 推流**：MJPG 硬件解码 + H.264 编码，1080p30
7. **相机标定**：fx=689.58, fy=686.99, cx=982.87, cy=394.59，RMS=0.98px
8. **ty 修正**：从 10 → 80 → **67**（= 10 + 57，几何正对值）
9. **蓝牙禁用 + 编译修复**：`bt_stub.c` 空实现解决链接错误
10. **舵机控制设计**：方案 A（简单标定）/ 方案 B（正运动学校正）文档已完成

---

## 六、当前遇到的核心问题

### 1. wx 噪声
- 静止时 `wx` 仍有明显尖峰，可能误判运动状态

### 2. 5° 发令阈值导致初始延迟
- 小角度头部动作不会触发发令，需先晃过阈值

### 3. STM32 短距减速
- MCU 侧 `<3cm → 15%`、`<6cm → 20%`、`<15cm → 30%` 速度衰减，小位移响应极慢

### 4. 纯开环
- 上位机只发目标位置，**不知道机械臂实际在哪**
- 没有下位机回传当前关节角/末端位姿

### 5. UART 裸协议
- 10 字节 raw int16，无帧头/长度/CRC
- 单向通信，无反馈

### 6. 舵机与机械臂耦合
- J4（Servo1）参与逆运动学，改变 J4 会导致 J1/J2/J3 重新解算
- 不能独立调舵机不调机械臂

### 7. 视觉算力浪费
- 每帧仍跑完整 PnP 解算和 3D 绘制，但结果只用于显示

---

## 七、已尝试的调试方案

| 尝试 | 结果 |
|------|------|
| 纯视觉 PID（死区 6°，KP=0.9, KI=0.04）| **Deprecated**，振荡严重，PnP 噪声被放大 |
| 纯积分控制（CHASE_K=0.08） | 最稳定但响应慢，视觉链路已废弃 |
| NRF24 IMU roll → tz（原始参数） | 工作，但极性反了，已修正 |
| NRF24 预测器 k 值调低（0.70→0.35） | 噪声减少，响应更稳 |
| Roll + Yaw 双轴并存（或逻辑发令） | **当前版本**，编译通过，待上机验证 |

---

## 八、用户想做的事

### 短期（当前迭代）
1. **J4 舵机标定**：方案 A 简单映射，IMU_roll → Servo1，机械臂到位后调一次
2. **下位机传回 J1/J2/J3**：为后续正运动学校正做准备
3. **通信协议升级**：帧头+长度+CMD+payload+CRC8，双向通信

### 中期
1. **上位机闭环**：有常数 + 低频反馈后，做正运动学校正
2. **参数热加载**：`/tmp/pid.conf` 或 YAML
3. **CSV 记录**：按发令频率记录输入/输出

### 长期
1. **上位机 ROS2 化**：节点化，YAML 热加载
2. **下位机 FreeRTOS**：多任务，100Hz 轨迹插值
3. **仿真**：SolidWorks → URDF → PyBullet
4. **手眼标定**：Eye-in-Hand

---

## 九、问题

1. J4 舵机标定的 `K_SERVO` 大概范围是多少？（0.5~1.0？）
2. 下位机能否提供 6 个几何常数（L1, L2, L_connect, L_end, Offset_Upper, Offset_Fore）？
3. 当前 UART 115200 波特率，加反馈帧后是否足够？
4. 是否需要先让下位机支持"小变化不重置 S 曲线"？
