# 可穿戴机械臂上位机系统 — 当前项目概述

## 一、一句话概括

当前项目是在 RK3588 (ELF2) 上运行的可穿戴机械臂上位机：以 NRF24 无线 IMU 为主控输入，通过 A-inverse 姿态解耦、8 状态运动预测和 11 字节 UART 协议驱动 STM32；视觉链路负责推流、手势状态机，并已接入 PnP 静止态 yaw 零飘修正。

## 二、硬件组成

| 组件 | 型号/规格 | 职责 |
|------|----------|------|
| 上位机 | RK3588 (ELF2) | NRF24 接收、运动预测、NPU/RGA 推理、RTMP/RTSP 推流、WebSocket、HTTP API、UART 下发 |
| 下位机 | STM32H723 | 逆运动学、S-curve、电机/舵机驱动 |
| 摄像头 | Realtek USB Camera (`/dev/video21`) | 1920x1080@30fps，视觉推理和推流 |
| 无线 IMU | NRF24L01+ + 陀螺仪 | 发送 roll/pitch/yaw 和 wx/wy/wz |
| 机械臂 | 三轴 + J4/J5 双舵机云台 | J1/J2/J3 负责机械臂位置，J4/J5 负责末端云台 |
| 通信 | UART `/dev/ttyS9` @ 115200 | 11B raw frame: 5x int16 + 1x flag |

## 三、当前软件架构

```
src/
  main.cpp            # 启动 WiFi/RuleEngine/NPU/RGA/UART/HTTP/NRF24/IMU2/推流/WS
  rga_npu.cpp/h       # 视觉链路 + RuleEngine socket + NRF24 控制核心
  nrf24_linux.c/h     # NRF24 SPI 驱动、IMU 帧解析、历史缓冲、A-init 5 帧平均
  uart_comm.cpp/h     # UART 11 字节发送、RX 文本检测、诊断日志
  gst_rtmp.cpp/h      # RTMP 推流
  gst_rtsp.cpp/h      # RTSP 回退推流
  stream_manager.cpp  # RTMP 探测和 RTMP/RTSP 自动选择
  ctrl_server.cpp/h   # HTTP API @ 8080
  ws_client.cpp/h     # WebSocket 注册和 100ms 心跳
  wifi.cpp/h          # WiFi + DHCP/static fallback
```

## 四、主控链路

```
IMU → NRF24 → RK3588 SPI
        → angle/gyro 历史缓冲
        → A-inverse: R_rel = R_current * R_init.t()
        → Pitch/Yaw 双状态机 + EndpointPredictor
        → 球坐标 tx/ty/tz + J4/J5 舵机映射
        → UART 11B(flag=0x00/0x01)
        → STM32 → 电机/舵机
```

关键实现：

- 控制周期：50ms GLib 定时器。
- NRF24 RX：5ms 轮询，22B payload，角度和角速度帧顺序不固定。
- A-init：握手线程触发，RX 线程收集 5 帧平均后置 `g_r_init_set=1`。
- 发令阈值：Pitch/Yaw 均为 5°。
- 发令间隔：主窗口/第二窗口 150ms，普通/静止态 200ms。
- 位置死区：tx/ty/tz 任一变化小于 1cm 时不发。

## 五、运动学和舵机

当前球坐标参数：

```cpp
l1 = 8, l2 = 5, l3 = 40, l4 = 28, k = 1.6
tx = l3 * sin(yaw) * cos(pitch)
ty = l4 - l1 * sin(pitch) + l3 * cos(pitch) * cos(yaw)
tz = l2 + l1 * cos(k*pitch) + l3 * sin(k*pitch) * cos(yaw)
```

舵机映射：

- J4/俯仰：`servo1 = 55 + K * delta_pitch`，抬头侧 `K=-0.8`，低头侧 `K=-1.65`，限幅 `[-90, 90]`
- J5/水平：`servo2 = 50 + 0.4 * delta_yaw`，限幅 `[0, 270]`
- 预测帧 `flag=0x00` 时，坐标使用预测值，同时发送当前计算出的 J4/J5 舵机角，便于现场测试俯仰和偏航跟随。

## 六、视觉链路

```
USB Camera → GStreamer → NV12
    → Body YOLO-Pose(best.rknn, 640x640)
    → ROI → Face Landmark 468(face_landmark_468_fp16.rknn, 192x192)
    → 12 点 PnP + OSD
    → RuleEngine v2 socket
    → RTMP/RTSP 推流
```

PnP 当前状态：

- 使用 468 点中的 12 个稳定点构造 PnP。
- 过滤重投影误差大于 25px、镜像解、旋转跳变大于 60°的帧。
- OneEuroFilter 当前关闭。
- 固定安装偏角使用 `R_mount = Ry(-14°) * Rx(-0.10rad)`。
- 按机械臂目标 yaw 查 `PNP_YAW_CALIBRATION` 表做插值补偿。
- 静止态把 `corrected_hy_deg` 写入共享状态，供 IMU 控制线程更新 `R_bias_total`。

RuleEngine v2：

- Python 服务：`scripts/rule_engine_server.py`
- Socket：`/tmp/rule_engine.sock`
- 输入：312B，`44f + 20f + 7q`
- 输出：56B，`7q`
- 20 点 = COCO 17 点 + Face 3 点，C++ 端做左右点 Observer 视角交换。

## 七、握手和通信

握手时序：

```
UART RX "init success"
  → 发送 FF AA 验证帧
  → g_uart_block_tx=1，屏蔽普通 UART 发送 7s
  → g_uart_block_tx=0
  → g_wait_a_init=1
  → NRF24 RX 线程采 5 帧平均并置 g_r_init_set=1
  → NORMAL
```

UART TX：

```
[x][y][z][k1][k2][flag]
```

- 前 10 字节：5 个 int16 小端。
- 第 11 字节：`0x00` 预测，`0x01` 确定。
- `[UART-TX]` 通过 `diag_log()` 同时写终端和 `/tmp/uart_complete_diag.log`。
- `[UART-RX]` 原始数据写 `/tmp/cmd`，终端打印大多已注释。
- RX 文本检测 `init success`、`move_success`。

## 八、推流和云端

- 启动时 `probe_rtmp_server()` 探测 `rtmp://47.93.162.124:1935/live/device-003`。
- RTMP 可达：启动 WebSocket，等待注册确认后推 RTMP。
- RTMP 不可达：回退本地 RTSP。
- WebSocket 地址：`ws://47.93.162.124/ws?deviceId=device-003`
- 注册帧：`{"type":"frame_ts"}`
- 心跳：100ms，带 timestamp、elapsed、frame_count、device。

## 九、端侧 HTTP API

- `GET /status`：pose_mode、stream_type、uart_move_complete、head_stationary、arm_stable
- `POST /mode?type=face/body`
- `POST /calib?mode=0/1/2/3`
- `POST /servo?k1=...&k2=...`：直接发一次测试指令，并写 `/tmp/servo_calib.txt`
- `POST /cmd?action=rebaseline/nrf24_reset`

注意：`/servo` 与 mode=1/2 标定循环当前未完全闭合；周期标定发令仍发送固定 `NRF_SERVO1_DEG/NRF_SERVO2_DEG`。

## 十、已知问题

1. Body 模型耗时占 AI 总耗时主要部分。
2. `wx` 静止噪声仍可能误判运动。
3. 5° 发令阈值导致小角度动作延迟。
4. STM32 短距减速导致小位移跟随慢。
5. 上位机没有关节角/末端位姿反馈，仍是开环为主。
6. A-init 姿态变化下俯仰极性仍需验证。
7. PnP 修正使用绝对值积分，`KI_PNP=0.05`，存在持续累计风险。
8. PnP `pnp_valid_cnt >= 1` 即触发，未做连续多帧一致性过滤。
9. 云端离散控制和端侧连续坐标控制协议尚未统一。

## 十一、下一步

- PnP 修正改为更稳定的误差积分/限幅策略。
- PnP 连续多帧一致性过滤。
- 补齐 `/servo` 与标定模式周期发令联动。
- J4/J5 上机标定。
- 下位机回传关节角或末端位姿。
- 参数热加载、CSV 记录和离线回放。
