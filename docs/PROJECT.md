# ELF2 (RK3588) 可穿戴机械臂上位机系统

## 一、当前快照

当前分支为 `imu-victor-hat`，标签语义为 `imu-pnp-fuse1.0`。活跃主控路径是：

```
NRF24 无线 IMU → A-inverse 姿态解耦 → 8 状态运动预测
    → 球坐标目标 + J4/J5 舵机映射
    → UART 11 字节帧(flag=0x00/0x01) → STM32
```

视觉链路仍在运行：Body YOLO-Pose + Face Landmark 468 + 12 点 PnP + OSD/推流。与旧记录不同，PnP 结果现在已经接入 IMU 控制线程，用于静止态 yaw 零飘修正；但该修正仍是实验态。

## 二、硬件与通信

- **上位机**：RK3588 (ELF2)，负责 NRF24 接收、运动预测、NPU/RGA 推理、RTMP/RTSP 推流、WebSocket 心跳、端侧 HTTP API、UART 下发。
- **下位机**：STM32H723，负责逆运动学、S-curve 轨迹规划、电机和舵机驱动。
- **摄像头**：Realtek USB Camera (`/dev/video21`)，当前管线按 YUY2 1920x1080@30fps 处理。
- **无线 IMU**：NRF24L01+ + 陀螺仪，22B payload，包含角度帧和角速度帧。
- **UART**：`/dev/ttyS9` @ 115200，主控下发 11 字节裸帧：`x,y,z,k1,k2` 五个 int16 小端 + 1 字节 flag。
- **flag**：`0x00` 表示预测坐标，`0x01` 表示确定/静止态坐标。

## 三、已完成模块

### NRF24 IMU 控制链路

- `nrf24_linux.c/h`：spidev + sysfs GPIO 驱动，4MHz SPI，5ms 轮询 RX 状态，解析角度帧 `0x55 0x53` 和角速度帧 `0x55 0x52`。
- RX 线程维护 `wy/wz/wx` 历史、角度历史和 A-init 所需的 5 帧平均。
- `nrf24_control_update()` 由 GLib 50ms 定时器驱动，独立于视觉帧率。
- A-inverse 使用 `R_rel = R_current * R_init.t()`，避免简单欧拉角相减带来的初始安装姿态耦合。
- Pitch/Yaw 双轴各自运行状态机和终点预测器。
- 运动学已改为参数化球坐标：
  - 当前默认 profile `far_l3_55`：`l1=11.08`, `l2=6.92`, `l3=55`, `l4=28`, `k=1.6`
  - 保留旧 profile `mid_l3_40`：`l1=8`, `l2=5`, `l3=40`, `l4=28`, `k=1.6`，J4=55、抬头 -0.8、低头 -1.65、J5 yaw 0.4、position pitch sign=+1，用于后续远/中/近距离切换
  - `tx = l3*sin(yaw)*cos(pitch)`
  - `ty = l4 - l1*sin(pitch) + l3*cos(pitch)*cos(yaw)`
  - `tz = l2 + l1*cos(k*pitch) + l3*sin(k*pitch)*cos(yaw)`
- 舵机映射：
  - J4：`servo1 = 65 + K*delta_pitch`，抬头侧 `K=+0.5`，低头侧 `K=+1.65`，限幅 `[-90, 90]`
  - J5：`servo2 = 50 + 0.2*delta_yaw`，限幅 `[0, 270]`

### 握手和 UART

- 主程序等待 UART RX 文本 `init success`。
- 发送 `FF AA ...` 验证帧。
- `g_uart_block_tx=1` 屏蔽普通 UART 发送 7 秒，等待下位机归位。
- 7 秒后清除屏蔽，触发 A-init。
- NRF24 RX 线程收集 5 帧 IMU 平均值并置 `g_r_init_set=1`。
- 正常发令进入 `NORMAL`。
- 退出时发送 `AA FF ...` 结束帧。

### 视觉与 RuleEngine

- `best.rknn`：Body YOLO-Pose，640x640，输出 17 个 COCO keypoints。
- `face_landmark_468_fp16.rknn`：Face Landmark，192x192 ROI，输出 468 点。
- PnP 使用 12 个 FaceMesh 稳定点，带重投影误差、镜像解和旋转跳变过滤。
- 固定安装偏角通过 `R_mount = Ry(-14°) * Rx(-0.10rad)` 组合到 PnP 姿态。
- 根据机械臂目标 yaw 查 `PNP_YAW_CALIBRATION` 表并插值补偿。
- C++ 通过 Unix socket 调 Python `rule_engine_server.py`，协议为 312B 输入、56B 输出。

### 推流、云端和端侧控制

- 启动时探测 RTMP 服务器，连通则推 RTMP 并启动 WebSocket 心跳；不可达则回退本地 RTSP。
- WebSocket 注册帧为 `{"type":"frame_ts"}`，心跳周期 100ms，上报 timestamp、elapsed、frame_count、device。
- 端侧 HTTP API 监听 8080：
  - `/status`
  - `/mode?type=face/body`
  - `/calib?mode=0/1/2/3/4`
  - `/servo?k1=...&k2=...`
  - `/cmd?action=rebaseline/nrf24_reset`

## 四、编译命令

```bash
cd /home/elf/work/twice
g++ -std=c++17 -O2 \
  src/main.cpp src/rga_npu.cpp src/gst_rtsp.cpp src/gst_rtmp.cpp \
  src/stream_manager.cpp src/ctrl_server.cpp src/ws_client.cpp \
  src/uart_comm.cpp src/wifi.cpp \
  src/nrf24_linux.c src/imu2_i2c.c src/bluetooth_spp.c \
  -o build/cc \
  $(pkg-config --cflags --libs gstreamer-1.0 gstreamer-app-1.0 gstreamer-rtsp-server-1.0 dbus-1) \
  -I/usr/include/opencv4 -lopencv_core -lopencv_imgproc -lopencv_calib3d \
  -lrknnrt -lrga -lwpa_client -lpthread
```

## 五、当前问题

1. Body 模型仍是视觉链路主要瓶颈。
2. `wx` 静止噪声仍会影响状态判断。
3. 5° 发令阈值会让小角度动作延迟触发。
4. 下位机短距减速使小位移响应偏慢。
5. 上位机仍没有关节角/末端位姿反馈，本质上不是完整闭环。
6. A-init 随机姿态下俯仰极性仍需继续上机验证。
7. PnP 零飘修正当前是绝对值积分，`KI_PNP=0.05`，非误差积分。
8. PnP 当前 `pnp_valid_cnt >= 1` 即触发修正，未做连续多帧一致性过滤。
9. `/servo` 会直接发一次测试指令并写 `/tmp/servo_calib.txt`，但 mode=1/2 的周期标定发令仍使用固定舵机值，标定循环尚未完全闭合。
10. 云端离散状态控制与端侧连续坐标控制语义仍未统一。

## 六、下一步

- PnP 修正策略改为误差积分或带限幅的稳定校正。
- PnP 有效帧改为连续多帧一致性过滤。
- 补齐 `/servo` 与 mode=1/2 标定循环的实际联动。
- J4/J5 上机标定，确认极性、基线和机械限位。
- 下位机回传关节角或末端位姿，形成可观测闭环。
- 参数热加载和日志回放，减少反复编译调参。
