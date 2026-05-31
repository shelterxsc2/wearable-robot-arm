# 可穿戴机械臂 — RTMP 云端推流 + 端侧控制 + NRF24 IMU 控制

> **分支**: `imu-rtm/sp-main`  
> **核心路径**: NRF24 无线 IMU → 8 状态运动状态机 → UART → STM32  
> **视觉链路**: MJPG → mppjpegdec → identity handoff → AI 绘制 → H264 → FLV → RTMP  
> **云端交互**: WebSocket 注册 + 心跳 (device-003)  
> **端侧控制**: HTTP API @ 8080 (模式/标定/舵机/命令)

---

## 硬件组成

| 组件 | 型号/职责 |
|------|----------|
| 上位机 | RK3588 (ELF2) — IMU 接收、运动预测、RTMP 推流、端侧控制 |
| 下位机 | STM32H723 — 逆运动学、S-curve、电机/舵机驱动 |
| 摄像头 | Realtek USB Camera (0bda:5858) — 1920×1080@30fps MJPG |
| 无线 IMU | NRF24 + 陀螺仪 — 头戴/颈挂，发送 roll/yaw/pitch + wx/wy/wz |
| 机械臂 | 可穿戴三轴 + 末端双舵机云台 (J1~J3 + J4/J5) |

---

## 数据流

### 控制链路
```
IMU (头部) → NRF24 无线 → RK3588 SPI
                              ↓
                    8 状态运动状态机 + 终点预测器
                              ↓
                    目标位姿 (tx,ty,tz) + 舵机 (k1,k2)
                              ↓
                    UART 115200 @ 5~7Hz → STM32 → 电机
```

### 视觉/推流链路
```
USB Camera (MJPG 1920×1080@30fps)
    → v4l2src → mppjpegdec → identity handoff
        → process_frame (YOLOv8-Pose / Face 3D PnP + RGA 绘制)
        → appsrc → mpph264enc → h264parse → flvmux → rtmpsink
```

### 云端/端侧交互
```
RTMP: rtmp://47.93.162.124:1935/live/device-003
WS  : ws://47.93.162.124/ws?deviceId=device-003
HTTP: http://<ELF2_IP>:8080/{status,mode,calib,servo,cmd}
```

---

## 编译

```bash
cd /home/elf/work/twice

g++ -std=c++17 -O2 -Isrc \
  src/main.cpp src/rga_npu.cpp src/gst_rtsp.cpp src/gst_rtmp.cpp \
  src/stream_manager.cpp src/ctrl_server.cpp src/ws_client.cpp \
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

启动后自动行为：
1. 连接 WiFi (iQOO 12)
2. 探测 RTMP 服务器 (`47.93.162.124:1935`)
3. 若 RTMP 可达 → 推 RTMP + 启动 WebSocket
4. 若 RTMP 不可达 → 降级为本地 RTSP (`rtsp://<IP>:8554/stream`)
5. 启动端侧 HTTP 控制服务器 (端口 8080)
6. 启动 NRF24 控制定时器 (50ms)

---

## 端侧控制 API

| 端点 | 方法 | 说明 |
|------|------|------|
| `/status` | GET | 返回当前模式、帧计数、运行时间 |
| `/mode` | POST `mode=face/body` | 切换 AI 模式 |
| `/calib` | POST `action=start/stop/save` | 标定模式控制 |
| `/servo` | POST `k1=50&k2=145` | 实时调整舵机角度 |
| `/cmd` | POST `action=rebaseline` | 重新标定 baseline |

---

## 文件结构

```
src/
  main.cpp            # 主入口：WiFi探测、WS/RTMP/RTSP初始化、控制服务器
  rga_npu.cpp         # 视觉链路 + NRF24 控制核心（8状态机+终点预测）
  nrf24_linux.c/h     # NRF24 SPI 驱动 + IMU 帧解析 + 历史缓冲
  uart_comm.cpp/h     # UART 通信（10 字节 raw int16）
  gst_rtmp.cpp/h      # GStreamer RTMP 推流（identity handoff → appsrc）
  gst_rtsp.cpp/h      # GStreamer RTSP 推流（appsink 桥接）
  stream_manager.cpp/h# 推流管理器：RTMP探测 + 自动选择 RTMP/RTSP
  ctrl_server.cpp/h   # 端侧 HTTP 控制服务器（零依赖 socket 实现）
  ws_client.cpp/h     # WebSocket 客户端（注册帧 + 100ms心跳）
  wifi.cpp/h          # WiFi 连接（wpa_supplicant）
docs/               # 项目文档（控制设计、标定指南、调试日志、审计提示）
scripts/            # 标定脚本
calib/              # 相机标定参数
models/             # RKNN 模型（best.rknn / face_best.rknn）
```

---

## 关键设计决策

### 1. identity handoff 替代 appsink
- `appsink` 的 `new-sample` signal 在 `gst_parse_launch` + `GstPipeline` 中存在不触发的兼容性问题（卡 PAUSED）
- 改用 `identity` 的 `handoff` signal（同步 callback），pipeline 稳定进入 PLAYING
- `mppjpegdec` 输出 NV12 高度对齐到 16 的倍数（1080→1088），handoff 中做 stride padding 校正

### 2. 初始化时探测 RTMP，条件启动 WebSocket
- `probe_rtmp_server()` 做 TCP 非阻塞 connect，3 秒超时
- RTMP 可达 → 推 RTMP + 启动 `ws_worker_thread`
- RTMP 不可达 → 降级 RTSP + 不启动 WS（避免 connect 阻塞 75s）

### 3. 安全的 Ctrl+C 退出
- 信号处理函数只设原子标志，不直接调用 `g_main_loop_quit`
- GLib 100ms 定时器 `check_quit_timer` 在同线程安全 quit
- `gst_rtmp.cpp` 中 `rtmpsink` 关闭放到后台线程，避免网络超时阻塞主线程退出

---

## 已知问题

1. **wx 噪声**：静止时 wx 仍有尖峰，可能误判运动状态
2. **5° 阈值延迟**：小角度头部动作不触发发令
3. **STM32 短距减速**：小位移响应极慢
4. **纯开环**：上位机不知道机械臂实际位置
5. **identity handoff 同步阻塞**：`process_frame` 耗时若 >33ms 会降低有效帧率

---

## 下一步

- J4 舵机标定（方案 A：简单映射）
- 下位机回传关节角 / 末端位姿
- 通信协议升级（帧头 + CRC + 双向）
