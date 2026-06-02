# 可穿戴机械臂 — 三状态手势识别 + NRF24 IMU 控制

> **分支**: `imu-3states-hat`  
> **核心路径**: 20 关键点手势状态机 → UART → STM32 + NRF24 无线 IMU → A-inverse 矩阵解耦 → 8 状态运动状态机  
> **视觉链路**: USB Camera YUY2 1920×1080@30fps → RGA YUYV→NV12 → 两阶段 NPU 推理 + Python Socket RuleEngine  
> **RuleEngine**: 3 状态手势识别（Idle / Mode1 / Mode2），基于 COCO 17 点 + 面部 3 点  
> **云端交互**: WebSocket 注册 + 心跳 (device-003)  
> **端侧控制**: HTTP API @ 8080 (模式/标定/舵机/命令)

---

## 硬件组成

| 组件 | 型号/职责 |
|------|----------|
| 上位机 | RK3588 (ELF2) — IMU 接收、运动预测、RTMP 推流、端侧控制、双模型 NPU 推理、手势状态机 |
| 下位机 | STM32H723 — 逆运动学、S-curve、电机/舵机驱动 |
| 摄像头 | Realtek USB Camera (0bda:5858) — YUY2 1920×1080@30fps |
| 无线 IMU | NRF24 + 陀螺仪 — 头戴/颈挂，发送 roll/yaw/pitch + wx/wy/wz |
| 机械臂 | 可穿戴三轴 + 末端双舵机云台 (J1~J3 + J4/J5) |

---

## 数据流

### 控制链路（双轨并行）

**手势控制链路:**
```
USB Camera → Body NPU (17 COCO kpts) + Face LM (3 点)
                              ↓
                    RuleEngine (Python Socket 服务)
                              ↓
                    3 状态手势状态机 (Idle/Mode1/Mode2)
                              ↓
                    UART 115200 → STM32 → 电机/舵机
```

**IMU 控制链路:**
```
IMU (头部) → NRF24 无线 → RK3588 SPI
                              ↓
                    A-inverse: R_current × R_init^T (OpenCV 矩阵解耦)
                              ↓
                    8 状态运动状态机 + 终点预测器
                              ↓
                    目标位姿 (tx,ty,tz) + 舵机 (k1,k2)
                              ↓
                    UART 115200 @ 5~7Hz → STM32 → 电机
```

### 视觉/推流链路（两阶段）
```
USB Camera (YUY2 1920×1080@30fps)
    → v4l2src → identity handoff (YUY2)
        → RGA 硬件 convert_yuyv_to_nv12 (NV12)
            → process_frame
                ├─ Stage 1: best.rknn (YOLO-Pose, 640×640) → 17 COCO keypoints
                │              └─ 估计面部 ROI（鼻肩几何）
                ├─ Stage 2: RGA imcrop/imresize/imcvtcolor (192×192 NV12)
                │              └─ face_landmark_468_fp16.rknn → 468 3D landmarks
                ├─ 12-point PnP → solvePnP → 3D 立方体 / 坐标轴 overlay
                └─ RuleEngine Socket → Python NPU 推理 → 手势状态 OSD
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

g++ -std=c++17 -O2 -I/usr/include/opencv4 -Isrc \
  src/main.cpp src/rga_npu.cpp src/gst_rtsp.cpp src/gst_rtmp.cpp \
  src/stream_manager.cpp src/ctrl_server.cpp src/ws_client.cpp \
  src/uart_comm.cpp src/wifi.cpp \
  src/nrf24_linux.c src/bt_stub.c \
  -o build/cc \
  $(pkg-config --cflags --libs opencv4 gstreamer-1.0 gstreamer-app-1.0 gstreamer-rtsp-server-1.0) \
  /usr/lib/aarch64-linux-gnu/libwpa_client.a \
  -lrknnrt -lrga -lpthread -lm -ldl
```

依赖：
- `rknn-toolkit-lite2==2.3.2`（Python RuleEngine 服务）
- `librknnrt.so` ≥ 2.3.2（C++ Body/Face NPU 推理）

---

## 运行

```bash
sudo ./build/cc
```

启动后自动行为：
1. 连接 WiFi (iQOO 12)
2. 启动 RuleEngine Python 服务 (`scripts/rule_engine_server.py`)，监听 `/tmp/rule_engine.sock`
3. 探测 RTMP 服务器 (`47.93.162.124:1935`)
4. 若 RTMP 可达 → 推 RTMP + 启动 WebSocket
5. 若 RTMP 不可达 → 降级为本地 RTSP (`rtsp://<IP>:8554/stream`)
6. 启动端侧 HTTP 控制服务器 (端口 8080)
7. 启动 NRF24 控制定时器 (50ms)

**A-init 时序（重要）**：程序启动后不会立即发令。需等待 STM32 完成上电 homing 并发送第一个 `"move complete"`，上位机才会捕获当前 IMU 姿态作为 `R_init`，此后 A-inverse 解耦生效，控制正式开始。

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
  main.cpp            # 主入口：WiFi探测、WS/RTMP/RTSP初始化、控制服务器、RuleEngine服务生命周期
  rga_npu.cpp         # 视觉链路 + RuleEngine Socket + NRF24 控制核心（A-inverse、8状态机、预测器）
  rga_npu.h           # 视觉/控制对外接口声明
  nrf24_linux.c/h     # NRF24 SPI 驱动 + IMU 帧解析 + 历史缓冲（roll/pitch/yaw + wy/wz）
  uart_comm.cpp/h     # UART 通信（10 字节 raw int16 + 诊断日志 + move_complete 检测）
  uart_loopback_test.cpp  # UART 回环测试工具（/dev/ttyS9 @ 115200）
  gst_rtmp.cpp/h      # GStreamer RTMP 推流（identity handoff → RGA YUYV→NV12 → appsrc）
  gst_rtsp.cpp/h      # GStreamer RTSP 推流（appsink 桥接）
  stream_manager.cpp/h# 推流管理器：RTMP探测 + 自动选择 RTMP/RTSP
  ctrl_server.cpp/h   # 端侧 HTTP 控制服务器（零依赖 socket 实现）
  ws_client.cpp/h     # WebSocket 客户端（注册帧 + 100ms心跳）
  wifi.cpp/h          # WiFi 连接（wpa_supplicant）
  bt_stub.c           # Bluetooth SPP stub（替代 dbus 依赖）
scripts/
  rule_engine_server.py  # Python Unix Socket 推理服务（rknn-toolkit-lite2）
  calibrate.py           # 标定脚本
  calibrate.py           # 标定脚本
docs/               # 项目文档（控制设计、标定指南、调试日志、审计提示）
calib/              # 相机标定参数
models/             # RKNN 模型（best.rknn / face_landmark_468_fp16.rknn / rule_engine_handcraft.rknn）
```

---

## 关键设计决策

### 1. 视觉输入：YUY2 @ 30fps + RGA 硬件转码
- **摄像头输出**：YUY2 (YUV 4:2:2) 1920×1080@30fps，非 MJPG。
- **RGA 转换**：`convert_yuyv_to_nv12()` 使用 RGA 硬件加速将 YUYV 转为 NV12，零 CPU 开销。
- **Pipeline**：`v4l2src` 直接抓 YUY2 → `identity handoff` → RGA 转 NV12 → AI 处理 → `appsrc` → H264 编码 → RTMP。

### 2. A-inverse 矩阵解耦（替代标量 baseline 减法）
- **问题**：IMU 安装存在非零初始 Euler 角，标量减法（`current - init`）在耦合姿态下失效。
- **方案**：将 IMU Euler 角转为旋转矩阵 `R_current`，与初始矩阵 `R_init` 做矩阵乘法 `R_rel = R_current × R_init^T`，再转回 Euler 角得到解耦的 `rel_roll/rel_pitch/rel_yaw`。
- **延迟初始化**：`R_init` 不在第一个 valid IMU 帧捕获，而是等待 STM32 发送第一个 `"move complete"`（上电 homing 完成后），确保机械臂处于已知 idle 姿态后再锁定坐标系。

### 3. 控制轴映射
| IMU 轴 | 控制量 | 极性 |
|--------|--------|------|
| Pitch (`wy`) | `tz`（垂直） | 低头 → tz 增大（臂上升） |
| Yaw   (`wz`) | `tx/ty`（水平） | 头左偏 → tx 负（左移） |

- Roll 轴不再参与控制，仅保留在历史缓冲中。
- `wy` 用于 Pitch 轴运动上下文和预测器；`wz` 用于 Yaw 轴。

### 4. RuleEngine：Python Socket 推理服务
- **问题**：C++ `librknnrt.so`（即使 v2.3.2）对 `Greater`/`Less` 等比较操作产生错误结果，导致状态机行为异常。
- **方案**：RuleEngine 推理完全迁移到 Python 子进程，通过 Unix domain socket (`/tmp/rule_engine.sock`) 与 C++ 主程序通信。
- **模型**：`rule_engine_handcraft.rknn`（手写 ONNX 导出，仅使用 RKNN 兼容算子）。
- **输入**：20 关键点 `[1,20,2] FLOAT` + valid_mask `[1,20] FLOAT` + 7 个 INT64 状态反馈。
- **输出**：7 个 INT64（state, right_hold, left_hold, center_hold, neutral_hold, right_keep_miss, left_keep_miss）。
- **协议**：176 字节原始二进制发送 → 56 字节原始二进制接收。零序列化开销。

### 5. 手势识别状态机
- **3 状态**：Idle (0) / Mode1 (1) / Mode2 (2)
- **触发条件**：基于右手腕相对位置（高举 → Mode1，侧举 → Mode2，放下 → Idle）
- **消抖机制**：需连续 5 帧确认同一手势才切换状态
- **视角补偿**：COCO left/right 在发送前进行 observer 视角交换（1↔2, 3↔4, ..., 15↔16）
- **Face 点处理**：Face LM 失败时，3 个面部关键点设为 0.0f，valid_mask=0.0f

### 6. 两阶段 NPU 推理（Body → Face）
- **Stage 1**：`best.rknn` 在 640×640 上检测 17 COCO 关键点，提取鼻/肩几何估算面部 ROI。
- **Stage 2**：RGA 硬件 `imcrop` + `imresize` + `imcvtcolor` 将 ROI 转为 192×192 NV12，送入 `face_landmark_468_fp16.rknn` 提取 468 个面部 3D 关键点。
- **PnP**：从 468 点中选取 12 个 MediaPipe 稳定点，通过 `solvePnP` 解算头部姿态，叠加 3D 立方体和坐标轴。
- **性能**：Body NPU ~46ms，Face NPU ~3ms，RGA 预处理 <1ms，总 AI 耗时 ~54ms（≈18.4 fps）。

### 7. RGA NV12 stride 对齐约束
- `wrapbuffer_virtualaddr` 的 `wstride` 在 NV12 模式下必须 **16 字节对齐**。
- 面部 ROI 宽度在 `imcrop` 前需对齐：`roi_w = (roi_w / 16) * 16`。
- ROI 高度设为与宽度相等（正方形），且 `roi_x`/`roi_y` 需为偶数。

### 8. RKNN 输出内存分配
- `rknn_create_mem` 必须使用 `rknn_query` 返回的 `attr->size`，而非 `n_elems * sizeof(element)`。
- 对于 FP16 模型，`attr->size` 可能包含 padding 或 stride 开销，手动计算会导致堆损坏。

### 9. identity handoff 同步回调
- `appsink` 的 `new-sample` signal 在 `gst_parse_launch` + `GstPipeline` 中存在不触发的兼容性问题（卡 PAUSED）。
- 改用 `identity` 的 `handoff` signal（同步 callback），pipeline 稳定进入 PLAYING。
- `mppjpegdec` 输出 NV12 高度对齐到 16 的倍数（1080→1088），handoff 中做 stride padding 校正。

---

## 已知问题

1. **Body 模型瓶颈**：`best.rknn` 占 AI 总耗时 85%（~46ms），是整体帧率的主要瓶颈。
2. **STM32 短距减速**：MCU 对 <3cm/<6cm/<15cm 位移分别降速到 15%/20%/30%，小位移响应极慢。
3. **纯开环**：上位机无关节角或末端位姿反馈，无法做闭环修正。
4. **UART complete 检测脆弱**：`g_uart_move_complete` 由 `strstr(rx_text, "complete")` 文本匹配设置，非二进制协议。
5. **identity handoff 同步阻塞**：`process_frame` 耗时若 >33ms 会降低有效帧率。

---

## 下一步

- Body 模型轻量化（320×320 输入或更轻架构）以突破 18 fps 瓶颈
- J4 舵机标定（方案 A：简单映射）
- 下位机回传关节角 / 末端位姿（闭环控制准备）
- UART 通信协议升级（帧头 + CRC + 双向二进制）
- RuleEngine 手势扩展（更多手势、自定义触发条件）
