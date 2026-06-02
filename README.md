# 可穿戴机械臂 — v2 手势状态机 + NRF24 IMU 控制

> **分支**: `imu-v2state-hat`  
> **核心路径**: NRF24 无线 IMU → 8 状态运动预测 → UART → STM32；20 关键点 + bbox → RuleEngine v2 → 手势状态机  
> **视觉链路**: 两阶段 NPU 推理（Body → ROI → Face 468 landmarks）+ Python Socket RuleEngine v2  
> **RuleEngine**: 基于 `rule_engine_v2.rknn`，输入 20 关键点 + bbox + valid_mask + 7 状态反馈  
> **云端交互**: WebSocket 注册 + 心跳 (device-003)  
> **端侧控制**: HTTP API @ 8080

---

## 硬件组成

| 组件 | 型号/职责 |
|------|----------|
| 上位机 | RK3588 (ELF2) — IMU 接收、运动预测、RTMP/RTSP 推流、双模型 NPU 推理、手势状态机 |
| 下位机 | STM32H723 — 逆运动学、S-curve、电机/舵机驱动 |
| 摄像头 | Realtek USB Camera (0bda:5858) — 1920×1080@30fps YUY2 |
| 无线 IMU | NRF24 + 陀螺仪 — 头戴/颈挂，发送 roll/pitch/yaw + wx/wy/wz |
| 机械臂 | 可穿戴三轴 + 末端双舵机云台 (J1~J3 + J4/J5) |

---

## 数据流

### 控制链路（双轨并行）

**手势控制链路（RuleEngine v2）:**
```
USB Camera → Body NPU (17 COCO kpts) + Face LM (3 点)
                              ↓
                    C++ Socket 客户端（kpts + bbox + valid_mask + state_fb）
                              ↓
                    Python RuleEngine Socket 服务（rule_engine_v2.rknn）
                              ↓
                    3 状态手势状态机（Idle / Mode1 / Mode2）
                              ↓
                    UART → STM32 → 电机/舵机
```

**IMU 控制链路:**
```
IMU (头部) → NRF24 无线 → RK3588 SPI
                              ↓
                    A-inverse 矩阵解耦（R_current × R_init^T）
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
    → v4l2src → identity handoff → RGA(YUYV→NV12)
        → process_frame
            ├─ Stage 1: best.rknn (YOLO-Pose, 640×640) → 17 COCO keypoints
            │              └─ 估计面部 ROI（鼻肩几何）
            ├─ Stage 2: RGA imcrop/imresize/imcvtcolor (192×192)
            │              └─ face_landmark_468_fp16.rknn → 468 3D landmarks
            ├─ 12-point PnP → solvePnP → 3D 立方体 / 坐标轴 overlay
            └─ RuleEngine v2 Socket → Python NPU 推理 → 手势状态 OSD
        → appsrc → mpph264enc → h264parse → flvmux → rtmpsink / rtph264pay
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

g++ -std=c++17 -O2 \
  src/main.cpp src/rga_npu.cpp src/gst_rtsp.cpp src/gst_rtmp.cpp \
  src/stream_manager.cpp src/ctrl_server.cpp src/ws_client.cpp \
  src/uart_comm.cpp src/wifi.cpp \
  src/nrf24_linux.c src/bt_stub.c \
  -o build/cc \
  $(pkg-config --cflags --libs gstreamer-1.0 gstreamer-app-1.0 gstreamer-rtsp-server-1.0) \
  -I/usr/include/opencv4 -lopencv_core -lopencv_imgproc -lopencv_calib3d \
  -lrknnrt -lrga -lwpa_client -lpthread
```

依赖：
- `rknn-toolkit-lite2==2.3.2`（Python RuleEngine 服务）
- `librknnrt.so` ≥ 2.3.2（C++ Body/Face NPU 推理）

---

## 运行

```bash
# 1. 启动 RuleEngine Python 服务（先起，确保 socket 就绪）
python3 scripts/rule_engine_server.py &

# 2. 启动主程序
sudo ./build/cc
```

启动后自动行为：
1. CPU 调频至性能模式
2. 连接 WiFi (iQOO 12)
3. 初始化 NPU（Body + FaceLM）+ RGA
4. 初始化 UART (`/dev/ttyS9` @ 115200)
5. 启动端侧 HTTP 控制服务器 (端口 8080)
6. 启动 NRF24 接收线程 + 50ms 控制定时器
7. 探测 RTMP 服务器 → 自动选择 RTMP（+WebSocket）或 RTSP

---

## RuleEngine v2 协议

### 输入（C++ → Python，312 字节）

| 字段 | 格式 | 字节数 | 说明 |
|------|------|--------|------|
| kpts | `40 × float32` | 160 | 20 个关键点 interleaved `[x0,y0,...,x19,y19]` |
| bbox | `4 × float32` | 16 | 检测框 `(x1, y1, x2, y2)` |
| valid_mask | `20 × float32` | 80 | 各点有效性 `0.0/1.0` |
| state_fb | `7 × int64` | 56 | 跨帧状态寄存器 |
| **总计** | | **312** | `struct: 44f20f7q` |

### 输出（Python → C++，56 字节）

| 字段 | 格式 | 字节数 | 说明 |
|------|------|--------|------|
| state + 6 个计数器 | `7 × int64` | 56 | `struct: 7q` |

---

## 端侧控制 API

| 端点 | 方法 | 说明 |
|------|------|------|
| `/status` | GET | 返回当前模式、推流类型、move_complete、head_stationary、arm_stable |
| `/mode` | POST `type=face/body` | 切换 AI 模式 |
| `/calib` | POST `mode=0/1/2` | 标定模式控制（0=关闭, 1=锁定+调舵机, 2=扫描测试） |
| `/servo` | POST `k1=50&k2=145` | 实时调整舵机角度 |
| `/cmd` | POST `action=rebaseline/nrf24_reset` | 重新标定 baseline 或复位 NRF24 |

---

## 文件结构

```
src/
  main.cpp            # 主入口：初始化、推流、控制服务器、RuleEngine 服务生命周期
  rga_npu.cpp         # 视觉链路 + RuleEngine Socket 客户端 + NRF24 控制核心
  rga_npu.h           # 视觉/控制对外接口声明
  nrf24_linux.c/h     # NRF24 SPI 驱动 + IMU 帧解析 + 历史缓冲
  uart_comm.cpp/h     # UART 通信（raw int16 发送 + 文本 complete 检测）
  gst_rtmp.cpp/h      # GStreamer RTMP 推流（identity handoff）
  gst_rtsp.cpp/h      # GStreamer RTSP 推流（appsink 桥接）
  stream_manager.cpp/h# 推流管理器：RTMP 探测 + 自动选择 RTMP/RTSP
  ctrl_server.cpp/h   # 端侧 HTTP 控制服务器（零依赖 socket 实现）
  ws_client.cpp/h     # WebSocket 客户端（注册帧 + 100ms 心跳）
  wifi.cpp/h          # WiFi 连接（wpa_supplicant）
scripts/
  rule_engine_server.py  # Python Unix Socket 推理服务（rknn-toolkit-lite2 v2）
  calibrate.py           # 标定脚本
docs/               # 项目文档（控制设计、标定指南、调试日志、审计提示）
calib/              # 相机标定参数
models/             # RKNN 模型（best.rknn / face_landmark_468_fp16.rknn / rule_engine_v2.rknn）
```

---

## 关键设计决策

### 1. RuleEngine v2：Python Socket 推理服务
- **模型**：`rule_engine_v2.rknn`，10 个输入（kpts + bbox + valid_mask + 7 state）
- **输入协议**：312 字节 `44f20f7q`（kpts 40f + bbox 4f + valid_mask 20f + state_fb 7q）
- **bbox 作用**：为模型提供全局空间上下文，辅助区分相似手势在不同人体位置时的语义
- **NPU 隔离**：Python 端固定使用 `NPU_CORE_0`，与 C++ Body/Face 的三核负载隔离，避免多进程 NPU 互锁
- **Warmup**：启动时用 dummy zeros 预跑一遍，消除首次 `rknn.inference()` 的初始化延迟（否则会导致 GStreamer 流线程阻塞 6s+）

### 2. A-inverse 矩阵解耦
- 用 `R_rel = R_current × R_init^T` 消除 IMU 初始安装角，替代早期的标量 baseline 减法
- **延迟初始化**：等待下位机首次 `move_complete` 后才捕获 `R_init`，防止初始姿态噪声污染基准
- 控制轴从 Roll/wx 切换到 Pitch/wy（更符合人头俯仰对应机械臂升降的物理直觉）

### 3. 手势识别状态机
- **3 状态**：Idle (0) / Mode1 (1) / Mode2 (2)
- **20 关键点构成**：COCO 17 点 + 面部 3 点（左嘴角、右嘴角、下巴）
- **Observer 视角交换**：COCO left/right 成对点在发送前互换（1↔2, 3↔4, ..., 15↔16），使模型始终以观察者视角判断左右
- **Face 点处理**：Face LM 成功时填入 3 个面部坐标；失败时设为 `0.0f + valid_mask=0`

### 4. 两阶段 NPU 推理（Body → Face）
- **Stage 1**：`best.rknn` 在 640×640 上检测 17 COCO 关键点，提取鼻/肩几何估算面部 ROI
- **Stage 2**：RGA 硬件 `imcrop` + `imresize` + `imcvtcolor` 将 ROI 转为 192×192，送入 `face_landmark_468_fp16.rknn`
- **PnP**：从 468 点中选取 12 个稳定点（眼、鼻、嘴、眉），通过 `solvePnP` 解算头部姿态，叠加 3D 立方体和坐标轴（仅显示，控制已废弃）
- **OneEuroFilter**：当前禁用（`PNP_USE_FILTER 0`），依赖硬丢弃策略（重投影误差 >25px、镜像解、旋转跳变 >60°）

### 5. RGA NV12 stride 对齐约束
- `wrapbuffer_virtualaddr` 的 `wstride` 在 NV12 模式下必须 **16 字节对齐**
- 面部 ROI 宽度在 `imcrop` 前需对齐：`roi_w = (roi_w / 16) * 16`
- ROI 高度设为与宽度相等（正方形），且 `roi_x`/`roi_y` 需为偶数（NV12 色度子采样）

### 6. RKNN 输出内存分配
- `rknn_create_mem` 必须使用 `rknn_query` 返回的 `attr->size`，而非 `n_elems * sizeof(element)`
- 对于 FP16 模型，`attr->size` 可能包含 padding，手动计算会导致堆损坏

### 7. 7 模块滚动时序统计
- 30 帧滑动窗口统计：`get_frame`、`rga_preprocess`、`npu_body`、`roi_crop`、`npu_face_lm`、`draw`、`encode_push`

---

## 已知问题

1. **Body 模型瓶颈**：`best.rknn` 占 AI 总耗时 ~85%，是整体帧率的主要瓶颈
2. **wx 噪声**：静止时 wx 仍有尖峰，可能误判运动状态
3. **5° 发令阈值延迟**：小角度头部动作不触发发令
4. **STM32 短距减速**：小位移响应极慢（`<3cm → 15%` 速度）
5. **纯开环**：上位机不知道机械臂实际位置，无关节角反馈
6. **UART 协议不一致**：发送侧仍是 10 字节 raw int16，接收侧已实现帧头+CRC，但未双向统一
7. **identity handoff 同步阻塞**：`process_frame` 耗时若 >33ms 会降低有效帧率
8. **视觉算力浪费**：PnP 解算和 3D 绘制仅用于 OSD 显示，控制链路已废弃

---

## 下一步

- J4 舵机标定（方案 A：简单映射，基础设施 `/tmp/calib_mode.txt` + `/tmp/servo_calib.txt` 已就绪）
- Body 模型优化（轻量化或降分辨率以突破帧率瓶颈）
- UART 协议升级（统一为帧头+CRC 双向通信，下位机回传关节角）
- RuleEngine 手势扩展（更多状态或自定义触发条件）
