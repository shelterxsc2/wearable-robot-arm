# 可穿戴机械臂 — `imu-victor-hat` 分支

> **分支**: `imu-victor-hat`  
> **标签**: `imu-pnp-fuse1.0` — PnP 视觉零飘修正闭环首次集成  
> **核心路径**: NRF24 无线 IMU → A-inverse 矩阵解耦 → 8 状态运动预测/终点预测 → 11 字节 UART(flag=0x00/0x01) → STM32  
> **视觉链路**: 两阶段 NPU 推理（Body → ROI → Face 468 landmarks）+ Python Socket RuleEngine v2  
> **RuleEngine**: 基于 `rule_engine_v2.rknn`，输入 20 关键点 + bbox + valid_mask + 7 状态反馈  
> **本分支重点**: 
> - **imu-pnp-fuse1.0 新增**：PnP 视觉零飘修正闭环（静止态绝对值积分，R_bias_total 累积修正矩阵）
> - NRF24 IMU 控制：运动学公式重构(l1~l4)、发令策略分化、舵机映射、位置死区、flag标志位
> - 握手时序重构：`g_uart_block_tx` 延时拦截 + A-init（5帧平均）
> - 云端 LL-HLS 监控系统对接
> **云端交互**: WebSocket 注册 + 心跳 + LL-HLS 低延迟直播 (~3s)  
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

**IMU 控制链路:**
```
IMU (头部) → NRF24 无线 → RK3588 SPI
                              ↓
                    A-inverse 矩阵解耦（R_current × R_init^T）
                              ↓
                    8 状态运动状态机 + 终点预测器
                              ↓
                    球坐标目标 (tx,ty,tz) + 舵机 (servo1,servo2)
                              ↓
                    UART 11字节 @ 115200 → STM32 → 电机
```

**手势控制链路:**
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

### 视觉/推流链路
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

### 云端交互
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

**握手时序**：
```
init success → FF AA 验证帧 → 7s 延时(g_uart_block_tx=1) → A-init → NORMAL
```
- 7s 延时期间：UART 发送被 `g_uart_block_tx` 完全拦截（不打印、不发数据）
- A-init 完成后：`g_uart_block_tx=0`，恢复正常发令

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
| `/calib` | POST `mode=0/1/2/3` | 标定模式控制（0=关闭, 1=锁定+调舵机, 2=扫描测试, 3=PnP yaw 多点标定） |
| `/servo` | POST `k1=50&k2=145` | 实时调整舵机角度 |
| `/cmd` | POST `action=rebaseline/nrf24_reset` | 重新标定 baseline 或复位 NRF24 |

---

## 文件结构

```
src/
  main.cpp            # 主入口：初始化、推流、控制服务器、RuleEngine 服务生命周期
  rga_npu.cpp/h       # 视觉链路 + RuleEngine Socket 客户端 + NRF24 控制核心
  nrf24_linux.c/h     # NRF24 SPI 驱动 + IMU 帧解析 + 历史缓冲
  uart_comm.cpp/h     # UART 通信（11字节发送含flag + 文本 complete 检测）
  gst_rtmp.cpp/h      # GStreamer RTMP 推流（identity handoff）
  gst_rtsp.cpp/h      # GStreamer RTSP 推流（appsink 桥接）
  stream_manager.cpp/h# 推流管理器：RTMP 探测 + 自动选择 RTMP/RTSP
  ctrl_server.cpp/h   # 端侧 HTTP 控制服务器（零依赖 socket 实现）
  ws_client.cpp/h     # WebSocket 客户端（注册帧 + 100ms 心跳）
  wifi.cpp/h          # WiFi 连接（wpa_supplicant）
  bluetooth_spp.c/h   # BLE（已禁用，bt_stub.c 空实现）
scripts/
  rule_engine_server.py  # Python Unix Socket 推理服务（rknn-toolkit-lite2 v2）
  calibrate.py           # 标定脚本
docs/
  midterm-report-prompt.md  # 中期检查报告生成 Prompt（含云端系统）
  audit-prompt.md           # 代码审计 Prompt
calib/              # 相机标定参数
models/             # RKNN 模型（best.rknn / face_landmark_468_fp16.rknn / rule_engine_v2.rknn）
```

---

## 关键设计决策

### 1. 11 字节 UART 协议（新增 flag 标志位）
- 前 10 字节：`[x][y][z][k1][k2]` 各 int16 小端
- 第 11 字节：`flag`
  - `0x00` = 预测/猜测坐标（头部运动中，predictor 输出）
  - `0x01` = 确定/最终坐标（头部静止后，实际角度）
- 下位机可据此区分预测阶段（允许 overshoot）和确定阶段（精确到位）

### 2. `g_uart_block_tx` 延时拦截
- 替代 `g_host_state` 的多用途状态机，简化逻辑
- 发完 `FF AA` 验证帧后置位，7s 后清零
- 延时期间：任何 UART 发送（包括标定/HTTP）都被拦截，不打印、不发数据
- NRF24 RX 线程不受影响，持续接收 IMU 数据

### 3. 球坐标运动学（参数化 l1~l4）
```
tx = l3·sin(yaw)·cos(pitch)
ty = l4 - l1·sin(pitch) + l3·cos(pitch)·cos(yaw)
tz = k·(l2 + l1·cos(pitch) + l3·sin(pitch)·cos(yaw))
```
当前参数：`l1=8`, `l2=5`, `l3=40`, `l4=28`, `k=1.6`（单位 cm）

### 4. 舵机映射（新定义）
- J4 俯仰：`servo1 = 30 + (-1.2)·Δpitch`，限幅 [-90°, +90°]
- J5 水平：`servo2 = 50 + 0.4·Δyaw`，限幅 [0°, 270°]
- 0° = 竖直向下，正 = 向内（低头），负 = 向外（抬头）

### 5. PnP 视觉零飘修正闭环（imu-pnp-fuse1.0 新增）
- **PnP 解算**：Face 468 landmarks → `solvePnP` → 提取 head yaw/pitch（屏幕显示欧拉角）
- **偏移校准**：`pnp_yaw_correction = hy_deg - 14.0`（安装偏角补偿）
- **静止态触发**：`is_stop && is_stop_yaw` 且 PnP 连续有效时执行修正
- **修正策略**：绝对值积分 `yaw_delta = -KI_PNP · pnp_yaw_correction`，`KI_PNP = 0.25`
- **累积矩阵**：`R_bias_total = R_delta × R_bias_total`，左乘在 IMU 当前姿态上
- **Pitch 修正**：当前关闭（`pitch_delta = 0`）
- **静止态发令**：Yaw 静止态不再掐死，允许 PnP 修正驱动机械臂微动

### 6. 发令策略分化
- **Yaw**：`STATE_ACCEL_TO_CONST` 主窗口发一次预测令；静止态允许发令（供 PnP 修正微动）
- **Pitch**：主窗口/第二窗口/静止态均发令（保持微调能力）

### 7. A-init 改进（5帧平均）
- RX 线程在 10ms 高频下收集 5 帧 IMU 数据算平均
- 相比单帧捕获，显著降低初始姿态抖动

---

## 已知问题

1. **Body 模型瓶颈**：`best.rknn` 占 AI 总耗时 ~85%
2. **wx 噪声**：静止时 wx 仍有尖峰
3. **5° 发令阈值延迟**：小角度头部动作不触发发令
4. **STM32 短距减速**：小位移响应极慢（`<3cm → 15%` 速度）
5. **纯开环**：上位机不知道机械臂实际位置，无关节角反馈
6. **俯仰极性偶发反转**：A-init 时头部姿态不同导致矩阵解耦符号不稳
7. **云-端控制协议割裂**：云端离散状态(x,y,z)，端侧连续坐标(tx,ty,tz)
8. **云端延迟**：RTMP ~3s，WebSocket 100ms
9. **PnP 绝对值积分漂移**：当前 `yaw_delta = KI·pnp_yaw` 是对绝对角度积分，非误差积分，Pnp 绝对值非零时 R_bias_total 会持续累积（预期行为）
10. **PnP 单帧即触发**：当前 `pnp_valid_cnt >= 1` 即置 `correction_ready`，未做连续多帧一致性过滤

---

## 版本历史

### `imu-pnp-fuse1.0` (当前)
- PnP 视觉零飘修正闭环首次集成
- A-init 改进为 5 帧平均
- 运动学参数调整为 `l1=8, l2=5, l3=40, l4=28, k=1.6`
- PnP 偏移链路简化（删除旋转矩阵偏移，直接用欧拉角）
- UART 打印精简

## 下一步

- PnP 修正策略优化（误差积分替代绝对值积分）
- PnP 连续多帧一致性过滤（避免单帧噪声触发修正）
- J4 舵机标定上机实测
- Body 模型优化（轻量化突破帧率瓶颈）
- 下位机回传关节角（UART 双向通信）
- 云-端协议统一（离散状态 vs 连续坐标融合）
- 参数热加载
