# ELF2 (RK3588) 可穿戴机械臂视觉上位机系统

## 一、项目概述

本项目是一个运行在 **RK3588 (ELF2 开发板)** 上的视觉感知与目标位姿规划系统，作为**可穿戴三轴机械臂的上位机**。机械臂背在人体背部，末端搭载摄像头，用于实时跟踪人脸并控制机械臂运动。

### 硬件组成
- **上位机**: RK3588 (ELF2)，负责视觉感知、AI 推理、位姿计算、通信控制
- **摄像头**: Realtek USB Camera3 (`/dev/video21`)，支持 1920×1080@30/60fps，输出 YUYV/MJPEG
- **机械臂**: 可穿戴三轴机械臂 + 末端两舵机云台
  - J1: 腰部，绕竖直 Z 轴 360° 旋转（Pan）
  - J2: 大臂俯仰
  - J3: 小臂俯仰
  - J4/J5: 末端舵机云台（俯仰 + 偏航），使摄像头可指向半球任意方向
- **通信**: 蓝牙 BLE SPP（接收遥控器指令）、串口 UART/USB（与下位机通信）

### 下位机职责
下位机负责机械臂的 **LQR 控制闭环**，包括电机驱动、状态估计、安全限位等。RK3588 只负责**视觉感知 + 目标位姿生成**，通过串口下发目标位姿给下位机。

---

## 二、当前已完成的工作

### 1. 视觉感知链路（已完成）
- **视频采集**: GStreamer `v4l2src` 从 USB 摄像头采集 1920×1080@30fps YUYV
- **格式转换**: RGA 硬件将 YUYV → NV12（替代了 software videoconvert，降低延迟约 35~40ms）
- **RTSP 推流**: 1080p30 H.264 硬件编码，支持客户端实时观看
- **AI 推理**: RKNN NPU 加载双模型：
  - `face_best.rknn`: 人脸检测，20 个关键点，实际使用 6 个（双眼、鼻尖、双嘴角、下巴）
  - `best.rknn`: YOLOv8n-pose 人体检测，17 个 COCO 关键点
- **PnP 姿态估计**: OpenCV `solvePnP` 计算人脸相对于相机的 3D 位姿（`rvec`, `tvec`）
- **滤波**: OneEuroFilter 对 6 自由度位姿去抖
- **头部姿态提取**: 从滤波后的 `rvec` 解算 yaw/pitch/roll，以及相机相对于人脸的位置误差（`pos_yaw_err`, `pos_pitch_err`）
- **目标跟踪**: FaceTracker + NMS，稳定锁定单个人脸
- **绘制**: RGA 硬件在 NV12 帧上绘制人脸关键点、3D 坐标轴、立方体框、姿态指示器

### 2. 通信与控制（部分完成）
- **蓝牙 BLE SPP**: 框架已实现，但 `main.cpp` 中临时禁用（注释状态）
  - `0x11`: BODY 模式（人体姿态检测）
  - `0x12`: FACE 模式（人脸跟踪）
- **WiFi 连接**: 自动连接预设 WiFi，DHCP + 静态 IP fallback，用于 RTSP 推流
- **UART 串口驱动**: `uart_comm.cpp/h` 已编写，支持两种格式：
  - `Pose6D` 四元数位姿（28 字节 payload，`CMD_TARGET_POSE 0x10`）
  - `ArmTarget` 简化指令（10 字节 int16，`CMD_ARM_TARGET 0x30`）
  - 回环测试 `build/uart_test` 已通过（5月11日），设备节点 `/dev/ttyS9`，波特率 **921600**
  - **✅ 基础通信已验证**：固定目标值下发 → STM32 接收 → 逆运动学 → 机械臂运动，链路已通
  - **⚠️ 尚未接入视觉闭环**：`rga_npu.cpp` 未调用发送函数，人脸位姿未转化为控制目标
- **协议现状**: STM32 下位机实际接收的是 **10 字节 raw 格式**（无帧头、无 CRC），与 `shared/protocol.h` 中定义的 `[0xAA][0x55][LEN][CMD][payload][CRC8]` 标准帧格式不一致。详见 `wearable-robot-arm/docs/architecture.md` 中"通信不匹配说明"。

### 3. 性能指标
- **端到端延迟（Frame → PnP）**: 约 **56~78 ms**，baseline 约 58~62 ms
  - 帧周期（30fps）: ~33 ms
  - USB 传输: ~3~5 ms
  - RGA 转换 + NPU 推理 + 后处理 + PnP: ~15~25 ms
- **NPU 负载**: YOLOv8-pose 640×640 推理约 10~15 ms，30fps 绰绰有余

### 4. 摄像头当前状态（⚠️ 固件故障）
- **硬件**: Realtek USB Camera3 (`0bda:5858`)，原生支持 USB3.0
- **故障现象**: 固件错误导致被识别为 **USB2.0 设备**
- **影响**: YUYV 1920×1080 在 USB2.0 带宽下只能跑 **~5fps**（物理瓶颈：4.15MB/帧 × 5fps ≈ 20MB/s，占满 USB2.0 实际带宽）
- **Workaround**: 已临时将 GStreamer caps 改为 `framerate=5/1`，OneEuroFilter `freq` 参数同步调整为 **5.0 Hz** 适配低帧率
- **恢复方法**: 待摄像头固件修复或更换设备后，恢复 `framerate=30/1` 和 OneEuroFilter `freq=30.0/15.0`

---

## 三、下一步待完成的工作

### 优先级 1: 串口通信与目标位姿下发（核心闭环）
**目标**: RK3588 把计算出的目标位姿通过串口发给下位机，下位机用 LQR 跟踪。

**需要确定的事项**:
- 串口设备节点（`/dev/ttyS1`? `/dev/ttyUSB0`?）
- 波特率（建议 115200 或 921600）
- 通信协议帧格式（帧头 + CMD + payload + CRC）
- 位姿表示方式：
  - **位置**: `(x, y, z)`，单位 mm 或 m
  - **姿态**: 四元数 `(qx, qy, qz, qw)`（推荐，无万向节锁）或欧拉角 `(roll, pitch, yaw)`
- 下位机回传格式：当前末端（摄像头）位姿 `(x, y, z, qx, qy, qz, qw)`

**RK3588 端的计算逻辑**:
1. 接收下位机回传的当前相机位姿 `T_base→cam_current`
2. 通过 PnP 得到人脸相对于相机的位姿 `T_cam→face`
3. 计算人脸在基座系中的位置: `P_face = T_base→cam_current × t_cam→face`
4. 根据当前模式（正脸/侧脸）和期望距离 D，计算目标相机位姿 `T_base→cam_target`
5. 打包下发给下位机

### 优先级 2: 蓝牙遥控器扩展
**目标**: 遥控器不止切换 FACE/BODY，还要控制跟踪距离和侧脸角度。

**建议指令集**:
| 指令 | 含义 |
|------|------|
| `0x11` | BODY 模式 |
| `0x12` | FACE 正脸模式 |
| `0x13` | FACE 左侧脸模式 |
| `0x14` | FACE 右侧脸模式 |
| `0x21` | 跟踪距离 45 cm |
| `0x22` | 跟踪距离 60 cm |
| `0x23` | 跟踪距离 75 cm |

**期望距离 D 的物理意义**:
- 目标相机位置 = 沿"人脸中心 → 当前相机"方向后退 D cm
- 即保持摄像头在人脸正前方（或侧前方）D 距离处

### 优先级 3: 目标位姿生成算法

#### 正脸模式
在人脸本地坐标系中，目标相机偏移量为 `(0, 0, -D)`，即人脸正前方 D cm。

#### 侧脸模式
在人脸本地坐标系中，目标相机偏移量绕 Y 轴旋转侧脸角度 θ：
- 左侧脸: `(-D·sinθ, 0, -D·cosθ)`
- 右侧脸: `(+D·sinθ, 0, -D·cosθ)`

> **待确定**: 侧脸角度 θ 取多少？30°、45°、60°？还是遥控器无极调节？

#### 坐标系转换流程
```
T_base→cam_target = T_base→face × T_face→cam_target
```
其中 `T_face→cam_target` 由上述偏移量和朝向构造（相机 Z 轴指向人脸中心）。

### 优先级 4: 人脸丢失 Fallback 策略
**待确定**: 检测不到人脸时：
- **保持原位**（freeze）？
- **回到预设 home 姿态**？
- **根据最后一帧速度预测追踪一段**？

### 优先级 5: 侧脸模型鲁棒性测试
当前 FACE 模型使用 6 个关键点（双眼、鼻尖、双嘴角、下巴）。虽然用户反馈半脸时模型仍能找到可见点，但大角度侧脸（>45°）时 PnP 稳定性需要实测验证。

**测试方法**: 观察不同侧脸角度下的 `[Latency]` 日志，看姿态输出是否平滑、是否有跳变。

---

## 四、关键坐标系定义

### 人脸坐标系（由 PnP 3D 模板定义）
- **原点**: 鼻尖附近
- **+X**: 人脸左侧（从人脸自身视角看）
- **+Y**: 人脸下方
- **+Z**: 人脸后方（远离相机方向）
- **人脸正面方向**: -Z（指向相机）

### 相机坐标系
- **+Z**: 相机光轴指向场景（指向人脸）
- PnP 输出 `rvec/tvec` 表示: **人脸坐标系 → 相机坐标系** 的变换

### 机械臂基座坐标系
- **原点**: 机械臂 J1 关节中心（或下位机定义的原点）
- **+Z**: 竖直向上
- 下位机回传的当前位姿在此坐标系下定义

---

## 五、文件结构

```
twice/
├── src/
│   ├── main.cpp                     # 主程序：初始化 WiFi/NPU/RGA/蓝牙/RTMP/WebSocket
│   ├── gst_rtmp.cpp/h               # GStreamer RTMP 推流 + RGA YUYV→NV12 转换
│   ├── gst_rtsp.cpp/h               # GStreamer RTSP 服务器（备用）
│   ├── rga_npu.cpp/h                # RGA 预处理 + NPU 推理 + PnP + 滤波 + 绘制
│   ├── wifi.cpp/h                   # WiFi 连接 + DHCP
│   ├── bluetooth_spp.c/h            # BLE GATT 客户端，连接 BT24 接收遥控器指令
│   ├── uart_comm.cpp/h              # UART 串口驱动，与下位机通信
│   ├── ws_client.cpp/h              # 极简 WebSocket 客户端
│   └── uart_loopback_test.cpp       # 串口回环测试
├── models/
│   ├── best.rknn                    # 人体姿态模型 (YOLOv8n-pose 17点)
│   └── face_best.rknn               # 人脸模型 (6点关键点用于 PnP)
├── scripts/
│   ├── calibrate.py                 # 相机标定脚本
│   ├── bt_agent.py                  # 蓝牙自动配对 Agent
│   └── bt_test.py                   # 蓝牙测试
├── calib/
│   ├── images/                      # 标定采集图像
│   ├── calib_result.png             # 标定可视化结果
│   ├── camera_matrix.npy/txt        # 相机内参
│   └── dist_coeffs.npy/txt          # 畸变系数
├── build/
│   └── cc                           # 编译产物
├── docs/
│   └── PROJECT.md                   # 本文件
└── wearable-robot-arm/              # Git 仓库：完整上下位机项目（含 STM32 代码、协议定义、架构文档）
    ├── docs/                          # 架构文档、协议规范、路线图
    │   ├── architecture.md            # 坐标变换方案、正运动学需求、手眼标定方法
    │   ├── roadmap.md                 # 开发状态与待办事项
    │   └── calibration.md             # 相机标定与 PnP 问题分析
    ├── shared/protocol.h              # 上下位机共享通信协议定义
    ├── host-rk3588/                   # 上位机代码（与本项目 src/ 同步）
    └── mcu-stm32/                     # 下位机 STM32H723 固件（Keil MDK）
```

---

## 六、编译命令

```bash
cd /home/elf/work/twice
g++ -DNULL=0 -o build/cc src/main.cpp src/wifi.cpp src/gst_rtsp.cpp src/rga_npu.cpp src/bluetooth_spp.c src/uart_comm.cpp \
  `pkg-config --cflags gstreamer-1.0 gstreamer-rtsp-server-1.0 opencv4 dbus-1` \
  `pkg-config --libs gstreamer-1.0 gstreamer-rtsp-server-1.0 opencv4 dbus-1` \
  -lgstapp-1.0 -lrknnrt -lrga -lwpa_client -lpthread -lstdc++ -lm -O3
```

---

## 七、已做的优化决策记录

1. **USB 摄像头替代 MIPI**: 原 MIPI 摄像头传输距离受限，改用 USB3.0 Realtek 摄像头 (`/dev/video21`)
2. **YUYV + RGA 硬件转换**: 放弃 GStreamer `videoconvert` 软件转换，改用 RGA `imresize` 硬件 YUYV→NV12，节省 ~35~40ms 延迟
3. **GStreamer 缓冲深度最小化**: 所有 `queue`/`appsink`/`appsrc` 的 `max-buffers` 设为 1
4. **PnP 提前**: `solvePnP` 和姿态角计算完成后立即输出延迟日志，再进行 3D 绘制
5. **重投影误差检查已禁用**: 用户信任模型在半脸情况下的鲁棒性，已注释掉 reproj_error > 35px 的过滤逻辑
6. **日志精简**: 只保留 `[Latency]`、模式切换、错误日志，其他周期性日志已注释
7. **鼻子 Z 坐标修正**: 从 `-70` 改回 `-90`，解决 PnP 镜像歧义性导致的 3D 框跳变问题

---

## 八、改动记录

### 2026-05-15

**背景**: 用户要求启用 WebSocket 连云服务器，并排查 RTMP 推流失败问题。

**改动内容**:

1. **启用 WebSocket 上报** (`main.cpp`)
   - 取消注释 `ws_worker_thread` 的创建代码
   - 编译时加入 `ws_client.cpp`
   - WebSocket 自动连接 `ws://47.93.162.124/ws?deviceId=device-003`，每 100ms 发送心跳帧

2. **修复 ALSA 音频设备** (`gst_rtmp.cpp`)
   - 确认 `hw:3,0` 为 USB 无线麦克风设备，保持不变

3. **采集分辨率从 1080p 降至 480p** (`gst_rtmp.cpp`)
   - **原因**: `v4l2-ctl` 确认该 Realtek 摄像头 (`0bda:5858`) 在 YUYV 格式下 1920×1080 只能跑 **5fps**（USB 2.0 带宽瓶颈），而 640×480 可跑 **30fps**
   - `v4l2src` caps: `1920×1080 YUY2` → `640×480 YUY2`
   - `appsrc` caps: `1920×1080 NV12` → `640×480 NV12`
   - RGA 转换尺寸: `1920×1080` → `640×480`
   - 编码码率: `4Mbps` → `1.5Mbps`
   - **NPU 输入不变**: `rga_npu.cpp` 内部仍通过 `imresize` 将 `640×480 NV12` 拉伸到 `640×640 RGB` 给模型，处理方式与之前完全一致

4. **1080p 版本备份**
   - 原始 `gst_rtmp.cpp`（1920×1080 采集）备份至 `backup/2026-05-16/`
   - 若后续摄像头支持 YUYV 1080p30（如更换为 USB 3.0 摄像头），可从备份恢复

**影响**:
- AI 推理链路（NPU 输入 640×640、后处理、PnP、UART 下发）**完全不受影响**
- RTMP 推流画面从 1080p 降为 480p
- 延迟不受影响，帧率从 5fps 恢复至 30fps

---

## 八、关键问题分析：PnP 镜像解跳变

### 现象
USB 摄像头替换后，正脸时 3D 框没有中间状态：人脸稍偏左→框偏左很多，人脸稍偏右→框直接跳到偏右很多。

### 根因
PnP 的 6 个 3D 关键点（双眼、鼻尖、嘴角、下巴）近似共面。当 **fx 估算不准** + **3D 点过于共面** 时，`solvePnP` 的迭代法容易在"真实解"和"镜像解"之间横跳。

| 条件 | MIPI (real) | USB (twice) |
|------|------------|-------------|
| fx | `1371` ✅ 准确标定 | `689.58` ✅ 已标定 |
| 鼻子 Z | `-70`（比眼睛深 10mm） | `-90`（比眼睛深 30mm） |
| 结果 | 稳定，不跳变 | `-70` 时跳变；`-90` 后稳定 |

**结论**：`-70` 本身没错，但在 fx 不准时让 3D 点过于接近共面，放大了 PnP 歧义性。`-90` 增强了非共面性（鼻子深 30mm），消除了歧义。

---

## 十、YUYV vs MJPG 格式对比分析（30fps 基准）

### 背景
该摄像头支持两种输出格式：YUYV（未压缩）和 MJPG（JPEG 压缩）。以下对比基于 **USB3.0 正常工作状态**（即摄像头修复后的目标状态）。

### 带宽与帧率

| 格式 | 单帧数据量 | USB3.0 传输时间 | USB2.0 实际帧率 |
|------|-----------|----------------|----------------|
| **YUYV 1080p** | 1920×1080×2 = **4.15 MB** | ~3~5 ms | **~5 fps**（带宽瓶颈） |
| **MJPG 1080p** | 压缩后 **~100~300 KB** | ~0.5~1 ms | **30 fps** 无压力 |

> 注：当前摄像头因固件故障降级为 USB2.0，YUYV 1080p 只能跑 5fps。MJPG 不受此影响。

### Frame → PnP 延迟拆解（30fps）

| 环节 | YUYV 路径 | MJPG 路径 | 差异 |
|------|----------|----------|------|
| USB 传输 | ~3~5 ms | ~0.5~1 ms | MJPG 省 ~3~4 ms |
| 格式转换 | RGA YUYV→NV12 **<1 ms** | mppjpegdec 硬件解码 **~5~10 ms** | MJPG 多 ~5~9 ms |
| NPU 推理 | ~10~15 ms | ~10~15 ms | 持平 |
| PnP + 绘制 | ~10~20 ms | ~10~20 ms | 持平 |
| 其他开销 | ~5~10 ms | ~5~10 ms | 持平 |
| **Frame→PnP 总计** | **~56~62 ms（实测）** | **~55~65 ms（预估）** | **基本持平** |

### 结论

1. **USB3.0 正常时**：YUYV 和 MJPG 的 Frame→PnP 延迟几乎一样（~56~62ms）。YUYV pipeline 更简单（RGA 一步格式转换），MJPG 多了 JPEG 解码环节，省下的 USB 传输时间被解码时间抵消。

2. **USB2.0 fallback 时（当前故障状态）**：
   - YUYV：只能 5fps，帧周期 200ms，总延迟 >200ms
   - MJPG：仍可达 30fps，Frame→PnP 仍保持 ~55~65ms
   - **这是 MJPG 的唯一优势：在 USB 带宽受限时维持高帧率**

3. **建议**：
   - 摄像头修好后（恢复 USB3.0），**继续使用 YUYV**，pipeline 更简洁，省去 JPEG 编解码环节
   - 仅在 USB2.0 fallback 或需要同时跑多路视频时，才考虑 MJPG

## 九、相机标定结果与方法

### 标定结果（2026-04-26）
| 参数 | 值 |
|------|-----|
| **fx** | 689.58 |
| **fy** | 686.99 |
| **cx** | 982.87 |
| **cy** | 394.59 |
| **k1** | -0.140459 |
| **k2** | 0.270074 |
| **p1** | 0.000120 |
| **p2** | 0.003055 |
| **k3** | -0.395587 |
| **RMS 误差** | 0.9759 px |

> 已写入 `rga_npu.cpp` 和 `dist_coeffs`。

### 为什么需要标定
准确的 fx/fy/cx/cy + 畸变系数能让：
- PnP 深度估计 (`tvec[2]`) 更准确
- 3D 框透视比例更真实
- 彻底杜绝镜像解风险

### 标定步骤

#### 1. 准备棋盘格
打印一张棋盘格（推荐 **9×6 内角点**，即 10×7 个黑白方块），每个格子边长 **25mm**，贴在硬纸板或平板上。

#### 2. 运行标定脚本
```bash
cd /home/elf/work/twice
python3 scripts/calibrate.py
```

脚本会自动：
- 打开 USB 摄像头 (`/dev/video21`)
- 实时检测棋盘格角点
- 按 `[空格]` 保存检测到角点的帧（建议保存 15~20 张不同角度）
- 按 `[c]` 开始标定计算
- 输出 `fx, fy, cx, cy` 和畸变系数 `k1, k2, p1, p2, k3`

#### 3. 更新代码
把标定结果填入 `src/rga_npu.cpp`：
```cpp
static const cv::Mat CAMERA_MATRIX = (cv::Mat_<float>(3,3) <<
    fx, 0.0f, cx,
    0.0f, fy, cy,
    0.0f, 0.0f, 1.0f);

static const cv::Mat DIST_COEFFS = (cv::Mat_<float>(1,5) << k1, k2, p1, p2, k3);
```

> 注意：当前 `DIST_COEFFS` 是 `zeros(4,1)`，标定后应改为 1×5 的畸变系数。同时 `projectPoints` 调用处的 `DIST_COEFFS` 参数类型需要匹配。


---

## 十一、2026-05-16 最新改动记录

### 当前状态
- **工作版本**: MJPG 1920×1080 @ 30fps + `mppjpegdec` 硬件解码
- **备份版本**: YUYV 1920×1080 @ 5fps（`backup/yuyv-5fps/`）

### 1. MJPG 改造（解决 USB2.0 带宽瓶颈）

由于摄像头固件故障降级为 USB2.0，YUYV 1080p 只能跑 5fps。改用 MJPG 恢复 30fps：

| 文件 | 改动 |
|------|------|
| `gst_rtsp.cpp` | Pipeline: `YUY2` → `image/jpeg → mppjpegdec → NV12` |
| `gst_rtsp.cpp` | `new_sample_cb`: 去掉 `convert_yuyv_to_nv12()`，改为直接 `memcpy`（mppjpegdec 已输出 NV12） |
| `rga_npu.cpp` | OneEuroFilter `freq` 恢复为 **30.0/15.0 Hz** |

### 2. 10 秒停止问题修复

**根因**: `appsrc` 默认 `max-bytes=200000`，而一帧 NV12 为 3.1MB。加上误加的 `is-live=TRUE`，导致 30fps 下 `appsrc` queue 堆积，pipeline 状态机异常。

**修复**:
- 去掉 `is-live=TRUE` / `do-timestamp=TRUE`
- 增大 `appsrc max-bytes` 到 `1920×1080×2×2 ≈ 8MB`
- 去掉 `GST_BUFFER_COPY_META`（`mppjpegdec` 的硬件帧 meta 复制到新 buffer 会导致下游 encoder 读取越界）

### 3. YUYV 5fps 全备份

完整 YUYV 5fps 版本已备份至：
```
backup/yuyv-5fps/
```
包含所有源码、模型、标定文件，可直接编译运行。与当前 MJPG 版本的区别仅在于：
- `gst_rtsp.cpp`: YUY2 pipeline + RGA `convert_yuyv_to_nv12()`
- `rga_npu.cpp`: OneEuroFilter `freq=5.0/5.0 Hz`

### 4. 已知问题

#### PnP 解算异常
- **现象**: 人脸关键点检测正常（2D 点位置正确），但 3D 立方体有畸变/抖动
- **可能原因**: 
  - MJPG 有损压缩导致关键点位置有亚像素级偏移，PnP 对位置敏感
  - `mppjpegdec` 输出 buffer 有 stride/padding（实测 buffer size=3,133,440，理论 NV12=3,110,400），`memcpy` 只复制理论大小，可能截断数据
- **状态**: 待修复

---

> 编译命令（当前 MJPG 版本）:
> ```bash
> cd /home/elf/work/twice
> g++ -DNULL=0 -o build/cc src/main.cpp src/wifi.cpp src/gst_rtsp.cpp src/rga_npu.cpp src/bluetooth_spp.c src/uart_comm.cpp \
>   `pkg-config --cflags gstreamer-1.0 gstreamer-rtsp-server-1.0 opencv4 dbus-1` \
>   `pkg-config --libs gstreamer-1.0 gstreamer-rtsp-server-1.0 opencv4 dbus-1` \
>   -lgstapp-1.0 -lrknnrt -lrga -lwpa_client -lpthread -lstdc++ -lm -O3
> ```
