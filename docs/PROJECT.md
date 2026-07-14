# fourth 项目架构

> 当前代码快照：2026-07-13。迁移来源和未完成验收见 `MIGRATION.md` 与 `HANDOFF.md`。

## 1. 系统目标

在 RK3588 上将视觉、头部/腰部姿态、云端/本地命令和机械臂 H7 控制组合成单一上位机，同时保持实时推流和可扩展情景模式。

## 2. 硬件边界

| 设备 | 职责 |
|---|---|
| RK3588 ELF2 | 本项目运行平台；视觉、控制语义、通信和推流。|
| STM32H723 | 逆运动学、S-curve、电机/舵机执行。|
| USB Camera | 1920×1080 视频输入。|
| NRF24 头部 IMU | 头姿/角速度输入。|
| 板载 IMU2 | 双 IMU 相对姿态参考。|
| `/dev/ttyS9` | RK3588 到 H7 的机械臂通信。|

本项目不需要额外 STM32 bridge。

## 3. 运行时架构

```text
REST ───────┐
Cloud WS ───┤
Bluetooth ──┼→ control_router → mode/profile/target/annotation
Voice future┤
Gesture ────┘

Camera → GStreamer → RGA/RKNN Body ─┬→ Face landmarks/PnP
                                    ├→ INTRO/INTERVIEW observations
                                    ├→ raised-wrist ROI → async Hand sidecar → gesture policy → control_router
                                    └→ H264 → RTMP or RTSP

NRF24 + IMU2 → relative head pose/state machine ─┐
Visual PnP correction ───────────────────────────┼→ target + J4/J5
Mode-specific control ───────────────────────────┘
                                                   ↓
                                             uart_comm mutex
                                                   ↓
                                               STM32H723
```

## 4. 线程/进程

- 主进程：初始化、GStreamer loop、控制和清理。
- GStreamer 回调：同步主视觉推理和 OSD。
- NRF24 RX 线程：采集头部 IMU。
- IMU2 线程：100 Hz 板载姿态采样。
- GLib 50 ms timer：`nrf24_control_update()`。
- HTTP server 线程：端侧 REST。
- WebSocket worker：注册、心跳和下行命令。
- Bluetooth worker：三字节遥控协议。
- Python RuleEngine 子进程：`/tmp/rule_engine.sock`。
- Python HandPipeline 子进程：`/tmp/hand_pipeline.sock`。
- Python VoiceKWS 可降级子进程：`/tmp/voice_kws.sock`，Sherpa-ONNX CPU 推理。
- C++ gesture worker：从有界队列取 ROI，与 HandPipeline 通信。
- C++ voice worker：阻塞读取新鲜关键词事件并调用统一控制路由。

## 5. 模块地图

| 模块 | 责任 |
|---|---|
| `main.cpp` | 初始化顺序、sidecar 生命周期、硬件、服务器和推流。|
| `rga_npu.cpp/h` | Body/Face/PnP、场景视觉、NRF 控制核心和模式状态。|
| `gesture_overlay.cpp/h` | 手部 ROI 异步队列、socket client、左右手 OSD和控制接入。|
| `gesture_control.cpp/h` | 手势到模式/profile 的纯决策状态机。|
| `rule_mode_control.cpp/h` | RuleEngine `0→1/0→2` 到 INTRO/INTERVIEW 的边沿决策。|
| `control_router.cpp/h` | 多来源统一语义入口。|
| `cloud_command.cpp/h` | 云端 JSON 兼容适配。|
| `voice_control.cpp/h` | 语音事件新鲜度检查、模式词映射和路由接入。|
| `arm_power_control.cpp/h` | trial0 对等展开/安全收起、确认窗口及 NRF 发令门控。|
| `cloud_report.cpp/h` | 语音动作产生的 zoom/view/track/annotation/record 上行队列。|
| `first_person_control.cpp/h` | FIRST_PERSON 纯映射。|
| `ctrl_server.cpp/h` | HTTP API。|
| `uart_comm.cpp/h` | H7 物理通信和写互斥。|
| `nrf24_linux.c/h` | NRF24 驱动、解帧和历史。|
| `imu2_i2c.c/h` | 板载 IMU2。|
| `gst_rtmp.cpp/h` | 云端 RTMP 管线。|
| `gst_rtsp.cpp/h` | 本地 RTSP 管线。|
| `stream_manager.cpp/h` | 云端探测和流类型选择。|
| `ws_client.cpp/h` | 云端 WS 上报和下行。|
| `bluetooth_spp.c/h` | HC-08 遥控。|

## 6. 启动顺序

1. 安装 SIGINT/SIGTERM 处理器，清理临时状态，设置 CPU 性能模式和网络；
2. 并行 fork RuleEngine 与 HandPipeline，等待两个 socket 实际就绪；
3. 初始化 Body、Face、RGA 和手势 worker；
4. 初始化蓝牙、UART、HTTP、NRF24、IMU2；
5. 探测 RTMP，失败则 RTSP；
6. 启动 H7 握手和 50 ms NRF 控制定时器；
7. 进入 GStreamer loop。

任一 sidecar 启动失败或运行中退出都会触发统一关闭。Ctrl+C、SIGTERM、推流失败和初始化失败共用同一逆序清理路径；主进程关闭时会停止并回收 sidecar、线程、socket 和监听端口。sidecar 还设置父进程死亡信号，主进程被强制杀死时不会长期变成孤儿。

H7 握手、A-init 和 `g_uart_block_tx` 是安全链的一部分，不应为了调试视觉随意删除。

## 7. 模式与控制

模式为 FACE、BODY、INTRO、INTERVIEW、FIRST_PERSON。详细输入、发令周期和接入方向见 `CONTROL.md`。

视觉约 13～15 FPS 是观察更新速率；机械命令由死区和最小时间间隔节流。INTRO/INTERVIEW 目前约 700 ms 级发令限制，FIRST_PERSON 150 ms，普通头控还依赖运动状态。

## 8. 数据与模型

- `models/best.rknn`：Body YOLO-Pose；
- `models/face_landmark_468_fp16.rknn`：Face 468；
- `models/rule_engine_v2.rknn`：当前运行的 7 维反馈 RuleEngine；
- `models/mode_rule_engine_fp16.rknn`：历史遗留模型，当前构建不加载；
- `models/hand/*`：hand detector、landmark、embedder、canned classifier；
- `models/sherpa-onnx-*`、`models/silero_vad.onnx`：离线中文 KWS 与软 VAD；
- `calib/`：相机内参与畸变；
- `/tmp/*.sock`：两个 sidecar socket；
- `/tmp/calib_mode.txt`、`/tmp/servo_calib.txt`：临时标定控制；
- `/tmp/cmd`：UART RX 调试记录。

## 9. 当前技术债

- `rga_npu.cpp` 同时承载视觉和大量控制逻辑，后续应按模块拆分，但需先建立回归测试；
- control router 缺少真正的来源优先级/租约；
- 云端协议缺真实包固定测试；
- PnP 修正仍是实验态；
- 手势情景控制已接入（每 3 帧采样、连续 3 个有效结果）但缺准确率、误触发和机械实机验收；
- 完整系统没有关节反馈、动力学和碰撞闭环；
- 多份历史设计文档描述旧参数，已加状态标记，当前数值以源码为准。

## 10. 构建与测试

构建命令见根 `README.md`；交接验收和推荐顺序见 `HANDOFF.md`；视觉细节见 `VISION.md`；仿真边界见 `../simulation/README.md`。
