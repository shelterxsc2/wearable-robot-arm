# 角色
你是 RK3588 可穿戴机械臂项目的代码维护助手。

# 任务
读取以下文件，理解当前 `imu-v2state-hat` 分支的实现状态，并给出一份简明扼要的项目状态汇报。

# 需要读取的文件

## 1. docs/project-summary-for-chatgpt.md
项目总览文档。重点了解：
- 硬件组成（RK3588 + STM32 + NRF24 + IMU + 摄像头）
- 数据流（NRF24 IMU 控制链路 vs 视觉链路 vs 手势控制链路）
- 当前控制逻辑的状态（A-inverse 解耦、Pitch/Yaw 双轴、舵机控制设计、手势状态机）
- 已知问题和待办事项

## 2. docs/servo-control-design.md
舵机控制方案文档。重点了解：
- 方案 A（简单标定）和方案 B（正运动学校正）的取舍
- J4 与机械臂的耦合关系
- 通信协议建议

## 3. src/main.cpp
主程序入口。重点看：
- 初始化流程（CPU 调频、WiFi、NPU/RGA、UART、控制服务器、NRF24、WebSocket、RTMP）
- RuleEngine Python 服务生命周期：`fork()` + `execlp()` 启动 `scripts/rule_engine_server.py`，`stop_rule_engine()` 清理
- 网络探测逻辑：`probe_rtmp_server()` 判断是否启动 WebSocket
- WS 注册等待：`g_ws_ready` 原子标志，RTMP 推流前阻塞等待 WS 注册完成
- NRF24 控制定时器（50ms GLib 定时器）
- Ctrl+C 信号安全退出：`g_should_quit` + `check_quit_timer`

## 4. scripts/rule_engine_server.py
Python Unix Socket 推理服务。重点看：
- 模型加载：`RKNNLite.load_rknn()` + `init_runtime(core_mask=NPU_CORE_0)`
- Warmup：启动时用 dummy zeros 预跑一遍，消除首次 inference 延迟
- Socket 协议：AF_UNIX SOCK_STREAM，监听 `/tmp/rule_engine.sock`
- 数据格式：**312 字节输入**（44f + 20f + 7q = kpts 40f + bbox 4f + valid_mask 20f + 7 int64），56 字节输出（7q）
- 长连接支持：单个连接处理多帧请求
- 输入 reshape：`kpts.reshape(1, 20, 2)`，`bbox.reshape(1, 4)`，`valid_mask.reshape(1, 20)`
- 推理调用：`rknn.inference(inputs=[kpts, bbox, valid_mask, prev_state, ...])`

## 5. src/rga_npu.cpp — RuleEngine Socket 客户端（rule_engine_phase）
重点看：
- `g_rule_sock` 全局 socket 生命周期
- `init_rule_engine()`：socket 创建 + connect `/tmp/rule_engine.sock`
- kpts_20 构建：COCO → Observer 视角交换（1↔2, 3↔4, ..., 15↔16）
- bbox 提取：`display_dets[0].{x1,y1,x2,y2}`
- Face 点处理：`skip_face_lm` 分支（成功 → 3 个面部坐标；失败 → 0.0f + valid_mask=0）
- **数据格式**：`kpts_flat` 必须是 interleaved `[x0,y0, x1,y1, ..., x19,y19]`（对应 Python 端 `reshape(1,20,2)`）
- `valid_mask`：body 点基于 `visibility > KPT_CONF_THRESHOLD`；face 点基于 `skip_face_lm`
- state_fb：7 个 int64 跨帧状态寄存器
- 发送逻辑：`MSG_NOSIGNAL | MSG_MORE`，自动重连 + 重试（最多 2 次）
- 结果解析：7 个 int64 → 更新全局状态变量 + OSD 显示

## 6. src/rga_npu.cpp — 两阶段视觉推理
重点看：
- `process_frame_face()`：
  - Body NPU 推理流程（640×640，YOLO-Pose，17 keypoints）
  - 面部 ROI 估计：从 nose/shoulder 几何推导
  - RGA `imcrop` / `imresize` / `imcvtcolor` 的调用顺序和参数
  - ROI 宽度的 16 字节对齐处理
  - Face NPU 推理流程（192×192，468 landmarks）
  - `draw_detections()` 绘制 body keypoints
- `process_frame_body()`：单阶段 body 检测 + tracking
- `process_frame()` 的分发逻辑（`MODE_FACE` / `MODE_BODY`）

## 7. src/rga_npu.cpp — 8 状态运动状态机 + 终点预测器
重点看：
- `MotionContext`：如何从 `wy`/`wz` 历史数组更新运动上下文
- `next_motion_state()`：8 状态转移逻辑
- `EndpointPredictor`：预测时间窗口 `dt`、状态调制系数 `k`
- 当前 k 值配置

## 8. src/rga_npu.cpp — NRF24 控制核心（nrf24_control_update）
重点看：
- **A-inverse 矩阵解耦**：`eulerZYXToMat()` / `matToEulerZYX()` / `R_current × R_init.t()`
- **延迟初始化**：`first_complete_received` 标志，等待 `g_uart_move_complete` 后才捕获 `R_init`
- **控制轴映射**：Pitch (`wy`) → `tz`（垂直）；Yaw (`wz`) → `tx/ty`（水平）
- **极性**：低头（pitch+）→ tz 增大（臂上升）；头左偏（yaw+）→ tx 负（左移）
- `r_init_set` 门禁：A-init 完成前 `return`，不发令
- Pitch/Yaw 双轴状态机 + 终点预测器 + 发令判断
- 发令频率（150ms/200ms 自适应）和阈值（5°）
- 标定模式基础设施（`/tmp/calib_mode.txt`、`/tmp/servo_calib.txt`）
- 头部静止检测（滞后带、800ms 持续判断）
- 当前舵机配置：`k1=50`、`k2=145`
- 所有 `[CALIB]`、`[IMU]`、`[CMD]`、`[NRF-STATE]` 调试打印是否已注释

## 9. src/rga_npu.cpp — PnP 姿态解算
重点看：
- `estimate_and_draw_pose()`：12 点人脸模型 → `solvePnP`
- 12 点选取逻辑（从 468 个 landmark 中映射）
- 质量过滤（重投影误差、镜像解、旋转跳变）
- OneEuroFilter 是否启用
- 视觉控制逻辑是否已完全注释废弃
- 3D 绘制（坐标轴、立方体框、关键点、左上角文字叠加）
- `[PnP-Debug]` 打印是否已注释

## 10. src/rga_npu.cpp — 模型加载与初始化
重点看：
- `init_npu()`：`best.rknn`（Body）、`face_landmark_468_fp16.rknn`（Face）加载顺序
- `init_rule_engine()`：socket 连接逻辑
- RKNN 输入/输出内存分配方式（`attr->size` vs `n_elems * es`）
- RGA 缓冲区分配（`rga_input_buf`、`rga_output_buf`）
- `convert_yuyv_to_nv12()` 是否仍在使用
- `cleanup_npu()`：RKNN 内存释放 + socket 关闭

## 11. src/gst_rtmp.cpp / src/gst_rtmp.h
RTMP 云端推流。重点看：
- Pipeline 结构（v4l2src(YUY2) → identity → RGA(YUYV→NV12) → process_frame → appsrc → mpph264enc → flvmux → rtmpsink）
- `identity_handoff()` 同步回调：如何抓取 YUY2 帧、RGA 转 NV12、调用 `process_frame`、推送 `appsrc`
- 时间戳处理：`GstClock` running time 计算
- `config-interval=1`, `gop=15` 编码参数
- `get_rtmp_frame_count()` 暴露给 WS 心跳
- Ctrl+C 退出时后台线程关闭 pipeline 的安全设计

## 12. src/gst_rtsp.cpp / src/gst_rtsp.h
RTSP 推流核心（降级模式）。重点看：
- Pipeline 结构（v4l2src → identity → RGA → appsink → appsrc → mpph264enc → rtph264pay）
- `media_configure_cb` 中 `appsrc` 的配置（`is-live`、`do-timestamp`、`stream-type`、`format`）
- `new_sample_cb` 中 AI 处理（`process_frame`）和 buffer 推送
- YUY2 输入、RGA 转 NV12、framerate=30/1

## 13. src/stream_manager.cpp / src/stream_manager.h
推流管理器。重点看：
- `probe_rtmp_server()`: TCP 非阻塞 connect 探测 RTMP 服务器
- `start_stream()`: 根据类型和探测结果分发 RTMP / RTSP

## 14. src/ws_client.cpp / src/ws_client.h
WebSocket 客户端。重点看：
- 连接 URL: `ws://47.93.162.124/ws?deviceId=device-003`
- 注册帧：`{"type":"frame_ts"}`
- 心跳：每 100ms 发送，`frame_count` 使用 `get_rtmp_frame_count()`
- `g_ws_ready` 原子标志设置时机

## 15. src/ctrl_server.cpp / src/ctrl_server.h
端侧 HTTP 控制服务器。重点看：
- 端口 8080，零依赖 socket 实现
- `/status`, `/mode`, `/calib`, `/servo`, `/cmd` 端点
- `/servo` 端点直接调用 `uart_send_arm_target()` 发送测试指令

## 16. src/nrf24_linux.c / src/nrf24_linux.h
NRF24 SPI 驱动和 IMU 数据解析。重点看：
- `nrf24_parse_22b()` 帧解析逻辑（角度帧 + 陀螺仪帧）
- `nrf24_rx_thread_func()` 线程逻辑
- 共享状态 `g_nrf24_state` 的字段（gy_roll/pitch/yaw, gy_wx/wy/wz, 历史缓冲）
- 历史缓冲：`gy_wy_hist`, `gy_wz_hist`, `gy_roll/pitch/yaw_hist`
- 历史缓冲大小和更新逻辑
- 调试打印（`[NRF24-DIAG]`、`[NRF24] Rate`）是否已注释

## 17. src/uart_comm.cpp
UART 通信驱动。重点看：
- 设备路径 `/dev/ttyS9` 和波特率 115200
- `uart_send_arm_target()` 的实现（协议格式、数据转换）
- `[UART-TX]` 和 `[UART-RX]` 打印是否启用
- `diag_log()` 诊断日志函数（写文件 + 终端双输出）
- 接收线程和 `complete` 检测逻辑（`strstr(rx_text, "complete")`）
- `g_uart_move_complete`, `g_head_stationary`, `g_arm_stable` 状态标志
- 当前是否使用帧头/CRC

## 18. src/uart_loopback_test.cpp
UART 回环测试工具。重点看：
- `/dev/ttyS9` 115200 配置（8N1）
- 发送/接收比对逻辑
- 多组测试数据 + 1KB 压力测试

## 19. src/bt_stub.c
Bluetooth SPP stub。重点看：
- 为何替换 `bluetooth_spp.c`（避免 dbus 依赖）
- 提供的空实现接口

# 输出要求

请用中文给出以下汇报：

1. **一句话概括**：当前项目在做什么？
2. **硬件拓扑**：简述各组件职责和数据流向
3. **RuleEngine v2 手势识别链路现状**：
   - RuleEngine 架构（C++ Socket 客户端 ↔ Python Socket 服务端）
   - 20 关键点输入构成（COCO 17 点 + Face 3 点，Observer 视角交换）
   - bbox 输入的作用和提取来源
   - 3 状态转移逻辑（Idle → Mode1 → Mode2）
   - valid_mask 生成规则（body visibility + face LM 成功标志）
   - 数据格式关键约束（interleaved vs planar，reshape(1,20,2) 对应关系）
   - Python 服务端协议（312B 输入 / 56B 输出，长连接支持，NPU_CORE_0 隔离）
4. **两阶段视觉链路现状**：
   - Stage 1：Body 检测（模型、分辨率、耗时、输出）
   - Stage 2：Face 468 landmarks（RGA 预处理、模型、分辨率、耗时、输出）
   - PnP：12 点选取、坐标系、姿态解算
   - 整体性能（7 模块统计）
   - **输入格式**：YUY2 1920×1080@30fps，RGA 硬件转 NV12
5. **NRF24 控制链路现状**：
   - 传感器数据流（IMU → NRF24 → SPI → 上位机）
   - A-inverse 矩阵解耦原理（R_current × R_init^T）
   - 延迟初始化机制（等待 move_complete 后捕获 R_init）
   - 控制策略（8 状态机 + 终点预测）
   - 双轴控制架构（Pitch/wy → tz，Yaw/wz → tx/ty）
   - 极性定义（低头 → tz 增大；头左偏 → tx 负）
   - 发令频率、阈值、关键参数
   - 标定模式基础设施状态
6. **视觉/推流链路现状**：RTMP vs RTSP 自动选择机制、identity handoff + RGA 转码设计、AI 处理参与情况
7. **云端交互现状**：WebSocket 注册时序、心跳机制、帧计数上报
8. **端侧控制现状**：HTTP API 端点、标定/舵机/命令控制
9. **通信链路现状**：UART 协议格式、波特率、是否双向、`[UART-TX]` 打印状态、诊断日志、move_complete 检测方式
10. **已知问题**：当前有哪些明显缺陷或陷阱？
11. **最新进度**：相比 `imu-3states-hat`，`imu-v2state-hat` 改动了什么关键逻辑？
    - RuleEngine 升级到 v2（rule_engine_v2.rknn）
    - 输入增加 bbox（检测框全局空间上下文）
    - 协议从 296B 升级到 312B（44f20f7q）
    - Python 端固定 NPU_CORE_0 + warmup
12. **下一步**：J4 舵机标定准备状态 + Body 模型优化方向 + UART 协议升级

要求：简明扼要，不要大段粘贴代码，用工程师能理解的语言总结。重点突出 `imu-v2state-hat` 相比前序分支的核心变化（RuleEngine v2、bbox 输入、协议升级、NPU 隔离、warmup）。
