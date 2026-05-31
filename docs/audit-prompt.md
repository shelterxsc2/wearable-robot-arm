# 角色
你是 RK3588 可穿戴机械臂项目的代码维护助手。

# 任务
读取以下文件，理解当前 `imu-rtmp-main` 分支的实现状态，并给出一份简明扼要的项目状态汇报。

# 需要读取的文件

## 1. docs/project-summary-for-chatgpt.md
项目总览文档。重点了解：
- 硬件组成（RK3588 + STM32 + NRF24 + IMU + 摄像头）
- 数据流（NRF24 IMU 控制链路 vs 视觉链路）
- 当前控制逻辑的状态（Roll/Yaw 双轴、舵机控制设计）
- 已知问题和待办事项

## 2. docs/servo-control-design.md
舵机控制方案文档。重点了解：
- 方案 A（简单标定）和方案 B（正运动学校正）的取舍
- J4 与机械臂的耦合关系
- 通信协议建议

## 3. src/main.cpp
主程序入口。重点看：
- 初始化流程（CPU 调频、WiFi、NPU/RGA、UART、控制服务器、NRF24、WebSocket、RTMP）
- 网络探测逻辑：`probe_rtmp_server()` 判断是否启动 WebSocket
- WS 注册等待：`g_ws_ready` 原子标志，RTMP 推流前阻塞等待 WS 注册完成
- NRF24 控制定时器（50ms GLib 定时器）
- Ctrl+C 信号安全退出：`g_should_quit` + `check_quit_timer`

## 4. src/gst_rtmp.cpp / src/gst_rtmp.h
RTMP 云端推流。重点看：
- Pipeline 结构（v4l2src → mppjpegdec → identity → appsrc → mpph264enc → h264parse → flvmux → rtmpsink）
- `identity_handoff()` 同步回调：如何抓取 NV12 帧、调用 `process_frame`、推送 `appsrc`
- 时间戳处理：`GstClock` running time 计算
- `config-interval=1`, `gop=15` 编码参数
- `get_rtmp_frame_count()` 暴露给 WS 心跳
- Ctrl+C 退出时后台线程关闭 pipeline 的安全设计

## 5. src/stream_manager.cpp / src/stream_manager.h
推流管理器。重点看：
- `probe_rtmp_server()`: TCP 非阻塞 connect 探测 RTMP 服务器
- `start_stream()`: 根据类型和探测结果分发 RTMP / RTSP

## 6. src/ws_client.cpp / src/ws_client.h
WebSocket 客户端。重点看：
- 连接 URL: `ws://47.93.162.124/ws?deviceId=device-003`
- 注册帧：`{"type":"frame_ts"}`
- 心跳：每 100ms 发送，`frame_count` 使用 `get_rtmp_frame_count()`
- `g_ws_ready` 原子标志设置时机

## 7. src/ctrl_server.cpp / src/ctrl_server.h
端侧 HTTP 控制服务器。重点看：
- 端口 8080，零依赖 socket 实现
- `/status`, `/mode`, `/calib`, `/servo`, `/cmd` 端点

## 8. src/nrf24_linux.c / src/nrf24_linux.h
NRF24 SPI 驱动和 IMU 数据解析。重点看：
- `nrf24_parse_22b()` 帧解析逻辑（角度帧 + 陀螺仪帧）
- `nrf24_rx_thread_func()` 线程逻辑
- 共享状态 `g_nrf24_state` 的字段（gy_roll/yaw/pitch, gy_wx/wy/wz, 历史缓冲）
- 历史缓冲大小和更新逻辑

## 9. src/uart_comm.cpp
UART 通信驱动。重点看：
- 设备路径和波特率
- `uart_send_arm_target()` 的实现（协议格式、数据转换）
- `[UART-TX]` 和 `[UART-RX]` 打印是否启用
- 接收线程和 `complete` 检测逻辑
- 当前是否使用帧头/CRC

## 10. src/gst_rtsp.cpp
RTSP 推流核心（降级模式）。重点看：
- Pipeline 结构（v4l2src → mppjpegdec → tee → appsink → appsrc → mpph264enc → rtph264pay）
- `media_configure_cb` 中 `appsrc` 的配置（`is-live`、`do-timestamp`、`stream-type`、`format`）
- `new_sample_cb` 中 AI 处理（`process_frame`）和 buffer 推送

## 11. src/rga_npu.cpp（分段读取）

### 第 470~675 行：8 状态运动状态机 + 终点预测器
重点看：
- `MotionContext`：如何从 `wx`/`wz` 历史数组更新运动上下文
- `next_motion_state()`：8 状态转移逻辑
- `EndpointPredictor`：预测时间窗口 `dt`、状态调制系数 `k`
- 当前 k 值配置

### 第 676~1011 行：`nrf24_control_update()` 完整函数
重点看：
- Roll 轴控制：读取 `gy_roll`/`gy_wx`、状态机、预测器、发令判断
- Yaw 轴控制：读取 `gy_yaw`/`gy_wz`、状态机、预测器、发令判断
- "或"逻辑合并发令：`roll_should_cmd || yaw_should_cmd`
- 坐标生成：`tx`/`ty`（Yaw 驱动）和 `tz`（Roll 驱动）
- `ty = 67` 的固定值和几何意义
- 发令频率（150ms/200ms 自适应）和阈值（5°）
- 标定模式基础设施（`/tmp/calib_mode.txt`、`/tmp/servo_calib.txt`）
- 头部静止检测（滞后带、800ms 持续判断）
- 当前舵机配置：`k1=50`、`k2=145`
- 所有 `[CALIB]`、`[IMU]`、`[CMD]`、`[NRF-STATE]` 调试打印是否已注释

### 第 1013~1336 行：`estimate_and_draw_pose()`
重点看：
- PnP 解算流程（6 点人脸模型 → solvePnP）
- 质量过滤（重投影误差、镜像解、旋转跳变）
- OneEuroFilter 是否启用
- 视觉控制逻辑是否已完全注释废弃
- 3D 绘制（坐标轴、立方体框、关键点、左上角文字叠加）

# 输出要求

请用中文给出以下汇报：

1. **一句话概括**：当前项目在做什么？
2. **硬件拓扑**：简述各组件职责和数据流向
3. **NRF24 控制链路现状**：
   - 传感器数据流（IMU → NRF24 → SPI → 上位机）
   - 控制策略（8 状态机 + 终点预测）
   - 双轴控制架构（Roll → tz，Yaw → tx/ty）
   - 发令频率、阈值、关键参数
   - 标定模式基础设施状态
4. **视觉/推流链路现状**：RTMP vs RTSP 自动选择机制、identity handoff 设计、AI 处理参与情况
5. **云端交互现状**：WebSocket 注册时序、心跳机制、帧计数上报
6. **端侧控制现状**：HTTP API 端点、标定/舵机/命令控制
7. **通信链路现状**：UART 协议格式、波特率、是否双向、打印状态
8. **已知问题**：当前有哪些明显缺陷或陷阱？
9. **最新进度**：相比之前版本，最近改动了什么关键逻辑？
10. **下一步**：J4 舵机标定的准备状态

要求：简明扼要，不要大段粘贴代码，用工程师能理解的语言总结。
