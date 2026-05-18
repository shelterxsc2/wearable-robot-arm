# RK3588 视觉上位机

## 文件说明

| 文件 | 说明 |
|------|------|
| `main.cpp` | 主程序：CPU 调频 → WiFi → NPU → RGA → RTSP/RTMP |
| `rga_npu.cpp/h` | **核心视觉链路**：RGA 预处理 → NPU 推理 → PnP → OneEuro 滤波 → 绘制 |
| `gst_rtsp.cpp/h` | GStreamer RTSP 服务器（MJPG→NV12→H.264） |
| `gst_rtmp.cpp/h` | GStreamer RTMP 推流（连云备份） |
| `wifi.cpp/h` | WiFi 连接 + DHCP/静态 IP fallback |
| `bluetooth_spp.c/h` | BLE GATT 客户端，接收遥控器指令（当前临时禁用） |
| `uart_comm.cpp/h` | UART 串口驱动，与下位机通信 |
| `ws_client.cpp/h` | 极简 WebSocket 客户端（连云服务器） |
| `uart_loopback_test.cpp` | 串口回环测试 |

## 编译

```bash
g++ -DNULL=0 -o cc main.cpp wifi.cpp gst_rtsp.cpp rga_npu.cpp bluetooth_spp.c \
  `pkg-config --cflags gstreamer-1.0 gstreamer-rtsp-server-1.0 opencv4 dbus-1` \
  `pkg-config --libs gstreamer-1.0 gstreamer-rtsp-server-1.0 opencv4 dbus-1` \
  -lgstapp-1.0 -lrknnrt -lrga -lwpa_client -lpthread -lstdc++ -lm -O3
```

> 若启用 WebSocket 或 UART，需把对应 `.cpp` 加入编译命令。

## 模型文件

| 模型 | 用途 | 输入 | 输出 |
|------|------|------|------|
| `models/face_best.rknn` | 人脸检测 | 640×640 RGB | 6 关键点（双眼、鼻尖、嘴角、下巴）用于 PnP |
| `models/best.rknn` | YOLOv8n-pose 人体检测 | 640×640 RGB | 17 COCO 关键点 |

## 标定

```bash
cd scripts
python3 calibrate.py
```

标定结果已写入 `rga_npu.cpp` 中的 `CAMERA_MATRIX` 和 `DIST_COEFFS`。

## 与下位机通信

当前 `uart_comm.cpp` 已实现：
- `uart_send_target_pose()` — 发送 `Pose6D`（但 STM32 尚未解析标准帧格式）
- `uart_send_arm_target()` — 发送简化 10 字节指令（X,Y,Z cm + k1,k2 °）
- `uart_start_receiver()` — 启动接收线程，可注册 `CURRENT_POSE` 回调

详见根目录 [`docs/architecture.md`](../docs/architecture.md) 中的坐标变换方案和通信不匹配说明。
