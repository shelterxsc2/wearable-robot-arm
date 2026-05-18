# RK3588 视觉上位机

## 文件说明

| 文件 | 说明 |
|------|------|
| `main.cpp` | 主程序：初始化 WiFi/NPU/RGA/蓝牙/RTSP |
| `rga_npu.cpp/h` | RGA 预处理 + NPU 推理 + PnP + 滤波 + 绘制 |
| `gst_rtsp.cpp/h` | GStreamer RTSP 服务器（MJPG→NV12→H.264） |
| `gst_rtmp.cpp/h` | GStreamer RTMP 推流（云端备份方案） |
| `wifi.cpp/h` | WiFi 连接 + DHCP/静态 IP fallback |
| `bluetooth_spp.c/h` | BLE GATT 客户端，接收遥控器指令 |
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

## 模型文件

- `models/face_best.rknn` — 人脸检测（6 关键点用于 PnP）
- `models/best.rknn` — YOLOv8n-pose 人体检测（17 COCO 关键点）

## 标定

```bash
cd scripts
python3 calibrate.py
```

标定结果已写入 `rga_npu.cpp` 中的 `CAMERA_MATRIX` 和 `DIST_COEFFS`。

