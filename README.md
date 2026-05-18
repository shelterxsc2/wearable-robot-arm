# 可穿戴机械臂视觉控制系统

## 项目概述

本项目是一个**可穿戴三轴机械臂**的完整控制系统，由上位机（视觉感知 + 位姿规划）和下位机（电机 LQR 控制闭环）组成。

- **上位机**: RK3588 (ELF2 开发板)，运行 Linux，负责实时视觉感知、AI 推理、人脸 3D 位姿估计、目标位姿生成
- **下位机**: STM32 裸机开发，负责机械臂电机驱动、LQR 控制闭环、安全限位、状态回传
- **通信**: UART 串口（主），蓝牙 BLE SPP（遥控器指令）

## 仓库结构

```
.
├── docs/               # 设计文档、通信协议说明
├── shared/             # 上下位机共享定义（通信协议、数据结构）
├── host-rk3588/        # 上位机代码（RK3588，C/C++）
│   ├── models/         # RKNN NPU 模型文件
│   ├── scripts/        # Python 辅助脚本（标定、蓝牙测试等）
│   └── calib/          # 相机标定数据
└── mcu-stm32/          # 下位机代码（STM32，裸机/寄存器/HAL）
```

## 快速开始

### 上位机（RK3588）

```bash
cd host-rk3588
# 编译
g++ -DNULL=0 -o cc main.cpp wifi.cpp gst_rtsp.cpp rga_npu.cpp bluetooth_spp.c \
  `pkg-config --cflags gstreamer-1.0 gstreamer-rtsp-server-1.0 opencv4 dbus-1` \
  `pkg-config --libs gstreamer-1.0 gstreamer-rtsp-server-1.0 opencv4 dbus-1` \
  -lgstapp-1.0 -lrknnrt -lrga -lwpa_client -lpthread -lstdc++ -lm -O3
# 运行
./cc
```

### 下位机（STM32）

见 `mcu-stm32/README.md`

## 通信协议

上下位机通过 UART 串口通信，协议定义见 `shared/protocol.h` 和 `docs/protocol.md`。

## 开发状态

- [x] 视觉感知链路（采集 → AI → PnP → 推流）
- [x] 串口驱动框架
- [ ] 上位机-下位机控制闭环（待联调）
- [ ] 蓝牙遥控器扩展

