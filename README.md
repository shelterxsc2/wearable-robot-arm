# 可穿戴机械臂视觉控制系统

## 项目概述

本项目是一个**可穿戴三轴机械臂**的完整控制系统：

- **上位机**: RK3588 (ELF2 开发板)，负责实时视觉感知、AI 推理、人脸 3D 位姿估计、目标位姿生成
- **下位机**: STM32H723 (裸机 HAL)，负责机械臂电机驱动、S 曲线速度规划、MIT 力矩控制、重力补偿
- **通信**: UART 串口（主控），蓝牙 BLE SPP（遥控器指令）

## 仓库导航

| 文档 | 内容 |
|------|------|
| [`docs/architecture.md`](docs/architecture.md) | **系统架构、坐标变换方案、数据流、正运动学需求** |
| [`docs/protocol.md`](docs/protocol.md) | 上下位机 UART 通信协议帧格式 |
| [`docs/roadmap.md`](docs/roadmap.md) | 开发状态、待办事项、已知问题 |
| [`docs/calibration.md`](docs/calibration.md) | 相机标定、PnP 问题分析、MJPG/YUYV 对比 |
| [`host-rk3588/README.md`](host-rk3588/README.md) | 上位机编译、文件说明、模型信息 |
| [`mcu-stm32/README.md`](mcu-stm32/README.md) | 下位机工程结构、电机配置、导入说明 |

## 仓库结构

```
.
├── docs/                    # 设计文档
├── shared/                  # 上下位机共享定义（协议、数据结构）
│   └── protocol.h
├── host-rk3588/             # 上位机代码（RK3588，C/C++）
│   ├── main.cpp
│   ├── rga_npu.cpp/h        # 视觉链路：RGA + NPU + PnP + 滤波
│   ├── uart_comm.cpp/h      # UART 串口通信
│   ├── gst_rtsp.cpp/h       # RTSP 推流
│   ├── models/              # RKNN 模型
│   ├── scripts/             # Python 辅助脚本
│   └── calib/               # 相机标定数据
└── mcu-stm32/               # 下位机代码（STM32H723，Keil MDK）
    ├── Core/                # CubeMX 生成 + main.c
    ├── RoboticArmControlSDK/# 用户业务代码
    │   ├── API/             # 顶层控制逻辑
    │   ├── Core/            # 控制算法 + 电机驱动
    │   ├── Config/          # 结构体定义 + 机械参数
    │   └── Port/            # HAL 移植层
    └── MDK-ARM/             # Keil 工程文件
```

## 快速开始

### 上位机（RK3588）

```bash
cd host-rk3588
g++ -DNULL=0 -o cc main.cpp wifi.cpp gst_rtsp.cpp rga_npu.cpp bluetooth_spp.c \
  `pkg-config --cflags gstreamer-1.0 gstreamer-rtsp-server-1.0 opencv4 dbus-1` \
  `pkg-config --libs gstreamer-1.0 gstreamer-rtsp-server-1.0 opencv4 dbus-1` \
  -lgstapp-1.0 -lrknnrt -lrga -lwpa_client -lpthread -lstdc++ -lm -O3
./cc
```

### 下位机（STM32）

使用 Keil MDK 打开 `mcu-stm32/MDK-ARM/RoboticArmControlProject.uvprojx`，编译并烧录。

## 通信速览

- **物理层**: UART TTL，波特率 921600
- **帧格式**: `[0xAA][0x55][LEN][CMD][payload...][CRC8]`
- **关键命令**:
  - `0x01` HEARTBEAT — 双向保活
  - `0x10` TARGET_POSE — 上位机 → 下位机（目标 6DOF 位姿，28 字节）
  - `0x20` CURRENT_POSE — 下位机 → 上位机（当前末端位姿，28 字节）
  - `0x30` ARM_TARGET — 上位机 → 下位机（简化指令，10 字节）

> ⚠️ **注意**: 当前 STM32 实际使用的是简化 10 字节 raw 格式（X,Y,Z 小端 int16 cm + k1,k2 小端 int16 °），尚未采用 `shared/protocol.h` 中定义的完整帧格式。详见 [`docs/architecture.md`](docs/architecture.md) 中的通信不匹配说明。
