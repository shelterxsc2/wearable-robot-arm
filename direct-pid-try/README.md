# Direct PID Try — 人脸跟随验证版

## 版本说明

这是可穿戴机械臂视觉系统的**快速验证分支**，核心目标：人脸偏航（yaw）+ 俯仰（pitch）双轴跟随，舵机固定，只控制机械臂三个关节位置。

## 核心特点

| 特性 | 实现 |
|---|---|
| **偏航跟随** | 带死区的积分器（±5° 死区，±90° 限幅） |
| **俯仰跟随** | **PI 控制**（P 项快速响应 + I 项消除稳态） |
| **舵机** | 固定 90° / 157°，全程不动 |
| **通信** | 10 字节 raw 格式 UART（兼容 STM32 当前固件） |

## 关键参数

```cpp
// 人脸位置（基座系，原点为云台电机）
FACE_X_CM = 0.0f, FACE_Y_CM = 10.0f, FACE_Z_CM = 35.0f;
TRACK_DIST_CM = 57.0f;          // 人脸正前方水平距离
CAM_HEIGHT_OFFSET_CM = 5.0f;    // 相机比人脸高 5cm

// 偏航：纯积分
CHASE_K = 0.08f;
DEAD_ZONE_RAD = 5°;

// 俯仰：PI 控制
KP_PITCH = 2.0f;                // 比例系数
KI_PITCH = 0.06f;               // 积分系数
PITCH_BIAS_DEG = 11.5f;         // PnP 俯仰基准偏移（正视时归零）
PITCH_RANGE_CM = 15.0f;         // 上下活动范围
```

## 编译

```bash
cd direct-pid-try/src
g++ -DNULL=0 -o ../cc main.cpp wifi.cpp gst_rtsp.cpp rga_npu.cpp bluetooth_spp.c uart_comm.cpp \
  `pkg-config --cflags gstreamer-1.0 gstreamer-rtsp-server-1.0 opencv4 dbus-1` \
  `pkg-config --libs gstreamer-1.0 gstreamer-rtsp-server-1.0 opencv4 dbus-1` \
  -lgstapp-1.0 -lrknnrt -lrga -lwpa_client -lpthread -lstdc++ -lm -O3
```

## 文件结构

```
direct-pid-try/
├── src/              # RK3588 上位机源码
├── models/           # RKNN 模型（face_best.rknn + best.rknn）
├── scripts/          # 标定 + 蓝牙脚本
├── calib/            # 相机标定数据
└── docs/PROJECT.md   # 完整项目文档
```

## 与主线版本的区别

| | 主线 host-rk3588 | 本分支 direct-pid-try |
|---|---|---|
| 偏航控制 | 无 / 简化映射 | 带死区积分器 |
| 俯仰控制 | 无 | **PI 控制** |
| 舵机 | 动态跟踪 | 固定 90/157 |
| 坐标变换 | 未实现 | 简化固定几何 |
| 目标 | 验证 PnP + 基础闭环 | 验证双轴跟随可行性 |

## 状态

验证通过：人脸偏航/俯仰双轴均有跟踪响应，运动趋势正确，参数待进一步优化。
