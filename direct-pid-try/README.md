# Direct PID Try — 人脸跟随验证版

## 版本说明

这是可穿戴机械臂视觉系统的**快速验证分支**，核心目标：人脸偏航（yaw）单轴跟随，舵机固定，只控制机械臂三个关节位置。

> **注意**：俯仰跟随已禁用（固定高度 `z=40cm`），当前仅验证偏航（yaw）方向跟踪性能。

## 核心特点

| 特性 | 实现 |
|---|---|
| **偏航跟随** | 带死区的积分器 + 速度前馈 + 大角度 P boost + 冻结停稳 + 死区微调 |
| **俯仰跟随** | **已禁用**，固定 `z = 40cm` |
| **舵机** | 固定 50° / 145°，全程不动（SERVO1 为 360° 控速电机，50 = 停止） |
| **通信** | 10 字节 raw 格式 UART（兼容 STM32 当前固件） |
| **发送保护** | 60cm 软限幅（超限发送边界点并同步积分器）+ 20 帧强制突破防死锁 |

## 关键参数

```cpp
// 人脸位置（基座系，原点为云台电机）
FACE_X_CM = 0.0f, FACE_Y_CM = 10.0f, FACE_Z_CM = 40.0f;
TRACK_DIST_CM = 57.0f;          // 人脸正前方水平距离
CAM_HEIGHT_OFFSET_CM = 0.0f;    // z = 40 + 0 = 40

// 舵机固定角度
SERVO1_DEG = 50.0f;   // 360° 控速电机，50 = 停止
SERVO2_DEG = 145.0f;

// ---------- 偏航控制 ----------
DEAD_ZONE_RAD  = 5°;            // 死区：|yaw| ≤ 5° 时冻结 + 微调
CHASE_K        = 0.08f;         // 积分系数
FF_GAIN        = 0.35f;         // 速度前馈系数（基于 yaw 单帧变化量）
BOOST_THRESH   = 10°;           // P boost 触发阈值
BOOST_K        = 0.0125f;       // P boost 系数（|yaw|>10° 时额外叠加）
FINE_K         = 0.008f;        // 死区内微调系数
FINE_MAX       = 5°;            // 微调量上限 ±5°
yaw_delta_clamp= ±0.20 rad/帧;  // 前馈输入限幅，防 PnP 跳变冲击

// ---------- 发送调度 ----------
MAX_STEP_CM    = 60.0f;         // 单帧发送距离软上限（超限则发送边界点）
SKIP_FORCE     = 20 帧;         // 连续 SKIP 超过 20 帧强制发送（防死锁）
```

## 控制逻辑详解

### 偏航三状态机

```
┌─────────────────────────────────────────────────────────┐
│  |yaw| > 5° (非死区)                                     │
│  ├─ big_integ = CHASE_K * (yaw - sign(yaw)*5°)          │
│  ├─ feedforward = FF_GAIN * yaw_delta                   │
│  ├─ boost = BOOST_K * yaw  (仅 |yaw| > 10°)             │
│  └─ cum_yaw += big_integ + feedforward + boost          │
├─────────────────────────────────────────────────────────┤
│  |yaw| ≤ 5° (死区内)                                     │
│  ├─ 前 3 帧：继续累加 feedforward（缓冲）                │
│  ├─ 3 帧后：frozen = true，冻结 cum_yaw                  │
│  └─ 冻结后：fine_integ = FINE_K * yaw，±5° 限幅         │
│      cum_yaw = frozen_yaw + fine_offset                 │
└─────────────────────────────────────────────────────────┘
```

### 发送调度

1. **60cm 软限幅**：若当前 target 与上次发送距离 > 60cm，不丢弃，而是计算 60cm 边界点发送，并**同步拉回 `cum_yaw_offset`**，防止积分器持续漂移。
2. **20 帧强制突破**：若连续 20 帧因限幅无法发送，强制发送一次当前 target（安全网）。

> 历史变更：早期版本曾使用 `move_complete` + 7cm step guard，实测导致大幅度转头时发送死锁（机械臂完全静止），已删除。

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
| 偏航控制 | 无 / 简化映射 | 积分器 + 前馈 + P boost + 冻结微调 |
| 俯仰控制 | 无 | **已禁用**（固定高度） |
| 舵机 | 动态跟踪 | 固定 50/145 |
| 坐标变换 | 未实现 | 简化固定几何 |
| 目标 | 验证 PnP + 基础闭环 | 验证偏航单轴跟踪性能 |

## 状态

验证中：偏航方向跟踪趋势正确，大角度转头时通过 60cm 软限幅避免发送死锁，P boost 参数经多轮调试收敛至 `BOOST_K = 0.0125`。
