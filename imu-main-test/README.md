# IMU 主控测试分支 (imu-main-test)

本分支为 RK3588 可穿戴机械臂视觉跟踪系统的 **NRF24 IMU 控制链路测试版本**。

> 视觉链路（RTSP 推流 + 人脸检测）保留为纯可视化，**不驱动控制**。  
> 控制完全由 NRF24 无线 IMU（头部佩戴）驱动。

---

## 核心特性

### 1. 预测终点驱动

不是跟踪人脸当前位置，而是**预测人脸运动的最终停止位置**，提前发令让机械臂直接运动到该终点。目标：机械臂到位时，人脸刚好停止 → "同时到达"，模拟实时跟随效果。

预测模型：
```
pred_delta = wz × dt × k
```

| 状态 | k | 说明 |
|:---|:---:|:---|
| 加速→匀速 | **0.70** | 速度平台确立，预测最确定 |
| 匀速→减速 | **0.60** | 开始减速，可估算剩余位移 |
| 加速中 | 0.50 | 速度还会增加，不确定 |
| 匀速 | 0.30 | 不知何时停止，最保守 |
| 减速→停止 | 0.25 | 已接近停止，剩余位移少 |

自适应预测窗口 `dt`：
- |wz| < 20°/s → dt = 500ms
- |wz| < 60°/s → dt = 350ms
- |wz| ≥ 60°/s → dt = 250ms

### 2. 事件触发发令

| 触发窗口 | 条件 | 间隔 |
|:---|:---|:---:|
| **#1 主窗口** | 检测到 `加速→匀速`（速度平台确立） | 150ms |
| **#2 修正窗口** | 检测到 `匀速→减速`（运动即将结束） | 150ms |
| 常规修正 | 预测终点变化 > 5° | 200ms |
| 停止对准 | 人脸已停，消除预测残余误差 | 200ms |

### 3. 状态机（8 状态修复版）

修复了原版的结构性缺陷：
- `STATE_ACCEL` 增加了 `decel_trend` 出口（加速中可直接转入减速）
- `STATE_DECEL_STOP_TO_ACCEL` 增加了回退到减速的出口
- 过零检测（`has_crossed_zero`）避免摇头时被误判为单向运动

### 4. wz 历史分析（10ms 分辨率）

虽然控制决策周期是 50ms（GLib 定时器），但 NRF24 数据以 10ms 采样。状态机每次分析**最近 5 帧（50ms）的完整 wz 序列**，而不是只看一个瞬时值。

能检测：
- 峰值 / 谷值（判断运动幅度）
- 过零（识别摇头 / 往返运动）
- 趋势（基于 `first→latest` 的绝对值变化）

---

## 目录结构

```
imu-main-test/
├── src/
│   ├── main.cpp              # 主程序：WiFi / NPU / RGA / UART / NRF24 初始化
│   ├── rga_npu.cpp           # ★ 核心：预测终点 + 状态机 + 发令策略
│   ├── rga_npu.h
│   ├── nrf24_linux.c         # ★ NRF24 驱动：spidev + sysfs GPIO，RX 线程 5ms 轮询
│   ├── nrf24_linux.h
│   ├── uart_comm.cpp         # UART 驱动 + 协议打包
│   ├── uart_comm.h
│   ├── gst_rtsp.cpp          # GStreamer RTSP 推流（纯可视化）
│   ├── gst_rtmp.cpp
│   ├── wifi.cpp
│   └── ...
└── docs/
    └── motion-control-framework.md   # 运动控制框架设计文档
```

---

## 编译

```bash
cd /home/elf/work/twice
g++ -std=c++17 -O2 -Isrc \
  src/main.cpp src/rga_npu.cpp src/gst_rtsp.cpp src/gst_rtmp.cpp src/uart_comm.cpp src/wifi.cpp \
  src/nrf24_linux.c /tmp/bt_stub.c \
  -o build/cc \
  $(pkg-config --cflags --libs opencv4 gstreamer-1.0 gstreamer-app-1.0 gstreamer-rtsp-server-1.0) \
  /usr/lib/aarch64-linux-gnu/libwpa_client.a \
  -lrknnrt -lrga -lpthread -lm -ldl
```

> `/tmp/bt_stub.c` 内容：
> ```c
> void bluetooth_spp_stop(void) {}
> void bluetooth_spp_cleanup(void) {}
> ```

---

## 关键参数速查

| 参数 | 值 | 说明 |
|:---|:---:|:---|
| 控制周期 | 50ms | GLib 定时器触发 `nrf24_control_update()` |
| NRF24 采样 | ~100Hz | 有效帧率 10ms/帧 |
| wz 历史窗口 | 5 帧（50ms） | 状态机分析用 |
| 最小发令间隔 | 150~200ms | 主/修正窗口 150ms，常规 200ms |
| yaw 发令阈值 | 5° | 预测终点变化 <5° 不发 |
| 静止阈值 | \|wz_avg\| < 5°/s | 3 点平均 |
| 趋势阈值 | ±10°/s | first→latest 绝对值变化 |
| 跟踪距离 | 57cm | `target_x/y` 圆弧半径 |
| 预测限幅 | ±90° | `cum_yaw_offset` 上限 |

---

## 已知约束

1. **机械臂比人脸慢 5~10 倍**：云台 v_max ≈ 43°/s，人脸转头可达 200°/s+。预测控制不能突破物理带宽，只能减少"等看到才发令"的额外滞后。
2. **下位机 S 曲线重置**：每次收到 UART 指令 `Speed_Plan_State = init`。高频发令（<150ms）会毁灭性重置，速度永远起不来。当前最小间隔 150ms 是底线。
3. **方向反转惩罚**：下位机反向时速度骤降到 30%。摇头场景不可避免触发此惩罚，机械臂在边界处会"爬行"。
4. **预测误差**：简化模型 `wz × dt × k` 是经验公式，对于非梯形速度曲线（如突停、二次加速）会过冲或欠冲。停止后会补发精确对准指令消除残余误差。

---

## 硬件拓扑

```
头部 IMU (BMI088)
    ↓ NRF24L01+ 无线
RK3588 (ELF2)
    ├── NRF24 RX 线程 (~100Hz, 5ms 轮询)
    ├── 状态机 + 终点预测 (50ms)
    └── UART /dev/ttyS9 @ 115200
        ↓
    STM32H723 (裸机 HAL)
        ├── S 曲线 7 相速度规划
        ├── 逆运动学
        └── 3 轴电机 + 2 舵机
```

---

*本分支基于 RK3588 可穿戴机械臂项目，专注于 NRF24 IMU 预测控制链路的验证与迭代。*
