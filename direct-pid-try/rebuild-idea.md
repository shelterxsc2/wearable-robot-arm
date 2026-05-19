# Rebuild Idea & Debug Log

> 记录日期：2026-05-20  
> 基准版本：`bbe5a03` (direct-pid-try)  
> 作者：shelterxsc2 + Kimi Code CLI  
> 说明：本文档记录 2026-05-20 的实机调试过程、关键发现、与基准版本的差异对比，以及后续重构思路。代码层面保持 `bbe5a03` 不变，本文档作为独立的设计/调试记录附加到仓库中。

---

## 一、今日调试记录（2026-05-20）

### 1.1 代码同步
- 从云端 `wearable-robot-arm` 拉取最新版 `bbe5a03`
- 用 `direct-pid-try/` 覆盖本地调试代码
- 本地备份：`backup/debug-before-overwrite-20260519-230714/`

### 1.2 参数调整历程

| 参数 | bbe5a03 原值 | 调试过程 | 最终值 | 说明 |
|------|-------------|---------|--------|------|
| `CAM_HEIGHT_OFFSET_CM` | 5.0f | — | **0.0f** | 相机中性高度 Z 从 40cm → **35cm** |
| `SERVO2_DEG` | 157.0° | — | **145.0°** | 舵机2固定角度 |
| UART 发送周期 | 每 3 帧 (~10Hz) | — | **每 10 帧 (~3Hz)** | 降低控制频率，匹配机械臂响应能力 |
| `YAW_OFFSET` | 0.03 | — | **0.00** | 修复左右不对称（+1.7°单向偏移导致右转灵敏度≈左转2倍） |
| 偏航死区 | 5° | 5°→2°→5°→**6°** | **6°** | 死区是稳定性的核心，2°比PnP噪声还小 |
| 偏航 KI | 0.08 (纯积分) | 0.04→0.015→0.008→**0.040** | **0.040** | PI模式下配合6°死区使用 |
| 偏航 KP | 无 (纯积分) | 0.9 | **0.9** | 加P项加速响应，但依赖大死区过滤噪声 |
| 偏航控制方式 | 纯积分 | PI + 积分分离 → 纯PI | **PI (KP+KI)** | 最终保留PI，死区6°，KI=0.040 |

### 1.3 编译验证
- 每次修改后均用以下命令编译通过：
  ```bash
  g++ -DNULL=0 -o build/cc src/main.cpp src/wifi.cpp src/gst_rtsp.cpp \
    src/rga_npu.cpp src/bluetooth_spp.c src/uart_comm.cpp \
    `pkg-config --cflags gstreamer-1.0 gstreamer-rtsp-server-1.0 opencv4 dbus-1` \
    `pkg-config --libs gstreamer-1.0 gstreamer-rtsp-server-1.0 opencv4 dbus-1` \
    -lgstapp-1.0 -lrknnrt -lrga -lwpa_client -lpthread -lstdc++ -lm -O3
  ```

---

## 二、关键发现与结论

### 2.1 左右不对称的根因
- `YAW_OFFSET = +0.03 rad`（+1.7°）是**单向正值偏移**，硬编码在 `rvec_f` 上
- 导致：右转时有效误差 ≈ 左转的 **2 倍**
- 修复：清零 `YAW_OFFSET`

### 2.2 死区是决定稳定性的核心
- 死区 2°：PnP 噪声（±1~2°）被当成有效信号，P 项每帧跳变，系统高频振荡
- 死区 5°：明显好转，人正视时的微动被过滤
- 死区 6°：进一步改善，配合 KI=0.040 + KP=0.9 待验证

### 2.3 PI vs 纯积分的权衡
- **纯积分**：慢但稳，无过调，最终能对准。积分器的"惯性"天然平滑噪声。
- **PI（KP=0.9）**：P 项加速响应，但放大了 PnP 噪声。在小死区下导致抖动。
- **结论**：PI 可用，但必须有足够大的死区（≥5°）过滤噪声。死区比 P/I 参数更关键。

### 2.4 PnP 噪声水平（实测）
- 人脸正视时 `head_yaw` 波动约 **±1~3°**
- 俯仰方向 `head_pitch` 波动约 **±0.5°**
- 这是死区设计的基准线

### 2.5 运动生硬问题
- 表现：从运动到停止太硬，机械冲击明显
- 判断：**下位机缺少轨迹规划/速度平滑**
- 上位机不处理，待下位机优化（见"四、快速改进方案"）

---

## 三、与 bbe5a03 (direct-pid-try) 的对比

### 3.1 参数差异

```cpp
// bbe5a03 原值 → 2026-05-20 调试后值

// 1. 相机高度
static const float CAM_HEIGHT_OFFSET_CM = 5.0f;   // → 0.0f (Z 从 40→35)

// 2. 舵机角度
static const float SERVO2_DEG = 157.0f;            // → 145.0f

// 3. YAW 校准偏移
static const double YAW_OFFSET = 0.03;             // → 0.00

// 4. 发送频率
if (++uart_send_cnt % 3 == 0)                      // → % 10 (10Hz → 3Hz)

// 5. 偏航控制（最大改动）
// bbe5a03: 纯积分
static const float DEAD_ZONE_RAD = 5.0f * PI/180;
static const float CHASE_K = 0.08f;
static float cum_yaw_offset = 0.0f;
cum_yaw_offset += CHASE_K * (yaw_rad - DEAD_ZONE_RAD);

// 2026-05-20: PI 控制
dead_zone = 6.0f * PI/180;                         // 死区加大
KP_YAW = 0.9f;                                     // 新增 P 项
KI_YAW = 0.040f;                                   // I 项增益调整
MAX_I_YAW = 5.0f * PI/180;                         // I 项限幅 ±5°
cum_yaw_offset = KP_YAW * eff_yaw + cum_yaw_offset_i;
```

### 3.2 逻辑差异

| 维度 | bbe5a03 | 2026-05-20 调试版 |
|------|---------|------------------|
| 偏航控制 | 纯积分（I-only） | PI（P+I） |
| 死区 | 5° | **6°** |
| 积分增益 | 0.08 | **0.040** |
| P 项增益 | 无 | **0.9** |
| I 项限幅 | 无（硬限幅 ±90°） | **±5°** |
| YAW 偏移 | +0.03 rad | **0** |
| 发送频率 | 10 Hz | **3 Hz** |
| 相机 Z | 40 cm | **35 cm** |
| 舵机2 | 157° | **145°** |
| 俯仰控制 | PI（不变） | PI（不变） |

### 3.3 调试结论
- **俯仰方向未动**：KP_PITCH=2.0, KI_PITCH=0.06, 死区3°，目前未发现问题
- **偏航方向是主要调试对象**：从纯积分改成 PI，死区和 KI 是调参关键
- **发送频率从 10Hz 降到 3Hz**：降低上位机负载，匹配机械臂实际响应能力

---

## 四、快速改进方案（不改大架构，立即可做）

### 4.1 下位机加轨迹插值（解决生硬问题）

在下位机裸机代码中，加一个简单的一阶低通平滑：

```c
// 在逆运动学之前，对目标位置做平滑
#define ALPHA 0.1f  // 平滑系数，越小越柔和

float smooth_target_x = prev_target_x + ALPHA * (raw_target_x - prev_target_x);
float smooth_target_y = prev_target_y + ALPHA * (raw_target_y - prev_target_y);
float smooth_target_z = prev_target_z + ALPHA * (raw_target_z - prev_target_z);

// 然后用 smooth_target_* 做逆运动学
prev_target_x = smooth_target_x;
```

- 不改 RTOS，不改通信协议
- 效果：目标位置跳变被平滑，从运动到停止是斜坡而不是悬崖
- 调参：ALPHA 从 0.1 开始试，越小越柔和但越滞后

### 4.2 PyBullet 仿真起步

利用现有的 SolidWorks 3D 模型，先搭仿真环境：

1. **导出 URDF**：SolidWorks → URDF Exporter → 机械臂 URDF 文件
2. **PyBullet 加载**：
   ```python
   import pybullet as p
   p.connect(p.GUI)
   robot_id = p.loadURDF("robot_arm.urdf")
   ```
3. **虚拟相机 + 人脸**：在仿真里放置一个 RGB 相机，渲染简单人脸模型（或加载真实人脸点云）
4. **算法验证**：把上位机的 PID 逻辑搬到 Python，在仿真里跑闭环
5. **参数迁移**：仿真里验证好的参数，直接往实机上搬

**仿真是最大的杠杆**：不会摔机械臂、可一键重置、可精确注入噪声、可画完美响应曲线。

---

## 五、重构思路（如果从头再来）

### 5.1 当前架构的 6 个核心问题

1. **上位机是单线程大杂烩**：WiFi/GStreamer/RGA/NPU/UART 全耦合在一个进程，没有模块化
2. **通信协议裸奔**：10 字节 raw int16，无帧头/长度/CRC，3Hz 发送
3. **控制是开环瞎子**：上位机只发目标位置，不知道机械臂实际在哪
4. **控制频率太低**：3Hz 对于视觉伺服太慢，行业至少 50~100Hz
5. **没有轨迹规划**：下位机收到跳变目标点直接硬冲
6. **没有仿真**：所有调试在实机上硬调，风险高效率低

### 5.2 最优架构建议

#### 上位机：ROS2 Humble (RK3588)

```
camera_node       → /camera/image_raw (sensor_msgs/Image)
    ↓
vision_node       → /vision/face_pose (geometry_msgs/PoseStamped)
    ↓
control_node      → /control/cmd_vel (geometry_msgs/TwistStamped)
    ↓
comm_node         ↔ UART ↔ 下位机
    ↓
rviz2 / plotjuggler   可视化 + 数据分析
```

- **节点化**：视觉、控制、通信分离
- **热加载参数**：YAML 配置文件，`rqt_reconfigure` 实时调参
- **rosbag 记录**：事后回放分析
- **发速度指令，不是位置指令**：`/control/cmd_vel` 比 `/control/target_pose` 更抗跳变

#### 下位机：FreeRTOS + micro-ROS (STM32H723)

| 任务 | 频率 | 职责 |
|------|------|------|
| `motor_task` | 1kHz | 电机控制闭环 |
| `traj_task` | 100Hz | S 曲线/梯形速度插值 |
| `comm_task` | 100Hz | UART 收发，双向通信 |
| `safety_task` | 100Hz | 限位、过流、碰撞检测、急停 |

#### 通信升级

```
[0xAA][0x55][LEN][CMD][payload...][CRC16]
```

- 上位机 → 下位机：`CMD_TARGET_VEL`（6DOF 速度，24 字节）
- 下位机 → 上位机：`CMD_JOINT_STATE`（关节角）+ `CMD_END_EFFECTOR_POSE`（末端位姿）
- **频率：100Hz 双向**

### 5.3 迁移路线图

```
阶段0：仿真先行（2~3 周）
  └── SolidWorks → URDF → PyBullet
  └── 虚拟相机 + 人脸模型
  └── 仿真里调视觉伺服 PID，拿到验证过的参数

阶段1：通信升级（1 周）
  └── 加 CRC16 帧格式
  └── 下位机回传关节角 + 末端位姿
  └── 上位机解析回传，闭环有反馈

阶段2：上位机 ROS2 化（2 周）
  └── rga_npu.cpp → vision_node
  └── PID 逻辑 → control_node
  └── UART → comm_node
  └── YAML 热加载参数

阶段3：下位机 FreeRTOS 化（2~3 周）
  └── 上 FreeRTOS
  └── traj_task S 曲线插值
  └── 频率提到 100Hz

阶段4：手眼标定 + 速度控制（1~2 周）
  └── Eye-in-Hand 标定
  └── 上位机从"发位置"改成"发速度"
  └── 雅可比转关节速度，真正的视觉伺服闭环
```

### 5.4 分阶段验证路线（从简单到复杂）

```
阶段0：验证 PnP 稳定性（人脸固定偏头不动，看 head_yaw 稳不稳）
  ↓
阶段1：偏航闭环 + 舵机固定（当前所在阶段）
  ↓
阶段2：偏航闭环 + 舵机释放
  ↓
阶段3：加入俯仰闭环
  ↓
阶段4：加入滚转（如必要）
```

---

## 六、下一步工作清单

### P1：基础设施（优先）
- [ ] 日志对齐：只打印发送帧，P/I 分离显示
- [ ] PID 框架化：抽离为可调结构体，支持 `/tmp/pid.conf` 热加载
- [ ] 数据记录：按 3Hz 写 CSV，事后 Python 画响应曲线

### P2：快速改进（不改大架构）
- [ ] 下位机加一阶低通平滑（ALPHA=0.1），解决生硬问题
- [ ] PyBullet 仿真起步：URDF 导出 + 基础加载脚本

### P3：重构（长期）
- [ ] ROS2 上位机
- [ ] FreeRTOS + micro-ROS 下位机
- [ ] 可靠串口协议 + 双向 100Hz 通信
- [ ] 手眼标定 + 速度控制闭环

---

## 七、附件：当前运行参数（调试后）

### 偏航（水平面）
```
DEAD_ZONE_RAD = 6.0°
KP_YAW = 0.9
KI_YAW = 0.040
MAX_I_YAW = ±5°
YAW_OFFSET = 0.00
```

### 俯仰（高度 Z）
```
PITCH_BIAS_DEG = 11.5°
DEAD_ZONE_PITCH_RAD = 3.0°
KP_PITCH = 2.0
KI_PITCH = 0.06
MAX_I_PITCH = ±20°
PITCH_RANGE_CM = 15.0
NEUTRAL_Z_CM = 35.0
```

### 空间参数
```
FACE_X_CM = 0.0
FACE_Y_CM = 10.0
FACE_Z_CM = 35.0
TRACK_DIST_CM = 57.0
CAM_HEIGHT_OFFSET_CM = 0.0
SERVO1_DEG = 90.0
SERVO2_DEG = 145.0
UART_SEND_INTERVAL = 10 frames (~3Hz @ 30fps)
```
