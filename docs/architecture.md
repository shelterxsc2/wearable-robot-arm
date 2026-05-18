# 系统架构与坐标变换方案

## 1. 系统架构分层

```
┌─────────────────────────────────────────────────────────────┐
│  RK3588 (上位机) — 感知 + 规划层                              │
│  ├─ 视觉采集 (USB Cam → GStreamer)                          │
│  ├─ AI 推理 (RGA + NPU: face_best.rknn / best.rknn)         │
│  ├─ PnP 位姿估计 (OpenCV solvePnP)                          │
│  ├─ 滤波 (OneEuroFilter + FaceTracker)                      │
│  ├─ ▶ 坐标变换 (相机系 → 基座系 → 目标位姿) [待实现]         │
│  ├─ 视频推流 (RTSP/RTMP)                                    │
│  └─ 通信 (UART + 蓝牙 BLE)                                  │
└─────────────────────────────────────────────────────────────┘
                              │ UART 921600
                              ▼
┌─────────────────────────────────────────────────────────────┐
│  STM32H723 (下位机) — 执行 + 控制层                           │
│  ├─ UART 接收 / 指令解析                                     │
│  ├─ ▶ 逆运动学 (Coordinate_Inverse_Settlement)              │
│  ├─ S 曲线速度规划 (Speed_Plan_Update, 7 阶段状态机)         │
│  ├─ MIT 力矩控制 + 重力补偿                                  │
│  ├─ 电机驱动 (FDCAN: DMJ4310 + LK4005)                      │
│  └─ 舵机驱动 (PWM: LFD01M ×2)                               │
└─────────────────────────────────────────────────────────────┘
```

**设计原则**: RK3588 做"感知 + 规划"（算力充足，有 OpenCV），STM32 做"执行 + 控制"（硬实时，专注电机）。

---

## 2. 坐标系定义

### 2.1 人脸坐标系（PnP 3D 模板）

- **原点**: 鼻尖附近
- **+X**: 人脸左侧（从人脸自身视角看）
- **+Y**: 人脸下方
- **+Z**: 人脸后方（远离相机方向）
- **人脸正面方向**: `-Z`（指向相机）

PnP 输出的 `rvec/tvec` 表示: **人脸坐标系 → 相机坐标系** 的变换 `T_camera←face`。

### 2.2 相机坐标系

- **+Z**: 光轴指向场景（指向人脸）
- **+X**: 右
- **+Y**: 下

### 2.3 机械臂基座坐标系

- **原点**: J1 关节中心（背部电机/穿戴固定点）
- **+Z**: 竖直向上
- **+X**: 人脸右侧（穿戴者右侧）
- **+Y**: 人脸前方（穿戴者前方）

> 此坐标系由 `docs/protocol.md` 和 STM32 逆运动学共同定义。`Coordinate_Inverse_Settlement()` 中 `atan2(Y, X)` 计算水平转角，说明 XY 为水平面，Z 为竖直方向。

---

## 3. 核心问题：相机位姿 → 基座位姿的变换链

RK3588 得到的是 `T_camera←face`（相机系下人脸的位姿），但 STM32 逆运动学需要**基座系下的目标点** `(X, Y, Z)`。中间缺失两层：

```
T_base←camera = T_base←end(θ) × T_end←camera
                       ↑              ↑
                 机械臂正运动学      手眼标定（固定不变）
```

```
T_base←face = T_base←camera × T_camera←face
                    ↑                ↑
              刚算出来的            PnP 输出
```

**目标**：让相机到达人脸前方的某个**期望相对位姿** `T_face←camera_desired`：

```
T_base←camera_target = T_base←face × T_face←camera_desired
```

### 3.1 期望跟踪策略（业务常数）

| 模式 | `T_face←camera_desired` | 物理意义 |
|------|------------------------|---------|
| **正脸** | 平移 `(0, 0, -D)` | 相机在人脸正前方 D cm |
| **左侧脸** | 平移 `(-D·sinθ, 0, -D·cosθ)` | 从人脸左侧观察 |
| **右侧脸** | 平移 `(+D·sinθ, 0, -D·cosθ)` | 从人脸右侧观察 |

其中 `D` 为跟踪距离（如 30cm），`θ` 为侧脸角度（如 30°、45°）。相机 Z 轴始终指向人脸中心。

> 遥控器可通过 BLE 下发指令切换模式和距离。详见 [`docs/roadmap.md`](docs/roadmap.md)。

---

## 4. RK3588 端每帧计算流程（推荐实现）

```python
# 1. 从 STM32 接收当前关节角（或 CURRENT_POSE）
theta_gimbal, theta_upper, theta_fore = uart_recv_current_pose()

# 2. 正运动学：当前关节角 → T_base←end
t_base_to_end = forward_kinematics(theta_gimbal, theta_upper, theta_fore)

# 3. 手眼标定结果（离线标定一次，硬编码）
t_end_to_camera = ...  # 4×4 齐次矩阵

# 4. 当前相机在基座系下的位姿
t_base_to_camera = t_base_to_end @ t_end_to_camera

# 5. PnP 输出
t_camera_to_face = pnp_output()  # rvec/tvec → 4×4

# 6. 人脸在基座系下的位姿
t_base_to_face = t_base_to_camera @ t_camera_to_face

# 7. 期望的相机-人脸相对位姿（设计常数）
t_face_to_camera_desired = desired_pose(distance=D, angle=theta)

# 8. 目标相机位姿（基座系下）
t_base_to_camera_target = t_base_to_face @ t_face_to_camera_desired

# 9. 提取位置和姿态，下发 Pose6D
target_pos = t_base_to_camera_target[0:3, 3]        # (X, Y, Z) mm
target_quat = rotmat_to_quat(t_base_to_camera_target[0:3, 0:3])
uart_send_target_pose(target_pos, target_quat)
```

---

## 5. 目前缺失的关键数据

要做通这个链路，必须补充以下三块：

### 5.1 手眼标定 `T_end←camera`（最急迫）

相机经过舵机0 + 末端杆 `L_end=0.08245m` 安装，**精确的旋转+平移偏移目前未知**。

**获取方式**：
1. 在机械臂末端贴 ArUco 标记或棋盘格
2. 让机械臂处于 3~5 个不同姿态，RK3588 拍图并记录关节角
3. 用 OpenCV `calibrateHandEye()` 求解 `T_end←camera`

> 若暂时无法精标定，可先通过"伸直朝前"姿态下的 PnP 深度 + 卷尺测量，反推一个**粗模型**验证运动方向。

### 5.2 机械臂正运动学模型

STM32 上有逆运动学，但 RK3588 需要**正运动学**（关节角 → 末端位姿）。

**需要的数据**：

| 参数 | 当前已知 | 待确认 |
|------|---------|--------|
| 连杆长度 | L_connect=0.44, L1=0.45, L2=0.461, L_end=0.08245 | ✅ 已有 |
| 零位偏移 | Gimbal=0.75, Upper=0.07, Fore=-0.18 | ✅ 已有 |
| **关节旋转轴方向** | — | ⚠️ 待确认：云台绕 Z？大臂/小臂绕 X 还是 Y？ |
| **基座到 J1 的固定安装关系** | — | ⚠️ 待确认：基座系原点是否在 J1 电机轴交点？ |

> 旋转轴方向决定了正运动学的旋转矩阵怎么建。需要对照机械 CAD 或实际测量确认。

### 5.3 STM32 → RK3588 实时回传

RK3588 做正运动学必须知道**当前关节角**。

`shared/protocol.h` 已定义 `CMD_CURRENT_POSE` (0x20)，但 STM32 目前**没有主动发送**。需要确认：
- STM32 `main.c` 中是否周期性地打包并发送 `CURRENT_POSE`？
- RK3588 `uart_comm.cpp` 中的接收回调是否已注册并解析？

---

## 6. 简化方案（粗标定验证）

如果不想先做完整手眼标定 + 正运动学，可以用以下方式快速验证链路方向：

1. 把机械臂摆到"伸直朝前"的已知姿态，记录：
   - STM32 关节角
   - RK3588 PnP 输出的 `tvec = (x0, y0, z0)`
2. 用卷尺量此时人脸到背部基座的实际距离 `(X_real, Y_real, Z_real)`
3. 假设此时末端与基座的关系可通过几何近似（大臂+小臂+连接杆伸直），反推一个近似的 `T_base←camera`
4. 用这个粗模型跑起来，验证：人脸靠近时机械臂是前伸还是后退
5. **方向对了之后，再用 calibrateHandEye 精标定**

---

## 7. STM32 端收到目标后的处理

```
RK3588 下发 Pose6D (X, Y, Z, qx, qy, qz, qw)
           ↓
    STM32 UART 接收 (DMA Idle 中断)
           ↓
    提取位置 (X, Y, Z) 和末端姿态
           ↓
    若使用简化协议：直接解算逆运动学
    若使用完整协议：由姿态四元数推算舵机角度
           ↓
    Coordinate_Inverse_Settlement(X, Y, Z, Servo_Angle,
                                   &gimbal, &upper, &fore)
           ↓
    Speed_Plan_Update() → S 曲线规划 → MIT 控制 + 重力补偿
           ↓
    FDCAN 发送给 DMJ4310 / LK4005，PWM 更新 LFD01M
```

---

## 8. 通信不匹配说明

当前 STM32 `Robotic_Arm_Communication_HAL_STM32_Port.c` 实际使用的是**简化 raw 格式**：

```
[Byte0~1] X  (int16, cm, 小端)
[Byte2~3] Y  (int16, cm, 小端)
[Byte4~5] Z  (int16, cm, 小端)
[Byte6~7] Servo1 (int16, °, 小端)
[Byte8~9] Servo2 (int16, °, 小端)
```

这与 `shared/protocol.h` 中定义的 `[0xAA][0x55][LEN][CMD][payload][CRC8]` **帧格式不一致**：
- 无帧头 `0xAA 0x55`
- 无命令字 CMD
- 无 CRC8 校验
- 无 `Pose6D` 的 28 字节完整位姿

**风险**: UART 无帧同步，一旦错位（丢字节、干扰），后续所有解析都会错位，且无法自恢复。

**建议**: 后续统一改为 `shared/protocol.h` 的标准帧格式，RK3588 下发 `CMD_TARGET_POSE` (0x10)，STM32 回传 `CMD_CURRENT_POSE` (0x20)。

---

## 9. 舵机与机械臂的分层控制建议

机械臂有 5 个自由度（3 电机 + 2 舵机），建议分层：

| 层级 | 执行器 | 控制目标 | 响应速度 |
|------|--------|---------|---------|
| **外环（位置）** | 云台 + 大臂 + 小臂 | 让相机整体移动到人脸前方合适位置 | 慢（S 曲线，几百 ms） |
| **内环（姿态）** | 舵机0 + 舵机1 | 让人脸保持在图像中心，补偿俯仰/偏航 | 快（PWM 直控，几十 ms） |

这样 RK3588 的输出分为两部分：
1. **机械臂目标位姿** → 通过 UART 发 `Pose6D` → STM32 逆运动学 + 规划
2. **舵机目标角度** → 也可以走 UART，或直接由 STM32 根据人脸在图像中的像素误差做 PID（如果 STM32 能拿到误差）

> 当前 STM32 代码中舵机角度已包含在 10 字节简化指令中，这是合理的。
