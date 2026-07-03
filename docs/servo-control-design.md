# 可穿戴机械臂 — 舵机控制设计方案

> **日期**: 2026-05-28  
> **当前同步**: 2026-07-01，按 `imu-victor-hat` 当前代码校正
> **历史适用版本**: `imu-main-test-roll+yaw`（Roll + Yaw 双轴控制）
> **设计约束**: 结构不改、下位机运动学不改、J4 只调 1~2 次、低频反馈

---

## 当前代码状态（以源码为准）

- J4/J5 已不是“固定值 + 到位后再调一次”的纯方案 A，而是在每次正常发令时根据 Pitch/Yaw 动态映射：
  - J4/俯仰：`servo1 = 55 + K * delta_pitch`，抬头侧 `K=-0.8`，低头侧 `K=-1.65`，限幅 `[-90, 90]`
  - J5/水平：`servo2 = 50 + 0.4 * delta_yaw`，限幅 `[0, 270]`
- 预测帧 `flag=0x00` 时，坐标使用预测值，同时发送当前计算出的 J4/J5 舵机角，便于现场测试俯仰和偏航跟随。
- UART 目标帧当前是 11 字节：`x,y,z,k1,k2` 五个 int16 小端 + `flag`。
- `/servo?k1=...&k2=...` 会直接发一次测试指令并写 `/tmp/servo_calib.txt`；但 mode=1/2 的周期标定发令仍发送固定 `NRF_SERVO1_DEG/NRF_SERVO2_DEG`，尚未完全闭合到 `calib_servo1`。
- 下文“方案 A/B”保留为设计参考，不代表当前代码完全按该状态机实现。

## 背景与约束

### 机械结构

- **J4（SERVO1）**: 俯仰舵机，与摄像头直接相连，PWM 控制，范围 0~180°（源码限制 `0 ~ π`）
- **J5（SERVO2）**: 水平舵机，当前代码按 yaw 动态映射，基线 50°
- **舵机不是独立云台**: `Coordinate_Inverse_Settlement(X, Y, Z, Servo_Angle, ...)` 中 `Servo_Angle` 是**输入**，参与前臂有效长度 `L2_v` 的计算
- **耦合效应**: 改 J4 会改变前臂有效长度 → 下位机重新解算 J1/J2/J3 → 机械臂微动

### IMU 数据性质

- `gy_roll/yaw/pitch` 仅代表 **IMU 自身的欧拉角**，与视觉坐标系/机械臂坐标系**无直接对应**
- 当前代码中 `roll` 已映射为俯仰效果（抬头/低头），因此 J4 俯仰舵机与 `roll` 挂钩

### 核心约束（用户确认）

1. ✅ 有下位机几何常数
2. ✅ 低频反馈（~200ms）
3. ✅ 机械臂到位后再调舵机
4. ✅ **只调一次或两次**，不调第三次
5. ❌ 不改下位机运动学
6. ❌ 不改机械结构

---

## 方案 A：简单标定映射（推荐）

### 核心思路

不依赖正运动学。利用 IMU_roll 与 J4 之间的**单调映射关系**，通过一次性标定确定比例系数 `k`。

```
IMU_roll 变化 → 人头俯仰变化 → J4 补偿角度
```

### 标定方法

```cpp
// === 标定阶段（开机时执行一次）===
void servo_calibration(void)
{
    // 用户正对摄像头，保持头部水平
    baseline_roll    = g_nrf24_state.gy_roll;   // 记录当前 roll
    baseline_servo1  = 90.0f;                    // J4 水平中位

    // 用户缓慢抬头 ~15°，观察 J4 需要转多少度才能让画面保持水平
    // 记录：delta_roll = +15° 时，servo1 需要 +10°
    // 则 k = 10° / 15° ≈ 0.67
}
```

### 运行时逻辑

```cpp
// === 上位机状态机 ===

typedef enum {
    SERVO_STATE_ARM_MOVING,     // 机械臂在运动中，J4 保持基准
    SERVO_STATE_ARM_STABLE,     // 机械臂已到位，等待触发
    SERVO_STATE_SEND_ADJUST,    // 发 J4 补偿令（只发一次）
    SERVO_STATE_WAIT_DONE,      // 等待下位机执行
    SERVO_STATE_IDLE            // 完成，下次触发从 ARM_MOVING 开始
} ServoControlState;

void servo_control_update(float current_roll_deg)
{
    static ServoControlState state = SERVO_STATE_ARM_MOVING;
    static float last_adjust_roll = 0.0f;
    static uint64_t stable_time_us = 0;
    static uint64_t last_cmd_us = 0;

    const float ROLL_THRESHOLD_DEG = 5.0f;      // 触发阈值
    const float K_SERVO = 0.67f;                 // 标定系数，实验确定
    const uint64_t STABLE_TIMEOUT_US = 500000;   // 500ms 认为到位
    const uint64_t MIN_ADJUST_INTERVAL_US = 2000000; // 2s 内只调一次

    uint64_t now_us = get_us();

    switch (state) {
        case SERVO_STATE_ARM_MOVING:
            // 机械臂运动中，J4 固定基准值
            if (g_uart_move_complete) {
                stable_time_us = now_us;
                state = SERVO_STATE_ARM_STABLE;
            }
            break;

        case SERVO_STATE_ARM_STABLE:
            // 等机械臂稳定 + roll 变化足够大
            if (now_us - stable_time_us < STABLE_TIMEOUT_US)
                break;

            float delta_roll = normalize_angle_deg(current_roll_deg - baseline_roll);
            if (std::fabs(delta_roll) > ROLL_THRESHOLD_DEG &&
                now_us - last_cmd_us > MIN_ADJUST_INTERVAL_US)
            {
                state = SERVO_STATE_SEND_ADJUST;
            }
            break;

        case SERVO_STATE_SEND_ADJUST:
        {
            float delta_roll = normalize_angle_deg(current_roll_deg - baseline_roll);
            float servo1_new = baseline_servo1 + K_SERVO * delta_roll;

            // 限幅 0~180°
            if (servo1_new > 180.0f) servo1_new = 180.0f;
            if (servo1_new < 0.0f)   servo1_new = 0.0f;

            // 只发一次：保持 tx/ty/tz 不变，只改 J4
            uart_send_arm_target(last_tx, last_ty, last_tz,
                                 servo1_new, NRF_SERVO2_DEG);

            last_adjust_roll = current_roll_deg;
            last_cmd_us = now_us;
            state = SERVO_STATE_WAIT_DONE;
            break;
        }

        case SERVO_STATE_WAIT_DONE:
            // 等下位机执行完（或超时）
            if (g_uart_move_complete ||
                now_us - last_cmd_us > 300000)  // 300ms 超时
            {
                state = SERVO_STATE_IDLE;
            }
            break;

        case SERVO_STATE_IDLE:
            // 完成。下次机械臂重新运动时回到 ARM_MOVING
            if (!g_uart_move_complete) {
                state = SERVO_STATE_ARM_MOVING;
            }
            break;
    }
}
```

### 所需数据

| 数据 | 来源 | 必要性 |
|------|------|--------|
| `baseline_roll` | IMU 开机标定 | 必须 |
| `baseline_servo1` | 固定 90° | 必须 |
| `K_SERVO` | 实验标定 | 必须 |
| `g_uart_move_complete` | 下位机回传 "complete" | 必须 |

**不需要任何几何常数。不需要正运动学。不需要下位机传关节角。**

### 误差估算

- 标定误差：±2~3°（取决于标定者手感）
- 机械臂耦合误差：J4 调 20° → J2/J3 各微动 ~8° → 末端偏移 ~±2cm
- **综合误差：±3~5°（舵机角度）**，对画面影响可接受

### 优缺点

| 优点 | 缺点 |
|------|------|
| 实现极简，不需要几何常数 | 精度依赖标定，无闭环修正 |
| 不需要正运动学计算 | 人脸快速转动时，J4 一步到位可能不准 |
| 与下位机解耦，通信负担最小 | 无法补偿机械臂姿态带来的误差 |

---

## 方案 B：正运动学校正

### 核心思路

下位机传回 J1/J2/J3 + Servo1 实际值 → 上位机做**正运动学**算出末端实际位姿 → 对比 IMU 头部姿态 → 算出摄像头与目标的偏差 → 反解出需要的 J4 补偿。

### 正运动学推导（基于下位机逆运动学反推）

已知下位机逆运动学：

```c
L2_v = sqrt(L2² + L_end² - 2·L2·L_end·cos(Servo))
β    = asin(L_end·sin(Servo) / L2_v)     // 虚拟角度偏移
temp = acos((L1² + L2_v² - R²) / (2·L1·L2_v))
```

反推正运动学（由 J1, J2, J3, Servo → X, Y, Z, 光轴方向）：

```cpp
struct Pose3D {
    float x, y, z;           // 末端位置
    float pitch, yaw;        // 末端姿态（光轴方向）
};

Pose3D ForwardKinematics(float J1, float J2, float J3, float Servo)
{
    // 1. 由 J3 和 Servo 反推 temp
    float L2_v = sqrtf(L2*L2 + L_end*L_end - 2*L2*L_end*cosf(Servo));
    float beta = asinf(L_end * sinf(Servo) / L2_v);
    float temp = J3 - Offset_Fore + beta;   // 从逆运动学反推

    // 2. 整体距离 R（水平面投影）
    float R = sqrtf(L1*L1 + L2_v*L2_v - 2*L1*L2_v*cosf(temp));

    // 3. 整体俯仰角 α（基座到末端的连线与水平面夹角）
    // 从逆运动学: J2 = -(asin(L2_v*sin(temp)/R) + atan2(Z-L_connect, R) + Offset_Upper)
    float alpha = -J2 - Offset_Upper - asinf(L2_v * sinf(temp) / R);

    // 4. 末端位置
    Pose3D p;
    p.x = R * cosf(J1);           // J1 = atan2(Y, X) + π/2
    p.y = R * sinf(J1);
    p.z = L_connect + R * tanf(alpha);  // 简化，实际用 sin/cos

    // 5. 摄像头光轴方向 = 前臂末端方向 + Servo 补偿
    float arm_pitch = alpha + asinf(L2_v * sinf(temp) / R);  // 上臂+前臂总俯仰
    p.pitch = arm_pitch + Servo - PI/2;  // 光轴相对于水平面的俯仰
    p.yaw   = J1 - PI/2;                  // 光轴偏航

    return p;
}
```

> ⚠️ 以上为推导示意，实际实现需根据下位机代码精确核对符号和坐标系。

### J4 补偿计算

```cpp
float CalculateServoCompensation(float imu_roll, float imu_yaw,
                                  const Pose3D& end_effector,
                                  const Pose3D& head_pose)
{
    // 1. 头部中心位置（基于 IMU 和固定几何关系）
    Vector3 head_pos = head_pose.position;

    // 2. 摄像头当前位置 = 末端位姿
    Vector3 cam_pos = {end_effector.x, end_effector.y, end_effector.z};

    // 3. "摄像头应该指向哪里" = 头部中心 - 摄像头位置
    Vector3 target_dir = Normalize(head_pos - cam_pos);

    // 4. 摄像头当前朝向
    Vector3 cam_dir;
    cam_dir.x = cosf(end_effector.pitch) * cosf(end_effector.yaw);
    cam_dir.y = cosf(end_effector.pitch) * sinf(end_effector.yaw);
    cam_dir.z = sinf(end_effector.pitch);

    // 5. 俯仰偏差
    float pitch_error = asinf(target_dir.z) - asinf(cam_dir.z);

    // 6. J4 补偿 = 当前 Servo1 + pitch_error
    return current_servo1 + pitch_error * 180.0f / PI;
}
```

### 运行时状态机（与方案 A 相同结构，但 SEND_ADJUST 时用正运动学）

```cpp
case SERVO_STATE_SEND_ADJUST:
{
    // 1. 用下位机反馈的 J1/J2/J3 + Servo1 做正运动学
    Pose3D ee = ForwardKinematics(feedback.J1, feedback.J2,
                                   feedback.J3, feedback.Servo1);

    // 2. 由 IMU 算头部姿态
    Pose3D head = IMU_to_HeadPose(imu_roll, imu_yaw, imu_pitch);

    // 3. 算需要的 J4 补偿
    float servo1_new = CalculateServoCompensation(imu_roll, imu_yaw, ee, head);

    // 4. 限幅 + 发令
    CLAMP(servo1_new, 0.0f, 180.0f);
    uart_send_arm_target(last_tx, last_ty, last_tz, servo1_new, NRF_SERVO2_DEG);

    state = SERVO_STATE_WAIT_DONE;
    break;
}
```

### 所需数据

#### 几何常数（一次性，写入上位机源码）

| 常数 | 符号 | 用途 |
|------|------|------|
| 上臂长度 | `L1` | 正运动学 |
| 前臂长度 | `L2` | 正运动学 |
| 基座偏移 | `L_connect` | 正运动学 Z 轴偏移 |
| 舵机支架长度 | `L_end` | 舵机参与的几何计算 |
| 上臂零点偏移 | `Offset_Upper` | 关节角零点校正 |
| 前臂零点偏移 | `Offset_Fore` | 关节角零点校正 |

> **这些常数必须从下位机源码或 CAD 图纸获取。**

#### 运行时反馈（低频，~200ms）

| 数据 | 类型 | 用途 |
|------|------|------|
| `J1_target` 或 `J1_actual` | float/int16 | 正运动学输入 |
| `J2_target` 或 `J2_actual` | float/int16 | 正运动学输入 |
| `J3_target` 或 `J3_actual` | float/int16 | 正运动学输入 |
| `Servo1_actual` | float/int16 | 确认舵机实际位置 |
| `move_complete` | bool/flag | 判断机械臂是否稳定 |

### 误差估算

- 正运动学数值误差：±1~2mm（浮点运算）
- 关节角反馈误差（理论值 vs 实际值）：±2~5mm（机械滞后）
- 舵机 PWM 死区：±1~2°
- **综合末端误差：±1~2cm**
- J4 补偿精度：±2~3°

### 优缺点

| 优点 | 缺点 |
|------|------|
| 精度高，理论上可精确对准 | 需要下位机提供 6 个几何常数 |
| 可补偿机械臂姿态带来的误差 | 需要实现正运动学，代码复杂 |
| 有反馈闭环，可自动修正 | 需要下位机传回 J1/J2/J3，通信负担增加 |
| 标定一次后自动适应 | 计算开销大（三角函数每周期） |

---

## 两方案对比

| 维度 | 方案 A（简单标定） | 方案 B（正运动学校正） |
|------|-------------------|----------------------|
| **精度** | ±3~5°（够用） | ±2~3°（更准） |
| **复杂度** | 极简 | 中等 |
| **需要常数** | 不需要 | 6 个几何常数 |
| **需要反馈** | 只需要 `complete` | J1/J2/J3 + Servo1 + `complete` |
| **正运动学** | 不需要 | 需要 |
| **标定工作量** | 一次手动标定 k | 一次常数录入 + IMU-头部几何标定 |
| **抖动风险** | 相同（都调一次） | 相同（都调一次） |
| **维护性** | 好（与运动学解耦） | 差（常数变了要改代码） |

---

## 通信协议建议（如需反馈）

当前 UART 目标帧为 11 字节裸帧：前 10 字节是 `x,y,z,k1,k2` 五个 int16 小端，第 11 字节是 `flag`。若要传回数据，建议复用已定义的帧协议：

### 下位机 → 上位机 反馈帧（200ms 周期）

```
[0xAA][0x55][LEN][CMD][payload...][CRC8]
```

| 字节 | 内容 | 说明 |
|------|------|------|
| 0~1 | `0xAA 0x55` | 帧头 |
| 2 | `LEN = 14` | payload 长度 |
| 3 | `CMD = 0x21` | 当前位姿反馈 |
| 4~5 | `J1` | int16，单位 0.01° 或 0.1° |
| 6~7 | `J2` | 同上 |
| 8~9 | `J3` | 同上 |
| 10~11 | `Servo1` | int16，单位 0.1° |
| 12 | `Status` | bit0=complete, bit1=moving, bit2=error |
| 13 | `CRC8` | 校验 |

### 上位机 → 下位机 目标帧（当前现有）

```
[x, y, z, k1, k2, flag]  // 11 字节，5x int16 + 1x uint8
```

> `flag=0x00` 表示预测坐标，`flag=0x01` 表示确定/最终坐标。下位机新增反馈通道时应保持该目标帧兼容。

---

## 下一步决策

| 如果... | 选方案 A | 选方案 B |
|---------|---------|---------|
| 不想问 STM32 要常数 | ✅ | ❌ |
| 不想写正运动学 | ✅ | ❌ |
| 精度要求 "大致对准" | ✅ | 过度设计 |
| 精度要求 "严格对准脸中心" | 不够用 | ✅ |
| 下位机能传 J1/J2/J3 | 可用 | 必须 |

**我的推荐**: 先实现 **方案 A**（简单标定），跑起来看效果。如果标定后画面还是偏，再升级到 **方案 B**（正运动学校正）。

---

## 待确认事项

1. [ ] J4（SERVO1）的**极性**：IMU_roll 增大时，J4 应该增大还是减小？
2. [ ] `K_SERVO` 的**标定值**：实验确定（建议范围 0.5~1.0）
3. [ ] 下位机能否传回 `complete` 信号？（当前 `uart_comm.cpp` 已有文本解析，是否工作？）
4. [ ] 如需方案 B，下位机能否提供 6 个几何常数的精确值？
5. [ ] J4 的**初始基准角度**：当前正常控制使用 `SERVO1_BASELINE = 55.0f`；标定模式仍有历史固定值，需上机确认哪个是机械水平位
