# J4 舵机标定操作手册（方案 A）

> **目标**: 确定 `baseline_roll`、`baseline_servo1`、`K_SERVO` 三个参数  
> **工具**: 运行中的机械臂 + 上位机程序 + NRF24 IMU  
> **时间**: 约 5~10 分钟  
> **风险**: 低（舵机范围小，不会撞机械结构）

---

## 一、标定原理

```
J4_target = baseline_servo1 + K_SERVO × (current_roll − baseline_roll)
```

| 参数 | 含义 | 获取方式 |
|------|------|---------|
| `baseline_roll` | 人头水平时的 IMU_roll 读数 | 实测记录 |
| `baseline_servo1` | 人脸在画面中央时 J4 的角度 | 实测调整 |
| `K_SERVO` | 人头每转 1°，J4 补偿多少度 | 两次测量计算 |

---

## 二、标定前准备

1. **机械臂上电**，运行当前上位机程序
2. **IMU 佩戴好**（位置固定，标定过程中不要移动）
3. **人站在机械臂正前方**（跟踪距离约 57cm，即一臂距离）
4. **打开串口日志**（观察 `[NRF-DEBUG]` 输出）
5. **准备一个可以手动发 UART 指令的方式**（用于调 J4）：
   - 方法 A：临时写一个 `uart_send_arm_target(0, 67, 40, servo1, 145)` 的调试按钮
   - 方法 B：直接用现有程序，改源码里的 `NRF_SERVO1_DEG` 重新编译运行
   - 方法 C：通过 RTSP 画面观察，先不动 J4，只记录 roll

> **推荐方法 B**：在 `nrf24_control_update()` 里临时加一个手动输入逻辑，或者直接用 `uart_send_arm_target()` 发固定值。

---

## 三、详细标定步骤

### 步骤 1：找 baseline（基准姿态）

**动作**：
1. 用户保持**头部水平**（自然平视前方，不要刻意抬头或低头）
2. 等待机械臂稳定（`[NRF-DEBUG]` 输出稳定，tx/ty/tz 不再大幅变化）
3. 观察 RTSP 画面：**人脸应该在画面中央**
   - 如果人脸偏上/偏下，先不管，后续用 J4 补偿
   - 如果人脸偏左/偏右，调整身体位置，让自己正对摄像头

**记录数据**：

```
执行: 读取当前 [NRF-DEBUG] 输出
记录: baseline_roll = ____°      （例：+2.3°）
记录: baseline_servo1 = ____°    （当前发下去的 J4 值，例：90°）
```

> 如果当前画面人脸偏下，说明 J4 需要往上抬（减小角度或增大角度，取决于极性）。先记下来，步骤 2 再调。

---

### 步骤 2：确定 baseline_servo1（J4 中位）

**动作**：
1. 保持头部水平不动
2. 手动发几组不同的 J4 角度，观察画面：
   - 先试 `servo1 = 90°`
   - 再试 `servo1 = 80°`
   - 再试 `servo1 = 100°`
   - 再试 `servo1 = 70°`、`110°`...

3. 找到**人脸最接近画面中央**的那个 servo1 值

**记录**：
```
画面最正时的 servo1 = ____°      → 这就是 baseline_servo1
```

> ⚠️ **注意极性**：如果 servo1 从 90° 调到 80° 时人脸从偏下变到偏上，说明：
> - 80° 时摄像头朝上抬了 → 人脸往下走了 → **servo1 减小 = J4 往上抬**
> - 这个极性关系要记下来，写入代码（如果反了，K_SERVO 取负）

---

### 步骤 3：测 K_SERVO（比例系数）

**动作**：
1. 保持 `servo1 = baseline_servo1`，头部回到水平
2. 确认画面人脸在中央（步骤 2 的结果）
3. **缓慢低头**（自然低头看手机的角度，约 15°~20°）
4. 观察画面：人脸会**移出画面上方**（因为摄像头没动，人低了）
5. 逐步调整 J4，直到人脸回到画面中央：
   - 从 `baseline_servo1` 开始，每次 ±5° 试
   - 记录最终让人脸回正的 servo1 值

**记录**：
```
低头后 current_roll    = ____°   （例：+18.5°）
低头后补偿 servo1_new  = ____°   （例：110°）

delta_roll  = current_roll − baseline_roll  = ____°   （例：18.5 − 2.3 = 16.2°）
delta_servo = servo1_new − baseline_servo1 = ____°   （例：110 − 90 = 20°）

K_SERVO = delta_servo / delta_roll = ____              （例：20 / 16.2 ≈ 1.23）
```

> 如果 delta_roll 是负值（IMU 读数减小），而 delta_servo 是正值（J4 增大），则 K_SERVO 为负。这表示极性相反。

---

### 步骤 4：反向验证（抬头）

**动作**：
1. 头部回到水平（画面应在中央）
2. **缓慢抬头**（约 15°~20°）
3. 用步骤 3 算出的 K_SERVO，计算预测补偿：
   ```
   delta_roll_verify = current_roll_verify − baseline_roll
   servo1_predict = baseline_servo1 + K_SERVO × delta_roll_verify
   ```
4. 直接发 `servo1_predict`，观察画面
5. 如果人脸基本在中央 → **K_SERVO 准确**
6. 如果人脸偏上/偏下 → 微调 K_SERVO（偏大或偏小 0.1~0.2）

**记录**：
```
抬头后 current_roll_verify = ____°
预测 servo1_predict        = ____°
实际发下去后画面效果      = [ ] 正中  [ ] 偏上  [ ] 偏下

如需微调：K_SERVO 从 ____ 改为 ____
```

---

### 步骤 5：限幅确认

**动作**：
1. 找到人头俯仰的**自然极限范围**：
   - 最大低头时 roll ≈ ____°
   - 最大抬头时 roll ≈ ____°
2. 代入公式算 J4 范围：
   ```
   servo1_min = baseline_servo1 + K_SERVO × (roll_min − baseline_roll)
   servo1_max = baseline_servo1 + K_SERVO × (roll_max − baseline_roll)
   ```
3. 确认 `servo1_min` 和 `servo1_max` 在 **0°~180°** 范围内
4. 如果超出范围，说明 K_SERVO 太大，需要调小（牺牲精度换范围）

---

## 四、标定数据记录表

| 参数 | 数值 | 备注 |
|------|------|------|
| baseline_roll | ____° | 人头水平时 IMU 读数 |
| baseline_servo1 | ____° | 画面最正时的 J4 角度 |
| K_SERVO | ____ | servo/roll 比例 |
| 极性 | [ ] 同向 [ ] 反向 | roll↑ 时 servo1↑ 还是 ↓ |
| roll_min（最大低头） | ____° | 自然极限 |
| roll_max（最大抬头） | ____° | 自然极限 |
| servo1_min | ____° | 计算值，确认 ≥0° |
| servo1_max | ____° | 计算值，确认 ≤180° |
| 验证结果 | [ ] 通过 [ ] 需微调 | 低头+抬头两次验证 |

---

## 五、快速写入代码

标定完成后，修改 `src/rga_npu.cpp`：

```cpp
/* ========== J4 标定参数（方案 A） ========== */
static const float SERVO1_BASELINE = ____;     // 填入 baseline_servo1
static const float ROLL_BASELINE   = ____;     // 填入 baseline_roll
static const float K_SERVO         = ____;     // 填入 K_SERVO
```

并在发令逻辑中加入 J4 补偿：

```cpp
// 在 roll 发令之后、yaw 发令之后，或单独判断
delta_roll = normalize_angle_deg(current_roll_deg - ROLL_BASELINE);
servo1 = SERVO1_BASELINE + K_SERVO * delta_roll;

// 限幅
if (servo1 > 180.0f) servo1 = 180.0f;
if (servo1 < 0.0f)   servo1 = 0.0f;

uart_send_arm_target(tx, ty, tz, servo1, NRF_SERVO2_DEG);
```

---

## 六、常见问题

### Q1：低头时画面人脸偏上，抬头时偏下，正常吗？
**A**：正常。因为摄像头在机械臂末端，固定不动。人低头了，摄像头还在原高度，人脸就跑出画面上方了。J4 的作用就是让人低头时摄像头也往下跟。

### Q2：K_SERVO 标出来是负数，怎么办？
**A**：负数说明极性相反（roll 增大时 J4 应该减小）。直接保留负值写入代码即可，`servo1 = baseline + (-0.8) * delta_roll` 会自动往反方向补偿。

### Q3：标定时机械臂在动，画面不稳怎么办？
**A**：先让机械臂到位（等 `complete`），再标定 J4。或者临时固定机械臂（发相同的 tx/ty/tz 让机械臂停住）。

### Q4：低头 15° 时 K_SERVO=1.2，但抬头 15° 时画面不正，为什么？
**A**：可能是非线性。IMU_roll 和 J4 的关系在小角度近似线性，大角度可能不是。可以分段标定：
- 小角度（±10°）：用一个 K
- 大角度（±20°）：用另一个 K
- 或者直接用查表法（roll→servo1 的映射表）

---

## 七、下一步

标定完成后：
1. 填入参数到代码
2. 编译运行
3. 观察效果
4. 如果满意 → 保持方案 A
5. 如果不满意（非线性严重或精度不够）→ 升级到方案 B（正运动学校正）
