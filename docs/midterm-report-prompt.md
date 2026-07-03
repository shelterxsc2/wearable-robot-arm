# 中期检查报告生成 Prompt

> 本文档包含撰写《可穿戴机械臂上位机系统中期检查报告》所需的全部背景信息、技术细节和格式要求。阅读本文档后，无需查看源代码即可独立写出完整报告。

---

## 一、项目概述

**项目名称**：ELF2 (RK3588) 可穿戴机械臂上位机控制系统
**当前分支**：`imu-victor-hat`
**核心目标**：通过头戴/颈挂 NRF24 无线 IMU 实时控制背负式机械臂，实现"头部转动 → 机械臂跟随"的穿戴式人机协同。

### 1.1 硬件组成

| 组件 | 型号/规格 | 职责 |
|:---|:---|:---|
| 上位机 | RK3588 (ELF2 开发板) | IMU接收、运动预测、NPU推理、RTMP/RTSP推流、UART通信、端侧HTTP控制 |
| 下位机 | STM32H723 | 逆运动学、S-curve 7相速度规划、电机/舵机驱动 |
| 摄像头 | Realtek USB Camera (`/dev/video21`) | 1920×1080@30fps YUY2 |
| 无线IMU | NRF24L01+ + 陀螺仪 | ~100Hz发送 roll/pitch/yaw + wx/wy/wz |
| 机械臂 | 可穿戴三轴(J1~J3) + 末端双舵机云台(J4俯仰/J5水平) | J1:腰部Pan, J2:大臂俯仰, J3:小臂俯仰, J4:俯仰舵机, J5:水平舵机 |
| 通信 | UART TTL `/dev/ttyS9` @ 115200 | 11字节帧 (5×int16坐标 + 1×uint8标志) |

### 1.2 系统控制链路（三轨并行）

**链路A — IMU粗调链路（当前活跃核心）：**
```
IMU(头部) → NRF24无线 → RK3588 SPI
                    ↓
            A-inverse矩阵解耦 (R_rel = R_current × R_init^T)
                    ↓
            8状态运动状态机 + 终点预测器
                    ↓
            球坐标目标生成 (tx,ty,tz) + 舵机映射 (servo1,servo2)
                    ↓
            UART 11字节帧 → STM32 → 电机/舵机
```

**链路B — 手势/视觉链路（保留但非本阶段重点）：**
```
USB Camera → YOLO-Pose(best.rknn, 640×640) → 17 COCO关键点
                    ↓
            Face Landmark(468点, 192×192) + PnP
                    ↓
            Python RuleEngine v2 Socket服务
                    ↓
            3状态手势状态机 → UART → STM32
```

**链路C — 视觉 PnP 零飘修正闭环（当前已集成，实验态）：**
```
USB Camera → 两阶段NPU推理 → Face Landmark 468点
                    ↓
            12点PnP → solvePnP → (rvec, tvec)
                    ↓
            头部姿态欧拉角提取 (pnp_yaw, pnp_pitch)
                    ↓
            固定安装校正 + 机械臂目标 yaw 查表补偿
                    ↓
            静止态(is_stop && is_stop_yaw)触发 R_bias_total 累积修正
                    ↓
            允许静止态 yaw 发令 → UART flag=0x01 → STM32 微调
```

**链路D — 云端交互（Cloud Streaming Monitor）：**
```
上位机(ELF2)
    ├── RTMP推流 ──→ 云端MediaMTX/FFmpeg ──→ LL-HLS低延迟直播(约3s)
    ├── WebSocket注册帧 ──→ 云端Node.js服务 (ws://47.93.162.124/ws?deviceId=device-003)
    │                         ↓
    │                   100ms心跳 (frame_count, elapsed, timestamp)
    │
    └── 端侧HTTP API(8080) ←── 浏览器/Android WebView
                              /status, /mode, /calib, /servo, /cmd
```

**云端系统功能模块：**

| 模块 | 技术 | 功能 |
|:---|:---|:---|
| 实时播放 | LL-HLS (hls.js) | 低延迟直播(~3s)，浏览器/WebView兼容，视频高度可调(60~500vh) |
| 远程控制 | Vue 3 + WebSocket | 离散状态控制(x,y,z ∈ {0,1,2})，Minecraft风格3×3方向键，R/Y/P参数滑块 |
| 录播点播 | MongoDB + FFmpeg | 按时间段查询录像，20min分片自动合并，进度条拖拽，一键下载 |
| 用户系统 | JWT + Session + 设备指纹 | root/普通用户分级，首次登录绑定设备指纹(SHA256)，仅授权设备可登录 |
| Android App | WebView + Gradle | sensorLandscape横屏，LOAD_NO_CACHE无缓存，version.properties版本管理 |

**云端技术栈：**
- 后端：Node.js + Express + MongoDB + WebSocket(ws)
- 前端：Vue 3 (CDN) + HLS.js + 原生CSS
- 视频：FFmpeg + MediaMTX
- 安卓：Android WebView + Gradle 8.0

**云-端接口协议：**

| 接口 | 方向 | 协议 | 内容 |
|:---|:---:|:---|:---|
| RTMP推流 | ELF2 → 云 | RTMP | `rtmp://47.93.162.124:1935/live/device-003` |
| WebSocket注册 | ELF2 → 云 | WS | `{"type":"frame_ts"}` (500ms内) |
| WebSocket心跳 | ELF2 → 云 | WS | `{"type":"frame_ts","data":{"timestamp":..., "frame_count":...}}` (100ms) |
| HTTP状态查询 | 浏览器 → ELF2 | HTTP GET | `/status` → pose_mode, stream_type, uart_move_complete, head_stationary, arm_stable |
| HTTP模式切换 | 浏览器 → ELF2 | HTTP POST | `/mode?type=face/body` |
| HTTP标定 | 浏览器 → ELF2 | HTTP POST | `/calib?mode=0/1/2` |
| HTTP舵机测试 | 浏览器 → ELF2 | HTTP POST | `/servo?k1=50&k2=145` |
| HTTP指令 | 浏览器 → ELF2 | HTTP POST | `/cmd?action=rebaseline/nrf24_reset` |
| 云端方向键 | 浏览器 → 云 → ELF2 | WebSocket | (x,y,z)离散状态，模3递增/循环递减 |

**控制协议变更（v1.0.9重大重构）：**
- 从连续坐标控制 → 改为 (x,y,z) 离散状态组控制
- 每维取值 {0,1,2}，初始值 (1,1,1)
- 上/下键控制 x，左/右键控制 y，左上/右上键控制 z
- 加键 `(val + 1) % 3`，减键循环递减 `val != 0 ? val - 1 : 2`
- 中键复位为 (1,1,1)

---

## 二、核心算法与技术方案

### 2.1 A-inverse 姿态解耦

**问题**：IMU初始安装姿态未知，直接使用欧拉角减法会因初始 roll/yaw 的非线性耦合导致俯仰极性反转。

**方案**：
1. 上电后阻塞等下位机 `init success`
2. 发送 `FF AA` 验证帧
3. 等待7s归位延时（期间禁止UART发送）
4. 下一帧有效IMU数据时捕获 `R_init`
5. 后续每帧计算 `R_rel = R_current × R_init^T`
6. 从 `R_rel` 反解 ZYX 欧拉角得到 `rel_roll`, `rel_pitch`, `rel_yaw`

**关键函数**：
- `eulerZYXToMat(roll, pitch, yaw)` — ZYX顺序构建旋转矩阵
- `matToEulerZYX(R_rel)` — 反解欧拉角，处理奇异分支(sy < 1e-6)

### 2.2 8状态运动状态机

基于 50ms 窗口内 10ms 分辨率的 wz/wy 历史分析。

```
STATE_STOP → STATE_STOP_TO_ACCEL → STATE_ACCEL → STATE_ACCEL_TO_CONST
    ↑                                                                    ↓
    └────────────────────────────────────────────────────────────────────┘
    (循环)
    
STATE_ACCEL_TO_CONST → STATE_CONST_SPEED → STATE_CONST_TO_DECEL → STATE_DECEL_TO_STOP → STATE_STOP
```

**状态转移规则**：
- 静止阈值：`|wz| < 5°/s`（窗口平均）
- 加速趋势：`|wz_latest| > |wz_first| + 10°/s` 且不过零
- 减速趋势：`|wz_latest| < |wz_first| - 10°/s` 且不过零
- 过零瞬态：速度穿过零点不停留 → 直接转加速（不停顿）
- 状态确认：连续2周期一致才切换（过零反转除外）

### 2.3 终点预测器

**核心公式**：`pred_delta = wz × dt × k`

| 速度区间 | 预测窗口 dt | 说明 |
|:---|:---:|:---|
| |wz| < 20°/s | 500ms | 低速 |
| 20°/s ≤ |wz| < 60°/s | 350ms | 中速 |
| |wz| ≥ 60°/s | 250ms | 高速 |

**状态调制系数 k**：

| 状态 | k | 说明 |
|:---|:---:|:---|
| STATE_ACCEL_TO_CONST | **0.70** | ★主窗口，最确定 |
| STATE_CONST_TO_DECEL | 0.60 | 修正窗口 |
| STATE_ACCEL | 0.35 | 还在加速 |
| STATE_CONST_SPEED | 0.30 | 不确定 |
| STATE_DECEL_TO_STOP | 0.25 | 剩余位移少 |
| STATE_STOP / STATE_STOP_TO_ACCEL | 0.00/0.35 | 静止/刚启动 |

### 2.4 发令策略分化

**Yaw（偏航）**：
- ✅ 主窗口 `STATE_ACCEL_TO_CONST`：发一次预测令
- ✅ 静止态：允许发令，供 PnP 零飘修正驱动机械臂微动
- ✅ 其他运动态：按当前代码条件放行，但仍受 5° 阈值、150/200ms 间隔和 1cm 位置死区约束

**Pitch（俯仰）**：
- ✅ 主窗口/第二窗口：发预测令
- ✅ 静止态：发实际坐标（保持微调能力）
- 回退策略更宽松

**合并条件**：`should_cmd = pitch_should_cmd || yaw_should_cmd`

### 2.5 球坐标运动学

```cpp
const float l1 = 8.0f, l2 = 5.0f, l3 = 40.0f, l4 = 28.0f;
const float k = 1.6f;
last_tx = l3 * sin(cum_yaw) * cos(cum_pitch);
last_ty = l4 - l1 * sin(cum_pitch) + l3 * cos(cum_pitch) * cos(cum_yaw);
last_tz = l2 + l1 * cos(cum_pitch * k) + l3 * sin(cum_pitch * k) * cos(cum_yaw);
```

**角度限幅**：`cum_pitch_offset`, `cum_yaw_offset` 均 clamp 在 ±π/2

### 2.6 舵机映射

| 舵机 | 基线 | 比例 | 限幅 | 说明 |
|:---|:---:|:---:|:---:|:---|
| J4 (servo1, 俯仰) | 30° | K=-1.2 | [-90°, +90°] | 抬头→负(向外), 低头→正(向内) |
| J5 (servo2, 水平) | 50° | K=+0.4 | [0°, 270°] | 左偏→负(左), 右偏→正(右) |

### 2.7 视觉 PnP 零飘修正闭环（当前实现）

**背景**：IMU开环控制存在5°发令阈值、短距减速陷阱和长期零飘。视觉PnP提供独立头部姿态观测，当前已接入静止态 yaw 零飘修正。

**PnP解算流程**：
1. 从468个面部landmarks中选取12个稳定点（眼角、鼻尖、嘴角、眉心）
2. 构建3D模板点（以鼻尖为原点，+X向左，+Y向下，+Z向后）
3. `solvePnP` 解算 `rvec`, `tvec`
4. `cv::Rodrigues(rvec, R)` 转为旋转矩阵
5. `matToEulerZYX(R, roll, pitch, yaw)` 提取头部姿态

**硬丢弃策略（当前OneEuroFilter已禁用）**：
- 重投影误差 > 25px → 丢弃该帧
- 旋转跳变 > 60° → 丢弃该帧
- 镜像解（翻转）→ 丢弃该帧

**当前触发条件**：
```
g_r_init_set == 1
&& is_stop && is_stop_yaw
&& pnp_correction_ready == true
```

**当前修正方式**：
```cpp
pitch_delta = 0.0f;  // 当前关闭 pitch 修正
yaw_delta = -0.05f * pnp_yaw_correction;
R_bias_total = R_delta * R_bias_total;
R_corrected = R_bias_total * R_current;
R_rel = R_corrected * R_init.t();
```

**PnP 校正与 IMU 控制的关系**：

| 阶段 | 主导传感器 | flag | 策略 |
|:---|:---|:---:|:---|
| 头部运动中 | IMU (NRF24) | 0x00 | 预测终点，快速响应 |
| 头部刚静止 | IMU (实际角度) | 0x01 | 最终到位 |
| 静止态 | 视觉PnP + IMU | 0x01 | PnP yaw 修正 R_bias_total，补偿 IMU 零飘 |

**当前限制**：
- `pnp_valid_cnt >= 1` 即置 `pnp_correction_ready`，尚未做连续多帧一致性过滤
- 修正是绝对值积分，不是误差积分，PnP 绝对值非零时会持续累积
- pitch 修正入口已预留，当前 `KI_PNP_PITCH=0.0`，待 `POST /calib?mode=4` 生成 `/tmp/pnp_pitch_calib.csv` 并填入补偿表后开启
- 固定安装校正为 `R_mount = Ry(-14°) * Rx(-0.10rad)`，并按 `PNP_YAW_CALIBRATION` 表对目标 yaw 插值补偿

### 2.8 新增协议标志位

UART帧从10字节扩展为11字节：
```
[x][y][z][k1][k2][flag]  ← 前10字节 int16小端, 第11字节 uint8
```

| flag | 语义 | 触发条件 |
|:---:|:---|:---|
| 0x00 | **预测/猜测坐标** | 头部运动中 (`!is_stop` 或 `!is_stop_yaw`) |
| 0x01 | **确定/最终坐标** | 头部静止后 (`is_stop = true`) 或 人为设定(标定/HTTP) |

---

## 三、关键性能指标（请整理成表格）

### 3.1 时域指标

| 指标 | 数值 | 说明 |
|:---|:---:|:---|
| NRF24采样周期 | ~10ms (100Hz) | 独立于视觉帧 |
| 上位机控制周期 | 50ms (20Hz) | GLib定时器 |
| UART发送周期 | 150~400ms+ | 变速，最小间隔150ms(主窗口)/200ms(普通) |
| 下位机控制周期 | 1ms | S-curve规划 |
| 舵机响应 | ~50ms | J4/J5快速跟随 |

### 3.2 机械臂运动特性

| 末端位移 | 速度限制 | 估算到位时间 |
|:---|:---:|:---:|
| S < 3cm | 15% | **~1.5s+** (小位移陷阱) |
| 3cm ≤ S < 6cm | 20% | ~1.2s |
| 6cm ≤ S < 15cm | 30% | ~800ms |
| S ≥ 15cm | 100% | ~400ms |

### 3.3 AI推理耗时（RK3588 NPU三核负载）

| 阶段 | 模型 | 输入尺寸 | 占AI总耗时 | 备注 |
|:---|:---|:---:|:---:|:---|
| Stage 1 | best.rknn (YOLO-Pose) | 640×640 | **~85%** | 主要瓶颈 |
| Stage 2 | face_landmark_468_fp16.rknn | 192×192 | ~10% | RGA硬件裁剪后推理 |
| PnP + Draw | solvePnP + OSD | 1080p | ~5% | 已接入静止态 yaw 零飘修正 |

### 3.4 视觉PnP精度指标（目标/规划值）

| 指标 | 当前值 | 目标值 | 说明 |
|:---|:---:|:---:|:---|
| PnP重投影误差 | ~15px | <10px | 12点稳定点集的标定精度 |
| 偏航角精度 | ±3° | ±1° | solvePnP解算偏航的均方根误差 |
| 俯仰角精度 | ±2° | ±1° | solvePnP解算俯仰的均方根误差 |
| PnP 修正增益 | 0.05 | 待优化 | `KI_PNP`，绝对值积分 |
| 有效帧门槛 | 1帧 | 连续多帧 | 当前单帧即触发 |
| Pitch 修正 | 关闭 | 待验证 | 当前只修 yaw |

### 3.4 通信协议

| 链路 | 协议 | 数据量 | 速率 |
|:---|:---|:---:|:---|
| NRF24 RX | 22B帧 (角度+陀螺仪) | 22B | ~100Hz |
| UART TX | 11B坐标帧 | 11B | 2.5~6.7Hz |
| UART RX | 文本帧 (init_success/move_success) | 变长 | 事件触发 |
| RuleEngine Socket | 312B输入/56B输出 | 312B/56B | 30Hz |
| RTMP推流 | H.264 FLV | 1080p30 | 自适应码率 |

### 3.5 控制阈值

| 参数 | 值 | 说明 |
|:---|:---:|:---|
| 位置死区 | 1cm | tx/ty/tz任一变化<1cm不发令 |
| 俯仰发令阈值 | 5° | delta_pitch_deg |
| 偏航发令阈值 | 5° | delta_yaw_deg |
| 最小发令间隔 | 150ms(主窗口)/200ms(普通) | |
| 静止判断 | wx/wz < 2°/s进入, >5°/s退出 | 滞后带 |
| 机械臂到位确认 | 静止持续800ms | g_arm_stable |

---

## 四、结构框图要求

报告中需要包含以下框图（请用Mermaid或文字描述绘制）：

### 4.1 系统总体架构框图
- 三层：传感器层 → 上位机(RK3588) → 下位机(STM32)
- 标出各模块和通信接口

### 4.2 IMU控制链路数据流图
- 从NRF24 RX到UART TX的完整数据流
- 包含：帧解析 → 历史缓冲 → A-inverse → 状态机 → 预测器 → 球坐标 → 舵机映射 → 协议封装

### 4.3 8状态运动状态机图
- 完整状态转移图，标注转移条件和停留条件

### 4.4 上位机-下位机握手时序图
- init_success → FF_AA → 7s延时(g_uart_block_tx=1) → A-init → NORMAL
- 标出每个阶段的UART发送权限

### 4.5 视觉 PnP 零飘修正闭环框图
- 从 Camera 输入到 PnP 解算、固定安装校正、查表补偿、写入 `g_nrf24_state`
- IMU 控制线程在 `is_stop && is_stop_yaw` 时更新 `R_bias_total`
- 标注：视觉仅在静止态参与 yaw 零飘修正，运动态由 IMU 预测主导

### 4.6 软件模块依赖图
- main.cpp / rga_npu.cpp / nrf24_linux.c / uart_comm.cpp / ctrl_server.cpp 之间的调用关系

### 4.7 云-端协同架构图
- 完整云-端-下位机三层架构
- 标注：ELF2(上位机) ↔ 云端(Node.js) ↔ 浏览器/Android
- 标注：RTMP推流链、WebSocket心跳链、HTTP控制链、UART指令链
- 数据流向：IMU/Camera → ELF2 → 云端 → 用户；用户操作 → 云端 → ELF2 → STM32

---

## 五、报告格式要求

1. **标题**：可穿戴机械臂上位机控制系统 — 中期检查报告
2. **章节**：
   - 1. 项目背景与目标
   - 2. 系统架构（含框图）
   - 3. 关键技术方案（A-inverse、8状态机、预测器、球坐标）
   - 4. 性能指标与测试数据（表格）
   - 5. 当前进度（已完成/进行中/待完成）
   - 6. 存在问题与解决方案
   - 7. 下一步计划
3. **图表**：至少包含上述7个框图/flowchart
4. **数据**：所有性能指标必须以表格形式呈现
5. **术语**：统一使用本文档中的术语（如"A-inverse"、"8状态机"、"主窗口"等）

---

## 六、当前进度摘要（供报告参考）

### 已完成 ✅
- NRF24 SPI驱动 + IMU帧解析 + 历史环形缓冲
- A-inverse矩阵解耦（消除初始安装角）
- 8状态运动状态机 + 终点预测器
- 球坐标运动学重构 + 动态舵机映射
- 发令策略分化（Yaw主窗口/Pitch多窗口）
- UART协议升级：10字节 → 11字节（新增预测/确定标志位）
- 握手时序重构：init_success → FF_AA → 7s延时拦截 → A-init → NORMAL
- 位置死区(1cm) + 静止检测滞后带
- 端侧HTTP控制服务器
- RTMP/RTSP自适应推流
- 两阶段NPU推理 + PnP头部姿态解算 + 静止态 yaw 零飘修正闭环（实验态）
- 云端LL-HLS低延迟直播系统（约3s延迟）
- WebSocket注册+心跳（100ms周期）
- 离散状态远程控制协议(x,y,z ∈ {0,1,2})
- 用户系统(JWT + 设备指纹绑定)

### 进行中 🔄
- J4舵机标定上机实测
- 俯仰极性稳定性验证（A-init随机姿态影响）

### 待完成 ⏳
- Body模型优化（轻量化突破帧率瓶颈）
- 下位机回传关节角（UART双向协议统一）
- PnP 修正策略优化（误差积分替代绝对值积分）
- PnP 连续多帧一致性过滤
- PnP精度验证与OneEuroFilter重新评估
- 参数热加载（不编译调参）
- CSV记录与离线回放
- **云端控制与IMU控制的协议统一**（离散状态 vs 连续坐标的融合）
- **云-端延迟优化**（当前WebSocket心跳100ms，RTMP~3s，目标<1s）
- 多设备管理与权限分级完善

---

## 七、已知问题（供报告"存在问题"章节）

1. **Body模型瓶颈**：best.rknn占AI总耗时~85%，是推流帧率主要瓶颈
2. **wx静止噪声**：静止时wx仍有尖峰，可能误判运动状态
3. **5°发令阈值延迟**：小角度头部动作不触发发令
4. **STM32短距减速陷阱**：<3cm位移速度限制15%，到位时间~1.5s+
5. **纯开环**：上位机无关节角/末端位姿反馈
6. **俯仰极性偶发反转**：A-init时头部姿态不同导致矩阵解耦符号不稳
7. **PnP修正仍为实验态**：已接入静止态 yaw 零飘修正，但当前为绝对值积分且单帧触发
8. **PnP精度不足**：重投影误差~15px，OneEuroFilter已禁用，硬丢弃策略可能漏掉有效帧
9. **视觉-IMU融合较弱**：当前只用 PnP 在静止态修正 yaw 零飘，没有互补滤波或卡尔曼融合
10. **云-端控制协议割裂**：云端用离散状态(x,y,z)，端侧用连续坐标(tx,ty,tz)，两套控制语义不互通
11. **云端延迟瓶颈**：RTMP推流延迟~3s，WebSocket心跳100ms，无法满足实时遥控需求
12. **设备指纹安全限制**：root用户首次登录绑定设备后，无法临时授权新设备应急

---

> **请基于以上全部信息，撰写一份完整的中期检查报告。报告应包含详细的结构框图、数据流图、状态机图、时序图，以及所有性能指标的表格化呈现。**
