# ELF 控制链移植指南

## 1. 概述

本指南说明如何将上游 C++ 项目 `/home/time/work/elf_info/wearable-robot-arm/` 的新功能移植到当前 Python 项目 `/home/time/work/trial0/`。

上游是 RK3588 平台上的 C++ 实现，使用 RKNN/RGA NPU、GStreamer、BlueZ 等；
当前 `trial0` 是 DK-2500（Intel Core Ultra 5 225U）上的 Python 移植版，使用 OpenVINO、pyserial、smbus2 等。

移植目标：**保留控制算法一致性**，将平台相关的视觉/流媒体代码替换为 `trial0` 已有的 OpenVINO/Python 实现，或按需新增。

---

## 2. 架构对照

```text
上游 (RK3588/C++)                      trial0 (DK-2500/Python)
─────────────────────────────────      ─────────────────────────────────
nrf24_linux.c/h  (SPI/NRF24)    ──►    STM32F103 DataHub bridge → Stm32DataHubBridge
imu2_i2c.c/h    (I2C 腰IMU)     ──►    STM32F103 DataHub bridge → Stm32DataHubBridge
rga_npu.cpp/h   (控制核心)       ──►    elf_control_chain.py::Nrf24Controller
ctrl_server.cpp/h (HTTP控制)    ──►    elf_control_chain.py::ElfControlThread HTTP
bluetooth_spp.c/h (BLE遥控)     ──►    tools/ble_remote.py (bleak)
uart_comm.cpp/h (机械臂UART)    ──►    SerialUartArmSink / BridgeUartArmSink
main.cpp        (启动/握手)      ──►    make_elf_control_thread()
GStreamer RTMP/RTSP            ──►    camera_demo_lowlatency.py (ffmpeg RTMP)
WebSocket 云端心跳              ──►    camera_demo_lowlatency.py (websocket-client)
RuleEngine v2 socket           ──►    camera_demo_elf_pipeline.py::RuleEngineState (NumPy)
STM32F103 DataHub bridge        ──►    /home/time/work/stm32f103_datahub/ (外部维护)
```

---

## 3. 已移植 vs 未移植

| 上游功能 | trial0 状态 | 说明 |
|---|---|---|
| NRF24 22B IMU 解析 | ✅ 已移植 | `parse_nrf24_imu_payload()` 与上游一致；生产环境由 STM32F103 DataHub 桥以 `TYPE 0x52` 转发原始 22 字节载荷，Python 解码 |
| 头 IMU 欧拉角 + 陀螺仪 | ✅ 已移植 | 输出 roll/pitch/yaw/qw/qx/qy/qz/wx/wy/wz；桥转发 nRF24 原始 22 字节载荷，Python 解码并反算四元数 |
| 腰 IMU 欧拉角 + 陀螺仪 | ✅ 已移植 | 桥转发 JY61P 原始 WIT 帧，Python 解码出欧拉角与陀螺仪 |
| 双 IMU A-inverse (`R_init`, `R_imu2_init`) | ✅ 已移植 | `elf_control_chain.py` 已实现 |
| 8 态运动 FSM | ✅ 已移植 | `MotionContext` + `next_motion_state` |
| Endpoint predictor | ✅ 已移植 | 相同阈值和 k 表 |
| Spherical IK / servo mapping | ✅ 已移植 | `_compute_arm_pose()` |
| `far_l3_55` / `mid_l3_40` profile | ✅ 已移植 | `ArmKinematicsProfile` |
| profile 运行时切换 | ✅ 已移植 | HTTP `/profile?idx=0/1` + BLE `0x01` |
| pitch sign 切换 | ✅ 已移植 | HTTP `/cmd?action=toggle_pitch_sign` |
| PnP yaw 补偿表 | ✅ 已移植 | `interp_yaw_table()` + async PnP demo 已接入 |
| PnP pitch 补偿表 | ✅ 已移植 | `interp_pitch_table()` + async PnP demo 已接入 |
| mount 矩阵 `Ry(-14°)·Rx(-0.10rad)` | ✅ 已移植 | `PoseEstimator.R_mount` + `camera_demo_lowlatency.py` 已应用 |
| 12 点 FaceMesh PnP pipeline | ✅ 已移植 | `camera_demo_elf_pipeline.py::PoseEstimator` |
| PnP drift correction 增益 | ✅ 已对齐 | `KI_PNP=0.04`, `KI_PNP_PITCH_BIAS=0.02`, 3° deadband |
| PnP pitch bias correction | ✅ 已对齐 | 同上 |
| 7 秒 homing TX 阻塞 | ✅ 已移植 | `SerialUartArmSink` 收到 init_success 后阻塞 7 s；STM32 DataHub 桥模式下由 H7 决定 |
| `FF` 校验帧握手 | ✅ 已移植 | `UartArmSink.FF_VERIFY_FRAME` 10 字节 raw 帧；用于直接 H7 串口模式 |
| 标定模式 1–5 | ✅ 已移植 | `Nrf24Controller._run_calibration()` + CSV 输出 |
| `/servo` 写 `/tmp/servo_calib.txt` | ✅ 已移植 | `ElfControlThread.send_servo_test()` 持久化 k1 |
| `MODE_INTRO` / `MODE_INTERVIEW` | ✅ 已移植 | HTTP `/mode?type=intro/interview` + `update_intro/interview_control()` |
| HC-08 蓝牙遥控 | ✅ 已移植 | `tools/ble_remote.py`（bleak 可选依赖） |
| RuleEngine v2 socket | ✅ 已替代 | `camera_demo_elf_pipeline.py::RuleEngineState` 用 NumPy 在进程内实现同样的 7 维状态推理 |
| GStreamer 推流 / WebSocket | ✅ 已替代 | `camera_demo_lowlatency.py` 用 ffmpeg RTMP + websocket-client 实现等价推流与云端心跳 |

---

## 4. 协议与数据格式约定

### 4.1 STM32F103 DataHub → DK-2500 分类桥接协议

**真实 STM32 固件在 `/home/time/work/stm32f103_datahub/`，本仓库不再维护 STM32 代码。最新版本已将 USART2/3 RX 改为短中断 + ring buffer，并在固件目录提供 `BRIDGE_PROTOCOL.md` 和 `loopback_test.py` 作为协议参考。**

trial0 Python 端解析 STM32F103 通过 USART3 下发的分类桥接帧，波特率 `460800`。桥不对传感器数据做解码，只按来源分类转发原始字节。

#### 桥接帧格式

```text
A5 TYPE LEN SEQ PAYLOAD... CRC
```

字段：

- `A5`：帧头
- `TYPE`：来源类型
  - `0x51`：腰部 JY61P IMU 原始 WIT 帧（11 字节）
  - `0x52`：nRF24 头 IMU 原始 22 字节载荷
  - `0x53`：STM32H7 机械臂 VOFA JustFloat 流（28 字节，`float[7]`）
- `LEN`：payload 字节数
- `SEQ`：8 位序列号，255 回绕
- `PAYLOAD`：原始上游字节，不变
- `CRC`：`TYPE + LEN + SEQ + PAYLOAD` 的 8 位无符号和

Python 解析器：`elf_control_chain_stm32.py::Stm32DataHubBridge`。

#### 各 payload 解码

- **JY61P WIT 帧（11 字节）**：由 Python 端 `elf_control_chain_stm32._parse_wit_frame()` 解码
  - `0x53` 角度帧：`roll = int16/32768*180`，`pitch`、`yaw` 同理，单位度
  - `0x52` 陀螺仪帧：`wx = int16/32768*2000`，`wy`、`wz` 同理，单位 deg/s
- **nRF24 22 字节载荷**：由 `elf_control_chain.parse_nrf24_imu_payload()` 解码，包含四元数/欧拉角 + 陀螺仪
- **H7 VOFA 流（28 字节）**：调试/运动规划数据，由 `get_arm_debug_vofa()` 暴露为命名字段：
  - `s`：规划位移
  - `v`：规划速度
  - `a`：规划加速度
  - `motor0_target`：`LK4005_Motor_Handle[0].Motor_Position_Target`
  - `motor0_actual`：`LK4005_Motor_Handle[0].Motor_Position_Actual`
  - `error_s`：位移误差
  - `tail`：VOFA `INFINITY` 帧尾标记，不是数据

#### 机械臂目标下发帧（USART3 RX 已启用，短中断 ring buffer 处理）

DK-2500 通过 USART3 向桥下发目标，桥解析后通过 USART2 转发给 H7。下行使用分帧协议：

```text
AA 55 LEN CMD PAYLOAD... CRC
```

- `CRC` = `sum8(LEN + CMD + PAYLOAD)`

当前使用的命令：

| CMD | 名称 | LEN | Payload | 桥行为 |
|---|---|---|---|---|
| `0x01` | HEARTBEAT | `0` | 无 | 桥消费，不转发 |
| `0x30` | ARM_TARGET | `11` | `x,y,z,k1,k2`（int16 LE）+ `flag`（uint8） | 转发 11 字节 payload 到 USART2/H7 |
| `0x10` | TARGET_POSE | `28` | `float[7]` | 默认被桥消费，不转发 |

坐标目标帧示例：

```text
AA 55 0B 30 X(2) Y(2) Z(2) K1(2) K2(2) FLAG(1) CRC
```

- `LEN` = 11，`CMD` = `0x30`
- `X/Y/Z/K1/K2`：int16 LE
- `FLAG`：与 `SerialUartArmSink` 直接接 H7 时的第 11 字节一致
- `CRC`：`LEN + CMD + payload` 的 8 位无符号和

特殊 10 字节 H7 命令（裸发，不带 `AA 55`）会被桥原样转发：

```text
FF AA FF AA FF AA FF AA FF AA  -> H7 init 命令
AA FF AA FF AA FF AA FF AA FF  -> H7 retract/exit 命令
```

串口参数：`460800 8N1`。使用 `BridgeUartArmSink`（经 F103 桥转发）或 `SerialUartArmSink`（DK-2500 直接接 H7）。

> 说明：USART3 桥接链路原设计为 921600，但在当前 CH341 USB-TTL + 杜邦线布线条件下，bridge TX 信号会耦合到 arm RX 线上，导致 framed 下行 payload 错位。将链路降到 460800 后串扰可控，控制带宽仍足够。如果后续改用更短/屏蔽/差分的线材，可以重新在固件和 `DEFAULT_BRIDGE_BAUD` 中设回 921600。

端口自动探测：`Stm32DataHubBridge(port=None)`、`make_elf_control_thread(stm32_uart_port=None)` 以及 `tools/stm32_bridge_loopback.py` 都会短暂监听 USB 串口，找到正在发送 `A5` 桥接帧的端口作为 bridge；打开串口后默认等待 2 秒，让 F103 从 pyserial 触发的 DTR 复位中恢复。

硬件环回验证：

```bash
# 自动探测 bridge/arm 端口
python3 tools/stm32_bridge_loopback.py

# 或显式指定
python3 tools/stm32_bridge_loopback.py --bridge-port /dev/ttyUSB1 --arm-port /dev/ttyUSB0

# 链路诊断：分别测试噪声底、下行 target、上行 VOFA、串扰
python3 tools/stm32_bridge_diag.py

# 串扰专用探测：发送 CRC 错误帧，若 arm 口仍有数据则为物理串扰
python3 tools/stm32_bridge_crosstalk_probe.py
```

`stm32_bridge_loopback.py` 与固件目录的 `loopback_test.py` 对应，采用同步 ping-pong 方式模拟真实控制流：host 发送一帧 `AA 55 0B 30 ...` 后等待该命令从 arm 口出现，arm 再回一帧 VOFA，host 等待对应的 `A5 53` 桥接帧返回， quiet 时间后再进入下一轮。这种同步模式避免了廉价 USB-TTL 适配器在全双工并发时的 echo/混叠假象，能够严格校验每一帧的 payload 是否正确。如果 framed 下行命令出现错位/混叠，应先用 `stm32_bridge_crosstalk_probe.py` 判断是否存在物理串扰。

### 4.2 NRF24 空中载荷

STM32F103 桥把 nRF24 22 字节载荷作为 `TYPE 0x52` 桥接帧原样转发给 DK-2500。Python 端用 `parse_nrf24_imu_payload()` 解码，保留该函数也用于测试与离线回放。

原始 22 字节格式保持不变：

```text
[11 bytes orientation frame] [11 bytes gyro frame]
```

每帧：

```text
[0x55] [TYPE] [8 bytes data] [checksum]
```

- TYPE `0x59`：四元数 `q0/q1/q2/q3`，`q = int16 / 32768.0`
- TYPE `0x52`：陀螺仪 `wx/wy/wz`，`w = int16 / 32768.0 * 2000.0` deg/s
- checksum = `sum(bytes[0..9]) & 0xFF`

### 4.3 BLE 遥控协议（HC-08）

3 字节帧：

```text
[0x55] [CMD] [VAL]
```

| CMD | VAL | 含义 |
|---|---|---|
| 0x01 | 0x00 | profile = L3-40 |
| 0x01 | 0x01 | profile = L3-55 |
| 0x02 | 0x00 | scene = FACE |
| 0x02 | 0x01 | scene = INTRO |
| 0x02 | 0x02 | scene = INTERVIEW |
| 0x02 | 0x03 | scene = BODY |
| 0x03 | 0xXX | toggle pitch sign |

---

## 5. 分阶段移植计划

### Phase 1 — 安全修复与参数对齐（必须先做）

目标：消除接硬件前的安全隐患。

> **STM32F103 DataHub 固件在 `/home/time/work/stm32f103_datahub/` 由项目所有者维护，以下 1.1–1.3 需在桥固件中完成，不再属于 trial0 仓库。**

| # | 任务 | 涉及文件 | 工作量 | 风险 |
|---|---|---|---|---|
| 1.1 | 桥固件原样转发 JY61P WIT 帧和 nRF24 载荷，Python 按度解码 | `/home/time/work/stm32f103_datahub/UserApp/Src/app.c` | 小 | 高（不修正会误动作） |
| 1.2 | 桥固件确认不做单位转换 / 不截断 payload | `/home/time/work/stm32f103_datahub/UserApp/Src/app.c` | 小 | 高 |
| 1.3 | 桥固件 USART3 RX 解析目标帧并转发到 H7 | `/home/time/work/stm32f103_datahub/UserApp/Src/app.c` | 中 | 中 |
| 1.4 | ✅ Python `UartArmSink` 实现 7 秒 homing 阻塞 | `elf_control_chain.py` | 小 | 中 |
| 1.5 | ✅ Python 发送 `FF` 校验帧 | `elf_control_chain.py` | 小 | 低 |
| 1.6 | ✅ PnP 增益改 `KI_PNP=0.04`，pitch bias `0.02` | `elf_control_chain.py` | 小 | 低 |
| 1.7 | ✅ 加 3° PnP deadband | `elf_control_chain.py` | 小 | 低 |

### Phase 2 — HTTP / 遥控接口补齐

| # | 任务 | 涉及文件 | 工作量 | 风险 |
|---|---|---|---|---|
| 2.1 | ✅ `/mode?type=intro` 和 `/mode?type=interview` | `elf_control_chain.py` | 小 | 低 |
| 2.2 | ✅ `/profile?idx=0/1` | `elf_control_chain.py` | 小 | 低 |
| 2.3 | ✅ `/servo` 写 `/tmp/servo_calib.txt` | `elf_control_chain.py` | 小 | 低 |
| 2.4 | ✅ HC-08 BLE 监听（bleak） | `tools/ble_remote.py` | 中 | 中 |

### Phase 3 — 标定与 PnP 补偿

| # | 任务 | 涉及文件 | 工作量 | 风险 |
|---|---|---|---|---|
| 3.1 | ✅ 标定模式 1（servo calib）：读文件 + 固定位置驱动 | `elf_control_chain.py` | 中 | 中 |
| 3.2 | ✅ 标定模式 2（scan test）：J4 扫掠 | `elf_control_chain.py` | 中 | 中 |
| 3.3 | ✅ 标定模式 3/4（PnP yaw/pitch calib）：运动目标 + CSV 记录 | `elf_control_chain.py` + camera demo | 大 | 中 |
| 3.4 | ✅ 标定模式 5（head IMU 3×3 grid） | `elf_control_chain.py` | 中 | 中 |
| 3.5 | ✅ mount 矩阵 + 补偿表接入 controller | `elf_control_chain.py` / camera demo | 中 | 低 |
| 3.6 | ✅ 完整 12 点 PnP 视觉 pipeline（OpenVINO 复刻） | `camera_demo_elf_pipeline.py::PoseEstimator` | 大 | 中 |

### Phase 4 — 场景模式

| # | 任务 | 涉及文件 | 工作量 | 风险 |
|---|---|---|---|---|
| 4.1 | ✅ `update_intro_control()` | `elf_control_chain.py` | 大 | 中 |
| 4.2 | ✅ `update_interview_control()` | `elf_control_chain.py` | 大 | 中 |
| 4.3 | ✅ `send_face_home_from_scenario()` | `elf_control_chain.py` | 小 | 低 |

### Phase 5 — 可选

| # | 任务 | 涉及文件 | 工作量 | 风险 |
|---|---|---|---|---|
| 5.1 | RuleEngine v2 socket client | 新增模块 | 中 | 低 |
| 5.2 | GStreamer 推流 / WebSocket | 可选，非控制链 | 大 | 低 |

---

## 6. 关键代码映射

### 6.1 rga_npu.cpp → elf_control_chain.py

| 上游 | Python | 说明 |
|---|---|---|
| `MODE_FACE/BODY/INTRO/INTVIEW` | `ElfControlThread.pose_mode` | 已支持 `face`/`body`/`intro`/`interview` |
| `process_frame_body()` | `Nrf24Controller.update()` | 入口不同，核心算法一致 |
| `update_intro_control()` | `ElfControlThread.update_intro_control()` | 已移植 |
| `update_interview_control()` | `ElfControlThread.update_interview_control()` | 已移植 |
| `set_pose_mode()` | `ElfControlThread.set_pose_mode()` | 已扩展 intro/interview，并带 face-home 过渡 |
| `ArmKinematicsProfile` | `ArmKinematicsProfile` dataclass | 已对齐 |
| `quatToMat` / `eulerZYXToMat` | 已实现 | 已对齐 |
| `axisProjectionYawPitch` | 已实现 | 已对齐 |
| `estimate_and_draw_pose()` | `camera_demo_elf_pipeline.py::PoseEstimator` | 已用 OpenVINO 复刻 12 点 FaceMesh PnP |
| `interpolate_pnp_yaw_table()` | `interp_yaw_table()` / `interp_pitch_table()` | 已接入 async PnP demo |

### 6.2 STM32F103 DataHub / H7 机械臂 → Python

| 上游行为 | Python 当前 | 说明 |
|---|---|---|
| 桥接帧输出 | `Stm32DataHubBridge._parse_buffer()` | 已按 `A5 TYPE LEN SEQ PAYLOAD CRC` 解析 |
| 头 IMU + 腰 IMU 聚合 | `BridgeNrf24ImuSource` + `BridgeImu2Source` | 桥转发原始 WIT/nRF 载荷，Python 解码并反算四元数 |
| 机械臂目标下发 | `BridgeUartArmSink` / `SerialUartArmSink` | 桥模式使用下行帧 `AA 55 0B 30 X Y Z K1 K2 FLAG CRC`；直连 H7 发送 11 字节原始坐标帧 |
| A-init 5 帧 | `Nrf24ImuSource` 已统计 5 帧 | 已对齐 |
| `init success` / `FF` 握手 | `SerialUartArmSink` | 仅用于直接 H7 串口模式；桥模式由 F103/H7 处理，Python 侧直接标记 init_success |

### 6.3 bluetooth_spp.c → Python

已实现 `tools/ble_remote.py`：
- 用 `bleak` 扫描并连接 HC-08（MAC `F8:2E:0C:E3:99:C8`）
- 监听 UART RX characteristic 通知
- 收到 3 字节帧 `[0x55] [CMD] [VAL]` 后通过 HTTP 调用 `elf_control_chain` 的对应接口（profile/scene/toggle pitch sign）

---

## 7. 安全测试清单

在接上真实机械臂前，必须确认：

- [ ] 桥固件正确转发 JY61P WIT 帧（度）和 nRF24 22 字节载荷（`/home/time/work/stm32f103_datahub/`）
- [ ] Python WIT/nRF 解析器得到正确的 roll/pitch/yaw/wx/wy/wz
- [ ] 桥模式：F103 USART3 RX 正确解析 `AA 55 0B 30 ...` 目标帧并转发给 H7
- [x] Python 在收到 `init success` 后发送 `FF` 校验帧（直接 H7 串口模式）
- [x] Python 在 `FF` 发送后阻塞 TX 至少 7 秒（直接 H7 串口模式）
- [x] A-init 完成前（5 帧有效姿态）controller 不输出目标点
- [x] PnP drift correction 增益已改为 `0.04` / `0.02`
- [x] 标定模式有明确的急停/退出机制
- [x] `/mode` 切换时有 home 过渡（避免 mode 跳变导致机械臂甩动）

---

## 8. 测试策略

### 8.1 单元测试
- `tests/test_nrf24_parser.py`：NRF24 22 字节解析
- `tests/test_stm32_bridge.py`：UART 帧协议
- `tests/test_porting.py`：PnP 参数、握手、HTTP 接口、标定、场景模式、BLE 遥控

### 8.2 集成测试
1. **stub 模式**：`make_elf_control_thread(stub=True)` 跑完整控制循环，验证无异常。
2. **STM32 桥 loopback**：用 USB 转串口把 TX/RX 短接，验证帧收发。
3. **真实 STM32F103 DataHub 桥 + DK-2500**：桥 USART3 接 DK-2500，验证 `A5 TYPE LEN SEQ PAYLOAD CRC` 桥接帧解析、头/腰 IMU 数据。
4. **真实机械臂空载**：H7 串口接 DK-2500，验证 homing 7 秒阻塞、A-init、基础 face/body 控制。
5. **真实 IMU2**：由桥读取 JY61P 腰 IMU，验证腰部补偿效果。

---

## 9. 附录：上游关键文件清单

| 文件 | 用途 |
|---|---|
| `src/rga_npu.cpp/h` | 控制核心、场景模式、标定、PnP |
| `src/bluetooth_spp.c/h` | HC-08 蓝牙遥控 |
| `src/ctrl_server.cpp/h` | HTTP 控制接口 |
| `src/uart_comm.cpp/h` | UART 11 字节目标帧、握手 |
| `src/main.cpp` | 启动流程、7 秒 homing |
| `src/nrf24_linux.c/h` | NRF24 驱动、22 字节解析 |
| `src/imu2_i2c.c/h` | 腰 IMU2 I2C 驱动 |
| `docs/calibration-l3-55-2026-07-02.md` | L3-55 标定说明 |
| `docs/servo-control-design.md` | 舵机控制设计 |
| `docs/motion-control-framework.md` | 运动控制框架 |
| `docs/audit-prompt.md` | 审计与设计要点 |

## 10. 附录：STM32F103 DataHub 桥固件

该固件由项目所有者在 `/home/time/work/stm32f103_datahub/` 维护，**不属于 trial0 仓库**。

trial0 相关对接文件：

| 文件 | 用途 |
|---|---|
| `BRIDGE_PROTOCOL.md` | 分类桥接协议描述（`A5 TYPE LEN SEQ PAYLOAD CRC`）及下行帧格式 |
| `UserApp/Src/app.c` | 主循环，桥接帧转发、下行命令解析与路由 |
| `UserApp/Inc/app_config.h` | 波特率、桥接帧类型常量、下行命令常量 |
| `UserApp/Src/dk2500_link.c` | 调试快照帧（`APP_DEBUG_SNAPSHOT_ENABLE=0` 时不使用） |
| `UserApp/Src/jy61p.c` | JY61P WIT 帧解析 |
| `UserApp/Src/arm_uart.c` | H7 侧目标帧 / ASCII 解析 |
| `UserApp/Src/nrf24.c` | nRF24L01+ 驱动 |

> 注意：MDK-ARM 工程实际编译的是 `UserApp/Src` 与 `UserApp/Inc`；`Core/Src` 与 `Core/Inc` 是旧快照协议遗留，未被编译。

本仓库对应的 Python 解析器：`elf_control_chain_stm32.py::Stm32DataHubBridge`。
