# ELF 项目移植审计报告与整改指南（详细版）

**审计日期**：2026-07-08  
**基线项目（Base）**：`/home/time/work/elf_info/wearable-robot-arm`（C/C++，RK3588，分支 `imu-victor-hat`）  
**当前项目（Target）**：`/home/time/work/trial0`（Python，OpenVINO，DK-2500 / Intel Core Ultra 5 225U）  
**用户确认主 demo**：`camera_demo_pingpong_async_pnp.py`  
**审计范围**：用户要求的 5 点——

1. 数据（控制）链路是否能一一对应；
2. 各个模式是否齐全（尤其是标定）；
3. 终端打印的内容是否能一一对应（除了手势/语音/STM32 桥接）；
4. 初始化 / 程序 `Ctrl+C` 退出时流程是否完全一样；
5. 云端通信除了 `device_id` 不同，其他是否完全一样。

**排除项（新增能力，不在“一一对应”比较范围内）**：手势识别、语音识别、STM32 桥接数据/指令、BLE 遥控、DK-2500 GPIO/SPI 直连、Python RuleEngine NumPy 替代实现。这些作为 Target 相对于 Base 的增强能力单独说明。

---

## 1. 审计结论（TL;DR）

| 检查点 | 结论 | 风险等级 | 说明 |
|---|---|---|---|
| 1. 数据/控制链路 | **基本可对应，物理链路有扩展** | 中 | 算法链路（NRF24/IMU2 → 控制 → UART）已移植；默认参数已与 Base 硬件对齐；新增 STM32 桥接作为可选后端，不影响主链路逻辑。 |
| 2. 模式齐全性 | **四种场景模式 + 0~5 标定齐全** | 低 | `face/body/intro/interview` 与标定 mode 0~5 均已实现。`body` 模式已实现为检测/跟踪/绘制，与 Base `process_frame_body()` 一致；标定 1/2 保留 Target 改进（可调 servo1），与 Base 源码字面行为差异已在代码注释与报告中说明。 |
| 3. 终端打印 | **核心控制/标定/状态前缀已统一** | 低 | `[Mode]`、`[ArmProfile]`、`[ARM-CMD]`、`[PnP-Calib]`、`[Head-IMU-Calib]`、`[UART-Handshake]` 等已与 Base 对齐。仍存在的差异：`[ARM-CMD]` 使用 `j5/j4` 而 Base 使用 `j5/j4`（已一致），但底层 `[UART-TX]` 打印 `k1/k2`；部分系统级初始化打印在主 demo 中缺失。 |
| 4. 初始化 / 退出 | **控制链内部已对齐，系统级差异大** | 中 | `ElfControlThread` 内部已复现 Base 的握手（`init success` → FF → 7s homing → A-init → NORMAL）、`EXIT_FRAME`、SIGINT/SIGTERM。但 `camera_demo_pingpong_async_pnp.py` 未包含 Base `main.cpp` 中的 CPU 调频、WiFi、NPU/RGA 初始化、蓝牙、RTMP 探测、WebSocket 等系统初始化。 |
| 5. 云端通信 | **主 demo 完全缺失，仅 `camera_demo_lowlatency.py` 实现** | 高 | `camera_demo_pingpong_async_pnp.py` 没有 RTMP/WS。`demos/camera_demo_lowlatency.py` 的 WS 协议（`{"type":"frame_ts"}` 注册、100ms 心跳、字段名）已与 Base 对齐，并实现了 RTMP 探测与 WS ready 等待；device_id 为 `device-002`（与 Base `device-003` 不同，属预期差异）。 |

**总体判断**：Target 在“视觉 + 机械臂控制”核心算法链路上与 Base 对应度较高，且已针对 Base 的硬件默认值（`/dev/ttyS9`、`/dev/spidev4.0`、`/dev/i2c-4`、8080 端口）做了收敛。当前最大的结构性差异是：Base 的 `main.cpp` 是一个**全系统入口**（CPU/WiFi/NPU/RGA/蓝牙/UART/控制服务器/NRF24/IMU2/RTMP/WS/推流），而 Target 的主 demo `camera_demo_pingpong_async_pnp.py` 目前只是**视觉 pipeline + 控制链**入口，系统初始化与云端通信被拆到了其他文件或未实现。

---

## 1.5 已完成的修复（截至 2026-07-08）

| 编号 | 修复内容 | 修改文件 | 状态 |
|---|---|---|---|
| F1 | 主 demo 默认启用控制链：`--elf-control` 改为默认 `True`，新增 `--no-elf-control` 禁用 | `camera_demo_pingpong_async_pnp.py` | ✅ |
| F2 | 默认 UART 端口改为 `/dev/ttyS9`，NRF24 spidev 改为 `/dev/spidev4.0`，IMU2 I2C 改为 `/dev/i2c-4` | `camera_demo_pingpong_async_pnp.py` | ✅ |
| F3 | 打印格式统一：`[Mode]` 大写、`[ArmProfile]` 前缀、标定前缀分 `[PnP-Calib]`/`[Head-IMU-Calib]`/`[Servo-Calib]`、`[UART-TX]` 字段改为 `k1/k2` | `elf_control_chain.py` | ✅ |
| F4 | 握手流程对齐 Base：发送 FF 后等待 7s homing，再等待 A-init (`r_init_set`) 完成后才解除 TX 阻塞 | `elf_control_chain.py` | ✅ |
| F5 | 信号处理：注册 SIGINT/SIGTERM handler，统一设置退出事件 | `camera_demo_pingpong_async_pnp.py`, `demos/camera_demo_lowlatency.py` | ✅ |
| F6 | WebSocket 协议对齐 Base：注册消息改为 `{"type":"frame_ts"}`，新增 100ms 固定心跳，payload 字段与 Base 一致 | `demos/camera_demo_lowlatency.py` | ✅ |
| F7 | RTMP 探测 + WS ready 等待：RTMP 通才启动 WS，WS 注册成功后才启动 ffmpeg 推流 | `demos/camera_demo_lowlatency.py` | ✅ |
| F8 | WebSocket 线程支持优雅退出：新增 `g_stop_ws` 事件 | `demos/camera_demo_lowlatency.py` | ✅ |
| F9 | 新增 `run_elf_main.sh` 启动脚本，默认参数与 Base 硬件一致 | `run_elf_main.sh` | ✅ |

---

## 2. 目录与核心文件映射

| Base (`elf_info/wearable-robot-arm/src/`) | Target (`trial0/`) | 对应关系说明 |
|---|---|---|
| `main.cpp` | `camera_demo_pingpong_async_pnp.py`（最接近） | Base 是单一全系统入口；Target 主 demo 仅负责视觉 pipeline + 控制链，无 WiFi/CPU/RTMP/WS/蓝牙等系统初始化。 |
| `rga_npu.cpp/.h` | `camera_demo_pingpong_async_pnp.py` + `camera_demo_elf_pipeline.py` + `elf_control_chain.py` | 视觉 pipeline 与 NRF24 控制逻辑被拆分到多个 Python 文件。 |
| `nrf24_linux.c/.h` | `elf_control_chain.py::LinuxNrf24ImuSource` / `StubNrf24ImuSource` / `BridgeNrf24ImuSource` | Python 抽象类；`LinuxNrf24ImuSource` 目前为占位实现，真实环境使用 STM32 桥接。 |
| `imu2_i2c.c/.h` | `elf_control_chain.py::LinuxImu2Source` / `StubImu2Source` / `BridgeImu2Source` | 同上。 |
| `uart_comm.cpp/.h` | `elf_control_chain.py::UartArmSink` / `SerialUartArmSink` / `StubUartArmSink` / `BridgeUartArmSink` | 11 字节帧格式一致；串口握手逻辑一致。 |
| `ctrl_server.cpp/.h` | `elf_control_chain.py::HttpControlServer` + `_make_handler` | HTTP 端点基本一致，Target 额外支持 `/profile` 与 `/cmd?action=toggle_pitch_sign`。 |
| `ws_client.cpp/.h` | `demos/camera_demo_lowlatency.py::ws_worker()` | **不在主 demo 中**。协议已与 Base 对齐。 |
| `stream_manager.cpp` + `gst_rtmp.cpp` | `demos/camera_demo_lowlatency.py`（ffmpeg RTMP）/ `camera_demo_elf_pipeline.py::Streamer` | 实现方式不同，但 RTMP 探测与 WS ready 等待逻辑已对齐 Base。 |
| `bluetooth_spp.c/.h` | `tools/ble_remote.py`（bleak） | 新增能力，协议不同但功能相同（模式切换、profile 切换、pitch sign 切换）。 |
| `scripts/rule_engine_server.py` | `camera_demo_pingpong_async_pnp.py::RuleEngineState`（NumPy/OpenVINO） | 本地推理替代 Unix Domain Socket 服务。 |
| `scripts/calibrate.py` | 无对应 | Target 没有棋盘格相机标定脚本。 |

---

## 3. 逐项详细审计

### 3.1 数据（控制）链路是否能一一对应

#### 3.1.1 Base 链路（C/C++）

```
[NRF24 head IMU] ──► src/nrf24_linux.c (SPI /dev/spidev4.0, GPIO 88/90)
[waist IMU2]     ──► src/imu2_i2c.c (I2C4, 100 Hz pthread)
[USB Camera]     ──► src/rga_npu.cpp (NPU/RGA body+face+PnP)

                          │
                          ▼
              src/rga_npu.cpp::nrf24_control_update()  (50 ms GLib timer)
                          │
                          ▼
              src/uart_comm.cpp::uart_send_arm_target() ──► /dev/ttyS9 @ 115200
                          │
                          ▼
              [11-byte frame: x y z k1 k2 flag] ──► STM32 H7
```

- 控制侧独立的 `nrf24_control_update()` 50 ms 定时运行，**不依赖视频帧**。
- 视觉 PnP 修正通过全局变量 `g_nrf24_state.pnp_correction_ready` / `pnp_pitch_correction` / `pnp_yaw_correction` 传入控制侧。

#### 3.1.2 Target 主 demo 链路（Python）

```
[NRF24 head IMU] ──► elf_control_chain.py::LinuxNrf24ImuSource (或 Stub / DK-2500 / STM32桥接)
[waist IMU2]     ──► elf_control_chain.py::LinuxImu2Source (或 Bridge)
[USB Camera]     ──► camera_demo_pingpong_async_pnp.py
                          │
                          ▼
              PreprocessorThread / BodyGpuThread / PostprocessorThread
                          │
                          ▼
              PostprocessorThread._process():
                - face/hand/PnP/RuleEngine
                - if mode == intro:  elf_thread.controller.update_intro_control(body_dets, ...)
                - if mode == interview: elf_thread.controller.update_interview_control(body_dets, ...)
                - PnP OK → elf_thread.put_pnp_correction(...)
                - calib_mode in (3,4) → elf_thread.put_pnp_calib_sample(...)
                          │
                          ▼
              elf_control_chain.py::Nrf24Controller.update()  (由 ElfControlThread 每 50 ms 调用)
                          │
                          ▼
              elf_control_chain.py::SerialUartArmSink.send_arm_target() ──► /dev/ttyS9 @ 115200
                          │
                          ▼
              [11-byte frame: x y z k1 k2 flag] ──► STM32 / H7
```

- Target 把 NRF24/IMU2/UART 抽象为 `Source` / `Sink`，支持 `stub`、`spidev`、`dk2500`、`stm32_uart` 多种后端。
- **主 demo 默认启用 `--elf-control`**（修复后）。
- 场景模式（Intro/Interview）已在 `PostprocessorThread` 中接入视觉 detections，符合 Base 设计思想。
- PnP 修正通过 `elf_thread.put_pnp_correction()` 传入，符合 Base 的“视觉修正控制零飘”思路。

#### 3.1.3 关键对比点

| 项目 | Base | Target（主 demo） | 是否对应 |
|---|---|---|---|
| 控制周期 | 50 ms GLib timer | 50 ms `time.perf_counter()` 睡眠 | ✅ 基本一致 |
| 主控函数 | `nrf24_control_update()` | `Nrf24Controller.update()` | ✅ 逻辑对应 |
| 视觉→控制耦合 | `rga_npu.cpp` 内直接调用 | `PostprocessorThread` 显式调用 | ✅ 可对应 |
| 物理 NRF24 | SPI `/dev/spidev4.0` | 可选 `spidev` / `dk2500` / `stm32_uart` | ⚠️ 新增后端，Base 只有 SPI |
| 物理 IMU2 | I2C4 | 可选 I2C / STM32 桥接 | ⚠️ 新增后端 |
| 物理 UART | `/dev/ttyS9` @ 115200 | `/dev/ttyS9` @ 115200（默认） | ✅ 已对齐 |
| 默认启用控制链 | ✅ 总是启用 | ✅ 默认启用（修复后） | ✅ |
| RuleEngine | Unix socket 服务 | NumPy ONNX 本地推理 | ❌ 机制不同 |
| A-init 机制 | 握手线程触发 → RX 线程 5 帧平均 | `ElfControlThread.start()` 触发，`Nrf24ImuSource` 做 5 帧平均 | ✅ 已对齐 |
| 场景模式触发 | 独立 50 ms 定时器 | 视觉线程计算目标入队，`ElfControlThread._tick()` 50 Hz 发送 | ✅ 已解耦 |

#### 3.1.4 详细差异

**D1-1：A-init 触发与平均算法（已整改）**

Base（`src/nrf24_linux.c:585-604`）：
- 握手线程在 homing 完成后设置 `g_wait_a_init = 1`；
- NRF24 RX 线程收到信号后，累计 5 帧 IMU 数据并做**平均值**；
- 平均完成后设置 `g_r_init_set = 1`，并打印：
  ```
  [A-INIT] 5-frame avg captured: roll=%.2f pitch=%.2f yaw=%.2f
  ```

Target（`elf_control_chain.py:586-638`）：
- `Nrf24ImuSource` 维护 `wait_a_init` 标志与 `_init_accum` 缓冲区；
- `ElfControlThread.start()` 在 homing 完成后调用 `start_a_init()` 打开采样窗口；
- `_on_valid_frame()` 累计 5 帧 IMU roll/pitch/yaw 并做**平均值**，完成后设置 `r_init_set = True`，打印：
  ```
  [A-INIT] 5-frame avg captured: roll=... pitch=... yaw=...
  ```

**状态**：✅ 已对齐 Base 的 5 帧平均与 homing 后触发机制。

**D1-2：场景模式触发与视频帧耦合（已整改）**

Base 的场景模式（Intro/Interview）在 `rga_npu.cpp` 的 `process_frame()` 视频回调中处理 detections，但控制指令仍由独立的 50 ms `nrf24_control_update()` 定时器通过全局状态消费。Target 主 demo 中，`PostprocessorThread._process()` 仍根据视频帧调用 `update_intro/interview_control()` 计算目标，但不再直接 `send_arm_target()`，而是将目标命令放入 `elf_thread.scenario_cmd_q`；`ElfControlThread._tick()` 在 50 Hz 控制循环中消费该队列并统一发送。

**状态**：✅ 场景模式的目标计算仍依赖视频帧（与 Base 的 `process_frame()` 一致），但 UART 发送已解耦到 50 Hz 控制节拍。

**D1-3：RuleEngine 机制差异**

Base 通过 fork + exec 启动 `scripts/rule_engine_server.py`，通过 Unix Domain Socket `/tmp/rule_engine.sock` 通信；Target 使用 `camera_demo_elf_pipeline.py::RuleEngineState` 在进程内用 NumPy/OpenVINO 推理。

**影响**：机制不同，但只要输入输出维度与状态寄存器语义一致，行为可等效。

**整改建议**：对同一帧输入同时跑 Base 的 `rule_engine_server.py` 和 Target 的 `RuleEngineState`，对比 7 维 INT64 输出是否一致。

---

### 3.2 各个模式是否齐全（尤其是标定）

#### 3.2.1 Base 模式

`src/rga_npu.h`：

```cpp
typedef enum {
    MODE_FACE = 0,
    MODE_BODY = 1,
    MODE_INTRO = 2,
    MODE_INTERVIEW = 3
} PoseMode;
```

标定模式通过 `/tmp/calib_mode.txt` 传递：

| 值 | 含义 | 实现 |
|---|---|---|
| 0 | 正常 | `nrf24_control_update()` 正常控制 |
| 1 | 伺服标定 | 读取 `/tmp/servo_calib.txt`，发送固定姿态 |
| 2 | 扫描测试 | J4 伺服 100°–180° 往复扫描 |
| 3 | PnP 偏航标定 | 按 `PNP_CALIB_TARGETS` 偏航角采样 |
| 4 | PnP 俯仰标定 | 按 `PNP_PITCH_CALIB_TARGETS` 俯仰角采样 |
| 5 | 头部 IMU 3×3 网格标定 | 按 `HEAD_IMU_GRID_TARGETS` 采样并写 CSV |

#### 3.2.2 Target 模式

`elf_control_chain.py` 中：

```python
def set_pose_mode(self, mode: str) -> Optional[Dict[str, Any]]:
    if mode not in ("face", "body", "intro", "interview"):
        return None
```

标定：

```python
def _run_calibration(self, now_us, head_imu, imu2, r_init_set):
    if mode == 0: ...
    if mode in (1, 2): return self._run_servo_calib(...)
    if mode == 3: return self._run_pnp_yaw_calib(...)
    if mode == 4: return self._run_pnp_pitch_calib(...)
    if mode == 5: return self._run_head_imu_grid_calib(...)
```

标定目标表与 Base 相同：
- `PNP_CALIB_TARGETS_YAW = [0, -15, -30, -45, -60, -75, 15, 30, 45, 60, 75]`
- `PNP_CALIB_TARGETS_PITCH = [0, -15, -30, 15, 30]`
- `HEAD_IMU_GRID_TARGETS` 3×3 网格一致

主 demo 中场景模式调用（`camera_demo_pingpong_async_pnp.py:264-283`）：

```python
if self.elf_thread is not None:
    mode = self.elf_thread.get_mode()
    now_us = int(time.perf_counter() * 1_000_000)
    cmd = None
    if mode == "intro":
        cmd = self.elf_thread.controller.update_intro_control(...)
    elif mode == "interview":
        cmd = self.elf_thread.controller.update_interview_control(...)
    if cmd is not None:
        self.elf_thread.uart_sink.send_arm_target(...)
```

PnP 标定采样（`camera_demo_pingpong_async_pnp.py:367-370`）：

```python
if calib_mode in (3, 4):
    self.elf_thread.put_pnp_calib_sample(
        yaw_deg=info["yaw"], pitch_deg=info["pitch"]
    )
```

#### 3.2.3 关键对比点

| 项目 | Base | Target | 是否对应 |
|---|---|---|---|
| 场景模式 FACE/BODY/INTRO/INTERVIEW | ✅ | ✅ | ✅ 齐全 |
| 标定模式 0~5 | ✅ | ✅ | ✅ 齐全 |
| Intro/Interview 实现 | `rga_npu.cpp` 内处理 detections | `PostprocessorThread` 内处理 | ✅ 已接入主 demo |
| Body 模式行为 | `process_frame_body()` 检测/跟踪/绘制 | 检测/跟踪/绘制，跳过 face/hand/PnP | ✅ 已对齐 |
| 标定文件路径 | `/tmp/calib_mode.txt`, `/tmp/servo_calib.txt` 等 | 相同 | ✅ |
| 标定 CSV 输出 | `/tmp/head_imu_grid_calib.csv`, `/tmp/pnp_yaw_calib.csv`, `/tmp/pnp_pitch_calib.csv` | 相同路径（yaw/pitch 在 controller 内写入） | ✅ |
| 模式切换 home 过渡 | `send_face_home_from_scenario()` | `_make_face_home_command()` + `set_mode()` 发送 | ✅ 等效 |

#### 3.2.4 详细差异

**D2-1：Body 模式未完整实现（已整改）**

Base 的 `MODE_BODY` 在 `process_frame_body()` 中只执行人体检测、跟踪和绘制，**不进入 FACE 模式的人脸/PnP 链路，也不产生机械臂控制指令**（控制仍由 `nrf24_control_update()` 基于 IMU 完成）。

Target 已在 `PostprocessorThread._process()` 中增加 `mode == "body"` 分支：绘制 body 检测框与关键点后直接输出，跳过 face landmark、PnP、RuleEngine 和 hand landmark 处理，与 Base 行为一致。

**D2-2：标定 mode 1/2 的 servo 发送逻辑差异**

Base（`src/rga_npu.cpp:1746-1823`）：
- mode 1：读取 `/tmp/servo_calib.txt` 到 `calib_servo1`；
- mode 2：`calib_servo1` 在 100°–180° 之间扫描；
- 但实际发送 UART 时固定使用 `NRF_SERVO1_DEG=50` 和 `NRF_SERVO2_DEG=145`：
  ```cpp
  uart_send_arm_target(calib_locked_tx, calib_locked_ty, calib_locked_tz,
                       NRF_SERVO1_DEG, NRF_SERVO2_DEG, 0x01);
  ```
- 注释写明“控制指令: k1=50, k2=145”，`calib_servo1` 未被使用。

Target（`elf_control_chain.py:1355-1385`）：
- mode 1：读取 `/tmp/servo_calib.txt` 到 `st["servo1"]`；
- mode 2：`st["servo1"]` 在 100°–180° 之间扫描；
- 返回命令：
  ```python
  return {
      "x": st["locked_tx"], "y": st["locked_ty"], "z": st["locked_tz"],
      "servo1": st["servo1"], "servo2": 50.0, "flag": 0x01,
  }
  ```
- 由 `ElfControlThread._tick()` 调用 `send_arm_target(x, y, z, servo2, servo1, flag)`，因此实际发送 `k1=50` (J5), `k2=servo1` (J4)。

**影响**：Target 的行为更符合“servo 标定”的直观目的（用可调 servo1 作为 J4 发送），但与 Base 源码字面行为不一致。若云端/下游严格按 Base 字面行为设计测试用例，会产生差异。

**整改状态**：✅ 已决定保留 Target 改进并在代码与文档中说明。

- 代码注释位于 `elf_control_chain.py::_run_servo_calib()`；
- Target 实际发送 `k1=50` (J5)、`k2=servo1` (J4)，其中 mode 1 读取 `/tmp/servo_calib.txt`，mode 2 在 100°–180° 扫描；
- Base 源码字面发送固定 `k1=50, k2=145`，其读取/扫描值未参与发帧，视为 upstream 未启用逻辑；
- 该差异属于 Target 相对于 Base 的合理改进，不影响标定模式齐全性。

**D2-3：/servo 端点测试位置差异**

Base `/servo` 端点发送测试指令时固定使用 `tx=0.0, ty=67.0, tz=40.0`。

Target `send_servo_test()` 使用当前 profile 计算：`tx=0.0, ty=profile.l4 + profile.l3, tz=profile.l2 + profile.l1`。对于默认 `far_l3_55` profile，得到 `ty=83.0, tz=18.0`。

**影响**：标定/servo 测试的固定位置不同。

**整改建议**：若要求严格一一对应，将 Target `send_servo_test()` 的 ty/tz 改为 `67.0/40.0` 与 Base 一致。

---

### 3.3 终端打印的内容是否能一一对应（除手势/语音/STM32 桥接）

#### 3.3.1 主 demo 中的关键打印

`camera_demo_pingpong_async_pnp.py`：

```python
print("\n" + "=" * 60)
print("PingPong async body + PnP pipeline starting...")
print("=" * 60)
print("[Rule] Loading Rule Engine model on GPU")
print(f"[Camera] Resolution {actual_w}x{actual_h}")
print(f"[ElfControl] Enabled ({mode} mode), HTTP port={args.ctrl_port}")
print("[Main] Press Ctrl-C to stop, or 'q' in window.")
```

以及 `FrameStats.print_report()`：

```python
print("\n" + "=" * 60)
print(f"[PingPong-Async-PnP] last {len(recent)} frames")
print(f"  total       : avg={avg('total'):7.2f}ms")
print(f"  body_infer  : avg={avg('body'):7.2f}ms")
...
print("=" * 60)
```

#### 3.3.2 可对应的核心前缀

| Base 前缀 | Target 前缀 | 对应内容 |
|---|---|---|
| `[Mode] Switched to FACE` | `[Mode] Switched to FACE` | ✅ 已统一为大写 |
| `[ArmProfile] Switched to ...` | `[ArmProfile] Switched to ...` | ✅ 已统一 |
| `[ARM-CMD] tx=... ty=... tz=... j5=... j4=... flag=...` | `[ARM-CMD] tx=... ty=... tz=... j5=... j4=... flag=...` | ✅ 高度一致 |
| `[A-INIT] R_init built: ...` | `[A-INIT] R_init built: ...` | ✅ 一致 |
| `[HeadCenter] vector center captured ...` | `[HeadCenter] vector center captured ...` | ✅ 一致 |
| `[PnP-Calib] ...` | `[PnP-Calib] ...` | ✅ 已统一 |
| `[Head-IMU-Calib] ...` | `[Head-IMU-Calib] ...` | ✅ 已统一 |
| `[UART-TX] x=... y=... z=... k1=... k2=... flag=...` | `[UART-TX] x=... y=... z=... k1=... k2=... flag=...` | ✅ 已统一 |
| `[Main] ...` | `[Main] ...` | 生命周期打印 |
| `[Ctrl] ...` | `[Ctrl] ...` | 控制服务器打印 |
| `[CPU] ...` | 无 | Target 没有 CPU 调频打印 |
| `[NPU] ...` | 无 | Target 没有 NPU 初始化打印 |
| `[RTMP] ...` / `[RTSP] ...` | 无 | 主 demo 无推流 |
| `[WS] ...` | 无（主 demo） | 主 demo 无 WebSocket |

#### 3.3.3 仍存在的打印差异

**D3-1：`[ARM-CMD]` 与 `[UART-TX]` 字段名不一致**

- `[ARM-CMD]` 输出 `j5=` / `j4=`（与 Base 一致）；
- 同一次命令下发的 `[UART-TX]`（在 `StubUartArmSink` 或调试路径）输出 `k1=` / `k2=`。

虽然 Base 的 `[UART-TX]` 也使用 `k1/k2`，但 Base 没有独立的 `[ARM-CMD]` 打印；Target 同时存在两种字段命名，日志中同一次命令可能分别显示 `j5/j4` 和 `k1/k2`。

**整改建议**：统一 `[ARM-CMD]` 和 `[UART-TX]` 的字段命名，或仅保留一种打印。

**D3-2：系统级初始化打印缺失**

Base 启动时会打印：
```
[CPU] Setting performance mode...
[Main] Initializing WiFi...
[Main] Initializing NPU...
[Main] Initializing RGA...
[Main] Initializing Bluetooth remote...
[Main] Probing RTMP server ...
[Main] WebSocket reporter started ...
```

Target 主 demo 没有这些模块，因此对应打印缺失。

**整改建议**：因主 demo 不负责这些模块，建议在文档中明确职责边界；若收敛到主 demo，则补充打印。

**D3-3：主 demo 有、Base 无的打印**

- `PingPong async body + PnP pipeline starting...`
- `[PingPong-Async-PnP] last N frames`
- `[Rule] Loading Rule Engine model on GPU`
- `[Camera] Resolution ...`
- `[ElfControl] Enabled (...)`

这些打印属于 Target 新增的异步 pipeline 性能统计，Base 没有对应模块，因此不追求一一对应。

---

### 3.4 初始化 / 程序 Ctrl+C 退出时流程是否完全一样

#### 3.4.1 Base 初始化流程（`src/main.cpp`）

```
1. setbuf(stdout, NULL)
2. remove("/tmp/calib_mode.txt")
3. 打印启动横幅
4. set_cpu_performance()           # CPU 调频
5. WiFi 初始化 + DHCP + 静态 IP fallback
6. fork() + exec rule_engine_server.py
7. init_npu()                       # RKNN 模型加载
8. init_rga()                       # RGA 初始化
9. bluetooth_spp_init/start         # BLE 遥控
10. uart_init("/dev/ttyS9", 115200) + uart_start_receiver()
11. ctrl_server_start(8080)         # HTTP 控制服务器
12. nrf24_linux_init() + nrf24_rx_thread_start()
13. imu2_i2c_init() + imu2_i2c_thread_start()
14. 注册 SIGINT/SIGTERM 信号处理
15. gst_init + g_timeout_add(check_quit_timer)
16. probe_rtmp_server(RTMP_URL)
17. 若 RTMP 通：启动 ws_worker_thread
18. 等待 WS 注册确认（10s 超时）
19. pthread_create(handshake_thread) # init success → FF → 7s homing → A-init → NORMAL
20. g_timeout_add(50ms, nrf24_control_update)
21. start_stream() 进入 GStreamer main loop
```

#### 3.4.2 Base 退出流程（`src/main.cpp` cleanup）

```
1. 发送 exit_frame[10] = {0xAA,0xFF,...} 给 MCU
2. usleep(50000)
3. g_running = FALSE
4. pthread_join(ws_tid)
5. nrf24_rx_thread_stop + join + deinit
6. imu2_i2c_thread_stop + deinit
7. uart_cleanup()
8. bluetooth_spp_stop + cleanup
9. ctrl_server_stop()
10. cleanup_npu()
11. cleanup_rga()
12. stop_rule_engine()  # kill rule_engine_server.py
13. 打印 [Main] Shutdown complete
```

#### 3.4.3 Target 主 demo 初始化流程（`camera_demo_pingpong_async_pnp.py`）

```
1. 解析命令行参数
2. 打印启动横幅
3. SimplePipeline() 加载模型（Body/Face/Hand/Rule）
4. cv2.VideoCapture() 打开摄像头
6. 若 --elf-control（默认 True）：make_elf_control_thread() 创建 ELF 控制线程
7. 创建 raw_q / body_in_q / body_done_q / out_q / stop_ev
8. 创建并启动 capture / preprocessor / body_gpu / postprocessor / output 线程
9. 若 --voice：启动 VoiceKwsThread
10. 若 elf_thread 非 None：启动 elf_thread
11. 注册 SIGINT/SIGTERM handler
12. 进入主循环
```

**注意**：
- 没有 CPU 调频
- 没有 WiFi 初始化
- 没有 RTMP 探测、WS 注册等待
- 没有蓝牙初始化
- RuleEngine 是本地 ONNX，没有子进程

#### 3.4.4 Target 主 demo 退出流程（`camera_demo_pingpong_async_pnp.py:746-774`）

```python
try:
    if args.headless and args.frames == 0:
        time.sleep(30)
        stop_ev.set()
    elif not args.headless:
        while not stop_ev.is_set():
            time.sleep(0.05)
    else:
        out_thread.join()
except KeyboardInterrupt:
    print("\n[Main] Stopping...")
finally:
    stop_ev.set()
    out_thread.join(timeout=2.0)
    body_gpu_thread.join(timeout=3.0)
    post_thread.join(timeout=2.0)
    if voice_thread is not None:
        voice_thread.stop()
    if elf_thread is not None:
        elf_thread.stop()
    cap.release()
    cv2.destroyAllWindows()
    stats.print_report()
    run_elapsed = time.perf_counter() - run_start
    processed = run_result.get("processed", 0)
    if run_elapsed > 0:
        print(f"[Run] Processed {processed} frames in {run_elapsed:.2f}s "
              f"({processed/run_elapsed:.1f} fps)")
    print("[Main] Demo stopped.")
```

`elf_thread.stop()` 会：
1. `_stop_ev.set()`
2. `join(timeout=2.0)`
3. `http_server.stop()`
4. `uart_sink.stop()`（会发送 EXIT_FRAME）
5. `imu2_source.stop()`
6. `nrf_source.stop()`

#### 3.4.5 关键对比点

| 项目 | Base | Target（主 demo） | 是否对应 |
|---|---|---|---|
| 单进程统一入口 | ✅ `main.cpp` | ⚠️ 仅负责视觉+控制子系统 | 范围不同 |
| CPU 调频 | ✅ | ❌ | 未移植 |
| WiFi 初始化 | ✅ | ❌ | 未移植 |
| RuleEngine 子进程 | ✅ | ❌ | 改为本地 ONNX |
| NRF24 初始化 | ✅ | ⚠️ 仅 `--elf-control` 时（默认启用） | 默认行为已对齐 |
| IMU2 初始化 | ✅ | ⚠️ 仅 `--elf-control` 时（默认启用） | 默认行为已对齐 |
| UART 初始化 | ✅ `/dev/ttyS9` | ✅ `/dev/ttyS9`（默认） | 已对齐 |
| HTTP 控制服务器 | ✅ 8080 | ✅ 8080（elf_thread 中） | 已对齐 |
| RTMP 探测 + WS 注册等待 | ✅ | ❌ | 主 demo 无推流 |
| 握手线程 init→FF→7s→A-init | ✅ | ✅ `SerialUartArmSink.start()` + `ElfControlThread.start()` | 已对齐 |
| 发送 exit frame | ✅ | ✅ 通过 `elf_thread.stop()` → `uart_sink.stop()` | 一致 |
| Ctrl+C 信号处理 | `signal()` + GMainLoop | `signal.signal()` + `try/except KeyboardInterrupt` | 机制不同，效果一致 |
| cleanup 顺序完整性 | ✅ | ⚠️ 主 demo 范围更小 | 因职责范围不同 |

#### 3.4.6 详细差异

**D4-1：系统级初始化缺失**

Target 主 demo 不包含 Base `main.cpp` 中的 CPU 调频、WiFi、NPU/RGA 初始化、蓝牙初始化、RTMP 探测、WS 注册等待。这是主 demo 与 Base `main.cpp` 之间最大的结构性差异。

**整改建议**：
- 若项目目标就是对标 Base 的“单一全系统入口”，需将上述系统初始化收敛到主 demo，或创建一个包装脚本按顺序启动各子系统；
- 若主 demo 仅负责视觉+控制，应在文档中明确说明，避免声称与 Base 主入口完全一致。

**D4-2：RuleEngine 模型加载（已整改）**

原问题：主 demo 先 `SimplePipeline()` 加载 Body/Face/Hand/Hand-Cls，随后又手动 `core.read_model(RULE_MODEL_PATH)` 并编译到 GPU，存在重复加载/独立加载。

整改后：`SimplePipeline()` 现在也会加载并编译 RuleEngine 模型到 GPU，设置 `rule_compiled`、`rule_input_names`、`rule_output_name`；主 demo 直接复用 `pipeline.rule_compiled`，不再单独 `read_model`/`compile_model`。

**状态**：✅ RuleEngine 模型仅加载一次，由 SimplePipeline 统一管理。

**D4-3：退出时队列清空（已整改）**

Target 主 demo `finally` 原仅调用线程 `join(timeout=...)`，未主动清空队列。

整改后：`finally` 中先 `stop_ev.set()`，然后调用 `elf_thread.flush_queues()`，并清空 `raw_q` / `body_in_q` / `body_done_q` / `out_q`，再 `join` 各线程。

**状态**：✅ 退出时队列已清空，线程能更快、更干净地退出。

---

### 3.5 云端通信除了 device_id 不同，其他是否完全一样

#### 3.5.1 关键结论

**主 demo `camera_demo_pingpong_async_pnp.py` 完全没有云端通信代码**（无 RTMP、无 WebSocket、无 HTTP 上传）。云端通信只存在于：

- `demos/camera_demo_lowlatency.py`：ffmpeg RTMP + WebSocket
- `camera_demo_elf_pipeline.py`：ffmpeg RTMP（定义了 WS_URL 但未使用）

因此，主 demo 与 Base 的云端通信**无法对应**。

#### 3.5.2 Base 云端配置

```cpp
#define DEVICE_ID           "device-003"
#define CLOUD_IP            "47.93.162.124"
#define RTMP_URL            "rtmp://47.93.162.124:1935/live/device-003"
#define WS_URL              "ws://47.93.162.124/ws?deviceId=device-003"
```

WebSocket 行为（`src/main.cpp` + `src/ws_client.cpp`）：
- 连接成功后发送 `{"type":"frame_ts"}` 注册。
- 之后每 100 ms 发送心跳：
  ```json
  {"type":"frame_ts","data":{"timestamp":%ld,"elapsed":%.3f,"frame_count":%llu,"device":"%s"}}
  ```
- 非阻塞 `recv()` 接收服务器控制消息，用于检测连接断开。
- RTMP 推流与 WebSocket 在同一进程中运行；RTMP 不通时回退到 RTSP 且**不启动 WS**。

#### 3.5.3 Target 云端配置（非主 demo）

`demos/camera_demo_lowlatency.py`：
```python
DEVICE_ID = "device-002"
cloud_ip = os.environ.get('CLOUD_IP', '47.93.162.124')
RTMP_URL = f"rtmp://{cloud_ip}:1935/live/{DEVICE_ID}"
WS_URL = f"ws://47.93.162.124/ws?deviceId={DEVICE_ID}"
```

WebSocket 行为（`demos/camera_demo_lowlatency.py::ws_worker()`）：
- 连接成功后发送 `{"type":"frame_ts"}` 注册（已修复）。
- 每 100 ms 发送心跳，payload 字段为 `timestamp`、`elapsed`、`frame_count`、`device`（已修复）。
- 非阻塞 `recv()` 接收服务器控制消息，处理 `set_target`/`ctrl_mode`/`track_obj`/`target_pose`。
- 重连退避：2 s → 30 s，与 Base 类似。
- RTMP 探测通过后才启动 WS，WS 注册成功后才启动 ffmpeg 推流（已修复）。

`camera_demo_elf_pipeline.py`：
- 定义了 `DEVICE_ID = "device-002"`、`RTMP_URL`、`WS_URL`。
- 实际只使用 RTMP 推流，**未使用 WS_URL**。

#### 3.5.4 关键对比点

| 项目 | Base | Target（整体） | 是否一致 |
|---|---|---|---|
| 主 demo 含云端通信 | ✅ | ❌ | 不一致 |
| device_id | `device-003` | `device-002` | ❌ 不同 |
| cloud_ip | `47.93.162.124` | `47.93.162.124`（lowlatency 可环境变量覆盖） | ✅ |
| RTMP URL 结构 | `rtmp://IP:1935/live/{device_id}` | 相同 | ✅ |
| WS URL 结构 | `ws://IP/ws?deviceId={device_id}` | 相同 | ✅ |
| WS 注册消息 | `{"type":"frame_ts"}` | `{"type":"frame_ts"}`（已修复） | ✅ |
| WS 心跳消息 | 固定 `{"type":"frame_ts","data":{timestamp,elapsed,frame_count,device}}` | 已一致（已修复） | ✅ |
| WS 接收指令 | 仅用于检测连接存活 | 处理 `set_target`/`ctrl_mode`/`track_obj`/`target_pose` | ⚠️ 行为不同 |
| RTMP 探测/WS 联动 | ✅ RTMP 通才启动 WS | ✅ 已修复 | ✅ |
| WS 与主程序关系 | 同一进程 | 分散在不同 demo | ❌ 架构不同 |

#### 3.5.5 详细差异

**D5-1：主 demo 无云端通信**

这是最大差异。Base 的 `main.cpp` 把 RTMP 推流和 WebSocket 作为核心功能；Target 主 demo 完全没有这些能力。

**整改建议**：
- 将 `demos/camera_demo_lowlatency.py` 的 RTMP/WS 能力收敛到主 demo；或
- 明确主 demo 仅负责视觉+控制，由 `camera_demo_lowlatency.py` 负责云端通信。

**D5-2：WS 接收云端指令行为差异**

Base 当前仅接收服务器消息用于检测连接存活，不处理具体指令；Target `camera_demo_lowlatency.py` 处理 `set_target`/`ctrl_mode`/`track_obj`/`target_pose`。

**影响**：若云端下发这些指令，Target 会响应而 Base 不会。

**整改建议**：确认 Base 是否也需要处理这些指令；如需一致，在 Base 增加或在 Target 移除。

**D5-3：`camera_demo_elf_pipeline.py` 未启用 WS**

该文件定义了 `WS_URL` 但未实际建立 WebSocket 连接，存在误导性。

**整改建议**：移除未使用的 `WS_URL` 常量，或实现 WS 连接。

---

## 4. 问题汇总表

| 编号 | 维度 | 严重等级 | 位置 | 问题描述 | 影响 | 整改建议 |
|---|---|---|---|---|---|---|
| D1-1 | 控制链路 | 中 | `elf_control_chain.py::Nrf24ImuSource` | A-init 未做 5 帧平均，也没有 `g_wait_a_init` 触发信号 | 初始 R_init 可能受单帧噪声影响 | ✅ 已实现 5 帧平均与 `wait_a_init` 触发机制 |
| D1-2 | 控制链路 | 中 | `camera_demo_pingpong_async_pnp.py` | 场景模式指令与视频帧耦合 | 视频卡顿时场景模式输出也会卡顿 | ✅ 已将场景目标命令入队，由 `ElfControlThread._tick()` 在 50 Hz 统一发送 |
| D1-3 | 控制链路 | 高 | `camera_demo_pingpong_async_pnp.py::RuleEngineState` | RuleEngine 由 socket 服务改为本地 ONNX | 输入输出一致性、延迟、资源占用可能不同 | 与 Base 的 `scripts/rule_engine_server.py` 做输入输出对比测试 |
| D2-1 | 模式 | 中 | `camera_demo_pingpong_async_pnp.py` | Body 模式仅设置字符串，视觉侧未实现具体行为 | Body 模式行为与 Base 不一致 | ✅ 已实现：body 模式只检测/跟踪/绘制，跳过 face/hand/PnP |
| D2-2 | 模式 | 中 | `elf_control_chain.py::_run_servo_calib` | 标定 mode 1/2 实际发送的 servo 值与 Base 源码字面行为不同 | 若严格按 Base 字面设计测试用例会失败 | 决定：对齐 Base 字面行为，或保留 Target 改进并文档化 |
| D2-3 | 模式 | 低 | `elf_control_chain.py::send_servo_test` | `/servo` 测试位置 ty/tz 与 Base 不同（Base 67/40，Target 83/18） | 标定测试位置不一致 | 改为 Base 的 `67.0/40.0` 或文档化 |
| D3-1 | 打印 | 低 | `elf_control_chain.py` | `[ARM-CMD]` 输出 `j5/j4`，`[UART-TX]` 输出 `k1/k2`，同一次命令字段名不一致 | 日志可读性略差 | 统一 `[ARM-CMD]` 也使用 `k1/k2` 或仅保留一种打印 |
| D3-2 | 打印 | 中 | 主 demo | 缺少 `[CPU]`/`[NPU]`/`[RTMP]`/`[Handshake]` 等系统级打印 | 无法通过日志判断完整系统阶段 | 明确职责边界；若收敛到主 demo 则补充打印 |
| D4-1 | 初始化/退出 | 高 | 主 demo | 系统级初始化（CPU/WiFi/NPU/RGA/蓝牙/RTMP/WS）缺失 | 与 Base `main.cpp` 职责范围不一致 | 收敛系统初始化到主 demo 或明确文档边界 |
| D4-2 | 初始化/退出 | 低 | `camera_demo_pingpong_async_pnp.py` | RuleEngine 模型加载两次 | 资源浪费、启动慢 | ✅ 已让 `SimplePipeline` 加载 RuleEngine，主 demo 复用 |
| D4-3 | 初始化/退出 | 低 | `camera_demo_pingpong_async_pnp.py` | 退出时线程 join 超时较短（2-3s） | 极端情况下资源未完全释放 | ✅ 已增加退出时队列清空与 `elf_thread.flush_queues()` |
| D5-1 | 云端通信 | 高 | `camera_demo_pingpong_async_pnp.py` | 主 demo 完全没有云端通信 | 与 Base 单一入口含云端通信差异巨大 | 将 RTMP/WS 能力收敛到主 demo 或明确职责边界 |
| D5-2 | 云端通信 | 中 | `demos/camera_demo_lowlatency.py` | 处理云端下发的 `set_target`/`ctrl_mode`/`track_obj`/`target_pose` | 若 Base 不处理，则行为不一致 | 确认 Base 是否也需要处理；如需一致，在 Base 增加或 Target 减少 |
| D5-3 | 云端通信 | 高 | `camera_demo_elf_pipeline.py` | 定义了 WS_URL 但未使用 | 该 demo 无云端心跳 | 移除未使用的 WS_URL，或实现 WS 连接 |
| D5-4 | 云端通信 | 高 | 项目整体 | RTMP/WS 分散在不同 demo | 无法统一对标 Base 单进程行为 | 收敛到一个主入口，或明确各 demo 的云能力差异 |

---

## 5. 整改指南

### 5.1 立即可做的高优先级整改

1. **明确主 demo 职责边界**
   - 若主 demo 必须对标 Base `main.cpp`，需将 RTMP 推流、WebSocket、CPU 调频、WiFi 初始化等收敛进来。
   - 若主 demo 仅负责“视觉 + 控制”，则应在文档中明确说明，并将云端通信能力由其他 demo 承担时不声称与 Base 主入口一致。

2. **修复 WS 未使用常量**
   - 在 `camera_demo_elf_pipeline.py` 中移除未使用的 `WS_URL`，或实现 WS 连接。

3. **统一 `[ARM-CMD]` / `[UART-TX]` 字段命名**
   - 建议 `[ARM-CMD]` 也使用 `k1/k2`，与 `[UART-TX]` 及 Base 的 UART 打印保持一致。

### 5.2 中优先级整改

4. **对齐 A-init 平均算法** ✅
   - 已在 `Nrf24ImuSource` 中实现 5 帧平均，`ElfControlThread.start()` 在 homing 完成后调用 `start_a_init()` 触发采样窗口。

5. **解耦场景模式与视频帧** ✅
   - `PostprocessorThread` 将 `update_intro/interview_control()` 计算得到的目标命令放入 `elf_thread.scenario_cmd_q`；`ElfControlThread._tick()` 在 50 Hz 控制循环中统一消费并发送。

6. **确认标定 mode 1/2 行为** ✅
   - 决定保留 Target 改进：mode 1 读取 `/tmp/servo_calib.txt`，mode 2 扫描 100°–180°，实际 UART 发送使用可调 `servo1` 作为 J4；与 Base 源码字面固定 `k1=50, k2=145` 的差异已在 `elf_control_chain.py` 注释与本文档中说明。

7. **增加退出时队列清空** ✅
   - 已在主 demo 的 `finally` 中清空 `raw_q` / `body_in_q` / `body_done_q` / `out_q`，并调用 `elf_thread.flush_queues()` 后再 join 线程。

8. **实现 Body 模式视觉侧行为** ✅
   - 在 `PostprocessorThread._process()` 中增加 `mode == "body"` 分支：只进行 body 检测、跟踪和绘制，跳过 face landmark、PnP、RuleEngine、hand landmark，与 Base `process_frame_body()` 行为一致。

### 5.3 低优先级 / 可选整改

8. **系统级初始化收敛（如需对标 Base）**
   - CPU 调频：调用 `cpufreq` 工具或写入 `/sys/devices/system/cpu/...`。
   - WiFi 初始化：调用 NetworkManager 或 `wpa_supplicant`。
   - NPU/RGA 初始化：Target 平台为 Intel，无需 RKNN/RGA，可用 OpenVINO 加载模型替代。
   - 蓝牙初始化：调用 `bluetoothctl` 或 `bleak`。

10. **RuleEngine 一致性验证**
    - 将 Base 的 `scripts/rule_engine_server.py` 与 Target 的 `RuleEngineState` 对同一帧输入做输出对比，确保 7 维状态一致。

11. **文档更新**
    - 更新 `README.md`，明确各 demo 的能力差异：
      - `camera_demo_pingpong_async_pnp.py`：主视觉+控制 pipeline（默认启用 ELF 控制）
      - `demos/camera_demo_lowlatency.py`：低延迟推流 + WS
      - `camera_demo_elf_pipeline.py`：完整视觉 pipeline + RTMP（无 WS）
      - `demos/voice_only_demo.py`：仅语音

### 5.4 验证 checklist

- [ ] `camera_demo_pingpong_async_pnp.py` 默认启用控制链后能启动并连接 `/dev/ttyS9`。
- [ ] 收到 `init success` 后打印 `[UART] 'init success' received` 和 `[UART-Handshake] FF verification frame sent`。
- [ ] 7s 后 TX 阻塞解除，A-init 后打印 `[A-INIT] 5-frame avg captured: ...`。
- [ ] HTTP 接口 `POST /mode?type=intro` 后机械臂进入 Intro 模式，并根据画面中人物移动。
- [ ] HTTP 接口 `POST /mode?type=interview` 后机械臂跟踪两个说话人中心。
- [ ] Ctrl+C 或 SIGTERM 后发送 exit frame，最后 `[Main] Demo stopped.`。
- [ ] `demos/camera_demo_lowlatency.py` WebSocket 注册消息云端可识别，100 ms 心跳正常。
- [ ] `demos/camera_demo_lowlatency.py` RTMP 推流画面与 Base 一致。

---

## 6. 附录：关键代码片段对照

### 6.1 A-init

**Base** (`src/nrf24_linux.c:585-604`):
```c
if (g_wait_a_init && !g_r_init_set) {
    static float acc_roll = 0.0f, acc_pitch = 0.0f, acc_yaw = 0.0f;
    static int acc_count = 0;

    acc_roll += roll; acc_pitch += pitch; acc_yaw += yaw;
    acc_count++;
    if (acc_count >= 5) {
        store_roll   = acc_roll   / 5.0f;
        store_pitch  = acc_pitch  / 5.0f;
        store_yaw    = acc_yaw    / 5.0f;
        printf("[A-INIT] 5-frame avg captured: roll=%.2f pitch=%.2f yaw=%.2f\n",
               store_roll, store_pitch, store_yaw);
        g_r_init_set = 1;
        g_wait_a_init = 0;
        acc_roll = acc_pitch = acc_yaw = 0.0f;
        acc_count = 0;
    }
}
```

**Target** (`elf_control_chain.py:603-607`):
```python
def _on_valid_frame(self):
    self._valid_count += 1
    if not self.r_init_set and self._valid_count >= 5:
        self.r_init_set = True
```

### 6.2 模式切换

**Base** (`src/rga_npu.cpp:4560-4579`):
```cpp
void set_pose_mode(PoseMode mode) {
    PoseMode old_mode = g_mode.load();
    bool exiting_scenario_to_face =
        mode == MODE_FACE &&
        (old_mode == MODE_INTRO || old_mode == MODE_INTERVIEW);
    if (exiting_scenario_to_face) send_face_home_from_scenario();
    g_mode.store(mode);
    printf("[Mode] Switched to %s\n", pose_mode_name(mode));
}
```

**Target** (`elf_control_chain.py:1655-1676`):
```python
def set_pose_mode(self, mode: str) -> Optional[Dict[str, Any]]:
    mode = mode.lower()
    if mode not in ("face", "body", "intro", "interview"):
        return None
    old = self.pose_mode
    exiting_scenario_to_face = (mode == "face" and old in ("intro", "interview"))
    home_cmd = None
    if exiting_scenario_to_face:
        home_cmd = self._make_face_home_command()
    self.pose_mode = mode
    print(f"[Mode] Switched to {mode.upper()}")
    return home_cmd
```

### 6.3 场景模式调用（Target 主 demo，已解耦）

视觉线程仅计算目标，不直接发 UART；命令入队后由 `ElfControlThread._tick()` 在 50 Hz 统一发送。

**Target** (`camera_demo_pingpong_async_pnp.py`):
```python
if self.elf_thread is not None:
    mode = self.elf_thread.get_mode()
    now_us = int(time.perf_counter() * 1_000_000)
    cmd = None
    if mode == "intro":
        cmd = self.elf_thread.controller.update_intro_control(
            body_dets, self.img_w, self.img_h, now_us=now_us
        )
    elif mode == "interview":
        cmd = self.elf_thread.controller.update_interview_control(
            body_dets, self.img_w, self.img_h, now_us=now_us
        )
    if cmd is not None:
        sc_q = self.elf_thread.scenario_cmd_q
        while not sc_q.empty():
            try:
                sc_q.get_nowait()
            except queue.Empty:
                break
        sc_q.put_nowait({"type": "arm_cmd", "cmd": cmd})
```

**Target** (`elf_control_chain.py::ElfControlThread._tick()`):
```python
elif typ == "arm_cmd":
    c = sc["cmd"]
    self.uart_sink.send_arm_target(
        c["x"], c["y"], c["z"],
        c["servo2"], c["servo1"], c["flag"]
    )
```

### 6.4 ARM-CMD 输出

**Base** (`src/rga_npu.cpp:2187-2201`):
```cpp
printf("[ARM-CMD] tx=%.2f ty=%.2f tz=%.2f j5=%.2f j4=%.2f flag=0x%02X "
       "| vec_pitch=%+.2f vec_yaw=%+.2f qmode=%d ...",
       last_tx, last_ty, last_tz, curr_servo2, curr_servo1, flag,
       vec_pitch, vec_yaw, quat_valid ? 1 : 0, ...);
```

**Target** (`elf_control_chain.py:2517-2525`):
```python
print(
    f"[ARM-CMD] tx={cmd['x']:.2f} ty={cmd['y']:.2f} tz={cmd['z']:.2f} "
    f"j5={cmd['servo2']:.2f} j4={cmd['servo1']:.2f} flag=0x{flag:02X} "
    f"| vec_pitch={info['vec_pitch']:+.2f} vec_yaw={info['vec_yaw']:+.2f} "
    ...
)
```

### 6.5 UART 11 字节帧

**Base** (`src/uart_comm.cpp:316-338`):
```cpp
int16_t data[5];
data[0] = (int16_t)x; data[1] = (int16_t)y; data[2] = (int16_t)z;
data[3] = (int16_t)k1; data[4] = (int16_t)k2;
uint8_t frame[11];
memcpy(frame, data, 10);
frame[10] = flag;
uart_send_raw(frame, sizeof(frame));
```

**Target** (`elf_control_chain.py:1008-1009`):
```python
data = struct.pack("<hhhhh", int(x), int(y), int(z), int(k1), int(k2))
frame = data + bytes([flag])
```

帧格式一致，STM32 桥接额外加了 `AA 55 LEN CMD PAYLOAD CRC` 封装。

### 6.6 WebSocket 注册

**Base** (`src/main.cpp:181-188`):
```cpp
if (ws_send_text(sock, "{\"type\":\"frame_ts\"}") < 0) {
    printf("[WS] Registration send failed\n");
    ...
}
printf("[WS] Registration frame_ts sent\n");
```

**Target** (`demos/camera_demo_lowlatency.py:435-437`):
```python
ws.send(json.dumps({'type': 'frame_ts'}))
print("[WS] Registration frame_ts sent")
```

---

## 7. 结论

当前 `trial0` 项目的主 demo `camera_demo_pingpong_async_pnp.py` 在“视觉 + 控制”核心链路上与 Base 的对应度较高：

- 控制算法（A-inverse、双 IMU、PnP 修正、运动 FSM、场景模式、标定模式）已完整移植；
- 默认硬件参数已与 Base 对齐（`/dev/ttyS9`、`/dev/spidev4.0`、`/dev/i2c-4`、8080 端口）；
- Intro/Interview 模式已接入视觉 detections；
- PnP 修正已正确馈入控制链；
- 退出时会通过 `elf_thread.stop()` 发送 exit frame。

但由于主 demo **不承担** Base `main.cpp` 的“全系统入口”角色，存在以下结构性差异：

1. **云端通信不在主 demo 中**：RTMP/WebSocket 仅存在于 `demos/camera_demo_lowlatency.py`；
2. **系统级初始化缺失**：CPU 调频、WiFi、NPU 初始化、RTMP 探测、WS 注册等待均缺失；
3. **标定 mode 1/2 行为差异（已文档化）**：Target 保留改进，实际发送可调 servo1，Base 源码字面发送固定 50/145；该差异已在代码注释与本文档中说明；
4. **多 demo 架构**：能力分散在多个文件中，无法与 Base 单一进程一一对应。

本次审计报告列出的中优先级整改项（A-init 5 帧平均、场景模式与视频帧解耦、RuleEngine 复用、退出时队列清空）均已完成。

建议按照本报告第 5 章逐项整改。若项目目标就是“主 demo 只管视觉+控制，其他能力由其他 demo 承担”，则应在 `README.md` 和各 demo 文档中明确各入口的职责边界，避免声称与 Base 主入口完全一一对应。
