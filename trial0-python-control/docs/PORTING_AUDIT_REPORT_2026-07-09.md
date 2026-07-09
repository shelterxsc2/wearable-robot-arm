# ELF 跨平台移植审计报告与整改指南

审计日期：2026-07-09

## 0. 报告定位

本报告基于以下三套当前本地代码进行静态审计和可运行单元测试：

| 角色 | 路径 | 本报告简称 |
|---|---|---|
| 原始系统/移植基线 | `/home/time/work/elf_info/wearable-robot-arm` | Base |
| DK-2500/OpenVINO 移植目标 | `/home/time/work/trial0` | Target |
| STM32F103 数据汇聚桥固件 | `/home/time/work/stm32f103_datahub` | Bridge |

Target 当前不是 Git 仓库，因此无法用提交历史判断某项差异是有意设计还是临时改动。本报告只判断当前文件的实际行为。

Bridge 工程同时保留了 `Core/Inc|Src` 与 `UserApp/Inc|Src` 两份同名业务文件。Keil 工程和 Eclipse `.cproject` 实际引用 `UserApp`：

- `MDK-ARM/STM32F103C8T6_DataHub.uvprojx:397-419`
- `.cproject:45,78,122,155`

因此所有固件结论均以 `UserApp` 为准；`Core` 中的旧副本不作为现行固件证据。

### 0.1 实际运行入口

当前推荐的一键入口是：

```text
run_elf_all.py
  ├─ start_rtsp_server.sh
  └─ run_elf_main.sh
       └─ camera_demo_pingpong_async_pnp.py
            ├─ camera_demo_elf_pipeline.SimplePipeline
            ├─ elf_control_chain.py
            ├─ elf_control_chain_stm32.py（默认）
            ├─ voice/*
            ├─ system_init.py（仅显式 --wifi/--ble）
            └─ lowlatency_streamer.py
```

审计主对象是这条链路，而不是旧报告曾使用的独立 demo。

### 0.2 判定术语

- **一致**：输入、状态转换、输出协议和生命周期效果均可对应。
- **等效但不相同**：实现技术不同，但对外行为基本等效。
- **部分对应**：主路径存在，但至少一个重要条件、状态或错误路径不同。
- **不一致**：不能满足“除新增功能/设备号外完全一样”的要求。
- **无法静态确认**：必须连接摄像头、H7、F103、云服务或真实传感器验证。

---

## 1. 执行摘要

### 1.1 五项最终结论

| 检查项 | 结论 | 风险 | 核心原因 |
|---|---|---:|---|
| 1. 数据/控制链路模块一一对应 | **部分对应，不是一一等价** | 高 | 正常传感器和控制帧可闭合；Bridge 默认路径缺少真实 `init success/move complete` 握手，`nrf24_reset` 为无操作，RuleEngine 架构和反馈语义不同 |
| 2. 模式齐全，尤其标定 | **编号齐全，行为不完全一致** | 高 | FACE/BODY/INTRO/INTERVIEW 和标定 0~5 均存在；mode 1/2 有意偏离 Base，mode 5 首拍与文件覆盖语义不同，标定状态不落 `/tmp/calib_mode.txt` |
| 3. 终端打印一一对应 | **不能一一对应** | 中 | 控制核心前缀大体对应；系统初始化、握手、推流实现和错误路径打印不同，Target 还缺 Base 的阶段性打印 |
| 4. 初始化及 Ctrl+C 退出完全一样 | **不一样** | 高 | 默认 WiFi/BLE 未启用；初始化顺序不同；信号注册过晚；初始化异常不在统一 `finally` 内；Bridge 握手不同；清理顺序不同 |
| 5. 云通信及云/本地推流仅 device_id 不同 | **明确不成立** | 高 | WS 云 IP 与 RTMP 云 IP来源不一致；心跳计数和重连语义不同；Target 解析云端控制消息；编码器、帧率、GOP、分辨率、RTSP 架构均不同 |

### 1.2 可以确认已经正确对应的部分

1. Bridge 上行 `A5 TYPE LEN SEQ PAYLOAD CRC` 的帧头、类型、长度和加和校验一致。
2. Bridge 下行 `AA 55 LEN CMD PAYLOAD CRC` 的构造和固件解析一致。
3. 默认 `CMD=0x30, LEN=11` 的机械臂目标可由 F103 原样转发至 H7。
4. H7 init/retract 两个 10 字节特殊帧可由 F103 原样转发。
5. NRF24 22 字节载荷的 gyro/quaternion 解析已有 6 个单元测试通过。
6. 控制线程周期目标为 50 ms，与 Base 相同。
7. FACE/BODY/INTRO/INTERVIEW 四种场景名均存在并接入视觉线程。
8. 标定 mode 0~5 均有 Target 分支。
9. RTMP URL 和 WS URL 在默认常量条件下与 Base 结构一致，仅默认 `device-002`/`device-003` 不同。
10. WS 注册帧 `{"type":"frame_ts"}` 和 100 ms 心跳字段集合基本一致。

### 1.3 发布阻断项

在宣称“完成等价移植”之前，至少应关闭以下 P0：

| 编号 | 阻断项 |
|---|---|
| P0-1 | 为 Bridge 路径设计并实现可观测握手，禁止把 `init_success=True` 当作 H7 确认 |
| P0-2 | 恢复 7 秒归位屏蔽，或由 H7/F103 上报 `move complete` 后再 A-init/TX 解锁 |
| P0-3 | 修复 WS URL 使用硬编码云 IP，保证 RTMP 与 WS 指向同一云端 |
| P0-4 | 将所有初始化纳入统一异常清理域，并在启动后台线程前注册 SIGINT/SIGTERM |
| P0-5 | 明确 mode 1/2 采用 Base 字面行为还是修正后的预期行为，并同步两端文档与测试 |
| P0-6 | 修复当前 `test_stm32_bridge.py` 的 3 个失败并增加固件协议向量测试 |

---

## 2. 系统和模块映射

### 2.1 Base 到 Target

| Base | Target | 判定 | 说明 |
|---|---|---|---|
| `src/main.cpp` | `run_elf_all.py` + `camera_demo_pingpong_async_pnp.py` | 部分对应 | Base 单进程；Target 是 launcher + 主进程 + MediaMTX + FFmpeg |
| `src/rga_npu.cpp` | `camera_demo_elf_pipeline.py` + `camera_demo_pingpong_async_pnp.py` + `elf_control_chain.py` | 部分对应 | 算法被拆分，推理后端由 RKNN/RGA 改为 OpenVINO/OpenCV |
| `src/nrf24_linux.c` | `BridgeNrf24ImuSource`（默认）或 `Dk2500Nrf24ImuSource` | 等效扩展 | 默认物理路径变为 NRF24→F103→UART |
| `src/imu2_i2c.c` | `BridgeImu2Source`（默认） | 等效扩展 | 默认物理路径变为 JY61P→F103→UART |
| `src/uart_comm.cpp` | `BridgeUartArmSink`（默认） | 部分对应 | 坐标帧可对应，握手状态不可对应 |
| `src/ctrl_server.cpp` | `HttpControlServer` | 部分对应 | 主端点存在；Target 额外 `/profile`、toggle；`nrf24_reset` 无实际动作 |
| `src/ws_client.cpp` + `main.cpp::ws_worker_thread` | `lowlatency_streamer.py` | 部分对应 | 注册/心跳相近，地址、接收行为、计数和重连语义不同 |
| `src/gst_rtmp.cpp` | `LowLatencyStreamer` + FFmpeg | 不相同 | 编码管线、帧来源、GOP、帧率和统计点不同 |
| `src/gst_rtsp.cpp` | MediaMTX + FFmpeg RTSP publisher | 不相同 | Base 内嵌 RTSP server；Target 外部服务 |
| `src/stream_manager.cpp` | `LowLatencyStreamer.probe_and_start()` | 部分对应 | 有 RTMP 探测和 RTSP 回退，但 Target 还会探测 RTSP，失败后本地无流 |
| `src/wifi.cpp` | `system_init.py` | 可选等效 | 默认入口未传 `--wifi`，实际不执行 |
| `src/bluetooth_spp.c` | `system_init.py` + `tools/ble_remote.py` | 新实现 | 默认入口未传 `--ble`；协议栈和生命周期不同 |
| `scripts/rule_engine_server.py` | `SimplePipeline` 内本地模型 | 不相同 | 312B/56B Unix socket 服务变为进程内推理 |
| `scripts/calibrate.py` | 无 | 缺失 | 相机内参棋盘格标定工具未移植 |

### 2.2 Target 到 Bridge

| Target 常量/行为 | Bridge `UserApp` | 判定 |
|---|---|---|
| USART 460800 8N1 | `APP_DK_UART_BAUDRATE=460800` | 一致 |
| `TYPE_WAIST_IMU=0x51` | `APP_BRIDGE_TYPE_WAIST_IMU=0x51` | 一致 |
| `TYPE_NRF_IMU=0x52` | `APP_BRIDGE_TYPE_NRF_IMU=0x52` | 一致 |
| `TYPE_ARM_STREAM=0x53` | `APP_BRIDGE_TYPE_ARM=0x53` | 一致 |
| waist payload 11B | WIT `JY61P_FRAME_LEN` | 一致 |
| NRF payload 22B | `NRF24_PAYLOAD_WIDTH=22` | 一致 |
| arm VOFA payload 28B | `APP_BRIDGE_ARM_VOFA_FRAME_LEN=28` | 一致 |
| uplink CRC=sum(TYPE..PAYLOAD) | `app_sum8(TYPE..PAYLOAD)` | 一致 |
| downlink `AA 55` | `APP_DOWNLINK_FRAME_SOF0/1` | 一致 |
| heartbeat `0x01, LEN=0` | 消费、不转发 | 一致 |
| arm target `0x30, LEN=11` | 原样入 H7 11B 队列 | 一致 |
| target pose `0x10, LEN=28` | 默认消费、不转发 | 一致，但不能控制 H7 |
| raw init/retract 10B | 特殊前缀识别并原样转发 | 一致 |

注意：`UserApp/Inc/app_config.h:100` 的 `APP_DOWNLINK_ARM_TARGET_LEN=10` 与推荐的 11B 并不冲突；固件在 `app.c:1174-1186` 同时接受 10B 和 11B。Target 使用 11B 分支。

---

## 3. 检查项一：数据和控制链路

### 3.1 Base 主链路

```text
NRF24 SPI
  → 22B WIT gyro/quaternion
  → nrf24 RX thread
  → 5-frame A-init
  → R_head_delta / R_rel
  → pitch/yaw prediction state machines
  → visual PnP correction
  → kinematics + J4/J5 mapping
  → 11B raw UART frame
  → STM32H7
```

并行链路：

```text
I2C4 waist IMU → relative attitude compensation
camera → body/face/hand/PnP → control correction + overlay
HTTP/Bluetooth → mode/calibration/rebaseline commands
H7 UART RX text → init success / move complete
```

### 3.2 Target 默认主链路

```text
head NRF24 ─┐
waist JY61P ├→ STM32F103 → A5 classified frames → USB/UART 460800
H7 VOFA ────┘                                  │
                                               ├→ BridgeNrf24ImuSource
                                               ├→ BridgeImu2Source
                                               └→ arm debug state

camera/OpenVINO → PostprocessorThread → PnP/scenario queue
bridge IMU + PnP → Nrf24Controller @ 50 ms → BridgeUartArmSink
→ AA 55 0B 30 ... CRC → F103 → raw 11B → H7
```

### 3.3 传感器上行

#### 3.3.1 NRF24

- Base 当前文档描述 22B 载荷，实际 Target 解析 `0x55 0x52` gyro 与 `0x55 0x59` quaternion。
- Bridge 配置 `APP_NRF_FEED_PARSER_ENABLE=0`，即固件不改数据，直接封装 22B。
- Target `parse_nrf24_imu_payload()` 负责校验 WIT 子帧和单位换算。
- 6 个 NRF parser 测试通过，包括校验错误、头错误、顺序互换、角速度比例和 90° yaw。

判定：**载荷层一致，物理采集层为合理扩展**。

风险：

1. Base `nrf24_reset` 会触发实际复位；Target HTTP 分支只返回成功 JSON，没有调用 Bridge 或 NRF 驱动。
2. F103 上报的是最新 payload，Target 没有基于 bridge `SEQ` 明确阻止重复样本进入 A-init。
3. `_bridge_meta.drop_count` 的更新不足以构成端到端丢帧告警，HTTP `/status` 也未暴露每类 seq/drop。

#### 3.3.2 waist IMU

- Bridge USART1 为 230400，DMA circular，转发 gyro/angle/quaternion 三类合法 WIT 帧。
- Target `_parse_wit_frame()` 实际只处理 `0x51/0x52/0x53`，不处理 `0x59`。
- waist 控制需要欧拉角，`0x53` 足够；忽略 `0x59` 不会阻断当前控制，但浪费带宽且文档声称的转发能力与主机消费能力不对称。

判定：**当前控制所需数据可对应，但并非所有 Bridge 上行类型均有消费者**。

#### 3.3.3 H7 反馈

- Bridge 只把 28B VOFA `float[7]` 解为调试状态。
- Target `BridgeUartArmSink` 不从该反馈设置 `move_complete=True`。
- Base UART RX 文本用于 `init success` 和 `move complete`。

判定：**H7 调试数据可到达，但握手/运动完成反馈链断裂**。

### 3.4 控制下行

Target `build_arm_target_frame()`：

```text
AA 55 0B 30
  int16 x
  int16 y
  int16 z
  int16 k1
  int16 k2
  uint8 flag
CRC
```

Bridge `app_downlink_handle_host_frame()` 对 LEN=11 直接入队并转发 USART2，故协议闭合。

需要确认的量纲：

- Target `int(x/y/z)` 直接截断。
- Base `uart_send_arm_target()` 同样将控制层浮点位置转为 int16。
- Bridge 文档称 H7 将 int16 `/100.0` 转米；这必须与实际 H7 固件确认。H7 代码不在本次三个审计目录中，不能静态证明最终执行量纲。

### 3.5 握手链路重大差异

Base：

```text
等待 H7 文本 "init success"
→ 发送 FF AA ... init
→ block_tx=1
→ 等待 7 秒归位
→ 触发 A-init，收集 5 个新 IMU 帧
→ R_init_set
→ NORMAL / 允许目标发送
```

Target Bridge 路径：

```text
BridgeNrf24ImuSource.start() 打开串口
→ BridgeUartArmSink.start() 直接设置 init_success=True
→ 立即发送 FF AA ...
→ 因 sink 不是 SerialUartArmSink，跳过 7 秒等待
→ 立即触发 A-init
→ 最多等待 10 秒
→ 即使超时也 block_tx=False
```

证据：

- `elf_control_chain_stm32.py:562-567`
- `elf_control_chain.py:2397-2417`

影响：

1. H7 尚未初始化时主机也会声明成功。
2. A-init 可能在机械臂归位运动中采样。
3. 没有新 NRF 数据时，10 秒后仍允许发令。
4. `move_complete` 在发送目标后只会被置 False，Bridge 路径没有恢复 True 的来源。

判定：**控制帧协议对应，但安全状态机不对应，因此不能称整条控制链一一对应。**

### 3.6 RuleEngine 和视觉耦合

- Base 使用独立 Python server，固定 312B 输入/56B 输出。
- Target 在 `SimplePipeline` 内直接加载模型。
- Target 场景命令由视觉线程计算后进入容量 2 的 queue，再由控制线程消费。

后者解决了 UART 发送线程与视觉线程直接耦合，但在队列满时会丢弃/覆盖的具体策略需要纳入验收。RuleEngine 需要用同一输入集做逐元素输出、类别和延迟比较，现有测试没有覆盖。

### 3.7 检查项一整改

#### P0

1. 在 F103 上新增 H7 状态上行类型，至少包含：
   - `H7_INIT_SUCCESS`
   - `H7_MOVE_COMPLETE`
   - `H7_FAULT`
2. `BridgeUartArmSink.start()` 不得直接写 `init_success=True`。
3. `ElfControlThread.start()` 使用统一握手接口，而不是 `isinstance(SerialUartArmSink)` 分支。
4. A-init 超时时保持 `block_tx=True`，只允许显式降级参数解锁。
5. 实现真实 `nrf24_reset`，或返回 501/明确“不支持”，不能假成功。

#### P1

1. `/status` 增加每类 bridge seq、drop、CRC bad、last_rx_age_ms。
2. A-init 只计入 seq 变化且时间戳晚于 `start_a_init()` 的新帧。
3. 增加 H7 量纲和 int16 溢出/限幅契约测试。
4. 对 RuleEngine 建立 Base/Target golden dataset。

---

## 4. 检查项二：模式和标定

### 4.1 场景模式

| 模式 | Base | Target | 判定 |
|---|---|---|---|
| FACE | face 检测、PnP、IMU 控制 | 同类主路径 | 基本对应 |
| BODY | body 检测/跟踪/绘制 | 跳过 face/hand/PnP | 基本对应 |
| INTRO | 场景姿态控制 | 视觉线程计算，控制队列发送 | 功能存在，实现不同 |
| INTERVIEW | 双人/采访场景控制 | 视觉线程计算，控制队列发送 | 功能存在，实现不同 |

从 INTRO/INTERVIEW 回 FACE 时，Target 会发送 home 并请求 head center 重捕获，语义与 Base 相近。

### 4.2 标定模式总表

| mode | Base | Target | 齐全性 | 一致性 |
|---:|---|---|---|---|
| 0 | 正常控制 | 正常控制 | 齐全 | 基本一致 |
| 1 | 舵机标定 | 读取 `/tmp/servo_calib.txt` | 齐全 | 不一致 |
| 2 | 100°~180° 扫描 | 同范围扫描 | 齐全 | 不一致 |
| 3 | PnP yaw 多点标定 | 11 点 yaw 标定 | 齐全 | 高度相近 |
| 4 | PnP pitch 多点标定 | 5 点 pitch 标定 | 齐全 | 高度相近 |
| 5 | 头 IMU 3x3 网格 | 9 点网格 | 齐全 | 部分一致 |

### 4.3 mode 1/2

Base 计算了 `calib_servo1`，但最终调用：

```cpp
uart_send_arm_target(..., NRF_SERVO1_DEG, NRF_SERVO2_DEG, 0x01);
```

即源码字面行为仍发送固定值。Target 有注释明确说明它有意修复这一问题，实际发送可调/扫描值。

结论：

- 若目标是“逐字节复刻 Base”，Target 不一致。
- 若目标是“实现 Base 注释表达的真实标定用途”，Target 更合理，但应先修 Base 或建立正式协议版本，不能一边声称完全一致一边保留行为分叉。

另有字段语义风险：

- HTTP `/servo?k1=...&k2=...` 将 `k1` 写入 `/tmp/servo_calib.txt`。
- mode 1 的 Target 把该值放进返回字典 `servo1`，发送时映射到下行 `k2`。
- 注释中 J4/J5、servo1/servo2、k1/k2 多次交叉，容易把物理关节写反。

必须建立唯一命名表：

| 协议字段 | 控制变量 | 物理关节 | 当前约定 |
|---|---|---|---|
| k1 | servo2 | J5 yaw | yaw |
| k2 | servo1 | J4 pitch | pitch |

所有 API、日志和 CSV 应使用该表。

### 4.4 mode 3/4

一致部分：

- yaw 目标：`0,-15,-30,-45,-60,-75,15,30,45,60,75`
- pitch 目标：`0,-15,-30,15,30`
- PREPARE 5 秒，SAMPLE 8 秒
- 输出路径一致
- 标定时暂停正常跟随

差异：

1. Base 只在目标命令发送成功后启动 phase timer；Target 每 tick 构造命令，没有显式“发送成功后再推进”的状态。
2. Target 在 `send_arm_target()` 因 block/串口错误失败时，controller 内 phase 仍可能继续计时。
3. Target CSV 使用 append；Base 在新标定开始时会初始化/覆盖对应文件的行为应逐模式统一。
4. Target 完成后把 target index 置 -1；如果 mode 仍保持 3/4，下一 tick 的重入逻辑存在重新开始风险，需要实测。

### 4.5 mode 5

一致部分：

- 3x3 共 9 个目标。
- PREPARE/SAMPLE 时长一致。
- CSV 字段集合基本一致。
- 可记录 head、waist 和 relative attitude。

差异：

1. Base 进入 mode 5 后立即计算并发送第一个目标；Target 第一个 tick 初始化状态后直接 `return None`。
2. Base 开始标定时以 `"w"` 创建 CSV，保证一次运行一个干净数据集；Target 根据文件是否存在决定 header，然后 append，旧数据可能混入。
3. Base `waist_valid` 使用当前状态；Target 使用采样期间多数票。后者可能更稳健，但不相同。
4. Target 要求 `r_init_set` 且 `R_init` 存在才累计；Bridge 握手不可靠会直接造成 0 样本记录。
5. Target mode 5 只返回 `None`，当前实现没有像 Base 那样为每个网格点生成并发送机械臂目标。这是功能缺口，不只是首 tick 差异。

第 5 点为标定完整性的高风险问题：Target 的网格状态机记录目标名称，却没有驱动机械臂依次到九个目标姿态。

### 4.6 标定控制面差异

- Base `/calib` 写 `/tmp/calib_mode.txt`，视觉和控制逻辑通过文件共享。
- Target `/calib` 只改内存 `controller.calib_mode`。
- Target 启动不显式删除 `/tmp/calib_mode.txt`；不过它也不读取该文件，所以旧文件不会控制当前主链。
- 外部脚本若仍通过写 `/tmp/calib_mode.txt` 控制标定，在 Target 上失效。
- Base 文档只列 0~4，但源码已有 mode 5；Target README/报告必须以源码 0~5 为准。
- Base 有独立相机棋盘格 `scripts/calibrate.py`；Target 无对应工具。若“各个模式”包含相机内参标定，则仍不齐全。

### 4.7 检查项二整改

#### P0

1. 补齐 mode 5 的九点机械臂目标发送。
2. 标定 phase 只能在下行发送成功后开始计时。
3. mode 1/2 做正式决策：
   - 兼容模式：严格发送 Base 固定值；
   - 修正版：Base 和 Target 同时改为可调值，并升级协议/文档版本。
4. 统一 k1/k2/J4/J5 命名。

#### P1

1. 每次进入 mode 3/4/5 时原子地覆盖 CSV，并写入 run_id、版本、profile、device_id。
2. 防止完成态在 mode 未清零时自动重启。
3. `/status` 输出 phase、target、elapsed、sample_count、last_send_ok。
4. 提供 `/calib?action=start|stop|reset`，不要依赖裸整数和隐式文件。
5. 移植相机内参标定工具，或明确它属于部署前离线工具。

---

## 5. 检查项三：终端打印

### 5.1 可对应的核心打印

| 语义 | Base | Target | 判定 |
|---|---|---|---|
| 模式切换 | `[Mode] Switched to ...` | 同前缀 | 对应 |
| profile | `[ArmProfile] ...` | 同前缀 | 对应 |
| A-init | `[A-INIT] R_init built...` | 同前缀 | 对应 |
| head center | `[HeadCenter] ...` | 同前缀 | 对应 |
| 正常命令 | `[ARM-CMD] ...` | 同前缀和主要字段 | 基本对应 |
| PnP 标定 | `[PnP-Calib] ...` | 同前缀 | 基本对应 |
| 头 IMU 标定 | `[Head-IMU-Calib] ...` | 同前缀 | 部分对应 |
| UART 目标 | `[UART-TX] ...` | direct-H7 路径有 | 默认 Bridge 路径不对应 |

### 5.2 不能对应的系统打印

Base 默认总会出现或尝试：

- CPU performance
- WiFi 初始化和 IP
- RuleEngine 子进程
- NPU/RGA 初始化
- Bluetooth 初始化
- UART/NRF24/IMU2
- RTMP probe / RTSP fallback
- WebSocket
- handshake
- cleanup

Target 默认 `run_elf_main.sh` 没有 `--wifi` 和 `--ble`，因此相关打印不会出现。Target 的 OpenVINO、FFmpeg、MediaMTX、voice、gesture 和 Bridge 打印也没有 Base 对应项。

### 5.3 握手打印存在误导

Target 默认 Bridge 路径打印：

```text
[STM32-DataHub] Arm sink started, H7 init frame sent
[UART-Handshake] Starting A-init capture...
[UART-Handshake] A-init (R_init) captured
[UART-Handshake] TX unblocked
```

它不会打印 Base 的真实等待状态，且 `Arm sink started` 只证明串口 write 被调用，不证明 F103 转发成功或 H7 接收成功。

建议把日志分为：

```text
TX_QUEUED → BRIDGE_FORWARDED → H7_ACKED → HOMING_DONE → A_INIT_DONE → NORMAL
```

每一步必须来自可验证事件，不能由本地赋值模拟。

### 5.4 日志字段问题

1. Base/Target 同时混用 `j5/j4`、`k1/k2`、`servo2/servo1`。
2. Target `LowLatencyStreamer._probe_url()` 标注返回 `bool`，失败时实际返回 exception 对象，打印文本依赖异常字符串。
3. Bridge 默认 `verbose=False`，正常下行不打印 `[UART-TX]`，现场无法逐条关联 `[ARM-CMD]` 与串口帧。
4. 日志没有统一 timestamp、thread、seq、command_id。
5. Target Python stdout 未像 Base `setbuf(stdout,NULL)` 一样显式无缓冲；通过重定向运行时可能延迟。

### 5.5 检查项三整改

1. 采用统一结构化前缀：

```text
[ARM-CMD] ts_us=... cmd_id=... x=... y=... z=... k1_j5=... k2_j4=... flag=...
[BRIDGE-TX] cmd_id=... bytes=... write_ok=...
[BRIDGE-RX] type=... seq=... crc_ok=... age_ms=...
```

2. 启动时打印完整有效配置，密码除外。
3. 每个初始化阶段打印 `START/OK/DEGRADED/FAIL`.
4. 退出时打印每个资源的 stop 结果。
5. 使用 `python -u` 或 `PYTHONUNBUFFERED=1`。
6. 建立日志 golden test，只比较 Base 共有语义，不要求 Python/OpenVINO 新增日志与 Base 对齐。

结论：**排除手势、语音、Bridge 新增日志后，仍不能做到终端输出一一对应。**

---

## 6. 检查项四：初始化和 Ctrl+C

### 6.1 Base 初始化顺序

```text
删除 /tmp/calib_mode.txt
→ CPU performance
→ WiFi + DHCP/static fallback
→ RuleEngine server
→ NPU
→ RGA
→ Bluetooth
→ UART + RX
→ HTTP server
→ NRF24
→ IMU2
→ signal handler
→ GStreamer
→ RTMP probe / WS
→ handshake thread
→ 50 ms control timer
→ stream loop
```

### 6.2 Target 当前顺序

```text
launcher 先启动 MediaMTX
→ 主程序加载全部视觉模型
→ 打开 camera
→ 构造 control objects
→ 启动 voice
→ 探测 RTMP/启动 WS/FFmpeg（可能阻塞 10s）
→ 可选 WiFi/Bluetooth
→ 启动视觉线程
→ 启动 control bridge/握手
→ 可选 BLE remote
→ 最后注册 SIGINT/SIGTERM
→ 主循环
```

### 6.3 初始化差异和风险

1. **WiFi 顺序相反**：Target 在可选 WiFi 初始化之前就探测云端。
2. **WiFi 默认关闭**：`run_elf_main.sh` 不传 `--wifi`。
3. **Bluetooth 默认关闭**：不传 `--ble`。
4. **无 CPU performance 对应步骤**。
5. **RuleEngine 改为进程内模型**。
6. **推流早于控制链启动**，Base 则先启传感器和控制服务器。
7. **信号注册过晚**：模型加载、camera open、voice、WS/FFmpeg、线程启动期间按 Ctrl+C 走默认处理。
8. **统一 `try/finally` 太晚**：大量初始化发生在 `try` 之前，异常时不会执行主清理。
9. **camera open 未检查 `cap.isOpened()`**。
10. **`ll_streamer.probe_and_start()` 的返回值未用于禁用对象**，后续仍逐帧调用 `write_frame`。
11. **`run_elf_all.py --no-stream` 仍先启动 MediaMTX**，参数只透传给主程序。

### 6.4 Base 退出顺序

```text
SIGINT/SIGTERM 只置标志
→ main loop 安全退出
→ 发送 H7 retract
→ 等 50 ms
→ 停 WS
→ 停 NRF24 / IMU2
→ UART cleanup
→ Bluetooth
→ HTTP
→ NPU/RGA
→ RuleEngine
```

### 6.5 Target 退出顺序

主程序：

```text
signal handler 设置 stop event
→ 清空视觉队列
→ join 部分视觉线程（超时）
→ stop voice
→ elf_thread.stop()
   → stop control loop
   → stop HTTP
   → 发送 retract + 50ms
   → stop IMU2
   → stop bridge serial
→ stop WS/FFmpeg
→ release camera
→ destroy windows
```

launcher：

```text
SIGINT main process group
→ 最多等 15s
→ 超时 SIGKILL
→ SIGTERM MediaMTX
```

### 6.6 退出差异和风险

1. Base 先发 retract 再设置全局 running false；Target 先停视觉和控制线程再发 retract。
2. Target 先停 HTTP，再发 retract；Base 是 retract 后较晚停 HTTP。
3. Target 停 WS/FFmpeg 晚于机械臂；Base stream loop 已返回后清理。
4. `wifi_thread` 没有 join/cancel。
5. `ble_remote` 没有显式 stop。
6. capture/preprocessor threads 没有逐个 join。
7. 所有 join 都允许超时后继续释放共享资源，可能造成后台线程访问已关闭对象。
8. `LowLatencyStreamer.stop()` WS join 仅 2 秒，线程可能在最长 5 秒 websocket connect 或重连等待中。
9. `run_elf_all` 15 秒后 SIGKILL，若主进程卡在模型/驱动调用，retract 不保证发送。
10. Bridge sink 只有本地 `init_success=True` 时发送 retract；该字段并非 H7 状态。
11. 初始化中途异常时，FFmpeg、voice、camera、bridge 可能没有统一清理。

### 6.7 检查项四整改

#### P0：统一生命周期

主程序应采用资源栈：

```python
stop_ev = Event()
install_signal_handlers(stop_ev)
resources = ExitStack()
try:
    # 按依赖顺序初始化，每成功一个立即注册 cleanup
    ...
    run(stop_ev)
finally:
    stop_ev.set()
    resources.close()
```

建议顺序：

```text
signal
→ config validation
→ CPU/device performance policy
→ WiFi
→ models
→ camera
→ Bridge/control + verified handshake
→ HTTP/BLE/voice
→ RTMP probe + WS + stream
→ visual workers
```

退出顺序使用严格反序，并在最前面发送安全 retract：

```text
block new commands
→ retract and wait/ack
→ stop producers
→ drain/stop workers
→ stop cloud/stream
→ stop HTTP/BLE/voice
→ close bridge/camera/models
```

#### P1

1. 所有非 daemon worker 必须 join 成功；超时记录为 shutdown failure。
2. 为初始化每一阶段增加 fault-injection test。
3. `run_elf_all --no-stream` 不启动 MediaMTX。
4. launcher 根据子进程退出码返回非零，不要总是 `sys.exit(0)`。
5. 增加断电/串口拔出/云端断开时的安全策略。

结论：**正常 Ctrl+C 大概率能触发 retract，但流程与 Base 不完全相同，异常初始化和超时路径还不具备同等级保证。**

---

## 7. 检查项五：云通信和推流

### 7.1 默认配置

| 配置 | Base | Target | 判定 |
|---|---|---|---|
| device_id | `device-003` | `device-002` | 预期差异 |
| 默认 cloud IP | `47.93.162.124` | `47.93.162.124` | 默认一致 |
| RTMP | `rtmp://IP:1935/live/{id}` | 同结构 | 默认一致 |
| WS | `ws://IP/ws?deviceId={id}` | 同结构 | 默认一致 |
| probe timeout | 3000 ms | 3000 ms | 一致 |
| WS ready timeout | 10000 ms | 10000 ms | 一致 |

### 7.2 云地址不一致缺陷

Target：

```python
get_default_rtmp_url():
    cloud_ip = /tmp/cloud_ip.txt or CLOUD_IP or "47.93.162.124"

get_default_ws_url():
    return "ws://47.93.162.124/ws?deviceId=..."
```

因此设置 `CLOUD_IP` 或 `/tmp/cloud_ip.txt` 时：

```text
RTMP → 新 IP
WS   → 47.93.162.124
```

这会导致视频和注册落到不同服务器，是明确功能错误。

### 7.3 WebSocket 协议

一致部分：

- 首帧为纯 `{"type":"frame_ts"}`。
- 发送后等待约 200 ms 并置 ready。
- 心跳字段：timestamp、elapsed、frame_count、device。
- 目标周期 100 ms。
- 重连初始 2 秒，乘 1.5，上限 30 秒。

差异：

1. Base `start_time` 在线程开始时设置一次；Target 每次重连重置，`elapsed` 语义不同。
2. Base `frame_count` 来自 GStreamer 实际 RTMP handoff；Target 在每次 `write_frame()` 调用前加一，即使 FFmpeg 已死也可能继续增长。
3. Base 收到云消息只用于存活检测并丢弃；Target JSON 解析并打印 `set_target/target_pose/ctrl_mode/track_obj`。
4. Target 有额外 `_data_queue` 上行数据通道，Base 无对应。
5. Base raw WebSocket client 仅支持 `ws://`；Target 使用 websocket-client，握手、ping/pong、关闭行为不同。
6. Target 对所有 `ws.recv()` 异常一概忽略，可能隐藏协议错误。
7. `ws_url=None` 时仍可能启动 worker；自定义配置必须验证非空。

### 7.4 RTMP 推流

Base：

- GStreamer。
- 摄像头 MJPEG 解码、AI、硬件 H264、FLV/RTMP。
- 默认 camera 1920x1080@30。
- mpph264enc 参数由 Base pipeline 固定。
- frame_count 在实际流处理位置统计。

Target：

- OpenCV 取帧和 OpenVINO 后处理。
- 把带 overlay 的 BGR raw frame 写入 FFmpeg stdin。
- 默认 `--stream-fps=20`。
- 编码器按可用性动态选择 VAAPI/QSV/libx264。
- bitrate/maxrate 4M。
- GOP 为 `fps//2`，默认 10。
- 分辨率默认 camera 实际值，参数 `--stream-width/height` 当前解析但未用于 `probe_and_start()`，属于无效参数。

结论：**URL 类似，媒体行为明显不相同。**

### 7.5 本地 RTSP

Base：

- 进程内 GStreamer RTSP server。
- RTMP probe 失败直接回退本地 RTSP。
- server 与视觉 pipeline 同生命周期。

Target：

- `run_elf_all.py` 先启动独立 MediaMTX。
- 主程序通过 FFmpeg 向 `rtsp://127.0.0.1:8554/stream` 发布。
- 主程序还会探测 RTSP 端口；失败则完全不推流。
- launcher 管理 MediaMTX 生命周期。
- RTSP transport 默认 UDP，可环境变量改 TCP。

结论：**对用户可提供相似 URL，但架构、进程、编码、传输和失败行为均不同。**

### 7.6 推流启动逻辑

Base：

```text
probe RTMP
  UP → start WS → wait ready → start RTMP
  DOWN → start embedded RTSP
```

Target：

```text
probe RTMP
  UP → start WS → wait ready → start FFmpeg RTMP
  DOWN → probe external RTSP
           UP → start FFmpeg RTSP
           DOWN → local only
```

Target `run_elf_all.py` 虽先启动 MediaMTX，正常一键路径下回退可用；直接运行 `run_elf_main.sh` 时不能保证。

### 7.7 检查项五整改

#### P0

1. 只解析一次 cloud config：

```python
cloud_ip = read_cloud_ip()
rtmp_url = ...
ws_url = ...
assert urlparse(rtmp_url).hostname == urlparse(ws_url).hostname
```

2. `frame_count` 只在 FFmpeg stdin write 成功且子进程存活时增加；更严格可读取 FFmpeg progress。
3. 明确云下行控制策略：
   - 完全兼容 Base：只做存活检测；
   - 新协议：真正鉴权、校验并接入控制器，不只打印。

#### P1

1. 固定并文档化媒体规格：分辨率、fps、bitrate、GOP、profile、pixel format、RTSP transport。
2. 修复 `--stream-width/--stream-height` 无效。
3. 将 RTMP/RTSP/WS 配置放入一个 dataclass 并在启动打印。
4. 对 FFmpeg early exit、broken pipe、WS reconnect 做状态机和健康上报。
5. 增加云端 mock integration test，验证注册必须先于 RTMP publish。

结论：**“除了 device_id 其他完全一样”明确为假。**

---

## 8. 测试结果和覆盖边界

### 8.1 已执行

解释器：

```text
/home/time/miniconda3/envs/trial0/bin/python
```

结果：

| 测试组 | 结果 |
|---|---|
| `test_nrf24_parser.py` | 6/6 通过 |
| `test_porting.py` | 15/15 通过 |
| `test_stm32_bridge.py` | 12/15 通过，3 失败 |

Bridge 失败：

1. `test_bridge_arm_sink`
   - 测试期望首字节为 framed target `AA 55`。
   - 当前 sink.start() 会先写 H7 init `FF AA`。
   - 表明实现已改但测试未同步拆分启动帧和目标帧。
2. `test_bridge_arm_sink_stop_sends_retract`
   - 实际 buffer 含 start init + stop retract。
   - 测试只期望 retract。
3. `test_bridge_source_counts_valid_frames`
   - 重复 poll 后 `r_init_set` 仍为 False。
   - 需要明确测试是否调用 `start_a_init()`，以及 controller 是否应按“新 seq”而非“每次 poll 同一快照”计数。

这三项不能作为“只是测试旧了”直接忽略。它们说明握手/A-init 契约尚未被稳定定义。

### 8.2 未执行

本次未连接真实硬件，以下不能静态证明：

- F103 实际烧录固件是否等于当前 `UserApp` 源码。
- USART3 460800 在目标 USB-UART 上的持续全双工稳定性。
- F103→H7 的 11B 分帧和 5ms inter-frame gap 是否满足 H7 ReceiveToIdle。
- H7 的单位、限位、flag 和特殊帧实现。
- NRF24 RF channel/address/data rate 与发射端完全一致。
- camera/OpenVINO 各模型运行。
- 云服务器注册和推流接受行为。
- Ctrl+C 时机械臂实际 retract。
- WiFi/BLE 权限和系统命令。

### 8.3 必须新增的自动测试

1. Python 与 C 固件共享协议向量：
   - uplink 三类型；
   - downlink heartbeat/10B/11B/28B；
   - CRC good/bad；
   - partial frame/resync/echo/dedup。
2. 握手状态机单测：
   - 无 init ack 不解锁；
   - homing 未完成不 A-init；
   - 5 个新 seq 才完成；
   - timeout 保持安全。
3. 标定 0~5 虚拟时钟测试。
4. mode 5 九条目标命令和九行 CSV 测试。
5. 生命周期 fault injection：
   - model load fail；
   - camera fail；
   - bridge open fail；
   - WS fail；
   - FFmpeg fail；
   - Ctrl+C at each stage。
6. 云配置一致性测试。
7. 日志关键阶段 golden test。

---

## 9. 分阶段整改指南

### 9.1 阶段 A：安全和协议闭环（P0）

目标：任何情况下都不能在 H7 未归位、A-init 无效或 Bridge 不可观测时放开控制。

任务：

1. 定义 Bridge 状态上行帧，例如 `TYPE=0x54`：

```text
event_id, state, error, command_seq, timestamp_ms
```

2. F103 解析 H7 `init success/move complete`，转换为状态帧。
3. Python 建立显式状态机：

```text
DISCONNECTED
→ BRIDGE_READY
→ INIT_SENT
→ H7_READY
→ HOMING
→ A_INIT
→ NORMAL
→ RETRACTING
→ STOPPED / FAULT
```

4. 所有普通 target 只允许在 NORMAL 发送。
5. retract 在任何已打开串口状态均允许发送。
6. 超时转 FAULT，不自动解锁。

验收：

- 拔掉 H7 时永不出现 NORMAL。
- 拔掉 NRF24 时永不出现 A_INIT_DONE。
- 重复/旧 seq 不推进 A-init。
- Ctrl+C 总能观察到 retract queued/forwarded。

### 9.2 阶段 B：标定闭环（P0/P1）

1. 修 mode 5 目标运动。
2. 统一 mode 1/2 决策。
3. 统一 k1/J5 与 k2/J4。
4. 每次标定创建新 run。
5. phase 以 target send ack 为起点。
6. 完成后自动回 mode 0 或保持 COMPLETE，不得静默重启。
7. 标定 CSV 加 metadata。

验收：

- 每种模式生成预期命令序列。
- 断开串口不会推进 phase。
- 退出再进入不混入旧 CSV。
- mode 5 恰好 9 个目标、9 条结果。

### 9.3 阶段 C：生命周期收敛（P0/P1）

1. 把信号注册移到 `main()` 最前。
2. 用 `ExitStack`/统一资源管理。
3. 让 `run_elf_all --no-stream` 真正跳过 MediaMTX。
4. 管理 WiFi/BLE threads。
5. 所有 worker 可中断、可 join。
6. 失败退出码向 launcher 传播。

验收：

- 在 10 个初始化阶段分别注入异常，无残留 Python/FFmpeg/MediaMTX。
- Ctrl+C 退出码和日志一致。
- 无 daemon thread 在资源释放后继续运行。

### 9.4 阶段 D：云和媒体契约（P0/P1）

1. 单一 cloud config。
2. 固定媒体规格或明确“等效而非相同”。
3. 修 frame_count。
4. 决定云下行消息是否属于正式能力。
5. 建 mock cloud，记录 WS 与 RTMP 时序。

验收：

- 任意 `CLOUD_IP` 下 RTMP/WS host 相同。
- 注册先于 publish。
- FFmpeg 退出后 frame_count 停止。
- RTMP 失败可靠回退本地 RTSP。

### 9.5 阶段 E：可观测性和回归（P1/P2）

1. 加 timestamp、seq、cmd_id。
2. `/status` 暴露健康指标。
3. 修复所有现有测试。
4. 在 CI 中运行无硬件测试。
5. 建立 HIL 测试脚本和发布 checklist。

---

## 10. 硬件在环验收清单

### 10.1 上行

- [ ] 记录 10 分钟 waist 0x51，无 CRC bad，seq 丢失率达标。
- [ ] 记录 10 分钟 NRF 0x52，无重复样本推进 A-init。
- [ ] H7 VOFA 0x53 七个 float 与 H7 发送值一致。
- [ ] 同时运行三路时 USART3 TX queue 不溢出。

### 10.2 下行

- [ ] heartbeat 不出现在 H7 UART。
- [ ] LEN=10 target 转为 11B+pad。
- [ ] LEN=11 target 原样转发。
- [ ] bad CRC 不转发。
- [ ] init/retract 原样 10B。
- [ ] 连续 1000 条 target 无粘包、错位和重复。

### 10.3 安全状态机

- [ ] H7 不在线时不解锁。
- [ ] F103 不在线时主程序明确失败。
- [ ] NRF 不在线时 A-init 失败且不解锁。
- [ ] homing 期间普通 target 被拒绝。
- [ ] Ctrl+C 发 retract。
- [ ] SIGTERM 发 retract。
- [ ] launcher 超时前主程序完成 retract。

### 10.4 模式和标定

- [ ] FACE/BODY/INTRO/INTERVIEW 实际行为逐项录像对照。
- [ ] mode 1 可调关节与 API 字段一致。
- [ ] mode 2 扫描范围和机械限位安全。
- [ ] mode 3 共 11 点。
- [ ] mode 4 共 5 点。
- [ ] mode 5 共 9 点且机械臂实际移动。
- [ ] 标定中断后可安全恢复。

### 10.5 云和推流

- [ ] 默认云 IP 和自定义云 IP。
- [ ] WS 注册在 500ms 内。
- [ ] 心跳 100ms，字段值正确。
- [ ] frame_count 与服务端收到帧一致。
- [ ] 云断开自动重连。
- [ ] RTMP 失败回退 RTSP。
- [ ] RTSP 客户端可播放指定分辨率/fps/GOP。

---

## 11. 最终判定

当前 Target 已经完成了主要算法和协议的移植，并成功引入手势、语音和 STM32F103 汇聚桥。正常数据帧和机械臂目标帧在 Target 与现行 Bridge `UserApp` 固件之间能够闭合。

但按用户提出的五个严格问题，当前不能给出“完全一致”结论：

1. 模块大体可映射，但握手反馈、NRF reset、RuleEngine 和 H7 状态链不能一一对应。
2. 模式编号齐全，但 mode 1/2 和 mode 5 行为不一致，mode 5 还缺机械臂九点运动闭环。
3. 核心控制日志可对应，完整终端打印不能对应。
4. 正常 Ctrl+C 有清理路径，但初始化顺序、默认模块、异常清理、Bridge 握手和资源停止顺序均不完全相同。
5. 云端和推流绝不只是 device_id 不同；至少存在云 IP 分裂缺陷，以及 WS、frame_count、编码参数和 RTSP 架构差异。

建议当前状态定义为：

> **功能移植基本成形，协议主路径可用，但尚未达到行为等价和安全发布标准。**

完成第 9.1 至 9.4 阶段并通过第 10 节 HIL 清单后，才适合将结论提升为“除明确新增能力和硬件适配外，核心行为等价”。
