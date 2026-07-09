# ELF 视觉 + 语音关键词识别 Pipeline

本项目在 Ubuntu 24.04 上运行一个低延迟的视觉 pipeline，同时支持基于 `sherpa-onnx` 的实时语音关键词识别（KWS）。关键词识别可以使用 CPU（INT8）或 Intel NPU（FP32，OpenVINO EP）。

视觉链路参考自 `/home/time/work/elf_info/wearable-robot-arm/src/rga_npu.cpp`，主要流程为：

```text
摄像头 → 人体姿态检测（YOLOv8s-pose / yolo26n-pose）→ 人脸 ROI 估算 →
Face Landmark 468 → 12 点 PnP 头部位姿 → 3D 立方体/坐标轴 overlay →
本地显示 / RTMP 推流 / 规则引擎 → 机械臂控制链（可选）
```

---

## 1. 目录结构

```text
/home/time/work/trial0/
├── camera_demo_pingpong_async_pnp.py     # 【主文件】异步 body 推理 + NPU 人脸/手部 + PnP + 控制链
├── camera_demo_elf_pipeline.py           # ELF 视觉 pipeline 模块（被主文件 import）
├── camera_demo_pingpong.py               # Ping-pong pipeline 模块（被主文件 import）
├── elf_control_chain.py                  # ELF 机械臂硬件控制链（Python port，默认 stub 后端）
├── stm32_bridge_utils.py                 # STM32 DataHub 桥串口探测工具
├── requirements.txt                      # Python 依赖
├── run_elf_main.sh                       # 启动脚本：运行主文件 + 默认硬件参数
├── README.md                             # 本文档
│
├── demos/                                # 其他独立 demo / 参考实现
│   ├── camera_demo_lowlatency.py         # 纯视觉低延迟推流入口（修复 ffmpeg 时间戳）
│   ├── camera_demo_pingpong_async_pnp_bench.py # 带内存/带宽/功耗采集的 benchmark 版本
│   └── voice_only_demo.py                # 仅语音 KWS 的最小 demo
│
├── models/                               # 视觉模型（OpenVINO IR / PyTorch）
│   ├── yolov8s-pose_openvino_model/
│   ├── yolov8s-pose.pt
│   ├── yolov8n-pose_openvino_model.bak/  # yolov8n-pose 备份
│   ├── yolov8n-pose.pt.bak
│   ├── yolo26n-pose (1)_openvino_model/
│   ├── yolo26n-pose (1).pt
│   ├── best_wflw_v8_pose20_openvino_model/
│   └── best_wflw_v8_pose20.pt
│
├── voice/                                # 语音 KWS 库
│   ├── __init__.py
│   ├── audio.py                          # 音频采集、重采样、增益
│   ├── config.py                         # 默认参数与环境变量
│   ├── kws.py                            # SherpaKwsSpotter 封装
│   ├── thread.py                         # VoiceKwsThread 后台线程
│   ├── vad.py                            # Silero VAD 门控
│   ├── keywords.txt                      # 当前关键词文件
│   └── keywords_raw.txt                  # 原始文本关键词
│
├── tools/                                # 辅助脚本
│   ├── apply_openvino_patch.py           # 给 sherpa-onnx 打 OpenVINO 补丁
│   ├── build_rule_engine_onnx.py         # 规则引擎 ONNX 构建
│   ├── camera_diag.py                    # 摄像头诊断
│   ├── voice_diag.py                     # 麦克风 / KWS 链路诊断（支持 wav 注入）
│   ├── voice_live_kws_diag.py            # 实时麦克风 → VAD → KWS 逐块诊断
│   └── benchmark_gpu_npu.py              # GPU vs NPU 并发干扰测试
│
├── tests/                                # 测试脚本
│   ├── test_yolo26s_pose.py
│   ├── test_yolo_compare.py
│   ├── validate_fps_fix.py
│   └── test_stm32_bridge.py              # STM32 UART 桥协议单元测试
│
├── docs/                                 # 设计文档与移植指南
│   └── PORTING_GUIDE.md                  # 上游 C++ → Python 移植计划
│
├── eval_results/                         # 性能评测结果与绘图
│   ├── analyze_eval.py                   # 解析 perf-eval 日志并生成对比表
│   ├── *_bench.py                        # 各类 benchmark 脚本
│   ├── *.txt / *.log / *-plots/          # 原始数据与可视化
│   └── ...
│
├── third_party/                          # 外部源码/运行时依赖
│   ├── onnxruntime-1.24.1/               # ONNX Runtime 头文件
│   ├── openvino_pip_runtime/             # OpenVINO 运行时（编译参考）
│   ├── ort_lib_symlink/                  # ORT 库软链接（编译参考）
│   └── sherpa_onnx-1.13.3/               # sherpa-onnx 源码与构建目录
│
├── sherpa_onnx-1.13.3 -> third_party/sherpa_onnx-1.13.3
├── hardware/
│   ├── ax201_firmware/
│   └── README.md                         # 指向外部 STM32F103 DataHub 桥固件
├── logs/
│   └── ffmpeg_lowlatency.log
└── archive/
    └── .venv/                            # 旧虚拟环境，可删除
```

---

## 2. 运行环境

- **OS**：Ubuntu 24.04.4 LTS
- **Python**：3.12（conda 环境 `trial0`）
- **Conda 路径**：`/home/time/miniconda3/envs/trial0/bin/python`
- **CPU/GPU**：Intel Core Ultra 5 225U / Arrow Lake-U Graphics
- **NPU**：Intel OpenVINO NPU EP（Meteor Lake NPU）
- **摄像头**：Realtek USB UVC，1920×1080，约 20 fps
- **麦克风**：USB Composite Device Audio（sounddevice 设备号 `6`，1 通道 @ 48000 Hz）

### 激活环境

```bash
conda activate trial0
```

如果没有激活，也可以直接用完整路径：

```bash
/home/time/miniconda3/envs/trial0/bin/python camera_demo_elf_pipeline.py ...
```

### BIOS / UEFI 设置

当前主机（Intel Core Ultra 5 225U）的 BIOS 已配置好，OpenVINO 可直接识别 `CPU/GPU/NPU`。若换主板或重置 BIOS，请检查以下选项：

| 选项 | 建议 | 说明 |
|---|---|---|
| **Internal Graphics / iGPU** | **Enabled** | 必须开启，否则 OpenVINO GPU 插件无法使用 |
| **Intel AI Boost / NPU** | **Enabled** | 必须开启，NPU 才会在 PCI 上出现（本机显示为 `Meteor Lake NPU`） |
| **Above 4G Decoding** | **Enabled** | 建议开启，有利于 GPU/NPU 显存映射 |
| **Resizable BAR** | **Enabled** | 建议开启，可提升 GPU 显存访问效率 |
| **VT-d / IOMMU** | 可选 | 本项目不强制需要，保持默认即可 |
| **Secure Boot** | 可选 | Ubuntu 24.04 通常可保持开启；若使用自定义内核模块再考虑关闭 |

验证 NPU 是否被系统识别：

```bash
lspci | grep -i npu
# 00:0b.0 Processing accelerators: Intel Corporation Meteor Lake NPU (rev 05)

ls -la /dev/accel*
# crw-rw---- root render 261, 0 /dev/accel0

/home/time/miniconda3/envs/trial0/bin/python -c "import openvino as ov; print(ov.Core().available_devices)"
# ['CPU', 'GPU', 'NPU']
```

---

## 3. 快速开始

### 3.1 无窗口测试模式（推荐）

```bash
cd /home/time/work/trial0
/home/time/miniconda3/envs/trial0/bin/python camera_demo_elf_pipeline.py \
  --headless \
  --no-stream \
  --voice \
  --voice-provider openvino \
  --frames 100
```

- `--headless`：不弹 GUI，结果保存到 `/tmp/elf_frame.jpg`
- `--no-stream`：不推 RTMP/RTSP
- `--voice`：启动语音 KWS 线程
- `--voice-provider openvino`：使用 NPU（FP32 模型）
- `--voice-device` 默认是 `6`（USB 麦克风），程序会自动开启其 ALSA Auto Gain Control
- `--frames 100`：处理 100 帧后自动退出

### 3.2 带 GUI 窗口运行

```bash
cd /home/time/work/trial0
/home/time/miniconda3/envs/trial0/bin/python camera_demo_elf_pipeline.py \
  --voice \
  --voice-provider openvino
```

按 `q` 退出。

### 3.3 仅视觉，不使用语音

```bash
/home/time/miniconda3/envs/trial0/bin/python camera_demo_elf_pipeline.py \
  --headless --no-stream --frames 100
```

### 3.4 使用 CPU 推理（更稳定，准确率更高）

```bash
/home/time/miniconda3/envs/trial0/bin/python camera_demo_elf_pipeline.py \
  --headless --no-stream --voice --voice-provider cpu --frames 100
```

### 3.5 关闭 VAD 前端

```bash
/home/time/miniconda3/envs/trial0/bin/python camera_demo_elf_pipeline.py \
  --voice --voice-provider openvino --voice-no-vad
```

### 3.6 仅语音关键词识别（无视觉）

如果你怀疑视觉 pipeline 影响了语音，可用这个最小 demo 做对比：

```bash
/home/time/miniconda3/envs/trial0/bin/python demos/voice_only_demo.py \
  --provider cpu \
  --threshold 0.1 \
  --score 0.5
```

### 3.7 麦克风/语音链路诊断

如果你发现“没有识别到关键词”，先用诊断工具检查麦克风是否有声音，以及 KWS 链路是否正常。

**1) 查看设备列表**

```bash
/home/time/miniconda3/envs/trial0/bin/python -m sounddevice
```

**2) 监听某个麦克风，看是否有音频输入**

```bash
/home/time/miniconda3/envs/trial0/bin/python tools/voice_diag.py --device 0 --duration 5
```

正常会有类似输出：

```text
rms= -12.3dB [########################      ] vad=False
rms= -10.1dB [##########################    ] vad=True
```

如果所有行都是 `rms=  -infdB`，说明该设备没有采集到声音，需要换设备号。

**3) 注入测试 wav，验证 KWS 链路**

```bash
/home/time/miniconda3/envs/trial0/bin/python tools/voice_diag.py \
  --inject-wav /home/time/work/sherpa/models/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20/test_wavs/en_0.wav \
  --keywords-file /home/time/work/sherpa/models/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20/test_wavs/keywords.txt \
  --provider openvino
```

预期输出包含：

```text
>>> 识别到关键词: LIGHT_UP
[Diag] 完成，耗时 0.82s，共识别 1 次: ['LIGHT_UP']
```

这说明 NPU + KWS 本身没问题；如果 pipeline 里仍不识别，问题通常在麦克风设备号、VAD 阈值或关键词文件不匹配。

**4) 实时逐块诊断**

```bash
/home/time/miniconda3/envs/trial0/bin/python tools/voice_live_kws_diag.py --duration 10
```

可以看到每一音频块的 RMS、VAD 状态、解码状态，方便判断 VAD 是否过于激进。

---

## 4. Pipeline 变体说明

除了主入口 `camera_demo_elf_pipeline.py` 之外，仓库还包含多个实验性/优化版本：

| 入口文件 | 用途 | 关键特性 |
|---|---|---|
| `camera_demo_elf_pipeline.py` | **主入口** | GPU body + NPU face/hand + PnP + rule engine + 语音 KWS |
| `camera_demo_lowlatency.py` | 低延迟推流 | 修复 ffmpeg 时间戳、控制推流间隔、V4L2 曝光调优 |
| `camera_demo_pingpong.py` | 双线程 ping-pong | 两个 worker 线程交替帧，CPU 预处理与 GPU body 推理重叠 |
| `camera_demo_pingpong_async_pnp.py` | 异步+NPU 优化 | body 异步请求池、face/hand 在 NPU、单后处理线程保序 |
| `camera_demo_pingpong_async_pnp_bench.py` | 综合 benchmark | 在 async 版基础上采集每模块时延、FPS、内存、内存带宽 |
| `elf_control_chain.py` | 机械臂控制链 | 头 IMU + 腰 IMU + 视觉 PnP 校正 + HTTP/BLE → UART（默认 stub 后端） |

### 4.1 Ping-pong / 异步版运行示例

```bash
# 双线程 ping-pong（GPU body + NPU face/hand）
/home/time/miniconda3/envs/trial0/bin/python camera_demo_pingpong.py \
  --headless --no-stream --frames 100

# 异步 body 推理 + NPU face/hand + PnP
/home/time/miniconda3/envs/trial0/bin/python camera_demo_pingpong_async_pnp.py \
  --headless --no-stream --frames 100

# 综合 benchmark（自动输出 JSON/TXT/绘图到 eval_results/）
/home/time/miniconda3/envs/trial0/bin/python camera_demo_pingpong_async_pnp_bench.py \
  --headless --no-stream --frames 1000
```

### 4.2 ELF 控制链

`elf_control_chain.py` 把原 C++ 控制链路（NRF24 头 IMU、腰 IMU、视觉 PnP 校正、外部 HTTP/BLE 控制 → 底层 UART）移植到 Python。默认使用 **stub 后端**，不连接真实硬件，可在 x86 上直接运行和验证算法。

真实后端当前提供两种选择：
- **STM32 DataHub 桥接**（默认）：NRF24 头 IMU、腰 IMU2 和机械臂控制共用一条 UART。
- **DK-2500 GPIO SPI**：通过板载 Super I/O GPIO 实现 bit-banged SPI 驱动 NRF24。

### 4.3 真实硬件后端依赖

启用真实硬件后端需要以下 Python 包（已在 `trial0` 环境预装）：

```bash
/home/time/miniconda3/envs/trial0/bin/pip install pyserial
```

| 后端 | 用途 | 依赖 | 设备权限 |
|---|---|---|---|
| `BridgeNrf24ImuSource` / `BridgeImu2Source` / `BridgeUartArmSink` | STM32 DataHub 桥接 | `pyserial` | 桥接 UART（如 `/dev/ttyUSB0`），需 `dialout` 组 |
| `Dk2500Nrf24ImuSource` | DK-2500 GPIO bit-banged SPI | 无额外 Python 依赖 | GPIO 设备 |
| `SerialUartArmSink` | 直接连 H7 机械臂（DK-2500 模式使用） | `pyserial` | `/dev/ttyUSB*` 或 `/dev/ttyACM*`，需 `dialout` 组 |

权限配置示例（一次性，重启后由 udev 规则接管）：

```bash
# 创建用户组（如不存在）
sudo groupadd -f i2c
sudo groupadd -f dialout

# 把当前用户加入组（注销重新登录后生效）
sudo usermod -aG i2c,dialout $USER

# I2C 设备 udev 规则
cat <<'EOF' | sudo tee /etc/udev/rules.d/99-i2c.rules
KERNEL=="i2c-[0-9]*", GROUP="i2c", MODE="0660"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=i2c
```

> 注意：`LinuxNrf24ImuSource`（基于 `/dev/spidev*`）已被移除，项目默认使用 STM32 DataHub 桥接或 DK-2500 GPIO SPI 后端。

### 4.4 DK-2500 开发板硬件连接

本项目实际运行在 **卓信创驰 DK-2500** 开发套件上（Intel Core Ultra 5 225U）。该板 40-Pin 调试接口提供 GSPI、I2C、UART、GPIO 等信号。

#### 40-Pin 接口与本项目相关的引脚

| PIN | SIGNAL | 用途 |
|---|---|---|
| 1 | 3.3V | 给 NRF24 / 传感器供电 |
| 6, 9, 14, 20, 25, 30, 34, 39 | GND | 地 |
| 3 | I2C1_DAT/GPIO1 | 腰 IMU2 I2C 数据 |
| 5 | I2C1_CLK/GPIO2 | 腰 IMU2 I2C 时钟 |
| 8 | UART TX(SIO) | 机械臂 UART 发送 |
| 10 | UART RX(SIO) | 机械臂 UART 接收 |
| 19 | GSPI_MOSI | NRF24 SPI MOSI |
| 21 | GSPI_MISO | NRF24 SPI MISO |
| 23 | GSPI_CLK | NRF24 SPI SCK |
| 24 | GSPICS0/GPIO22 | NRF24 SPI CSN |
| 7, 13, 15, 16, 18, 22, 26, 29... | GPIOx | 可选作 NRF24 CE/IRQ |

#### 当前 Linux 下的 GSPI 状态

DK-2500 的 GSPI 由板载 Super I/O / GPIO 扩展方案提供，**默认不会自动枚举为 `/dev/spidev*`**。当前系统：

```bash
$ ls /dev/spidev*
ls: cannot access '/dev/spidev*': No such file or directory

$ lspci | grep -iE 'serial|spi'
00:1f.5 Serial bus controller: Intel Corporation Device 8086:7723
```

00:1f.5 是 Intel 的 SPI 固件闪存控制器，通常不用于用户 SPI 外设。

进一步检查发现，当前 Ubuntu 6.17 内核**没有加载 DK-2500（Arrow Lake-U）的 GPIO 控制器驱动**，导致 `/sys/class/gpio/` 下没有 gpiochip，GPIO/SPI 均无法通过标准接口访问：

```bash
$ ls /sys/class/gpio/
export  unexport   # 没有 gpiochip

$ ls /sys/bus/acpi/devices/ | grep INTC10
INTC1092:00
INTC109F:00
INTC10CB:00       # Arrow Lake PCH GPIO ACPI 设备存在，但无匹配驱动
```

现有内核里的 `pinctrl-meteorlake` 只匹配 `INTC1082/1083/105E`，不匹配 Arrow Lake 的 `INTC1092/109F/10CB`。因此在没有厂商 BSP 或更新内核的情况下，**无法直接从用户空间控制 40-Pin 的 GPIO/SPI**。

#### 使用 GSPI 的几种方案

**方案 1：向卓信创驰索取 DK-2500 Linux BSP / GPIO-SPI 驱动（推荐）**

工业板卡厂商通常会提供专用驱动或设备树补丁，使 40-Pin 上的 GSPI 出现为 `/dev/spidev*` 或 `gpiochip*`。拿到驱动后按厂商文档加载即可。

**方案 2：USB 转 SPI 适配器（最快验证）**

用 USB 转 SPI 模块（如 FT232H、CP2130、CH341A 等）插到 DK-2500 的 USB3.0 口，再接 NRF24：

```text
DK-2500 USB ──► USB-SPI 适配器 ──► NRF24L01+ ──► 头 IMU
```

现有 `LinuxNrf24ImuSource` 基于 `spidev`。部分适配器（如加载 `spi-ch341-usb` 后的 CH341A）可生成 `/dev/spidev*`；否则需要重写 backend 用对应适配器的 Python 库（如 `pyftdi`）。

**方案 3：M.2 / PCIe SPI 扩展卡**

DK-2500 有 M.2 E-Key 和 B-Key，以及 CON1/CON2 扩展连接器。可接带 SPI 的 M.2 模块或 PCIe SPI 卡。

**方案 4：GPIO 软件模拟 SPI（bit-banging）**

速度低、CPU 占用高，不推荐用于 NRF24 实时 IMU 数据，但可作为最后手段。

#### 方案 4：直接用 IT8786 GPIO 做 bit-banged SPI（已实现）

仓库已提供 DK-2500 的 userspace 驱动，无需内核模块：

```bash
# GPIO 自检：会闪 USER_LED0 / USER_LED1
sudo /home/time/miniconda3/envs/trial0/bin/python tools/dk2500_gpio.py

# SPI loopback 测试（需要把 MOSI 和 MISO 短接）
sudo /home/time/miniconda3/envs/trial0/bin/python tools/dk2500_spidev.py

# NRF24 接收 demo
sudo /home/time/miniconda3/envs/trial0/bin/python elf_control_chain_dk2500.py
```

驱动文件：

| 文件 | 作用 |
|---|---|
| `tools/dk2500_gpio.py` | IT8786 Super I/O GPIO userspace 驱动 |
| `tools/dk2500_spidev.py` | 基于 GPIO 的 bit-banged SPI master |
| `tools/nrf24.py` | 通用 NRF24L01+ 寄存器驱动 |
| `elf_control_chain_dk2500.py` | `Dk2500Nrf24ImuSource` 后端 |

在 `elf_control_chain.py` 的 `make_elf_control_thread(..., nrf_backend="dk2500")` 中已集成该后端。

`elf_control_chain.py` 中的 `parse_nrf24_imu_payload()` 实现了与 `elf_info/wearable-robot-arm/src/nrf24_linux.c::nrf24_parse_22b()` 完全一致的 22 字节载荷解析：11 字节姿态帧（`0x55 0x59` + 四元数）和 11 字节陀螺仪帧（`0x55 0x52` + 角速度），均带校验和。

---

#### 方案 5：STM32F103C8T6 DataHub 桥（推荐最终方案）

DK-2500 的 40-Pin UART 已经被机械臂占用，且可用的 3.3V GPIO 太少，因此最干净的方案是外接一颗 **STM32F103C8T6（Blue Pill）** 作为传感器聚合桥：

```text
头 NRF24 IMU ──► STM32 (SPI/nRF24) ──┐
腰 JY61P IMU  ──► STM32 (USART1)    ──┼──► USART3 ──► DK-2500
H7 机械臂状态 ◄── STM32 (USART2)    ──┘
```

STM32F103 DataHub 桥负责：

1. 通过 nRF24 读取头 IMU 原始 22 字节载荷。
2. 通过 USART1 读取腰部 JY61P IMU 原始 WIT 帧。
3. 通过 USART2 监听 STM32H7 机械臂 VOFA JustFloat 流。
4. 以 **`A5 TYPE LEN SEQ PAYLOAD CRC`** 分类桥接协议把三类原始数据转发给 DK-2500（USART3，460800 波特率）：}, {
   - `0x51`：JY61P WIT 帧（11 字节）
   - `0x52`：nRF24 载荷（22 字节）
   - `0x53`：H7 VOFA 帧（28 字节）

真实固件由项目所有者维护，位于：

```text
/home/time/work/stm32f103_datahub/
```

本仓库仅保留对应的 Python 解析器 `elf_control_chain_stm32.py::Stm32DataHubBridge`，不再维护 STM32 固件。最新固件已对 USART2/3 RX 做了短中断改造（ring buffer），并把协议说明写在固件目录的 `BRIDGE_PROTOCOL.md` 中。

Python 端启用方式：

```python
from elf_control_chain import make_elf_control_thread

thread = make_elf_control_thread(
    stub=False,
    nrf_backend="stm32_uart",
    stm32_uart_port="/dev/ttyUSB0",  # 传 None 则自动探测哪个口在发 A5 桥接帧
    stm32_uart_baud=460800,
)
thread.start()
```

测试：

```bash
# 单元测试（纯软件，不依赖硬件）
/home/time/miniconda3/envs/trial0/bin/python tests/test_stm32_bridge.py

# 真实硬件双向环回测试（需要两个 USB-TTL：USART3@460800 + USART2@115200）
# 自动探测哪个口是 bridge（发 A5 帧的口），另一个当作 arm
python3 tools/stm32_bridge_loopback.py

# 或显式指定端口
python3 tools/stm32_bridge_loopback.py --bridge-port /dev/ttyUSB1 --arm-port /dev/ttyUSB0

# 链路诊断（分别测试下行/上行、噪声底、串扰）
python3 tools/stm32_bridge_diag.py

# 串扰专用探测：发送 CRC 错误帧，若 arm 口仍有数据则为物理串扰
python3 tools/stm32_bridge_crosstalk_probe.py
```

> **注意**：
> - DK-2500 接 F103 USART3 时，使用 `BridgeUartArmSink` 下发目标，串口参数 `460800 8N1`。Python 发送的下行帧格式为 `AA 55 0B 30 X Y Z K1 K2 FLAG CRC`，F103 校验后把 11 字节 payload（`X/Y/Z/K1/K2/FLAG`）原样通过 USART2 转发给 H7。
> - 460800 是当前布线（CH341 USB-TTL + 杜邦线）下的折中：原设计 921600 因 bridge TX 到 arm RX 的串扰导致 payload 错位，降到 460800 后可控。若后续改用更短/屏蔽/差分线材，可在固件和 `DEFAULT_BRIDGE_BAUD` 中恢复 921600。
> - DK-2500 直接接 H7 时，使用 `SerialUartArmSink`，并按 H7 的 UART 参数通信。
> - 若 framed 下行命令在 arm 口出现 payload 错位/混叠，先用 `tools/stm32_bridge_crosstalk_probe.py` 排除物理串扰。该脚本发送 CRC 错误帧，F103 不会转发任何字节；若 arm 口仍收到数据，说明 bridge TX 信号耦合到了 arm RX 线上。

---

## 5. 关键词配置

当前关键词文件：

```bash
/home/time/work/trial0/voice/keywords.txt
```

当前默认关键词均为中文：

| 关键词 | token 显示文本 |
|---|---|
| 拉远 | @拉远 |
| 拉近 | @拉近 |
| 介绍 | @介绍 |
| 正面 | @正面 |
| 采访 | @采访 |
| 开机 | @开机 |
| 关机 | @关机 |
| 录制 | @录制 |
| 停止 | @停止 |
| 并肩 | @并肩 |

### 5.1 修改关键词

直接编辑：

```bash
vim /home/time/work/trial0/voice/keywords.txt
```

格式为：

```text
音素序列 @显示文本
```

例如：

```text
l ā y uǎn @拉远
b ìng j iān @并肩
```

### 5.2 从原始文本生成 token 文件

如果你有原始文本关键词文件（如 `voice/keywords_raw.txt`）：

```text
拉远 @拉远
并肩 @并肩
```

用 `sherpa-onnx-cli text2token` 转换：

```bash
sherpa-onnx-cli text2token \
  --tokens /home/time/work/sherpa/models/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20/tokens.txt \
  /home/time/work/trial0/voice/keywords_raw.txt \
  /home/time/work/trial0/voice/keywords.txt
```

当前默认仅使用中文关键词；中文关键词一般使用拼音 token。

---

## 6. 性能评测

`eval_results/` 目录包含完整的评测框架与历史数据：

| 脚本 | 用途 |
|---|---|
| `cpu_camera_feed_bench.py` | 真实摄像头 feed 下的 CPU 30 fps 测试 |
| `cpu_synthetic_30fps_bench.py` | 合成帧 CPU 30 fps 测试 |
| `gpu_concurrent_bench.py` | GPU 并发 body/face/hand 测试 |
| `gpu_saturate_bench.py` | GPU 满载 saturate 测试 |
| `gpu_visual_concurrent_bench.py` | 可视化并发测试 |
| `gpu_visual_saturate_bench.py` | 可视化满载测试 |
| `analyze_eval.py` | 解析 `perf-eval` 日志并输出对比表 |

### 6.1 运行 benchmark

以 pingpong async benchmark 为例：

```bash
/home/time/miniconda3/envs/trial0/bin/python camera_demo_pingpong_async_pnp_bench.py \
  --headless --no-stream --frames 1000
```

程序结束时会输出：

```text
[Bench] 1000 frames in 52.84s (18.93 fps)
[Bench] JSON: /home/time/work/trial0/eval_results/pingpong_async_pnp_*.json
```

### 6.2 解析已有结果

```bash
cd /home/time/work/trial0/eval_results
/home/time/miniconda3/envs/trial0/bin/python analyze_eval.py
```

该脚本会解析 `*_perf.log` 与 `*_pipeline.log`，汇总 CPU/GPU/NPU/功耗/内存带宽等指标。

---

## 7. 自定义 sherpa-onnx 构建（已预装）

PyPI 上的 `sherpa-onnx` 不支持 `provider="openvino"`，因此需要自定义编译并替换 Python 包里的 `.so`。

当前环境已经预装好自定义版本，**一般情况下不需要重新编译**。下面记录构建步骤以便复现。

### 7.1 编译依赖

```bash
conda activate trial0
pip install sounddevice sherpa-onnx onnxruntime-openvino==1.24.1 soundfile
pip install "numpy<2.5,>=2" --force-reinstall --no-deps
```

> `onnxruntime-openvino==1.24.1` 自带 OpenVINO NPU EP，但会拉下 `numpy 2.5.0`，与 `openvino 2026.1.0` 冲突，必须降级到 `numpy<2.5`。

### 7.2 打 OpenVINO 补丁

```bash
cd /home/time/work/trial0
python tools/apply_openvino_patch.py
```

该脚本会修改 `third_party/sherpa_onnx-1.13.3/sherpa-onnx/csrc/session.cc` 和 `cmake/onnxruntime.cmake`。脚本已做幂等处理，重复执行不会重复插入。

### 7.3 编译

```bash
export INTEL_OPENVINO_DIR=/home/time/work/trial0/third_party/openvino_pip_runtime
export SHERPA_ONNXRUNTIME_INCLUDE_DIR=/home/time/work/trial0/third_party/onnxruntime-1.24.1/include
export SHERPA_ONNXRUNTIME_LIB_DIR=/home/time/work/trial0/third_party/ort_lib_symlink
export CXXFLAGS="-D_GLIBCXX_USE_CXX11_ABI=0"

cd /home/time/work/trial0/third_party/sherpa_onnx-1.13.3
mkdir -p build && cd build

cmake .. \
  -DSHERPA_ONNX_USE_PRE_INSTALLED_ONNXRUNTIME_IF_AVAILABLE=ON \
  -DSHERPA_ONNX_ENABLE_PYTHON=ON \
  -DSHERPA_ONNX_ENABLE_TTS=OFF \
  -DSHERPA_ONNX_ENABLE_SPEAKER_DIARIZATION=OFF \
  -DSHERPA_ONNX_ENABLE_PORTAUDIO=OFF \
  -DCMAKE_BUILD_TYPE=Release

make -j$(nproc)
```

> `CXXFLAGS="-D_GLIBCXX_USE_CXX11_ABI=0"` 必须设置，因为 `onnxruntime-openvino` Linux wheel 使用旧 ABI，否则链接时会因 `std::string` ABI 不一致失败。

### 7.4 安装自定义 Python 包

编译完成后，把产物替换到 conda 环境里：

```bash
SP=/home/time/miniconda3/envs/trial0/lib/python3.12/site-packages/sherpa_onnx/lib
BLD=/home/time/work/trial0/third_party/sherpa_onnx-1.13.3/build/lib
CAP=/home/time/miniconda3/envs/trial0/lib/python3.12/site-packages/onnxruntime/capi

# 备份原文件
cp "$SP/_sherpa_onnx.cpython-312-x86_64-linux-gnu.so" "$SP/_sherpa_onnx.cpython-312-x86_64-linux-gnu.so.bak"
cp "$SP/libsherpa-onnx-c-api.so" "$SP/libsherpa-onnx-c-api.so.bak"
cp "$SP/libsherpa-onnx-cxx-api.so" "$SP/libsherpa-onnx-cxx-api.so.bak"

# 替换自定义构建产物
cp "$BLD/_sherpa_onnx.cpython-312-x86_64-linux-gnu.so" "$SP/"
cp "$BLD/libsherpa-onnx-c-api.so" "$SP/"
cp "$BLD/libsherpa-onnx-cxx-api.so" "$SP/"

# 把 OpenVINO wheel 的 provider 库链接到 sherpa_onnx/lib，否则 ORT 找不到 provider shared object
for f in "$CAP"/lib*.so*; do
    ln -sf "$f" "$SP/$(basename "$f")"
done

ln -sf "$CAP/libonnxruntime.so.1.24.1" "$SP/libonnxruntime.so"
ln -sf "$CAP/libonnxruntime.so.1.24.1" "$SP/libonnxruntime.so.1"
```

因为自定义构建禁用了 TTS，`site-packages/sherpa_onnx/__init__.py` 里需要去掉不存在的符号导入：

- `GenerationConfig`
- `OfflineTtsSupertonicModelConfig`

去掉这两行即可正常导入。

---

## 8. 模型说明

### 8.1 视觉模型

| 模型 | 路径 | 用途 | 默认设备 |
|---|---|---|---|
| yolov8s-pose | `models/yolov8s-pose_openvino_model/` | 人体姿态检测 | GPU |
| yolov8n-pose | `models/yolov8n-pose_openvino_model.bak/` | 已备份，可恢复 | - |
| yolo26n-pose | `models/yolo26n-pose (1)_openvino_model/` | 备用姿态模型 | GPU |
| best_wflw_v8_pose20 | `models/best_wflw_v8_pose20_openvino_model/` | 人脸关键点 | GPU/NPU |
| face_landmark_468 | `/home/time/work/mymodel/face_landmark_468.onnx` | MediaPipe Face Mesh 468 点 | NPU/GPU |
| hand_landmarks_detector | `/home/time/work/mymodel/openvino_pipeline/models/onnx/hand_landmarks_detector.onnx` | 手部关键点 | NPU/GPU |
| keypoint_classifier | `/home/time/work/mymodel/openvino_pipeline/model/keypoint_classifier/keypoint_classifier.onnx` | 手势分类 | CPU |
| rule_engine_v2 | `/home/time/work/mymodel/rule_engine_v2.onnx` | 规则引擎 | GPU |

### 8.2 语音 KWS 模型

模型路径（在项目外）：

```bash
/home/time/work/sherpa/models/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20/
```

- `provider="cpu"`：使用 INT8 encoder/joiner
- `provider="openvino"`：自动切换为 FP32 encoder/joiner（NPU 对 INT8 支持有精度问题）

---

## 9. 工具脚本速查

| 脚本 | 用途 | 典型用法 |
|---|---|---|
| `tools/apply_openvino_patch.py` | 给 sherpa-onnx 源码打 OpenVINO 补丁 | `python tools/apply_openvino_patch.py` |
| `tools/voice_diag.py` | 麦克风/KWS 链路诊断 | `tools/voice_diag.py --device 0 --duration 5` |
| `tools/voice_live_kws_diag.py` | 实时逐块 KWS 诊断 | `tools/voice_live_kws_diag.py --duration 10` |
| `tools/camera_diag.py` | 摄像头诊断 | `tools/camera_diag.py` |
| `tools/benchmark_gpu_npu.py` | GPU vs NPU 并发干扰测试 | `tools/benchmark_gpu_npu.py` |
| `tools/build_rule_engine_onnx.py` | 规则引擎 ONNX 构建 | `tools/build_rule_engine_onnx.py` |
| `eval_results/analyze_eval.py` | 解析 perf-eval 日志 | `eval_results/analyze_eval.py` |

---

## 10. 常见问题

### 10.1 `The requested API version [27] is not available`

说明编译时使用的 ONNX Runtime 头文件版本与 `onnxruntime-openvino` wheel 不一致。请使用 `third_party/onnxruntime-1.24.1` 的头文件重新编译。

### 10.2 `Failed to load library ... libonnxruntime_providers_shared.so`

说明 ORT 找不到 OpenVINO provider 共享库。需要把 `onnxruntime/capi/*.so*` 链接到 `site-packages/sherpa_onnx/lib/`。

### 10.3 `duplicate case value` / `redeclaration of 'const char* npu_hybrid_env'`

说明 `apply_openvino_patch.py` 被执行了多次。当前脚本已做幂等处理，但如果源码已损坏，需要从干净源码恢复 `sherpa-onnx/csrc/session.cc` 后再执行一次：

```bash
cd /home/time/work/trial0
python tools/apply_openvino_patch.py
```

### 10.4 NumPy 版本冲突

```bash
pip install "numpy<2.5,>=2" --force-reinstall --no-deps
```

### 10.5 找不到麦克风

```bash
/home/time/miniconda3/envs/trial0/bin/python -m sounddevice
```

查看设备列表，然后用 `--voice-device <index>` 指定。

### 10.6 关键词识别不到

按顺序排查：

1. **麦克风没声音**：用 `tools/voice_diag.py --device <index> --duration 5` 检查。如果 `rms= -infdB`，换设备号。
2. **USB 麦克风增益太低**：部分 USB 麦克风（如 Jieli/Realtek composite）默认 AGC 关闭，声音极小。pipeline 现在会自动开启 ALSA `Auto Gain Control`；如果仍识别不到，可手动执行：
   ```bash
   amixer -c 1 set 'Auto Gain Control' on
   ```
3. **关键词不匹配**：确认你说的词在 `voice/keywords.txt` 里。当前默认是 `拉远/拉近/介绍/正面/采访/开机/关机/录制/停止/并肩`。
4. **VAD 过滤掉了**：默认 VAD 已经调得比较宽松（threshold=0.3，hangover=800ms），并且 KWS 现在会在每个音频块都解码，只在连续静音 10 个块后才 reset。如果仍然漏检，可尝试：
   - `--voice-vad-threshold 0.2 --voice-vad-hangover-ms 1200`
   - `--voice-no-vad` 彻底关闭 VAD
5. **模型链路问题**：用 `tools/voice_diag.py --inject-wav ...` 注入测试 wav，验证 KWS 本身能识别。
6. **实时诊断**：用 `tools/voice_live_kws_diag.py --duration 10` 可以直接看到每 0.1 秒的 VAD 状态、解码情况和识别结果。

### 10.7 NPU 推理慢或报 "Infer Request is busy"

NPU 通常一次只能处理一个请求。如果多个线程同时调用同一个 compiled model，需要加锁（`npu_lock`），或者使用 `ov.AsyncInferQueue` 管理异步请求。`camera_demo_pingpong_async_pnp.py` 已实现 body 的异步请求池，可参考。

### 10.8 GPU body 推理成为瓶颈

在 Arrow Lake-U 上，`yolov8s-pose` 在 GPU 上约 25-35ms。若需要更高帧率，可：

- 改用更小的模型（`yolov8n-pose` 或 `yolo26n-pose`）
- 使用 `camera_demo_pingpong.py` 或 `camera_demo_pingpong_async_pnp.py` 重叠预处理与推理
- 降低输入分辨率（默认 `BODY_INPUT_SIZE=640`）

---

## 11. 作者/备注

- 本项目为实验性 demo，主要验证 Intel NPU 上 `sherpa-onnx` KWS 的可行性。
- 当前默认关键词均为中文；如重新加入英文关键词，CPU INT8 路径通常更稳。
- VAD（Silero）始终运行在 CPU 上，只有 KWS 模型运行在 NPU。
- `elf_control_chain.py` 目前默认使用 stub 后端，真实硬件后端需根据实际机器人平台启用。

上游 `elf_info/wearable-robot-arm` 仍在持续更新，详细移植计划见 [`docs/PORTING_GUIDE.md`](docs/PORTING_GUIDE.md)。

## 与上游 Base 的对应关系

本项目是上游 C++ 项目 `/home/time/work/elf_info/wearable-robot-arm` 向 Python/OpenVINO 的移植版。由于平台差异（RK3588 → Intel Core Ultra 5 225U）和新增能力（手势、语音、STM32 桥接），各入口的职责与 Base 单一 `main.cpp` 不完全相同：

| 入口文件 | 对应 Base 模块 | 能力范围 |
|---|---|---|
| `camera_demo_pingpong_async_pnp.py` | `src/main.cpp` 中的视觉+控制部分 | 异步 body 推理 + 人脸/手部/PnP/规则引擎 + ELF 控制链（默认启用） |
| `demos/camera_demo_lowlatency.py` | `src/main.cpp` 中的 RTMP + WebSocket 部分 | 低延迟 ffmpeg RTMP 推流 + 云端 WebSocket 心跳 |
| `camera_demo_elf_pipeline.py` | `src/rga_npu.cpp` 视觉 pipeline | 完整视觉 pipeline + RTMP 推流（无 WebSocket） |
| `elf_control_chain.py` | `src/rga_npu.cpp::nrf24_control_update()` + `src/uart_comm.cpp` + `src/ctrl_server.cpp` | 机械臂控制算法、UART 输出、HTTP 控制接口 |

完整移植审计报告与整改指南见 [`docs/PORTING_AUDIT_REPORT.md`](docs/PORTING_AUDIT_REPORT.md)。

