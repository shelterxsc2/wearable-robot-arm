# fourth：RK3588 可穿戴机械臂上位机

> 交接快照：2026-07-14
> 当前分支：`no-arm-test`
> Git 基线：`025f3b3`（原 `imu-victor-hat`）
> 当前提交：`e995dc8` 起的无机械臂性能测试分支；代码已推送 GitHub，但机械动作尚未验收。

`fourth` 是当前可继续开发的独立目录。后续开发者不需要读取相邻目录或历史对话；外部项目名只用于说明代码来源，当前行为以本目录源码为准。

## 先读什么

1. `docs/HANDOFF.md`：当前进度、风险、验证状态和下一步。
2. `docs/MIGRATION.md`：从哪些项目迁了什么、刻意没迁什么。
3. `docs/PROJECT.md`：系统架构、模块边界和运行时数据流。
4. `docs/VISION.md`：人体、人脸、PnP、手部 ROI 与推流热路径。
5. `docs/CONTROL.md`：控制来源、模式、UART 所有权和安全约束。
6. `simulation/README.md`：无硬件测试的能力边界与命令。

若文档描述冲突，优先级是：当前源码 > `docs/HANDOFF.md` > `docs/PROJECT.md` / `docs/MIGRATION.md` > 专题设计与历史记录。

## 项目边界

本机 RK3588 负责：

- USB 摄像头取流、RKNN/RGA 视觉推理和 OSD；
- NRF24 头部 IMU、板载 IMU2 数据采集；
- REST、云端 WebSocket、蓝牙等控制入口的统一路由；
- 通过 `/dev/ttyS9` 与 STM32H723 机械臂通信；
- RTMP/RTSP 推流。

STM32H723 已负责逆运动学、S-curve 和电机/舵机执行。本项目不需要 trial0 的额外 STM32/F103 桥接层，也不允许多个模块绕过 `uart_comm` 直接争抢串口。

## 当前功能状态

| 模块 | 状态 | 说明 |
|---|---|---|
| FACE/BODY 视觉 | 已有、在用 | Body YOLO-Pose；FACE 叠加 FaceMesh/PnP。|
| INTRO/INTERVIEW | 基线已合入 | 可切换，仍需按实际场景继续验收和调参。|
| FIRST_PERSON | 已迁入 | 固定空间目标，头姿映射 J4/J5；需低速实机验收。|
| REST/蓝牙/云端 | 已统一入口 | 统一经过 `control_router`；云端协议仍需真实抓包核对。|
| 手部识别 | 已接情景控制、待实机验收 | 举手后裁 ROI；连续确认后经 `control_router` 切模式/profile，不直接写 UART。|
| 语音 | trial0 语义已迁入、待实机验收 | 12 个关键词；模式/profile/标注经过路由，开关机使用安全展开/收起状态机。|
| 仿真 | 部分可用 | 控制回放和旧/新 S-curve 对比；不代表完整机械动力学。|

## 视觉与手势当前节拍

- 摄像头声明 1920×1080@30 FPS；阶段实测平均约 16 FPS、5 秒窗口 P05/P50/P95 为 12/18/18，不能承诺恒定 15 FPS。
- 视觉控制观察值在每个已处理视觉帧更新；机械臂 UART 发令另有死区和时间限频，不能把“视觉 15 FPS”误写成“UART 15 Hz”。
- 手部旁路固定每 3 个视觉帧采样一次；一个采样帧若同时有左右手，会把两只手都交给异步 worker。
- 只有腕点高于同侧肩点时才产生手部 ROI；BODY/FIRST_PERSON 不运行手势裁剪。
- 以视觉 15 FPS 估算，手势约 5 轮/秒；连续 3 次确认约需 0.6 秒。
- 右上角分别显示 `Left Hand`、`Right Hand`。这里的左右是被拍摄者的 COCO 语义左右，不是屏幕左右。
- 控制要求 score ≥ 0.70、同侧连续 3 次一致，并有 1.5 秒全局冷却；`ILoveYou` 承担 trial0 的 `OK` 语义，优先确认待处理收起，否则返回 FACE。

## 构建

```bash
cd /home/elf/work/fourth
mkdir -p build
g++ -std=c++17 -O2 \
  src/main.cpp src/rga_npu.cpp src/gst_rtsp.cpp src/gst_rtmp.cpp \
  src/stream_manager.cpp src/gst_unified.cpp src/ctrl_server.cpp src/ws_client.cpp \
  src/control_router.cpp src/cloud_command.cpp src/cloud_report.cpp \
  src/arm_power_control.cpp src/first_person_control.cpp \
  src/gesture_control.cpp src/rule_mode_control.cpp src/gesture_overlay.cpp \
  src/voice_control.cpp \
  src/uart_comm.cpp src/wifi.cpp \
  src/nrf24_linux.c src/imu2_i2c.c src/bluetooth_spp.c \
  -o build/cc \
  $(pkg-config --cflags --libs gstreamer-1.0 gstreamer-app-1.0 \
    gstreamer-rtsp-server-1.0 dbus-1) \
  -I/usr/include/opencv4 \
  -lopencv_core -lopencv_imgproc -lopencv_calib3d \
  -lrknnrt -lrga -lwpa_client -lpthread
```

依赖包括 GStreamer、OpenCV、Rockchip RKNN Runtime、librga、DBus、wpa_supplicant client，以及 Python 的 `rknnlite`/`rknn-toolkit-lite2`。Hand/KWS 模型和 Sherpa ARM64 Python runtime 已提交；根目录 Body/Face/Rule RKNN 受 `.gitignore` 管理，克隆后需单独准备。

## 测试

无硬件安全测试：

```bash
python3 -m unittest tests.test_control_replay
python3 simulation/control_replay.py simulation/example_first_person.jsonl
g++ -std=c++17 -Isrc tests/first_person_control_test.cpp \
  src/first_person_control.cpp -o /tmp/first_person_control_test
/tmp/first_person_control_test
g++ -std=c++17 -Isrc tests/gesture_control_test.cpp \
  src/gesture_control.cpp -o /tmp/gesture_control_test
/tmp/gesture_control_test
g++ -std=c++17 -Isrc tests/rule_mode_control_test.cpp \
  src/rule_mode_control.cpp -o /tmp/rule_mode_control_test
/tmp/rule_mode_control_test
python3 -m py_compile scripts/rule_engine_server.py scripts/hand_pipeline_server.py \
  scripts/voice_kws_server.py
```

主程序会接触网络、摄像头、NRF24、蓝牙和 UART。实机运行前先确认机械臂活动空间、急停方式、串口设备和当前模式：

```bash
sudo ./build/cc
```

主程序自动启动 `rule_engine_server.py`、`hand_pipeline_server.py --mode roi_gesture --quiet`
和可降级的 `voice_kws_server.py`。语音 sidecar 或麦克风失败不会终止视觉与机械控制主链。

## 主要目录

```text
src/          C/C++ 主程序、视觉、控制、推流及硬件通信
scripts/      RuleEngine、HandPipeline、VoiceKWS sidecar 与测试脚本
models/       Body、Face、RuleEngine、Hand RKNN 模型
calib/        相机内参与畸变参数
tests/        无硬件单元/回放测试
simulation/   控制回放和 STM32 S-curve 仿真
docs/         当前交接文档、专题设计和历史记录
scenario-intro-mode/  旧 INTRO 快照，仅作历史对照，不是当前构建入口
```

## 关键安全约束

- 手势已能切换情景/profile，但尚未实机验收；运行前必须确认活动空间、退出回中和急停方式。
- 语音模块只能调用统一控制路由，不能直接持有 UART。
- VoiceKWS 为可降级 CPU sidecar，默认绑核 CPU 2、nice 10；退出不影响视觉主链。
- 5 分钟现场测试产生 32 次 KWS 事件（含开机/关机）；完成阈值标定、唤醒/二次确认前，不得连接可展开机械臂运行语音控制。
- `no-arm-test` 是分支/验收状态，不是编译期硬锁；主程序仍会打开 `/dev/ttyS9`，必须物理隔离机械臂或确认 H7 不可执行。
- 正常 SIGINT/SIGTERM 会回收主程序、三个 sidecar、VoiceKWS 的 `arecord`、8080/8554和三个 socket；SIGKILL、内核崩溃或断电无法执行完整清理，重启前仍应以 `pgrep`、`ss` 和 `/tmp/*.sock` 复核。
- 云端、REST、蓝牙、语音和手势共享接口语义，但最终设备命令必须经过端侧仲裁。
- 不新增 STM32 桥；本机已经采集传感器并与机械臂通信。
- 不要用仿真通过代替实机限位、极性、碰撞和通信时序验收。

详细未完成项与推荐开发顺序见 `docs/HANDOFF.md`。
