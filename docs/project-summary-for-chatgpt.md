# fourth 项目摘要（供新会话/新开发者）

> 这是可复制给新协作者的短摘要；完整交接以 `HANDOFF.md` 为准。

`fourth` 是 RK3588 可穿戴机械臂上位机，当前位于 `migration/trial0-rk3588`，Git 基线 `025f3b3`。它以原 `twice/imu-victor-hat` 的 C++ RKNN/RGA、NRF24/IMU2、UART、推流和 FACE/BODY/INTRO/INTERVIEW 为底座，选择性吸收 `trial0-python-control-20260710` 的统一控制、FIRST_PERSON、云命令和回放；手部能力参考 `third`。当前迁移改动尚未正式提交。

核心边界：本机已有传感器采集和 STM32H723 通信，不移植额外 STM32/F103 bridge，不用 trial0 的 Python 串口主控或 Intel/OpenVINO 视觉。所有最终机械命令由 `uart_comm` 发送；REST、云端、蓝牙以及未来语音/手势应经过 `control_router`。

视觉主链为 Body YOLO-Pose → 可选 Face 468/PnP → OSD → RTMP/RTSP，实际约 13～15 FPS。手部旁路用 COCO 腕肘和肩宽生成方形 ROI，经 RGA 变成 224×224，异步交给 Python RKNN sidecar。固定每 2 个视觉帧采样一轮，同轮可处理左右手；右上角分开显示左右手。手势目前只显示，不切情景、不写 UART。

最近修复过严重 RGA 边界问题：右边缘 ROI 曾被缩成 `48×352`，触发 Invalid argument 和系统 Bus error。当前 ROI 必须方形、偶数坐标、16 对齐、完全在帧内，边缘通过整体平移处理。

FIRST_PERSON 固定空间目标，头姿只映射 J4/J5；纯逻辑测试已存在，但实机极性、限位和退出回中尚未验收。云端下行解析已接入，但缺真实抓包；annotation 只有统一状态，未贯穿全部 OSD。语音以后使用外接模块输出语义命令。更多情景模式要逐个状态机迁移和验收。

下一步优先级：审查并拆分提交当前工作树；完成双手+人脸+人体+推流长测；低速验收 FIRST_PERSON；抓真实云包；完成 annotation；再设计手势决策与语音接入。不要把约 8 ms 单手推理误写成完整视觉时延，也不要把视觉 15 FPS 等同 UART 15 Hz。

首读文件：`README.md`、`docs/HANDOFF.md`、`docs/MIGRATION.md`、`docs/VISION.md`、`docs/CONTROL.md`。
