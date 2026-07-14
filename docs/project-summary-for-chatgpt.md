# fourth 项目摘要（供新会话/新开发者）

> 这是可复制给新协作者的短摘要；完整交接以 `HANDOFF.md` 为准。

`fourth` 是 RK3588 可穿戴机械臂上位机，当前位于已推送 GitHub 的 `no-arm-test`；历史基线为 `025f3b3`。它保留 RK3588 C++ RKNN/RGA、NRF24/IMU2、UART 和推流底座，吸收 trial0 的统一控制、FIRST_PERSON、12 词 KWS 与展开/收起语义，并参考 third 的手部能力。

核心边界：不移植额外 STM32/F103 bridge，不用 trial0 的 Python 串口主控或 Intel/OpenVINO 视觉。所有机械命令由 `uart_comm` 串行化；REST、云端、蓝牙、语音和手势经过统一控制层。`no-arm-test` 不是硬件锁，运行时必须物理隔离未验收机械臂。

视觉主链为 Body YOLO-Pose → 可选 Face 468/PnP → OSD → RTMP/RTSP。Hand 每 3 个视觉帧异步采样一轮，同轮可处理左右手；连续 3 次确认后可切情景/profile，`ILoveYou` 承担 OK/收起确认语义。interval=3+KWS 阶段平均约 16.25 FPS，短窗口仍可降至 12 FPS。

最近修复过严重 RGA 边界问题：右边缘 ROI 曾被缩成 `48×352`，触发 Invalid argument 和系统 Bus error。当前 ROI 必须方形、偶数坐标、16 对齐、完全在帧内，边缘通过整体平移处理。

FIRST_PERSON 固定空间目标，头姿只映射 J4/J5；实机极性、限位和退出回中尚未验收。云端缺真实抓包。KWS 已接 USB 麦克风，但 5 分钟产生 32 次事件，当前不得开放语音开关机。正常 SIGINT/SIGTERM 的进程、端口和 socket 清理已复验；SIGKILL、内核崩溃或断电仍需依赖父进程死亡信号和下次启动清理。

下一步优先级：隔离语音危险动作并标定 KWS；做 20～30 分钟热稳态；补初始化失败与 SIGKILL 后重启清理测试；低速验收 FIRST_PERSON/展开收起；抓真实云包；补 annotation 自动测试。不要把 Hand 单次推理或视觉 FPS 等同 UART 发令率。

首读文件：`README.md`、`docs/HANDOFF.md`、`docs/MIGRATION.md`、`docs/VISION.md`、`docs/CONTROL.md`。
