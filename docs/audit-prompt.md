# fourth 当前代码审计 Prompt

> 本文件用于把项目交给新的审计者。审计必须以当前源码为准，不得把 `scenario-intro-mode/` 历史快照当主工程。

## 角色与目标

你是一名 RK3588 视觉、嵌入式控制和并发安全审计者。请在不修改文件的前提下审查 `fourth`，重点回答：迁移是否保持单一硬件事实源、视觉热路径是否会被手势阻塞、多来源控制是否可能争抢 UART，以及当前测试能否支撑实机安全。

## 必读顺序

1. `README.md`
2. `docs/HANDOFF.md`
3. `docs/MIGRATION.md`
4. `docs/PROJECT.md`
5. `docs/VISION.md`
6. `docs/CONTROL.md`
7. `docs/DEVELOPMENT.md`
8. `simulation/README.md`

然后阅读以下源码：

- 生命周期：`src/main.cpp`
- 视觉/控制核心：`src/rga_npu.cpp`, `src/rga_npu.h`
- 手部链路：`src/gesture_overlay.cpp/h`, `src/gesture_control.cpp/h`, `scripts/hand_pipeline_server.py`
- RuleEngine：`scripts/rule_engine_server.py`
- 控制路由：`src/control_router.cpp/h`, `src/cloud_command.cpp/h`
- FIRST_PERSON：`src/first_person_control.cpp/h`
- 外部入口：`src/ctrl_server.cpp`, `src/ws_client.cpp`, `src/bluetooth_spp.c`
- 设备出口：`src/uart_comm.cpp/h`
- 传感器：`src/nrf24_linux.c/h`, `src/imu2_i2c.c/h`
- 推流：`src/gst_rtmp.cpp`, `src/gst_rtsp.cpp`, `src/stream_manager.cpp`
- 测试：`tests/`, `simulation/control_replay.py`

## 必查问题

### 迁移边界

- 是否仍以本机 RKNN/RGA、NRF24/IMU2、H7 UART 为事实源？
- 是否意外引入第二套串口主控或 bridge？
- trial0 云命令范围、缺省值和错误包是否安全？

### 视觉与手势

- `process_frame()` 热路径是否等待 Python 推理？
- 每 2 帧提交、双手两个槽位和“最新帧覆盖”是否存在线程竞态？
- pending/work buffer 生命周期是否安全？
- 左右手结果是否会串侧、陈旧或误绘？
- NV12 ROI 是否始终偶数、方形、16 对齐、完整在帧内？
- sidecar 断连、超时或异常返回会不会拖死推流？
- 约 8 ms 的口径是否被误用为完整系统性能？

### 控制与 UART

- 所有来源是否经过统一控制语义？
- 所有物理发送是否由 `uart_comm` 串行化？
- 模式切换、退出 FACE 回中、握手阻塞和 A-init 是否有竞态？
- FIRST_PERSON 的限位、死区和 150 ms 限频是否一致？
- 视觉 15 FPS 是否被错误等同为 UART 15 Hz？
- 手势举手门槛、连续 3 次确认、1.5 秒冷却、模式白名单和 generation 防陈旧结果是否正确？
- 手势是否始终经过 `control_router`，且没有直接写 UART？

### 可靠性

- fork 的 Python 子进程能否被可靠停止和回收？
- 部分初始化失败后的 free/close/join 是否完整？
- 网络、摄像头、NPU、RGA、NRF24、IMU2、蓝牙任一失效时是否安全降级？
- `/tmp` socket/状态文件是否可能残留并影响重启？

## 验证命令

先运行只读检查和无硬件测试：

```bash
git status --short
git diff --check
python3 -m unittest tests.test_control_replay
python3 simulation/control_replay.py simulation/example_first_person.jsonl
python3 -m py_compile scripts/rule_engine_server.py scripts/hand_pipeline_server.py
```

完整构建使用根 `README.md` 的命令。不要为了审计自动启动 `build/cc`，因为它会初始化硬件并可能发送机械臂指令。

## 输出格式

1. 先列 findings，按 critical/high/medium/low 排序；每项给文件和行号、触发条件、影响和建议。
2. 再列“已验证事实”和“尚未实机验证”，不要混写。
3. 单独给出并发/缓冲区所有权表。
4. 单独给出控制来源到 UART 的调用路径。
5. 最后给出最小修复顺序与建议补充的测试。
