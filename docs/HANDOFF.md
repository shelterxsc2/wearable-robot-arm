# fourth 开发交接说明

> 状态日期：2026-07-13。本文是下一位开发者的首要入口；所有结论均可在 `fourth` 目录内验证。

## 1. 当前结论

`fourth` 已从原 RK3588 主控工程上建立迁移分支，并完成统一控制入口、第一人称模式、云端下行适配、异步手部识别、手势情景控制、CPU 语音 KWS 和基础无硬件回放。当前二进制可编译，但整个迁移工作树尚未形成正式 Git 提交，语音、手势控制和整机综合验收也未完成。

继续开发时不要重新从相邻项目复制整个目录。应以 `fourth` 为唯一工作树，仅在需要追溯算法来源时阅读本文件的来源说明。

## 2. 移植来源

| 来源 | 在 fourth 中承担的作用 | 已迁内容 | 未迁/禁止迁入 |
|---|---|---|---|
| `twice` 的 `imu-victor-hat`，Git 基线 `025f3b3` | RK3588 设备底座 | RKNN/RGA Body+Face、PnP、NRF24/IMU2、UART、RTMP/RTSP、REST、蓝牙、INTRO/INTERVIEW | 无需再次复制；它已是当前 Git 历史基线。|
| `trial0-python-control-20260710`，分支同名，参考提交 `b9b9feb` | 控制语义与扩展方向 | 统一控制路由思路、FIRST_PERSON、云端命令类型、目标/标注/profile 状态、无硬件回放 | Intel/OpenVINO 视觉、Python 串口主控、额外 STM32/F103 bridge、直接替换现有设备链。|
| `third`，分支 `imu-3states-hat`，参考提交 `1dfbf36` 加其当时工作树手部实现 | 手部视觉参考实现 | Hand RKNN 模型、Python sidecar、腕肘 ROI 几何、21 点、embedder+canned classifier | 不把手势直接接情景；不在生产热路径跑整帧 palm detector；未采用已知异常的旧 classifier。|

这些路径只记录历史来源。即使相邻目录以后删除，`fourth` 仍应能够构建、运行和继续开发。

## 3. 当前工作树与构建状态

- 分支：`migration/trial0-rk3588`。
- HEAD：`025f3b3`，与原 `imu-victor-hat` 基线相同。
- 迁移代码、模型、测试和文档目前位于未提交修改/未跟踪文件中。
- `build/cc` 已在 2026-07-13 重编译成功；编译有旧代码的 ignored-return/narrowing 警告，没有新增编译错误。
- sidecar 已改为并行启动并等待 socket 就绪；Ctrl+C/初始化失败/推流失败统一逆序清理，实测不残留主进程、sidecar、8080/8554 或 socket。
- Python sidecar 通过 `py_compile`；FIRST_PERSON C++ 纯逻辑和 Python 回放有测试，但实机综合测试不能由这些测试替代。

提交前必须执行 `git status --short`，逐项确认模型、大文件、备份和用户原有修改，禁止盲目 `git add -A`。

已知历史自包含例外：`simulation/jitter_sim.py` 仍含旧 `twice/simulation` 绝对导入，`scenario-intro-mode/src/main.cpp` 也含旧 sidecar 绝对路径。两者都不是当前运行入口；若要恢复使用，应先改成 `fourth` 内相对路径并补测试，而不是依赖相邻目录。

## 4. 已完成的迁移

### 4.1 控制入口统一

`src/control_router.cpp/h` 将 REST、云端、蓝牙以及未来的 gesture/voice/local 标记成明确来源。已统一的状态包括模式、臂长 profile、FIRST_PERSON 空间目标和 annotation 开关。

`src/uart_comm.cpp` 是机械臂物理发送出口，并带全局写互斥。新增入口不得自行打开 `/dev/ttyS9`。

### 4.2 FIRST_PERSON

- 模式枚举：`MODE_FIRST_PERSON`。
- 默认空间目标：`(-20, 30, 20) cm`，可由 `/target` 或云命令更新。
- 头部 yaw/pitch 只映射到 J5/J4。
- 映射纯函数位于 `first_person_control.cpp` 并带关节限幅；运行时比较相邻目标，J4/J5 变化不足 2° 时不发新指令。
- 运行时仍保留 150 ms 最小 UART 发令周期；这与视觉帧率是两件事。

### 4.3 云端与 REST

- WebSocket 下行已交给 `cloud_command.cpp`，识别 `set_target`、`target_pose`、`set_zoom`、`set_view_mode`、`set_annotation_mode`、`track_obj`。
- REST 支持 `/status`、`/mode`、`/profile`、`/target`、`/annotation`，并保留 `/calib`、`/servo`、`/cmd`。
- 蓝牙场景协议新增 FIRST_PERSON 值 `55 02 04`。

云端字段目前采用兼容性解析，真实服务器数据范围尚未抓包确认。尤其不要假定 `target` 永远是离散 `-5..5`。

### 4.4 手部异步识别与情景控制

- Body 的 COCO 腕 9/10、肘 7/8 用于定位左右手。
- ROI 沿前臂方向外移 35%，用肩宽修正有效前臂长度，生成 96～360 像素方形框。
- 边缘 ROI 通过整体平移保持方形、偶数坐标和 16 像素尺寸对齐；禁止再次把边缘框分别缩成不同宽高。
- RGA 裁剪/缩放/颜色转换任一步失败都会停止该 ROI，不把错误继续传给 worker。
- ROI 转成 224×224 RGB 后，通过 Unix socket `/tmp/hand_pipeline.sock` 交给 Python sidecar。
- 只有腕点高于同侧肩点时生成 ROI；BODY/FIRST_PERSON 不裁手。
- 固定每 3 个视觉帧采样一轮；一轮包含当前可见的左右手，两手结果分开保存和显示。
- 右上角显示 `Left Hand` / `Right Hand`，语义是被拍摄者左右手。
- 置信度至少 0.70、连续 3 次一致才显示 stable；控制也要求同侧连续 3 个有效结果并带 1.5 秒冷却。
- `Open_Palm → INTRO`、`Closed_Fist → INTERVIEW`、左右 `Pointing_Up → profile 0/1`；`ILoveYou` 承担 trial0 的 OK 语义，优先确认待处理的收起请求，否则返回 FACE。
- 展开/收起已按 trial0 状态机接入：默认收起；展开后等待 7 秒并重新 A-init；收起先发安全 FACE 位姿，再等待 3 秒手势确认。
- INTRO/INTERVIEW 中只接受 `ILoveYou` 退出；语义动作经过 `control_router`，手势线程不直接写 UART。
- ROI 任务记录模式 generation，模式切换前排队的旧结果不能覆盖新命令。

最近一次关键故障是右边缘 ROI 被缩成 `48×352`，导致 RGA `Invalid argument`，随后系统出现 Bus error。修复位于 `gesture_hand_rois()` 和 `gesture_overlay_submit_latest()`；后续改 ROI 必须保留边界不变量。

## 5. 尚未完成或只完成一半

| 项目 | 当前程度 | 完成标准 |
|---|---|---|
| FIRST_PERSON 实机验收 | 代码和纯逻辑测试完成 | 低速确认 J4/J5 极性、机械限位、退出 FACE 回中、连续运行稳定。|
| 手势接情景 | 代码已接、纯决策测试完成 | 实测准确率/误触发，验收举手门槛、连续确认、冷却、退出回中和急停。|
| 手势性能 | 单手推理曾约 8 ms | 测人体+人脸+双手+推流长时间 FPS、P95/P99、温度和内存。|
| 视觉 15 FPS 控制输入 | 架构上每处理帧更新 | 实机证明稳定 13～15 FPS；区分视觉更新率和 UART 发令率。|
| 云端协议 | 兼容解析已写 | 用真实下行抓包覆盖所有类型、范围、错误包和重连。|
| annotation | 状态已统一 | 让开关贯穿全部 OSD 分支并补测试。|
| 语音 KWS | trial0 12 词语义、云端上报和开关机已接入 | 验收麦克风、误触发、录像服务响应、事件延迟和视觉 A/B FPS。|
| 情景模式扩展 | INTRO/INTERVIEW 基线 | 逐模式状态机迁移、回放、实机验收；禁止一次性大合并。|
| 控制闭环 | 视觉修正为实验态 | 优化 PnP 多帧门槛/积分策略；若需要完整闭环，增加下位机反馈。|
| 仿真 | 语义和 S-curve 部分覆盖 | 增加真实云包、传感器噪声、串口时序；动力学/碰撞需独立方案。|

## 6. 推荐下一步顺序

1. 保存一份干净的当前迁移提交：先审查 diff，再分“控制迁移、手势旁路、文档”提交。
2. 做 10～30 分钟实机视觉压力测试：单人单手、双手、手到四边、无人、sidecar 重启、RTMP/RTSP 两种路径。
3. 记录每秒视觉 FPS、Body/Face/Hand 时延、队列覆盖次数、RGA 错误、CPU/NPU 温度；不能只看平均 8 ms。
4. 低速验收 FIRST_PERSON 和退出 FACE 的回中动作。
5. 抓取真实云端下行包，固定协议样例并增加回放测试。
6. 完成 annotation 绘制开关和 REST/云端状态一致性。
7. 验收手势情景映射并补来源租约/急停仲裁；语音和更多情景模式使用同一控制路由。

## 7. 开发不变量

- `fourth` 是事实来源；专题文档不覆盖当前源码事实。
- 摄像头热路径不得等待 Python 手势推理。
- 手部队列必须“最新帧优先”且有界，不能无限积压。
- NV12 ROI 坐标为偶数、完全在帧内、宽高相等、尺寸 16 对齐。
- 手势控制虽已编译接入，但在明确实机验收前不得用于无急停保护的机械动作。
- 所有机械臂写操作经过 `uart_comm`；统一路由负责来源语义，UART 层负责串行化。
- 不移植额外 STM32 桥，不建立第二套传感器采集事实源。
- 不把视觉更新 15 FPS 等同于电机命令 15 Hz。
- 修改控制限位、极性、发令频率前必须说明机械风险并做低速测试。

## 8. 常用验证

完整构建命令见根 `README.md`。无硬件测试：

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
python3 -m py_compile scripts/rule_engine_server.py scripts/hand_pipeline_server.py
git diff --check
```

实机运行的观察点：

- 启动应打印 `fixed interval=3`；
- 左右手日志包含 `side=Left` / `side=Right`；
- 画面右上角有两行独立结果；
- 四个边缘都不出现非方形 ROI 或 RGA Invalid argument；
- 手势 sidecar 断开时视觉和控制主链继续运行；
- 举手并连续识别 3 次后应出现 `[GestureCmd]` 和对应 `[ControlRouter]`；非举手、BODY/FIRST_PERSON、低置信度及旧 generation 结果不得触发。

## 9. 重要文件

| 文件 | 作用 |
|---|---|
| `src/main.cpp` | 生命周期、sidecar、硬件和推流启动。|
| `src/rga_npu.cpp/h` | Body/Face/PnP、情景控制、NRF 控制和手部 ROI 产生。|
| `src/gesture_overlay.cpp/h` | 手部异步队列、RGA 预处理、sidecar 通信、OSD 和控制接入。|
| `src/gesture_control.cpp/h` | 连续确认、冷却、模式限制和标签映射纯逻辑。|
| `scripts/hand_pipeline_server.py` | Hand landmark/embedder/classifier socket 服务。|
| `src/control_router.cpp/h` | 多来源统一控制入口。|
| `src/cloud_command.cpp/h` | trial0 风格云端 JSON 适配。|
| `src/first_person_control.cpp/h` | FIRST_PERSON J4/J5 纯映射。|
| `src/ctrl_server.cpp` | REST API。|
| `src/uart_comm.cpp` | 唯一机械臂串口发送层。|
| `simulation/control_replay.py` | 无硬件控制语义回放。|
