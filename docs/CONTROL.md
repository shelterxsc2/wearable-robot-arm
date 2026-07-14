# 控制架构与安全边界

> 当前实现说明，更新于 2026-07-14；分支 `no-arm-test`，机械动作未验收。

## 1. 两个不同的频率

视觉观察阶段实测平均约 16 FPS，但短窗口可降至 12 FPS；NRF24 控制定时器为 50 ms；UART 命令还受模式死区、运动状态和最小周期限制。三者不能混为同一个“控制 FPS”。

## 2. 控制来源

`ControlSource` 已定义 REST、cloud、Bluetooth、gesture、voice、local。所有语义入口应调用 `control_router`：

- 模式切换；
- profile 切换；
- FIRST_PERSON 固定空间目标；
- annotation 状态。

当前路由主要统一接口和日志，还不是完整优先级仲裁器。多个来源同时下命令时的租约、抢占和超时策略仍需实现。

## 3. 模式

| 模式 | 主要输入 | 当前动作 |
|---|---|---|
| FACE | NRF24/IMU2 + Face PnP | 头控空间目标和视觉零飘修正。|
| BODY | Body pose | 人体视觉/跟踪基线。|
| INTRO | Body 右腕与躯干 | 介绍构图状态机，暂停普通 NRF 头控。|
| INTERVIEW | 多人头部/单人腕点 | 采访构图状态机，暂停普通 NRF 头控。|
| FIRST_PERSON | 头部 yaw/pitch | 固定 x/y/z，只调 J4/J5。|

从 INTRO、INTERVIEW、FIRST_PERSON 回 FACE 时存在回中与重新捕获头部中心逻辑，实机改动必须重点测试这一转换。

## 4. UART 所有权

STM32 目标帧为五个 int16 小端值加 flag，共 11 字节。`flag=0x00` 表示预测目标，`0x01` 表示确定/静止目标。

`uart_comm` 是唯一物理发送层，并通过互斥避免多个线程字节交叉。control router 不直接替代 UART；它解决“谁提出什么语义命令”，UART 层解决“如何安全串行发送”。

禁止：

- 语音、手势或云端模块自行打开串口；
- 加入第二套 Python 串口主控；
- 移植额外 STM32/F103 bridge；
- 绕过握手阻塞、限位和模式切换安全动作。

## 5. FIRST_PERSON

- 默认空间目标 `x=-20, y=30, z=20 cm`；
- J4 中心约 10°，J5 中心约 180°；
- 纯映射没有输入中心死区；运行时相邻 J4/J5 目标变化不足约 2°时不发新指令；
- 输出限制 J4 `[-90,90]`、J5 `[0,270]`；
- 运行时变化至少约 2°且距离上次发令至少 150 ms 才发送。

纯映射测试只验证公式和限幅，不验证实际机械极性和碰撞。

## 6. 接口概览

- `GET /status`
- `POST /mode?type=face|body|intro|interview|first_person`
- `POST /profile?id=0|1`
- `POST /target?x=...&y=...&z=...`
- `POST /annotation?enabled=true|false`
- `POST /calib?mode=0..5`
- `POST /servo?k1=...&k2=...`
- `POST /cmd?action=rebaseline|head_center|nrf24_reset`

云端适配类型见 `cloud_command.cpp`。参数范围和响应格式最终以 `ctrl_server.cpp` / `cloud_command.cpp` 为准。

## 7. 手势和语音的接入方向

手势当前控制流程：

```text
per-side stable gesture
  → raised-hand gate
  → 3-result confirmation + 1.5 s cooldown
  → gesture decision policy
  → control_router
  → existing mode safety transition
  → uart_comm
```

当前映射：`Open_Palm → INTRO`、`Closed_Fist → INTERVIEW`、左/右
`Pointing_Up → profile 0/1`、`ILoveYou → 收起确认（若无待确认则 FACE）`。INTRO/INTERVIEW 中只接受
`ILoveYou` 返回 FACE；BODY/FIRST_PERSON 不裁手、不接受手势命令。手势线程不直接写
UART，异步结果还携带模式 generation，旧模式结果会被丢弃。

与 trial0 一致，`rule_engine_v2.rknn` 在 FACE、INTRO、INTERVIEW 中持续更新 7 维
反馈状态。模型状态边沿 `0→1` 请求 INTRO，`0→2` 请求 INTERVIEW；`1/2→0`
不自动返回 FACE，退出仍必须使用连续 3 个有效 `ILoveYou` 结果。RuleEngine 请求同样
经过 `control_router` 和 mode generation 校验，推理期间产生的旧结果不会覆盖较新的
REST、云端、蓝牙或手势命令。

目前仍没有来源租约和急停仲裁，也没有“释放后才能再次触发”的状态；这些是实机开放前的剩余风险。

语音 KWS 已通过 `/tmp/voice_kws.sock` 输出 JSON Lines 事件。关键词语义与 trial0 一致：`原画/标注` 控制 OSD，`拉近/拉远` 控制 profile，`介绍/采访/正面/并肩` 控制模式，`录制/停止` 请求云端录像，`开机/关机` 进入展开/收起状态机。语音层不直接写普通目标帧。

机械臂默认保持收起且 NRF 目标发送被阻塞。`开机` 发送 `FF AA` 展开帧，等待 7 秒归位，重置并重新采集 A-init 后才开放 NRF/UART 目标发送。`关机` 仅在 FACE 且已展开时有效：先切换远距 profile、发送安全 FACE 位姿并冻结 NRF，随后要求在 3 秒内连续 3 次识别 `ILoveYou`；当前模型的 `ILoveYou` 对应 trial0 的 OK 确认语义。超时则取消收起并恢复控制，确认成功才发送 `AA FF` 收起帧。

`no-arm-test` 只是分支名称，不会阻止串口初始化或语义命令进入上述状态机。5 分钟 KWS 测试产生 32 次事件，含 `开机/关机`；增加唤醒/二次确认并完成噪声标定前，必须物理隔离机械臂或禁用语音开关机。

## 8. 待补控制能力

- 来源优先级、租约和命令超时；
- 云端真实协议回放和异常包测试；
- annotation 各模式自动回归与云端一致性；
- KWS 阈值、唤醒/二次确认和危险词独立授权；
- `/servo` 与周期标定循环闭合；
- PnP 多帧一致性与稳定积分；
- 关节角/末端位姿反馈（若要形成完整闭环）；
- 情景模式逐个验收与退出安全动作。
