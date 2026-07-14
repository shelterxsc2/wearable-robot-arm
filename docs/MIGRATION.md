# fourth 移植来源、进展与方向

> 迁移日期：2026-07-13；当前分支 `migration/trial0-rk3588`。本文描述选择性移植，不表示要把来源项目整体合并。

## 1. 为什么建立 fourth

原 RK3588 工程已经具备本机视觉、传感器采集和 STM32H723 通信；trial0 则包含更完整的控制语义、第一人称和未来多模态方向。`fourth` 的目标是保留已稳定的设备链，同时吸收可复用的上层控制能力。

最终原则是：

- 本机硬件事实源只有现有 NRF24、IMU2、摄像头和 H7 UART；
- RKNN/RGA C++ 热路径保持主导，不切回 Intel/OpenVINO/Python 主视觉；
- 新输入统一进入 control router，不产生多个串口拥有者；
- 云端 REST/WebSocket 语义可以共享，但端侧仍做模式转换与安全限制。

## 2. 来源 A：twice / imu-victor-hat

当前 Git 历史基线为 `025f3b3 Add interview mode and Bluetooth scene switching`。从该基线继承：

- `main.cpp` 生命周期、Wi-Fi、蓝牙、UART、NRF24、IMU2；
- RK3588 RKNN/RGA Body、Face 468、PnP；
- RTMP/RTSP 和 WebSocket 上报；
- FACE、BODY、INTRO、INTERVIEW；
- 球坐标控制、状态机、预测器、标定和 H7 11 字节协议；
- RuleEngine v2。

这些不是“待迁内容”，而是 `fourth` 的底座。

## 3. 来源 B：trial0-python-control-20260710

追溯点：来源分支 `trial0-python-control-20260710`，当时 HEAD `b9b9feb Update trial0 control and streaming runtime`。

### 已吸收

- 多来源统一控制语义：`control_router.cpp/h`；
- `FIRST_PERSON` 模式和独立 J4/J5 映射；
- profile、第一人称固定目标、annotation 统一状态；
- 云端命令适配：`set_target`、`target_pose`、`set_zoom`、`set_view_mode`、`set_annotation_mode`、`track_obj`；
- REST 新接口与蓝牙 FIRST_PERSON 值；
- 无硬件控制回放与 FIRST_PERSON 单元测试。

### 没有吸收

- Intel/OpenVINO 视觉实现；
- trial0 Python 串口主控；
- F103/额外 STM32 bridge；
- 独立传感器采集副本；
- 尚未逐项验收的所有情景模式；
- 本机语音识别实现。

未吸收不是遗漏：本机已经完成传感器与机械臂通信，重复 bridge 会造成坐标系、握手和 UART 双写竞争。语音后期使用外接模块，只需把语义事件接入统一控制路由。

## 4. 来源 C：third 手部实现

追溯点：参考目录当时分支 `imu-3states-hat`、HEAD `1dfbf36`；手部迁移还参考了该目录当时未完全提交到上述 HEAD 的工作树实现，因此 `fourth` 当前源码和本文几何说明才是最终事实。

### 已吸收

- `models/hand/` 中 detector、landmark、embedder、canned classifier；
- `scripts/hand_pipeline_server.py` 的 RKNN socket 推理；
- 腕肘与肩宽构造手部 ROI 的几何策略；
- 21 点与手势分类结果；
- C++ 异步 OSD worker。

### 当前与 third 的差异

`fourth` 的生产热路径已有 Body 腕点，因此直接裁腕部 ROI 后运行 landmark/classifier，不对整帧同步运行 palm detector。这样避免阻塞约 13～15 FPS 的主视觉链。palm detector 模型保留，可用于离线或未来低频兜底研究。

旧 `gesture_classifier_fp16.rknn` 因已知输出异常没有采用，当前使用 canned classifier 组合。

## 5. 当前迁移进度

| 能力 | 代码 | 测试/验证 | 状态 |
|---|---|---|---|
| 统一模式/profile/target/annotation | 已实现 | 构建通过，部分回放 | 需多来源冲突测试。|
| FIRST_PERSON 映射 | 已实现 | C++ 单元测试、Python 回放 | 需低速实机验收。|
| 云端下行适配 | 已实现 | 静态/构建验证 | 缺真实服务器抓包。|
| REST 扩展 | 已实现 | 构建验证 | 缺接口回归集合。|
| 蓝牙 FIRST_PERSON | 已实现 | 构建验证 | 缺现场遥控验收。|
| 手部腕肘 ROI | 已实现 | 单手阶段性运行 | 边缘崩溃已修；需四边/双手长测。|
| 左右手独立 OSD | 已实现 | 构建验证 | 需 VLC 实画面确认。|
| 每 3 帧手势采样 | 已实现 | 5 分钟 KWS 联测 | 需继续验证响应速度和长时稳定性。|
| 手势切情景 | 已实现 | 纯决策测试、构建通过 | 需准确率、误触发和低速实机验收。|
| 语音 | 未实现 | 不适用 | 等外接模块协议。|
| 更多情景模式 | 未实现 | 不适用 | 逐个迁移、逐个验收。|
| STM32 bridge | 不移植 | 不适用 | 本机现有链已覆盖。|

## 6. 手势当前细节

- 服务：`/tmp/hand_pipeline.sock`；
- 输入：224×224 RGB 手部 ROI；
- 采样：固定每 3 个已处理视觉帧一轮；
- 触发：腕点必须高于同侧肩点；BODY/FIRST_PERSON 不裁手；
- 双手：同轮最多提交左右两只手，有界最新帧队列；
- 显示：右上角左右手独立行、ROI 和 21 点；
- OSD 稳定条件：score ≥ 0.70 且连续 3 次一致；
- 控制条件：score ≥ 0.70、同侧连续 3 个有效结果、1.5 秒冷却；
- 映射：`Open_Palm/Closed_Fist/Pointing_Up/ILoveYou` 分别控制情景、profile 和返回 FACE；
- 控制经过 router，手势模块不直接调用 UART。

此前边缘框曾变成 `48×352` 并触发 RGA/系统 Bus error。当前必须保持 ROI 方形、偶数坐标、16 对齐和完整帧内边界。

## 7. 仿真迁移

`simulation/control_replay.py` 用 JSONL 回放模式、目标和头姿，验证 FIRST_PERSON 语义目标；原 S-curve 对比脚本继续保留。仿真不覆盖 RKNN、摄像头、传感器噪声、UART 字节时序、机械动力学和碰撞。

## 8. 后续迁移方向

### 第一阶段：收口当前迁移

- 拆分并提交当前工作树；
- 双手+人脸+人体+推流长时间性能测试；
- FIRST_PERSON/云端/蓝牙实机验收；
- annotation 全链路完成。

### 第二阶段：扩展情景

- 为每个 trial0 情景建立输入、状态、退出条件和安全动作；
- 先做无硬件回放，再做机械臂低速验收；
- 禁止一个提交同时加入多个未验收情景。

### 第三阶段：多模态输入

- 外接语音模块输出语义命令；
- 手势完成准确率和误触发评估后增加独立决策层；
- control router 增加来源优先级、租约、冷却和急停规则。

### 第四阶段：控制质量

- PnP 连续多帧一致性和稳定校正；
- 下位机反馈与可观测闭环；
- 参数热加载、统一遥测和真实日志回放；
- 若确有需要，再建设更完整的动力学/碰撞仿真。
