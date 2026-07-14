# simulation：无硬件回放与 S-curve 研究

> 当前状态：`no-arm-test` 的部分无硬件覆盖。这里的“通过”不能证明机械臂实机安全，也不能证明视觉帧率或 KWS 安全。

## 1. 两类仿真

### 控制语义回放

`control_replay.py` 读取 JSONL 事件，模拟模式、FIRST_PERSON 固定目标和头姿到 J4/J5 的结果。它用于验证从 trial0 吸收的控制语义没有破坏基础映射。

```bash
python3 simulation/control_replay.py simulation/example_first_person.jsonl
python3 -m unittest tests.test_control_replay
```

### STM32 S-curve 对比

`stm32_scurve_sim.py` 按 1 ms 周期复现旧/mid/current 三种规划逻辑，用于研究新目标打断、制动和短距离行为。相关 `analyze_*`、`compare_*`、`check_*` 和图片是历史分析工具/产物。

重要：部分 VOFA 数据来自旧版或次新版下位机，而不是当前 H7 固件。它们只能做趋势对比，不能作为当前固件的定量拟合依据。

## 2. 文件索引

| 文件 | 作用 | 当前建议 |
|---|---|---|
| `control_replay.py` | FIRST_PERSON/模式语义回放 | 当前回归入口。|
| `example_first_person.jsonl` | 示例事件 | 可扩充云端/蓝牙样例。|
| `stm32_scurve_sim.py` | old/mid/current S-curve | 控制研究；需核对固件版本。|
| `vofa_parser.py` | 解析历史 VOFA 文本 | 历史数据工具。|
| `compare_versions*.py` | 多版本绘图比较 | 研究脚本，部分参数可能过期。|
| `analyze_*`, `check_*` | 针对具体日志的分析 | 使用前阅读脚本输入假设。|
| `jitter_sim.py` | 抖动/死区研究 | 概念验证。|

`jitter_sim.py` 目前仍硬编码旧 `/home/elf/work/twice/simulation` 导入路径，因此不满足独立运行要求；它被保留为历史研究脚本。继续使用前应改为本目录相对导入并增加测试。`scenario-intro-mode/` 中的旧绝对路径同理，但该目录已归档。

## 3. S-curve 示例

```python
from stm32_scurve_sim import simulate_events

events = [
    (0.0, 1.50, 0x01),
    (6.0, 1.52, 0x01),
]
result = simulate_events(events, version="current", dt=0.001)
```

运行前应在实验记录中注明：使用的上位机提交、H7 固件提交、版本参数和输入日志来源。

## 4. 明确不覆盖

- 摄像头、RGA、RKNN 和推流 FPS；
- NRF24/IMU2 噪声、丢包和姿态标定；
- UART 字节级时序与 H7 实际握手；
- 电机/减速器真实动力学、柔性、负载和碰撞；
- 云端网络延迟、乱序和重连；
- 手势误识别和多来源控制冲突。

## 5. 推荐扩展

1. 保存真实云端下行 JSONL 并加入 parser 回放。
2. 给 control router 增加来源冲突、租约和超时测试。
3. 加入手势 stable/release/cooldown 的纯状态机测试后，再考虑接控制。
4. 将当前 H7 固件参数以版本化配置导入 S-curve 仿真。
5. 如要研究安全空间，另建带关节限位和碰撞模型的仿真，不在现有一维 S-curve 上过度推断。
