# Simulation 文件夹说明

## 重要前提
**`cmd.txt` 中的 VOFA 数据来自旧版/次新版下位机代码，非当前 `8efeb36`。**

这意味着：
- VOFA 中观察到的"频繁打断、加速度方波、急停晃动"等现象，是旧版行为的记录
- 当前版 `8efeb36` 的改进（cmd_type 分支、runtime safe 等）是在这些数据之后加入的
- 因此：用旧版数据验证新版逻辑时，只能做**定性对比**（趋势是否改善），不能做**定量拟合**（数值是否匹配）

## 文件清单

| 文件 | 说明 |
|------|------|
| `vofa_parser.py` | 解析 `../cmd.txt` 中的 VOFA 文本转义数据，提取 s,v,a 时间序列 |
| `stm32_scurve_sim.py` | 下位机 S-curve 仿真器，复现 `Speed_Plan_Update` 完整状态机，支持 old/mid/current 三版本切换 |

## 下位机仿真方法概述

`stm32_scurve_sim.py` 的核心是逐周期（dt=1ms）复现下位机代码：

1. **事件驱动**：模拟 UART 接收新指令的时刻，触发 `state = init`
2. **完整状态机**：idle → init → phase1→2→3→3_end→4→5→6→7，每个相位的积分公式与下位机 C 代码逐行对应
3. **版本差异**：
   - `old`：无条件 `init→phase1`，无 runtime safe，无 cmd_type 分支
   - `mid`：推测次新版行为——强制制动检查，无分层决策，无条件降速
   - `current`：`8efeb36` 完整逻辑——120% clamp + 制动距离迭代 + 0.6f 智能跳转 + phase5 runtime safe
4. **简化跟踪**：用一阶惯性模型模拟电机实际位置跟随规划轨迹（`actual += (plan - actual) * 0.05`）

## 用法示例

```python
from stm32_scurve_sim import simulate_events

# 模拟：长距离运动后收到微调指令
events = [
    (0.0, 1.50, 0x01),   # 长距离启动
    (6.0, 1.52, 0x01),   # idle 后微调
]
result = simulate_events(events, version='current', dt=0.001)
# result 包含 t, target, s, v, a, state 数组
```

## 待办

- [ ] 用户上传上位机仿真 Python 文件
- [ ] 对比上位机与下位机仿真方法的一致性
- [ ] 用两套仿真联合运行，验证参数鲁棒性
