# 视觉与手部旁路

> 当前实现说明，更新于 2026-07-13。性能数字是阶段性实测，不是硬保证。

## 1. 主视觉热路径

RTMP 与 RTSP 都在 MJPEG 解码后先将倒装摄像头画面旋转 180°，再把 1920×1080 NV12 帧交给 `process_frame()`。因此人体/手部推理、ROI 坐标、OSD 和最终推流共享同一个正向坐标系。主路径同步执行：

```text
Camera 30 FPS caps
  → RGA resize/color
  → best.rknn Body YOLO-Pose (17 COCO points)
  → FACE 模式：Face ROI + 468 landmarks + PnP
  → 模式控制观察值/OSD
  → 手部 ROI 轻量预处理提交
  → 编码推流
```

Body 模型是主要瓶颈，实际完整视觉通常约 13～15 FPS。GStreamer caps 的 30 FPS 是输入声明，不代表 NPU 控制链真正处理 30 FPS。

## 2. 模式差异

- `FACE`：跟踪目标人体，运行 Face 468 与 PnP，提供视觉校正。
- `BODY`：人体检测/跟踪和人体 OSD，不运行完整 Face PnP 控制。
- `INTRO`：复用人体点，使用介绍手/构图状态机。
- `INTERVIEW`：复用人体检测，选择两人中点或单人腕点构图。
- `FIRST_PERSON`：视觉仍推流，但机械控制主体来自头部 IMU 到 J4/J5 的映射。

## 3. 手部定位来源

当前生产旁路不先对整帧运行 palm detector，而是借助 Body 关键点：

- 左腕/右腕：COCO 9/10；
- 左肘/右肘：COCO 7/8；
- 腕置信度门槛 0.75，肘使用人体关键点门槛；
- 从肘到腕方向向外延伸 35%；
- 用肩宽修正偏短的前臂估计；
- ROI 边长约为有效前臂的 1.8 倍，限制在 96～360 像素。

ROI 必须满足：方形、边长 16 对齐、x/y 为偶数、完全位于画面内。靠近边缘时移动整个方框，不得单独缩窄宽或高。

## 4. 异步手势链

```text
Body wrists/elbows
  → square ROI
  → RGA crop → 224×224 NV12
  → RGA RGB conversion
  → bounded latest-frame queue (最多左右手两个任务)
  → C++ worker
  → /tmp/hand_pipeline.sock
  → Python Hand landmark + embedder + canned classifier
  → per-side result
  → OSD + gesture policy → control_router
```

固定每 3 个已处理视觉帧采样一轮。若一轮有两只手，两只手都进入最多两个槽位的有界队列；新采样帧覆盖尚未处理的旧队列，防止延迟持续增长。worker 推理不阻塞主视觉线程。

ROI 还要求同侧腕点高于肩点。FACE 运行完整手势映射；INTRO/INTERVIEW 保留
`ILoveYou` 退出路径；BODY/FIRST_PERSON 不生成手部 ROI。

右上角分别显示左右手状态。`Left/Right` 是被拍摄者自身的 COCO 左右，镜像画面下可能与屏幕左右相反。

## 5. 稳定结果与安全边界

- 手势得分门槛 0.70；
- 连续 3 次同类候选后产生 `Stable`；
- 新鲜结果约 0.9 秒，stable 最多保持约 1.5 秒；
- OSD stable 条件为 score ≥ 0.70、连续 3 次一致；
- 控制条件：score ≥ 0.70、同侧连续 3 个有效结果、全局冷却 1.5 秒；
- `gesture_overlay` 通过 `gesture_control` 调用 `control_router`，仍不直接调用 `uart_comm`；
- `ILoveYou` 是返回 FACE 的确认手势，替代 trial0 使用的 `OK`。

如未来接入场景，必须新增独立决策层，包含连续确认、释放确认、冷却时间、模式白名单、控制来源冲突处理和急停优先级。

## 6. 已知 RGA 故障与防回归

曾出现右边缘 ROI `[1858,578,48,352]`，RGA 报 `Invalid argument` 并使系统出现 Bus error。原因是边缘处分别缩小宽高，破坏了方形和 stride/rect 约束。

当前防护：

- ROI 生成阶段保持方形并向内平移；
- 提交前再次检查边界和对齐；
- crop/resize/color 任一步失败立即放弃该 ROI；
- 错误日志限频打印。

边缘、双手交叠、腕点跳变和无人场景是每次修改后的必测项。

## 7. 性能口径

“手部约 8 ms”指 sidecar 内单个 ROI 的阶段性推理耗时。它不包含 Body 模型、RGA、双手两次推理、编码、网络和调度尾延迟。

建议同时报告：

- 完整输出 FPS；
- Body、Face、每只 Hand 的平均/P95/P99；
- 单手和双手占比；
- 队列覆盖次数；
- RGA 错误数和 sidecar 重连次数；
- CPU/NPU 温度与 10～30 分钟持续运行结果。

## 8. 后续方向

1. 增加正式视觉性能统计而不是依赖零散终端日志。
2. 继续验证双手每 3 帧策略的手势响应和长期 FPS。
3. 评估腕肘 ROI 对遮挡、短袖/长袖、画面边缘和多人场景的召回率。
4. 必要时低频 palm detector 只作腕点缺失兜底，不能同步塞回主热路径。
5. 完成 annotation 总开关对所有 OSD 的一致控制。
