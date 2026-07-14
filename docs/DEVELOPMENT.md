# 开发、测试与故障排查

## 1. 开发前检查

```bash
git branch --show-current
git status --short
git diff --check
```

当前迁移工作树包含未提交和未跟踪文件。不要 reset、restore 或批量覆盖；先区分迁移改动、模型、构建产物和用户原有文件。

## 2. 构建与纯逻辑测试

完整构建命令见根 `README.md`。建议每次至少执行：

```bash
python3 -m unittest tests.test_control_replay
python3 simulation/control_replay.py simulation/example_first_person.jsonl
python3 -m py_compile scripts/rule_engine_server.py scripts/hand_pipeline_server.py
git diff --check
```

## 3. 安全运行层级

1. 纯逻辑测试：不接触硬件。
2. sidecar 单独启动：使用 Unix socket 和 NPU，但不启动机械控制。
3. 摄像头/推流测试：确认 UART 不可写或机械臂安全隔离。
4. 低速机械臂测试：确认急停、限位和空间。
5. 完整云端/蓝牙/机械臂联调。

不要从第 1 层测试通过直接推断第 5 层安全。

## 4. 常见日志

- `wrist-ROI async OSD ready ... fixed interval=2`：手势 worker 初始化。
- `side=Left/Right ... infer=...ms`：某侧手部结果。
- `waiting for sidecar` / `connected to sidecar`：Python 服务连接状态。
- `reject invalid ROI`：C++ 边界检查拒绝非法框，应调查上游关键点。
- `RGA ROI crop/resize/color failed`：RGA 阶段错误；保留完整 rect/stride 日志。
- `[ControlRouter] source=...`：统一控制入口日志。
- `[FIRST-PERSON-CMD]`：实际 FIRST_PERSON UART 目标已发送。
- `[Main] All sidecars ready`：两个 Python 服务均已创建 socket，主程序才继续初始化 NPU。
- `[Main] Shutdown complete`：Ctrl+C 逆序清理完成；随后不应残留 `build/cc`、sidecar、8080/8554 或两个 socket。

## 5. 生命周期验证

主程序负责 RuleEngine 和 HandPipeline 的完整生命周期，不应手工另开生产 sidecar。修改初始化或关闭流程后，至少验证一次：启动后存在主进程和两个子进程；发送一次 SIGINT；日志到达 `Shutdown complete`；`pgrep`、8080/8554 和 `/tmp/*.sock` 均无残留。

## 6. RGA Bus error 处理

若出现 RGA `Invalid argument` 后系统范围 Bus error：

1. 停止主程序和 sidecar；若普通进程也持续 Bus error，重启设备。
2. 保存 RGA 输出中的 src/dst rect、image stride、format。
3. 检查 NV12 偶数坐标、方形尺寸、16 对齐和帧边界。
4. 不要通过忽略返回值继续 resize/color。
5. 重编译后先测四边和角落，再长时间运行。

## 7. 提交建议

将当前大工作树拆成至少三类提交：

- 控制迁移与测试；
- 手势模型/sidecar/OSD；
- 文档与交接。

模型文件需确认仓库容量策略；`build/cc` 通常不应作为源码提交。手势控制提交信息应明确“代码已接入、实机未验收”，不能把纯决策测试描述成机械安全验证。
