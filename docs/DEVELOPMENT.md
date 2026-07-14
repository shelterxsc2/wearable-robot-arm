# 开发、测试与故障排查

## 1. 开发前检查

```bash
git branch --show-current
git status --short
git diff --check
```

当前主快照已提交到 `no-arm-test`。开始工作前仍应检查工作树，禁止覆盖用户后续修改。

## 2. 构建与纯逻辑测试

完整构建命令见根 `README.md`。建议每次至少执行：

```bash
python3 -m unittest tests.test_control_replay
python3 simulation/control_replay.py simulation/example_first_person.jsonl
python3 -m py_compile scripts/rule_engine_server.py scripts/hand_pipeline_server.py \
  scripts/voice_kws_server.py
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

- `wrist-ROI async control ready ... fixed interval=3`：手势 worker 初始化。
- `side=Left/Right ... infer=...ms`：某侧手部结果。
- `waiting for sidecar` / `connected to sidecar`：Python 服务连接状态。
- `reject invalid ROI`：C++ 边界检查拒绝非法框，应调查上游关键点。
- `RGA ROI crop/resize/color failed`：RGA 阶段错误；保留完整 rect/stride 日志。
- `[ControlRouter] source=...`：统一控制入口日志。
- `[FIRST-PERSON-CMD]`：实际 FIRST_PERSON UART 目标已发送。
- `[Main] All sidecars ready`：必需 RuleEngine/HandPipeline 已就绪；VoiceKWS 随后可降级启动。
- `[Main] Shutdown complete`：逆序清理完成标志；随后不应残留 `build/cc`、sidecar、8080/8554 或三个 socket。

## 5. 生命周期验证

主程序负责三个 sidecar 生命周期，不应手工另开生产实例。至少验证正常 SIGINT、timeout 和初始化失败：日志到达 `Shutdown complete`；`pgrep`、8080/8554 和三个 socket 均无残留。2026-07-14 性能测试曾在部分清理后遗留 root-owned Rule/Hand socket，这是当前待修问题。

## 6. RGA Bus error 处理

若出现 RGA `Invalid argument` 后系统范围 Bus error：

1. 停止主程序和 sidecar；若普通进程也持续 Bus error，重启设备。
2. 保存 RGA 输出中的 src/dst rect、image stride、format。
3. 检查 NV12 偶数坐标、方形尺寸、16 对齐和帧边界。
4. 不要通过忽略返回值继续 resize/color。
5. 重编译后先测四边和角落，再长时间运行。

## 7. 提交与分支

远端分支为 `origin/no-arm-test`。`build/cc`、缓存和日志不提交；根目录 Body/Face/Rule RKNN 仍受 `.gitignore` 管理，Hand/KWS 模型和 Sherpa runtime 已提交。任何机械相关提交必须区分代码接入与机械安全验收。
