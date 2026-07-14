# KWS/VAD RK3588 基准（2026-07-14）

测试命令：

```bash
taskset -c 2 nice -n 10 python3 scripts/benchmark_kws.py --duration 20 --no-vad
taskset -c 2 nice -n 10 python3 scripts/benchmark_kws.py --duration 20
```

边界：RK3588 CPU2（A55），`performance` governor，1.8 GHz，测试后温度约 43.5°C；
官方中文 WAV 循环，16 kHz，每 20 ms 按真实时钟喂入。Sherpa-ONNX CPU 单线程。
MemTotal 7,857 MiB。这是独立语音进程测试，不是完整视觉 pipeline A/B。

| 指标 | KWS only | KWS + Silero VAD 软门控 |
|---|---:|---:|
| 单核 CPU | 19.51% | 23.69% |
| 八核合计等效占用 | 2.44% | 2.96% |
| RTF（CPU time/audio time） | 0.1955 | 0.2373 |
| 稳态 RSS | 81.62 MiB（1.04%） | 84.95 MiB（1.08%） |
| 峰值 RSS | 81.82 MiB（1.04%） | 86.78 MiB（1.10%） |
| `decode_stream` 次数 | 62 | 62 |
| `decode_stream` 平均 | 57.04 ms | 54.47 ms |
| `decode_stream` P50/P95/P99 | 56.83/58.04/59.73 ms | 54.19/55.52/57.99 ms |
| `decode_stream` 最大 | 63.02 ms | 61.42 ms |
| VAD `accept_waveform` 平均 | - | 0.981 ms |
| VAD P50/P95/P99 | - | 1.216/2.101/2.366 ms |
| VAD 最大 | - | 3.076 ms |

解读：

- 一次 KWS 模型解码约 54--57 ms，但并非每 20 ms 音频块都执行；20 秒中只有 62 次。
- VAD 增量约 4.17 个单核百分点、3.33 MiB 稳态 RSS，实时性仍有较大余量。
- 开启 VAD 后 RTF 0.237，高于原方案的 0.10 目标；虽然能实时运行，但不应声称已达成该性能目标。
- 官方 WAV 不包含本项目自定义关键词，因此本轮只测算力和内存，不评估命中率。
