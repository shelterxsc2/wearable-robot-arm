# L3=55 PnP Calibration - 2026-07-02

> 文档状态：2026-07-02 的历史实测记录，保留用于追溯 `far_l3_55` PnP 表。它不是 2026-07-13 迁移后的完整验收结果；使用前需与 `src/rga_npu.cpp` 当前常量逐项核对。

Current profile: `far_l3_55`

## Yaw

`/tmp/pnp_yaw_calib.csv`

| target_yaw_deg | pnp_yaw_avg_deg | compensation_deg | samples | used |
|---:|---:|---:|---:|:---:|
| 0 | -7.2326 | 7.2326 | 116 | yes |
| -15 | -5.8080 | 5.8080 | 114 | yes |
| -30 | -8.4788 | 8.4788 | 115 | yes |
| -45 | -6.4065 | 6.4065 | 115 | yes |
| -60 | -4.1826 | 4.1826 | 115 | yes |
| -75 | -5.2863 | 5.2863 | 115 | yes |
| 15 | -7.1712 | 7.1712 | 116 | yes |
| 30 | -8.1047 | 8.1047 | 117 | yes |
| 45 | -5.6603 | 5.6603 | 116 | yes |
| 60 | -7.9074 | 7.9074 | 116 | yes |
| 75 | -13.9577 | 13.9577 | 117 | no |

Yaw `+75` was excluded because the PnP result was reported unreliable during capture and is discontinuous relative to neighboring points.

## Pitch

`/tmp/pnp_pitch_calib.csv`

| target_pitch_deg | pnp_pitch_avg_deg | compensation_deg | samples |
|---:|---:|---:|---:|
| 0 | 6.3915 | -6.3915 | 115 |
| -15 | 2.9546 | -2.9546 | 115 |
| -30 | 8.9463 | -8.9463 | 116 |
| 15 | 6.1471 | -6.1471 | 116 |
| 30 | 9.5954 | -9.5954 | 117 |
