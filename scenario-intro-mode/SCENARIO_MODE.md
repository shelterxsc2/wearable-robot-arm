# Scenario Intro Mode Snapshot

This folder is a standalone source snapshot of the current scene-mode work.
The repository root remains on the prior far/near two-profile control version.

Key entry point:
- `POST /mode?type=intro`

Current intro-mode behavior:
- Reuses the BODY 17-keypoint model; no extra object detector is added.
- Freezes NRF24 head-control updates while intro mode is active.
- Starts from `tx=-20, ty=85, tz=15`, `J4=65`, `J5=30`.
- Uses COCO right wrist index `10` as the presentation hand.
- Frames the target as `0.90 * right_wrist + 0.10 * torso_center`.
- J5 has hysteresis hold: enter hold within `6%` half-frame error, exit above `12%`.
- UART sends are limited by `4 deg` J5 deadband and `700 ms` minimum interval.
- When the right wrist returns near body center for `0.8 s`, space `x` moves toward `0` by `4 cm` every `0.9 s`.
- When the right wrist extends again, space `x` moves back toward the intro position `-20`.

Build from the repository root with the same command as the main project, replacing `src/` with `scenario-intro-mode/src/` if testing this snapshot directly.
