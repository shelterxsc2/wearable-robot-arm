#!/usr/bin/env python3
"""Hardware-free scenario/control replay for the RK3588 migration.

Input is JSON Lines. This deliberately simulates command orchestration and
first-person output only; it does not pretend to model arm dynamics.
"""

import argparse
import json
import sys


MODES = {"face", "body", "intro", "interview", "first_person"}


class Replay:
    def __init__(self):
        self.mode = "face"
        self.profile = 0
        self.target = [-20.0, 30.0, 20.0]

    def apply(self, event):
        kind = event.get("type")
        if kind == "mode":
            mode = str(event.get("mode", "")).lower()
            if mode not in MODES:
                raise ValueError(f"invalid mode: {mode}")
            self.mode = mode
            return {"event": "mode", "mode": mode, "source": event.get("source", "sim")}
        if kind == "profile":
            profile = int(event["profile"])
            if profile not in (0, 1):
                raise ValueError("profile must be 0 or 1")
            self.profile = profile
            return {"event": "profile", "profile": profile}
        if kind == "target":
            self.target = [float(event[k]) for k in ("x", "y", "z")]
            return {"event": "target", "xyz": self.target}
        if kind == "head":
            if self.mode != "first_person":
                return {"event": "no_uart", "reason": "not_first_person"}
            yaw = float(event.get("yaw", 0.0))
            pitch = float(event.get("pitch", 0.0))
            j4 = max(-90.0, min(90.0, 10.0 - pitch))
            j5 = max(0.0, min(270.0, 180.0 + yaw))
            return {"event": "uart_target", "xyz": self.target, "j4": j4,
                    "j5": j5, "flag": 1}
        raise ValueError(f"unknown event type: {kind}")


def run(stream_in, stream_out):
    replay = Replay()
    for line_no, line in enumerate(stream_in, 1):
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        try:
            result = replay.apply(json.loads(line))
            stream_out.write(json.dumps(result, ensure_ascii=False) + "\n")
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            stream_out.write(json.dumps({"event": "error", "line": line_no,
                                         "message": str(exc)}) + "\n")
            return 1
    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("events", nargs="?", help="JSONL file; stdin when omitted")
    args = parser.parse_args()
    if args.events:
        with open(args.events, "r", encoding="utf-8") as stream:
            return run(stream, sys.stdout)
    return run(sys.stdin, sys.stdout)


if __name__ == "__main__":
    raise SystemExit(main())
