# -*- coding: utf-8 -*-
"""Standalone keyword-spotting demo (no vision, no camera).

This is the smallest possible end-to-end KWS loop so you can tell whether the
vision pipeline is starving the audio thread or interfering with recognition.
"""
import argparse
import os
import sys
import time

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from voice import VoiceKwsThread

SHERPA_MODEL_DIR = "/home/time/work/sherpa/models/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20"
SHERPA_KEYWORDS_FILE = os.path.join(SCRIPT_DIR, "voice", "keywords.txt")
SHERPA_VAD_MODEL = "/home/time/work/sherpa/models/silero_vad.onnx"


def main() -> None:
    parser = argparse.ArgumentParser(description="Voice-only keyword spotting demo")
    parser.add_argument("--provider", type=str, default="cpu", choices=["cpu", "openvino"],
                        help="KWS execution provider")
    parser.add_argument("--device", type=int, default=6, help="sounddevice input device index")
    parser.add_argument("--capture-rate", type=int, default=48000, help="Microphone native sample rate")
    parser.add_argument("--channels", type=int, default=2, help="Microphone native channels")
    parser.add_argument("--blocksize", type=int, default=1024, help="Audio capture block size")
    parser.add_argument("--no-vad", action="store_true", help="Disable Silero VAD")
    parser.add_argument("--vad-threshold", type=float, default=None,
                        help="VAD speech threshold (lower = more sensitive)")
    parser.add_argument("--vad-hangover-ms", type=float, default=None,
                        help="VAD hangover after speech ends")
    parser.add_argument("--silence-reset-blocks", type=int, default=None,
                        help="Consecutive silent blocks before resetting KWS")
    parser.add_argument("--score", type=float, default=None,
                        help="KWS keywords score (lower = easier trigger)")
    parser.add_argument("--threshold", type=float, default=None,
                        help="KWS keywords threshold (lower = easier trigger)")
    parser.add_argument("--trailing-blanks", type=int, default=None,
                        help="KWS num trailing blanks")
    parser.add_argument("--gain", type=float, default=None, help="Fixed linear audio gain")
    parser.add_argument("--auto-gain", action="store_true", default=None,
                        help="Enable software auto-gain (default from config)")
    parser.add_argument("--no-auto-gain", action="store_true", help="Disable software auto-gain")
    parser.add_argument("--auto-gain-target-db", type=float, default=None,
                        help="Target RMS for auto-gain")
    parser.add_argument("--auto-gain-max-db", type=float, default=None,
                        help="Max auto-gain boost")
    parser.add_argument("--auto-gain-min-db", type=float, default=None,
                        help="Auto-gain noise floor")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Run for N seconds then exit (0 = run until Ctrl-C)")
    args = parser.parse_args()

    auto_gain = None
    if args.auto_gain:
        auto_gain = True
    elif args.no_auto_gain:
        auto_gain = False

    print("=" * 60)
    print("Voice-only KWS demo")
    print("=" * 60)
    print(f"  provider={args.provider}")
    print(f"  device={args.device}")
    print(f"  score={args.score}")
    print(f"  threshold={args.threshold}")
    print(f"  trailing_blanks={args.trailing_blanks}")
    print(f"  gain={args.gain}")
    print(f"  auto_gain={auto_gain}")
    print("=" * 60)
    print("Speak a keyword. Press Ctrl-C to stop.\n")

    voice = VoiceKwsThread(
        model_dir=SHERPA_MODEL_DIR,
        keywords_file=SHERPA_KEYWORDS_FILE,
        provider=args.provider,
        device_id=args.device,
        capture_rate=args.capture_rate,
        channels=args.channels,
        blocksize=args.blocksize,
        gain=args.gain,
        auto_gain=auto_gain,
        auto_gain_target_db=args.auto_gain_target_db,
        auto_gain_max_db=args.auto_gain_max_db,
        auto_gain_min_db=args.auto_gain_min_db,
        use_vad=not args.no_vad,
        vad_model=SHERPA_VAD_MODEL,
        vad_threshold=args.vad_threshold,
        vad_hangover_ms=args.vad_hangover_ms,
        silence_reset_blocks=args.silence_reset_blocks,
        keywords_score=args.score,
        keywords_threshold=args.threshold,
        num_trailing_blanks=args.trailing_blanks,
    )
    voice.start()

    t0 = time.time()
    detections = 0
    try:
        while True:
            kw, ts = voice.get_latest(consume=True) or (None, None)
            if kw:
                detections += 1
                print(f"[{time.strftime('%H:%M:%S')}] Detected: {kw}  (total={detections})")
            if args.duration and (time.time() - t0) >= args.duration:
                print(f"\nDuration {args.duration}s reached, stopping.")
                break
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        voice.stop()
        print(f"Total detections: {detections}")


if __name__ == "__main__":
    main()
