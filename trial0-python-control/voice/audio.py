# -*- coding: utf-8 -*-
"""Microphone capture thread with resampling."""
import queue
import re
import subprocess
import threading

import numpy as np
import sounddevice as sd
from scipy import signal

from . import config


def _enable_alsa_agc(device_id: int) -> None:
    """Try to enable ALSA Auto Gain Control for USB mics that are too quiet.

    Some USB microphones (e.g. Jieli/Realtek composite devices) ship with AGC
    disabled and produce near-silent output. This function parses the ALSA
    card index from the sounddevice name and turns AGC on if the control exists.
    """
    try:
        dev = sd.query_devices(device_id, "input")
        name = dev.get("name", "")
        m = re.search(r"\(hw:(\d+),\s*(\d+)\)", name)
        if not m:
            return
        card = m.group(1)
        # Check whether the device has an AGC control.
        out = subprocess.run(
            ["amixer", "-c", card, "contents"],
            capture_output=True,
            text=True,
            timeout=5,
            check=False,
        )
        if "Auto Gain Control" not in out.stdout:
            return
        subprocess.run(
            ["amixer", "-c", card, "set", "Auto Gain Control", "on"],
            capture_output=True,
            timeout=5,
            check=False,
        )
        print(f"[MicAudio] Enabled Auto Gain Control for ALSA card {card}")
    except Exception as e:
        # Never fail pipeline startup because of a mixer helper issue.
        print(f"[MicAudio] AGC enable skipped: {e}")


def _db_to_linear(db: float) -> float:
    return 10.0 ** (db / 20.0)


def _apply_gain(
    samples: np.ndarray,
    fixed_gain: float,
    auto_gain: bool,
    target_db: float,
    max_db: float,
    min_db: float,
) -> np.ndarray:
    """Apply a fixed linear gain plus optional block-wise AGC.

    The auto-gain only boosts blocks that are below ``target_db`` but still
    above ``min_db`` (pure noise floor), and it clamps the boost to ``max_db``
    to avoid runaway amplification of silence.
    """
    if not auto_gain:
        if fixed_gain == 1.0:
            return samples
        return np.clip(samples * fixed_gain, -1.0, 1.0)

    rms = np.sqrt(np.mean(samples.astype(np.float64) ** 2))
    if rms <= 0.0 or 20.0 * np.log10(rms) < min_db:
        extra_gain = 1.0
    else:
        target_lin = _db_to_linear(target_db)
        max_gain = _db_to_linear(max_db)
        extra_gain = min(max_gain, target_lin / max(rms, 1e-12))
    total_gain = fixed_gain * extra_gain
    return np.clip(samples * total_gain, -1.0, 1.0)


class MicAudioThread(threading.Thread):
    """Capture audio from the default/microphone device and push 16 kHz mono
    float32 blocks into ``out_queue``.

    The capture itself runs in the PortAudio callback; this thread only
    resamples the blocks and enqueues them, so it stays lightweight and can
    coexist with the heavy vision pipeline.
    """

    def __init__(
        self,
        out_queue: queue.Queue,
        device_id: int | None = None,
        capture_rate: int | None = None,
        channels: int | None = None,
        blocksize: int | None = None,
        gain: float | None = None,
        auto_gain: bool | None = None,
        auto_gain_target_db: float | None = None,
        auto_gain_max_db: float | None = None,
        auto_gain_min_db: float | None = None,
        stream_queue: queue.Queue | None = None,
    ) -> None:
        super().__init__(daemon=True)
        self.out_queue = out_queue
        self.device_id = device_id if device_id is not None else config.VOICE_DEVICE_ID
        self.capture_rate = capture_rate if capture_rate is not None else config.VOICE_CAPTURE_RATE
        self.channels = channels if channels is not None else config.VOICE_CHANNELS
        self.blocksize = blocksize if blocksize is not None else config.VOICE_BLOCKSIZE
        self.gain = gain if gain is not None else config.VOICE_GAIN
        self.auto_gain = auto_gain if auto_gain is not None else config.VOICE_AUTO_GAIN
        self.auto_gain_target_db = auto_gain_target_db if auto_gain_target_db is not None else config.VOICE_AUTO_GAIN_TARGET_DB
        self.auto_gain_max_db = auto_gain_max_db if auto_gain_max_db is not None else config.VOICE_AUTO_GAIN_MAX_DB
        self.auto_gain_min_db = auto_gain_min_db if auto_gain_min_db is not None else config.VOICE_AUTO_GAIN_MIN_DB
        self.stream_queue = stream_queue
        self.ratio = config.VOICE_SAMPLE_RATE / self.capture_rate
        self._stop_event = threading.Event()

    def run(self) -> None:
        raw_q: queue.Queue = queue.Queue()

        # Query the actual device so we don't request an unsupported channel
        # count or sample rate.  We always resample to 16 kHz afterward.
        try:
            dev = sd.query_devices(self.device_id)
        except Exception as e:
            print(f"[MicAudio] Failed to query device {self.device_id}: {e}")
            return

        # USB mics often need AGC enabled to reach usable levels.
        _enable_alsa_agc(self.device_id)

        max_ch = dev.get("max_input_channels", self.channels)
        channels = min(self.channels, max_ch) if max_ch and max_ch > 0 else self.channels
        actual_rate = int(dev.get("default_samplerate", self.capture_rate))
        ratio = config.VOICE_SAMPLE_RATE / actual_rate
        print(
            f"[MicAudio] Opening device {self.device_id} "
            f"({channels} ch @ {actual_rate} Hz), resample ratio={ratio:.4f}"
        )

        def callback(indata, frames, _time_info, status):
            if status:
                print(f"[MicAudio] sounddevice status: {status}")
            f32 = indata.astype(np.float32) / 32768.0
            if channels > 1 and f32.ndim > 1:
                mono = f32.mean(axis=1)
            else:
                mono = f32[:, 0] if f32.ndim > 1 else f32
            raw_q.put(mono.copy())

        try:
            with sd.InputStream(
                device=self.device_id,
                channels=channels,
                samplerate=actual_rate,
                dtype="int16",
                blocksize=self.blocksize,
                callback=callback,
            ):
                while not self._stop_event.is_set():
                    try:
                        block = raw_q.get(timeout=0.05)
                    except queue.Empty:
                        continue
                    try:
                        resampled = signal.resample(block, int(len(block) * ratio))
                    except Exception:
                        # Fallback linear interpolation if scipy fails for any reason.
                        x_old = np.linspace(0.0, 1.0, len(block))
                        x_new = np.linspace(0.0, 1.0, int(len(block) * ratio))
                        resampled = np.interp(x_new, x_old, block)
                    if self.stream_queue is not None:
                        stream_block = np.clip(
                            resampled * self.gain, -1.0, 1.0
                        ).astype(np.float32)
                        try:
                            self.stream_queue.put_nowait(stream_block)
                        except queue.Full:
                            try:
                                self.stream_queue.get_nowait()
                                self.stream_queue.put_nowait(stream_block)
                            except (queue.Empty, queue.Full):
                                pass
                    resampled = _apply_gain(
                        resampled,
                        fixed_gain=self.gain,
                        auto_gain=self.auto_gain,
                        target_db=self.auto_gain_target_db,
                        max_db=self.auto_gain_max_db,
                        min_db=self.auto_gain_min_db,
                    )
                    self.out_queue.put(resampled.astype(np.float32))
        except Exception as e:
            print(f"[MicAudio] InputStream failed: {e}")

    def stop(self) -> None:
        self._stop_event.set()
