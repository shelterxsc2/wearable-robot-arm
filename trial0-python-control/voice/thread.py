# -*- coding: utf-8 -*-
"""End-to-end voice KWS thread: microphone → (VAD) → Sherpa keyword spotter."""
import queue
import threading
import time

from .audio import MicAudioThread
from .kws import SherpaKwsSpotter
from .vad import SileroVadGate
from . import config


class VoiceKwsThread(threading.Thread):
    """Background thread that runs the full voice keyword-spotting chain.

    Usage:
        voice = VoiceKwsThread(provider='cpu', device_id=0, use_vad=True)
        voice.start()
        ...
        kw, ts = voice.get_latest(consume=True)  # None if no new keyword
        voice.stop()
    """

    def __init__(
        self,
        model_dir: str | None = None,
        keywords_file: str | None = None,
        provider: str | None = None,
        device_id: int | None = None,
        capture_rate: int | None = None,
        channels: int | None = None,
        blocksize: int | None = None,
        gain: float | None = None,
        auto_gain: bool | None = None,
        auto_gain_target_db: float | None = None,
        auto_gain_max_db: float | None = None,
        auto_gain_min_db: float | None = None,
        use_vad: bool | None = None,
        vad_model: str | None = None,
        vad_threshold: float | None = None,
        vad_hangover_ms: float | None = None,
        silence_reset_blocks: int | None = None,
        keywords_score: float | None = None,
        keywords_threshold: float | None = None,
        num_trailing_blanks: int | None = None,
        num_threads: int | None = None,
    ) -> None:
        super().__init__(daemon=True)
        self.audio_queue: queue.Queue = queue.Queue()
        self.audio_thread = MicAudioThread(
            self.audio_queue,
            device_id=device_id,
            capture_rate=capture_rate,
            channels=channels,
            blocksize=blocksize,
            gain=gain,
            auto_gain=auto_gain,
            auto_gain_target_db=auto_gain_target_db,
            auto_gain_max_db=auto_gain_max_db,
            auto_gain_min_db=auto_gain_min_db,
        )
        self.kws = SherpaKwsSpotter(
            model_dir=model_dir,
            keywords_file=keywords_file,
            provider=provider,
            device_id=device_id,
            keywords_score=keywords_score,
            keywords_threshold=keywords_threshold,
            num_trailing_blanks=num_trailing_blanks,
            num_threads=num_threads,
        )
        self.use_vad = use_vad if use_vad is not None else config.VOICE_USE_VAD
        self.silence_reset_blocks = silence_reset_blocks if silence_reset_blocks is not None else config.VOICE_SILENCE_RESET_BLOCKS
        self.vad = None
        if self.use_vad:
            self.vad = SileroVadGate(
                model_path=vad_model or config.SILERO_VAD_MODEL,
                threshold=vad_threshold if vad_threshold is not None else config.VOICE_VAD_THRESHOLD,
                hangover_ms=vad_hangover_ms if vad_hangover_ms is not None else config.VOICE_VAD_HANGOVER_MS,
                sample_rate=config.VOICE_SAMPLE_RATE,
            )
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._latest: tuple[str, float] | None = None
        self._last_keyword: str | None = None
        self._count = 0

    def start(self) -> None:
        self.audio_thread.start()
        super().start()
        print("[VoiceKws] Thread started")

    def run(self) -> None:
        vad_state = False
        silent_blocks = 0
        while not self._stop_event.is_set():
            try:
                block = self.audio_queue.get(timeout=0.05)
            except queue.Empty:
                continue

            self.kws.accept_waveform(block)

            # Decode on every block.  VAD is only used to decide when the stream
            # has truly gone silent so we can reset and avoid false triggers on
            # accumulated noise.  A brief VAD false-negative inside a keyword
            # will no longer drop the audio.
            kw = self.kws.decode()
            if kw:
                self._set_latest(kw)

            if self.vad is not None:
                prev_state = vad_state
                vad_state = self.vad.accept_waveform(block)
                if not vad_state:
                    silent_blocks += 1
                    if prev_state and silent_blocks >= self.silence_reset_blocks:
                        # Speech -> sustained silence: reset to avoid noise accumulation.
                        self.kws.reset()
                        silent_blocks = 0
                else:
                    silent_blocks = 0

    def _set_latest(self, keyword: str) -> None:
        with self._lock:
            self._latest = (keyword, time.time())
            self._last_keyword = keyword
            self._count += 1
        print(f"[VoiceKws] Detected: {keyword}")

    def get_latest(self, consume: bool = True) -> tuple[str, float] | None:
        """Return the latest detected keyword (and timestamp) if any."""
        with self._lock:
            latest = self._latest
            if consume:
                self._latest = None
            return latest

    @property
    def last_keyword(self) -> str | None:
        with self._lock:
            return self._last_keyword

    @property
    def total_count(self) -> int:
        with self._lock:
            return self._count

    def stop(self) -> None:
        self._stop_event.set()
        self.audio_thread.stop()
        self.join(timeout=1.0)
        self.audio_thread.join(timeout=1.0)
        print("[VoiceKws] Thread stopped")
