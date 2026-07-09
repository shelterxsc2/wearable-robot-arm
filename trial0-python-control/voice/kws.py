# -*- coding: utf-8 -*-
"""Sherpa-onnx streaming keyword-spotting wrapper."""
import time
from pathlib import Path

import numpy as np
import sherpa_onnx

from . import config


class SherpaKwsSpotter:
    """Thin wrapper around sherpa_onnx.KeywordSpotter.

    It handles the INT8/FP32 model selection (FP32 is required for the
    OpenVINO NPU EP on this hardware) and exposes a simple
    ``accept_waveform`` / ``decode`` interface.
    """

    def __init__(
        self,
        model_dir: str | None = None,
        keywords_file: str | None = None,
        provider: str | None = None,
        device_id: int | None = None,
        keywords_score: float | None = None,
        keywords_threshold: float | None = None,
        num_trailing_blanks: int | None = None,
        num_threads: int | None = None,
    ) -> None:
        self.model_dir = Path(model_dir or config.SHERPA_MODEL_DIR)
        self.keywords_file = Path(keywords_file or config.KEYWORDS_FILE)
        self.provider = provider or config.VOICE_PROVIDER
        self.device_id = device_id if device_id is not None else config.VOICE_DEVICE_ID
        self.keywords_score = keywords_score if keywords_score is not None else config.VOICE_KEYWORDS_SCORE
        self.keywords_threshold = keywords_threshold if keywords_threshold is not None else config.VOICE_KEYWORDS_THRESHOLD
        self.num_trailing_blanks = num_trailing_blanks if num_trailing_blanks is not None else config.VOICE_NUM_TRAILING_BLANKS
        self.num_threads = num_threads if num_threads is not None else config.VOICE_NUM_THREADS

        self.kws = self._create_kws()
        self.stream = self.kws.create_stream()

    def _create_kws(self) -> sherpa_onnx.KeywordSpotter:
        # The NPU OpenVINO EP currently produces incorrect results with the INT8
        # encoder/joiner, so fall back to FP32 when openvino is requested.
        if self.provider == "openvino":
            encoder = self.model_dir / "encoder-epoch-13-avg-2-chunk-16-left-64.onnx"
            decoder = self.model_dir / "decoder-epoch-13-avg-2-chunk-16-left-64.onnx"
            joiner = self.model_dir / "joiner-epoch-13-avg-2-chunk-16-left-64.onnx"
        else:
            encoder = self.model_dir / "encoder-epoch-13-avg-2-chunk-16-left-64.int8.onnx"
            decoder = self.model_dir / "decoder-epoch-13-avg-2-chunk-16-left-64.onnx"
            joiner = self.model_dir / "joiner-epoch-13-avg-2-chunk-16-left-64.int8.onnx"
            if not encoder.exists():
                encoder = self.model_dir / "encoder-epoch-13-avg-2-chunk-16-left-64.onnx"
            if not joiner.exists():
                joiner = self.model_dir / "joiner-epoch-13-avg-2-chunk-16-left-64.onnx"

        for p in (encoder, decoder, joiner, self.keywords_file):
            if not p.exists():
                raise FileNotFoundError(f"Voice KWS missing file: {p}")

        t0 = time.perf_counter()
        kws = sherpa_onnx.KeywordSpotter(
            tokens=str(self.model_dir / "tokens.txt"),
            encoder=str(encoder),
            decoder=str(decoder),
            joiner=str(joiner),
            keywords_file=str(self.keywords_file),
            num_threads=self.num_threads,
            sample_rate=config.VOICE_SAMPLE_RATE,
            feature_dim=80,
            keywords_score=self.keywords_score,
            keywords_threshold=self.keywords_threshold,
            num_trailing_blanks=self.num_trailing_blanks,
            provider=self.provider,
            device=self.device_id,
        )
        print(
            f"[VoiceKws] Model loaded in {(time.perf_counter() - t0) * 1000:.1f} ms "
            f"(provider={self.provider})"
        )
        return kws

    def accept_waveform(self, samples: np.ndarray) -> None:
        """Feed a block of 16 kHz mono float32 samples into the streaming decoder."""
        samples = np.asarray(samples, dtype=np.float32)
        if samples.ndim != 1:
            samples = samples.reshape(-1)
        if len(samples) == 0:
            return
        self.stream.accept_waveform(config.VOICE_SAMPLE_RATE, samples)

    def decode(self) -> str | None:
        """Run decode while the stream is ready and return the latest keyword, if any."""
        detected = None
        while self.kws.is_ready(self.stream):
            self.kws.decode_stream(self.stream)
            r = self.kws.get_result(self.stream)
            if r:
                detected = r
                self.kws.reset_stream(self.stream)
        return detected

    def reset(self) -> None:
        """Reset the streaming state (e.g. on speech->silence transition)."""
        self.kws.reset_stream(self.stream)
