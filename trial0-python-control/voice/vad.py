# -*- coding: utf-8 -*-
"""CPU-only Silero VAD gate.

Copied and slightly adapted from /home/time/work/sherpa/scripts/live_recognizer.py.
The model runs with onnxruntime CPUExecutionProvider.  It consumes 16 kHz mono
float32 audio in 512-sample windows (32 ms) and returns a speech/non-speech
decision every window.
"""
import numpy as np


class SileroVadGate:
    def __init__(
        self,
        model_path: str,
        threshold: float = 0.5,
        hangover_ms: float = 300.0,
        sample_rate: int = 16000,
        window_size: int = 512,
    ) -> None:
        import onnxruntime as ort

        self.threshold = threshold
        self.sample_rate = sample_rate
        self.window_size = window_size
        self.hangover_frames = max(1, int(hangover_ms / 1000.0 * sample_rate / window_size))

        sess_options = ort.SessionOptions()
        sess_options.intra_op_num_threads = 1
        sess_options.inter_op_num_threads = 1
        sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL

        self.session = ort.InferenceSession(
            model_path,
            sess_options=sess_options,
            providers=["CPUExecutionProvider"],
        )

        # LSTM states: [num_layers, batch, hidden]
        self.h = np.zeros((2, 1, 64), dtype=np.float32)
        self.c = np.zeros((2, 1, 64), dtype=np.float32)

        self._is_speech = False
        self._silent_count = 0
        self._buffer = np.array([], dtype=np.float32)

    def reset(self) -> None:
        """Reset internal LSTM states and hangover counters."""
        self.h.fill(0.0)
        self.c.fill(0.0)
        self._is_speech = False
        self._silent_count = 0
        self._buffer = np.array([], dtype=np.float32)

    @property
    def is_speech(self) -> bool:
        return self._is_speech

    def _process_window(self, window: np.ndarray) -> None:
        x = window.reshape(1, -1).astype(np.float32)
        prob, new_h, new_c = self.session.run(
            None,
            {
                "x": x,
                "h": self.h,
                "c": self.c,
            },
        )
        self.h = new_h
        self.c = new_c
        score = float(prob[0][0])

        if score >= self.threshold:
            self._is_speech = True
            self._silent_count = 0
        else:
            self._silent_count += 1
            if self._silent_count >= self.hangover_frames:
                self._is_speech = False
                self._silent_count = 0

    def accept_waveform(self, samples: np.ndarray) -> bool:
        """Append samples and process complete VAD windows.

        Returns the current speech/non-speech decision after consuming the new
        audio.  The decision is sticky over the hangover period.
        """
        samples = np.asarray(samples, dtype=np.float32)
        if samples.ndim != 1:
            samples = samples.reshape(-1)
        self._buffer = np.concatenate([self._buffer, samples])

        while len(self._buffer) >= self.window_size:
            window = self._buffer[: self.window_size]
            self._buffer = self._buffer[self.window_size :]
            self._process_window(window)

        return self._is_speech
