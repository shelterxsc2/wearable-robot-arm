# -*- coding: utf-8 -*-
"""ELF voice library: Sherpa-onnx keyword spotting.

The package wraps the existing sherpa KWS pipeline (microphone capture,
optional Silero VAD, streaming transducer keyword spotting) into a single
background thread that can be dropped into the ELF camera pipeline.
"""

try:
    import sherpa_onnx  # noqa: F401
    import onnxruntime  # noqa: F401
    import sounddevice  # noqa: F401
except ImportError as e:
    raise ImportError(
        "The voice library requires sherpa-onnx, onnxruntime and sounddevice. "
        "Install them in the active environment, e.g.\n"
        "  pip install sherpa-onnx onnxruntime sounddevice\n"
        "For NPU/OpenVINO EP support use the custom sherpa-onnx/onnxruntime "
        "build described in /home/time/work/sherpa/README.md and patches/."
    ) from e

from .thread import VoiceKwsThread
from .config import *

__all__ = ["VoiceKwsThread"]
