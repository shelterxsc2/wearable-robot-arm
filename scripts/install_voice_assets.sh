#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "$0")/.." && pwd)
VERSION=v1.13.4
RUNTIME=sherpa-onnx-${VERSION}-linux-aarch64-shared-cpu.tar.bz2
MODEL=sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20.tar.bz2
VAD=silero_vad.onnx
RUNTIME_URL=https://github.com/k2-fsa/sherpa-onnx/releases/download/${VERSION}/${RUNTIME}
MODEL_URL=https://github.com/k2-fsa/sherpa-onnx/releases/download/kws-models/${MODEL}
VAD_URL=https://github.com/k2-fsa/sherpa-onnx/releases/download/asr-models/${VAD}
MODEL_SHA256=68447f4fbc67e70eee3a93961f36e81e98f47aef73ce7e7ca00885c6cd3616a6
RUNTIME_SHA256=36c5a3c942358ed635471488f50a28a96181331c935b0dce75a02b7f49913dc2
VAD_SHA256=9e2449e1087496d8d4caba907f23e0bd3f78d91fa552479bb9c23ac09cbb1fd6

mkdir -p "$ROOT/downloads" "$ROOT/third_party" "$ROOT/models"
curl -fL --retry 5 -o "$ROOT/downloads/$RUNTIME" "$RUNTIME_URL"
curl -fL --retry 5 -o "$ROOT/downloads/$MODEL" "$MODEL_URL"
curl -fL --retry 5 -o "$ROOT/models/$VAD" "$VAD_URL"
echo "$MODEL_SHA256  $ROOT/downloads/$MODEL" | sha256sum -c -
echo "$RUNTIME_SHA256  $ROOT/downloads/$RUNTIME" | sha256sum -c -
echo "$VAD_SHA256  $ROOT/models/$VAD" | sha256sum -c -
tar -xjf "$ROOT/downloads/$RUNTIME" -C "$ROOT/third_party"
tar -xjf "$ROOT/downloads/$MODEL" -C "$ROOT/models"
python3 -m pip install --no-cache-dir --target "$ROOT/third_party/python" "sherpa-onnx==1.13.4"
echo "Installed official Sherpa-ONNX assets under $ROOT"
