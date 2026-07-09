# trial0 Python Control Package

This folder contains the organized Python runtime from /home/time/work/trial0 as of 2026-07-10.

## Included

- Python camera / control / STM32 bridge runtime.
- Voice keyword control package and keyword files.
- BLE remote helper.
- Tests and project docs.
- The local OpenVINO body pose model used by the trial0 pipeline: models/yolov8s-pose_openvino_model/.

## Excluded

The following were intentionally not copied into GitHub:

- third_party/ runtime/vendor trees.
- archive/, virtual environments, caches and __pycache__.
- Runtime logs and evaluation outputs.
- Local TLS key/cert files.
- Backup or unused model exports.

## External Runtime Assets

The current Python scripts still reference several external model locations used on the RK host, including:

- /home/time/work/mymodel/face_landmark_468.onnx
- /home/time/work/mymodel/openvino_pipeline/models/onnx/hand_landmarks_detector.onnx
- /home/time/work/mymodel/openvino_pipeline/model/keypoint_classifier/keypoint_classifier.onnx
- /home/time/work/mymodel/rule_engine_v2.onnx
- /home/time/work/sherpa/models/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20
- /home/time/work/sherpa/models/silero_vad.onnx

Those assets must be present on the target device or the paths must be adjusted before running the full demo.

## Current Control Behavior

- Auto power-on is enabled after init success while the microphone is unavailable.
- After power-on and A-init, the system stays in face mode for 10 seconds, then switches to first_person.
- First-person initial pose is x=-20, y=30, z=20, J4=10, J5=180.
- First-person mode uses head IMU only: yaw controls J5 and pitch controls J4.
- BLE remote continues to work in first-person mode via the local HTTP control server.
- Power-off and Ctrl+C first send the far_l3_55 face-home pose, wait 3 seconds, then send the retract frame.
