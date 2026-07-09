#!/usr/bin/env python3
"""Apply the OpenVINO NPU changes to sherpa-onnx 1.13.3 source.

The original patch files in /home/time/work/sherpa/patches/ have malformed
hunk counts and fail to apply.  This script makes the equivalent edits.
"""
import os
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
SHERPA_DIR = os.path.realpath(os.path.join(ROOT, "..", "third_party", "sherpa_onnx-1.13.3"))

if not os.path.isdir(SHERPA_DIR):
    print(f"Error: {SHERPA_DIR} not found.  Extract sherpa_onnx-1.13.3.tar.gz first.")
    sys.exit(1)

# ------------------------------------------------------------------
# 1. sherpa-onnx/csrc/session.cc
# ------------------------------------------------------------------
session_cc = os.path.join(SHERPA_DIR, "sherpa-onnx", "csrc", "session.cc")
with open(session_cc, "r", encoding="utf-8") as f:
    text = f.read()

# Add OpenVINO provider factory header
old = '#if defined(SHERPA_ONNX_ENABLE_SPACEMIT)\n#include "spacemit_ort_env.h"  // NOLINT\n#endif\n\nnamespace sherpa_onnx {'
new = '#if defined(SHERPA_ONNX_ENABLE_SPACEMIT)\n#include "spacemit_ort_env.h"  // NOLINT\n#endif\n\n#include "onnxruntime/core/providers/openvino/openvino_provider_factory.h"  // NOLINT\n\nnamespace sherpa_onnx {'
if old not in text:
    print("[session.cc] header insertion marker not found, skipping")
else:
    text = text.replace(old, new, 1)
    print("[session.cc] added OpenVINO provider header")

# Add kOpenVINO case in GetSessionOptionsImpl (skip if already present)
if 'case Provider::kOpenVINO:' in text:
    print("[session.cc] kOpenVINO case already present, skipping")
    old = None
else:
    old = '      break;\n    }\n  }\n  return sess_opts;\n}\n\nOrt::SessionOptions GetSessionOptions(const OnlineModelConfig &config) {'
new = '''      break;
    }
    case Provider::kOpenVINO: {
      if (std::find(available_providers.begin(), available_providers.end(),
                    "OpenVINOExecutionProvider") != available_providers.end()) {
        std::string device_type = "NPU";
        auto it = config.find("device_type");
        if (it != config.end()) {
          device_type = it->second;
        }
        std::string precision;
        auto pit = config.find("precision");
        if (pit != config.end()) {
          precision = pit->second;
        } else if (device_type == "NPU") {
          // NPU defaults to FP16 inference, which hurts accuracy for this
          // streaming transducer.  Use ACCURACY to get the best NPU precision.
          precision = "ACCURACY";
        }
        std::string ov_device = device_type;
        if (!precision.empty() && ov_device.find('_') == std::string::npos) {
          ov_device += "_" + precision;
        }
        SHERPA_ONNX_LOGE("Use OpenVINO Execution Provider with device %s",
                         ov_device.c_str());
        OrtStatus *status = OrtSessionOptionsAppendExecutionProvider_OpenVINO(
            sess_opts, ov_device.c_str());
        if (status) {
          const auto &api = Ort::GetApi();
          const char *msg = api.GetErrorMessage(status);
          SHERPA_ONNX_LOGE(
              "Failed to enable OpenVINO: %s. Available providers: %s. "
              "Fallback to cpu",
              msg, os.str().c_str());
          api.ReleaseStatus(status);
        }
      } else {
        SHERPA_ONNX_LOGE(
            "OpenVINOExecutionProvider is not available. Available providers: "
            "%s. Fallback to cpu!",
            os.str().c_str());
      }
      break;
    }
  }
  return sess_opts;
}

Ort::SessionOptions GetSessionOptions(const OnlineModelConfig &config) {'''
if old is not None:
    if old not in text:
        print("[session.cc] kOpenVINO case marker not found, skipping")
    else:
        text = text.replace(old, new, 1)
        print("[session.cc] added kOpenVINO provider case")

# Add hybrid NPU mode in the model-type overload (skip if already present)
if 'SHERPA_ONNX_NPU_HYBRID' in text:
    print("[session.cc] NPU hybrid logic already present, skipping")
    old = None
else:
    old = '''  /*
    Transducer models : Only encoder will run with tensorrt,
                        decoder and joiner will run with cuda
  */
  if (config.provider_config.provider == "trt" &&
      (model_type == "decoder" || model_type == "joiner")) {'''
new = '''  /*
    OpenVINO NPU: the NPU plugin on this hardware/software stack introduces
    accuracy drift in the streaming transducer decoder/joiner for some English
    keywords.  By default we now run the full chain on NPU to minimize memory
    bandwidth.  Set SHERPA_ONNX_NPU_HYBRID=1 to keep decoder/joiner on CPU.
  */
  const char *npu_hybrid_env = std::getenv("SHERPA_ONNX_NPU_HYBRID");
  if (npu_hybrid_env && npu_hybrid_env[0] == '1' &&
      config.provider_config.provider == "openvino" &&
      (model_type == "decoder" || model_type == "joiner")) {
    SHERPA_ONNX_LOGE(
        "OpenVINO NPU hybrid: running %s on CPU to avoid NPU drift",
        model_type.c_str());
    return GetSessionOptionsImpl(config.num_threads, "cpu",
                                 &config.provider_config);
  }

  /*
    Transducer models : Only encoder will run with tensorrt,
                        decoder and joiner will run with cuda
  */
  if (config.provider_config.provider == "trt" &&
      (model_type == "decoder" || model_type == "joiner")) {'''
if old is not None:
    if old not in text:
        print("[session.cc] hybrid marker not found, skipping")
    else:
        text = text.replace(old, new, 1)
        print("[session.cc] added NPU hybrid logic")

with open(session_cc, "w", encoding="utf-8") as f:
    f.write(text)

# ------------------------------------------------------------------
# 2. cmake/onnxruntime.cmake
# ------------------------------------------------------------------
cmake_file = os.path.join(SHERPA_DIR, "cmake", "onnxruntime.cmake")
with open(cmake_file, "r", encoding="utf-8") as f:
    text = f.read()

old = '''  if(DEFINED ENV{SHERPA_ONNXRUNTIME_INCLUDE_DIR})
    set(location_onnxruntime_header_dir $ENV{SHERPA_ONNXRUNTIME_INCLUDE_DIR})

    include_directories(${location_onnxruntime_header_dir})
  else()'''
new = '''  if(DEFINED ENV{SHERPA_ONNXRUNTIME_INCLUDE_DIR})
    set(location_onnxruntime_header_dir $ENV{SHERPA_ONNXRUNTIME_INCLUDE_DIR})

    include_directories(
      ${location_onnxruntime_header_dir}
      ${location_onnxruntime_header_dir}/onnxruntime/core/session
      ${location_onnxruntime_header_dir}/onnxruntime/core/providers/openvino
    )
  else()'''
if old not in text:
    print("[onnxruntime.cmake] include marker not found, skipping")
else:
    text = text.replace(old, new, 1)
    print("[onnxruntime.cmake] fixed include directories")

old = '''    if(WIN32)
      set_target_properties(onnxruntime PROPERTIES
        IMPORTED_LOCATION ${location_onnxruntime_lib}
        IMPORTED_IMPLIB ${location_onnxruntime_lib2}
        INTERFACE_INCLUDE_DIRECTORIES "${location_onnxruntime_header_dir}"
      )
    else()
      set_target_properties(onnxruntime PROPERTIES
        IMPORTED_LOCATION ${location_onnxruntime_lib}
        INTERFACE_INCLUDE_DIRECTORIES "${location_onnxruntime_header_dir}"
      )
    endif()'''
new = '''    if(WIN32)
      set_target_properties(onnxruntime PROPERTIES
        IMPORTED_LOCATION ${location_onnxruntime_lib}
        IMPORTED_IMPLIB ${location_onnxruntime_lib2}
        INTERFACE_INCLUDE_DIRECTORIES "${location_onnxruntime_header_dir};${location_onnxruntime_header_dir}/onnxruntime/core/session;${location_onnxruntime_header_dir}/onnxruntime/core/providers/openvino"
      )
    else()
      set_target_properties(onnxruntime PROPERTIES
        IMPORTED_LOCATION ${location_onnxruntime_lib}
        INTERFACE_INCLUDE_DIRECTORIES "${location_onnxruntime_header_dir};${location_onnxruntime_header_dir}/onnxruntime/core/session;${location_onnxruntime_header_dir}/onnxruntime/core/providers/openvino"
      )
    endif()'''
if old not in text:
    print("[onnxruntime.cmake] target properties marker not found, skipping")
else:
    text = text.replace(old, new, 1)
    print("[onnxruntime.cmake] fixed target include directories")

with open(cmake_file, "w", encoding="utf-8") as f:
    f.write(text)

print("\nOpenVINO patches applied successfully.")
