#ifndef TRON_VISION_NPU_MODEL_H
#define TRON_VISION_NPU_MODEL_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "tron_vision/npu_abi.h"

namespace tron_vision {

struct TensorInfo {
  std::string name;
  std::vector<uint32_t> dims;
  int format = 0;
  int type = 0;
  int quant = 0;
  int32_t zero_point = 0;
  float scale = 1.0F;
  uint32_t elements = 0;
};

// One model on the NPU through the runtime library loaded with dlopen. Not copyable, one context per object.
class NpuModel {
 public:
  NpuModel() = default;
  NpuModel(const NpuModel&) = delete;
  NpuModel& operator=(const NpuModel&) = delete;
  ~NpuModel();

  // Loads the library and the model, reads the tensor attributes and binds the model to `core` (0 to 2) alone.
  bool Open(const std::string& library_path, const std::string& model_path, int core, std::string& error);
  void Close();
  bool IsOpen() const {
    return context_ != 0;
  }

  const std::vector<TensorInfo>& Inputs() const {
    return inputs_;
  }
  const std::vector<TensorInfo>& Outputs() const {
    return outputs_;
  }
  const std::string& SdkVersion() const {
    return sdk_version_;
  }

  // One inference on an RGB image in the layout of input 0 (NHWC, 8 bit per channel); outputs stay quantized.
  bool Run(const uint8_t* image, size_t size, std::vector<std::vector<int8_t>>& outputs, std::string& error);

 private:
  bool Resolve(std::string& error);
  bool QueryTensors(std::string& error);

  void* library_ = nullptr;
  abi::Context context_ = 0;
  abi::InitFn init_ = nullptr;
  abi::QueryFn query_ = nullptr;
  abi::SetCoreMaskFn set_core_mask_ = nullptr;
  abi::InputsSetFn inputs_set_ = nullptr;
  abi::RunFn run_ = nullptr;
  abi::OutputsGetFn outputs_get_ = nullptr;
  abi::OutputsReleaseFn outputs_release_ = nullptr;
  abi::DestroyFn destroy_ = nullptr;
  std::vector<TensorInfo> inputs_;
  std::vector<TensorInfo> outputs_;
  std::string sdk_version_;
};

}  // namespace tron_vision

#endif  // TRON_VISION_NPU_MODEL_H
