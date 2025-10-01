#pragma once

#include "onnx_config.h"
#include <onnxruntime_cxx_api.h>

template <typename T> static void softmax(T &input) {
  float rowmax = *std::max_element(input.begin(), input.end());
  std::vector<float> y(input.size());
  float sum = 0.0f;
  for (size_t i = 0; i != input.size(); ++i) {
    sum += y[i] = std::exp(input[i] - rowmax);
  }
  for (size_t i = 0; i != input.size(); ++i) {
    input[i] = y[i] / sum;
  }
}

struct MNIST {
  MNIST() {}

  // 添加静态常量定义
  static constexpr int width_ = 28;
  static constexpr int height_ = 28;

  int64_t run(const std::array<float, width_ * height_> &image_data) {
    // 创建内存信息对象
    auto memory_info =
        Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);

    // 创建输入张量
    std::array<int64_t, 4> input_shape_{1, 1, width_, height_};
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info, const_cast<float *>(image_data.data()), image_data.size(),
        input_shape_.data(), input_shape_.size());

    // 准备输出容器和形状
    std::array<float, 10> results{};
    std::array<int64_t, 2> output_shape_{1, 10};
    Ort::Value output_tensor = Ort::Value::CreateTensor<float>(
        memory_info, results.data(), results.size(), output_shape_.data(),
        output_shape_.size());

    // 配置输入输出名称
    const char *input_names[] = {"Input3"};
    const char *output_names[] = {"Plus214_Output_0"};

    // 执行推理
    Ort::RunOptions run_options;
    session_.Run(run_options, input_names, &input_tensor, 1, output_names,
                 &output_tensor, 1);

    // 后处理
    softmax(results);
    auto max_it = std::max_element(results.begin(), results.end());
    return std::distance(results.begin(), max_it);
  }

private:
  Ort::Env env;
  Ort::Session session_{env, ONNX_MODEL_BASE_PATH L"/mnist.onnx",
                        Ort::SessionOptions{nullptr}};
};