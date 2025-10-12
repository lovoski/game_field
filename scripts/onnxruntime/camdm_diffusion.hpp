#pragma once

#include <onnxruntime_cxx_api.h>
#include "toolkit/utils.hpp"

class camdm_diffusion {
public:
  int diffusion_steps = 4;
  std::vector<std::string> joint_names;
  std::vector<float> posterior_log_variance_clipped, posterior_mean_coef1,
      posterior_mean_coef2;

  void setup(std::string onnx_filepath, std::string config_filepath) {
    session =
        Ort::Session(env, toolkit::string_to_wstring(onnx_filepath).c_str(),
                     Ort::SessionOptions{nullptr});
  }

private:
  const int joint_num = 26;
  const int past_points = 10;
  const int future_points = 45;

  Ort::Env env;
  Ort::Session session;
};