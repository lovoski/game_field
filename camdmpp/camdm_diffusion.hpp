#pragma once

#include "toolkit/utils.hpp"
#include "scripts/onnxruntime/utils.hpp"

class camdm_diffusion {
public:
  void setup(std::string onnx_filepath, std::string config_filepath);
  std::vector<float> inference();

private:
  nlohmann::json config;
  int joint_num = 78, past_points = 10, future_points = 45, feat_dim = 3,
      diffusion_steps = 4;
  std::vector<std::string> joint_names;
  std::vector<float> posterior_log_variance_clipped, posterior_mean_coef1,
      posterior_mean_coef2;

  std::size_t shape_to_size(std::vector<std::int64_t> &shape) {
    std::size_t size = 1;
    for (auto &s : shape)
      size *= s;
    return size;
  }

  std::vector<std::int64_t> x_t_shape, timestep_shape, past_motion_shape,
      traj_pose_shape, traj_trans_shape, style_idx_shape;

  std::vector<const char *> input_names = {"input_x",     "time_steps",
                                           "past_motion", "traj_pose",
                                           "traj_trans",  "style_idx"};
  std::vector<const char *> output_names = {"output"};

  Ort::Session session = Ort::Session(nullptr);
  Ort::Env env = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "diffusion");
  Ort::AllocatorWithDefaultOptions allocator;
};