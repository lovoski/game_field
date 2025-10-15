#include "scripts/onnxruntime/camdm_diffusion.hpp"
#include <fstream>
#include <iostream>
#include <spdlog/spdlog.h>

void camdm_diffusion::setup(std::string onnx_filepath,
                            std::string config_filepath) {
  std::ifstream config_file(config_filepath);
  if (!config_file.is_open()) {
    spdlog::error("Failed to open config file {0}", config_filepath);
    return;
  }
  config_file >> config;
  past_points = config["past_points"];
  future_points = config["future_points"];
  joint_num = config["joint_num"] + 3;
  diffusion_steps = config["diffusion_steps"];
  joint_names = config["joint_names"].get<std::vector<std::string>>();
  posterior_log_variance_clipped =
      config["posterior_log_variance_clipped"].get<std::vector<float>>();
  posterior_mean_coef1 =
      config["posterior_mean_coef1"].get<std::vector<float>>();
  posterior_mean_coef2 =
      config["posterior_mean_coef2"].get<std::vector<float>>();

  Ort::SessionOptions session_options;
  session_options.SetIntraOpNumThreads(1);
  session_options.SetGraphOptimizationLevel(
      GraphOptimizationLevel::ORT_ENABLE_ALL);
  try {
    session =
        Ort::Session(env, toolkit::string_to_wstring(onnx_filepath).c_str(),
                     session_options);
  } catch (const Ort::Exception &e) {
    spdlog::error("Error loading model {0}", e.what());
    return;
  }
  PrintNodeDetails(session, allocator, true);  // Print input details
  PrintNodeDetails(session, allocator, false); // Print output details

  timestep_shape = {1};
  x_t_shape = {1, joint_num, feat_dim, future_points};
  past_motion_shape = {1, joint_num, feat_dim, past_points};
  style_idx_shape = {1};
  traj_pose_shape = {1, 3, future_points + past_points};
  traj_trans_shape = {1, 3, future_points + past_points};
}

std::vector<float> camdm_diffusion::inference() {
  std::vector<float> x_t_data(shape_to_size(x_t_shape));
  std::vector<float> past_motion_data(shape_to_size(past_motion_shape), 0.0f);
  std::vector<int64_t> style_idx_data(shape_to_size(style_idx_shape), 0);
  std::vector<float> traj_pose_data(shape_to_size(traj_pose_shape), 0.0f);
  std::vector<float> traj_trans_data(shape_to_size(traj_trans_shape), 0.0f);

  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<float> d(0.0f, 1.0f);
  for (size_t i = 0; i < x_t_data.size(); ++i) {
    x_t_data[i] = d(gen);
  }

  Ort::MemoryInfo memory_info =
      Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

  for (int t = diffusion_steps - 1; t >= 0; t--) {
    Ort::Value x_t = Ort::Value::CreateTensor<float>(
        memory_info, x_t_data.data(), x_t_data.size(), x_t_shape.data(),
        x_t_shape.size());
    std::vector<std::int64_t> time_step_shape = {1};
    std::vector<std::int64_t> time_step_data = {(std::int64_t)t};
    Ort::Value time_step_tensor = Ort::Value::CreateTensor<std::int64_t>(
        memory_info, time_step_data.data(), time_step_data.size(),
        time_step_shape.data(), time_step_shape.size());

    std::vector<Ort::Value> onnx_inputs;
    onnx_inputs.push_back(std::move(x_t));
    onnx_inputs.push_back(std::move(time_step_tensor));
    onnx_inputs.push_back(Ort::Value::CreateTensor<float>(
        memory_info, past_motion_data.data(), past_motion_data.size(),
        past_motion_shape.data(), past_motion_shape.size()));
    onnx_inputs.push_back(Ort::Value::CreateTensor<float>(
        memory_info, traj_pose_data.data(), traj_pose_data.size(),
        traj_pose_shape.data(), traj_pose_shape.size()));
    onnx_inputs.push_back(Ort::Value::CreateTensor<float>(
        memory_info, traj_trans_data.data(), traj_trans_data.size(),
        traj_trans_shape.data(), traj_trans_shape.size()));
    onnx_inputs.push_back(Ort::Value::CreateTensor<std::int64_t>(
        memory_info, style_idx_data.data(), style_idx_data.size(),
        style_idx_shape.data(), style_idx_shape.size()));

    auto output_tensors =
        session.Run(Ort::RunOptions{nullptr}, input_names.data(),
                    onnx_inputs.data(), 6, output_names.data(), 1);

    float *pred_noise_data = output_tensors[0].GetTensorMutableData<float>();

    const float coef1 = posterior_mean_coef1[t];
    const float coef2 = posterior_mean_coef2[t];
    const float log_var = posterior_log_variance_clipped[t];
    const float variance = std::exp(log_var);

    for (size_t i = 0; i < x_t_data.size(); ++i) {
      float pred_x0 = pred_noise_data[i];
      float posterior_mean = coef1 * pred_x0 + coef2 * x_t_data[i];

      float noise = (t > 0) ? d(gen) : 0.0f;
      x_t_data[i] = posterior_mean + std::sqrt(variance) * noise;
    }
  }

  return x_t_data;
}