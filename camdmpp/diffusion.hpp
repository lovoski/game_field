#pragma once

#include "onnx_utils.hpp"
#include "ziggurat.hpp"

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <json.hpp>

class diffusion {
public:
  void setup(std::string onnx_filepath, std::string config_filepath);

  /**
   * Submit an inference task. The provided callback will be invoked on the main
   * thread when the result is ready. The vector contains the output tensor data
   * produced by the model.
   */
  void submit_inference(std::function<void(std::vector<float>)> cb);

  /**
   * This function should be called from th emain thread each frame to run the
   * submitted callbacks.
   */
  void process_completions();

  /**
   * Run the inference immediately on the main thread, this could block the
   * execution.
   */
  std::vector<float> run_model_inference();

  ~diffusion();

  // These variables should be modified from the main thread, acting as the
  // default inputs for the model
  std::vector<float> x_t_data, past_motion_data;
  std::vector<float> traj_facing_data, traj_pos_data;
  std::vector<int64_t> style_idx_data;

  nlohmann::json config;
  int joint_num, past_points, pose_token_dim, future_points, diffusion_steps;
  std::vector<std::string> joint_names;
  std::vector<float> posterior_log_variance_clipped, posterior_mean_coef1,
      posterior_mean_coef2;

  std::vector<float> data_std, data_mean;
  std::vector<std::string> input_names, output_names;
  std::vector<std::int64_t> x_t_shape, past_motion_shape, traj_facing_shape,
      traj_pos_shape, style_idx_shape;

private:
  // for random number generation
  uint32_t ziggurat_kn[128], ziggurat_jsr = 0;
  float ziggurat_fn[128], ziggurat_wn[128];

  Ort::MemoryInfo memory_info =
      Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

  Ort::Session session = Ort::Session(nullptr);
  Ort::Env env = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "diffusion");
  Ort::AllocatorWithDefaultOptions allocator;

  // Worker / queue members for async inference
  std::thread worker_thread;
  std::mutex queue_mutex;
  std::condition_variable queue_cv;
  std::queue<std::function<void()>> task_queue;

  std::mutex completion_mutex;
  std::queue<std::function<void()>> completion_queue;
  std::atomic_bool stop_worker{false};

  void worker_loop();
};