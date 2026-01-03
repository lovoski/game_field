#include "diffusion.hpp"
#include <condition_variable>
#include <fstream>
#include <iostream>
#include <mutex>
#include <queue>
#include <random>

inline std::size_t __shape_to_size(std::vector<std::int64_t> &shape) {
  std::size_t size = 1;
  for (auto &s : shape)
    size *= s;
  return size;
}

void diffusion::setup(std::string onnx_filepath, std::string config_filepath) {
  std::ifstream config_file(config_filepath);
  // load config file
  if (!config_file.is_open()) {
    std::cout << "Failed to open config file " << config_filepath << std::endl;
    exit(-1);
  }
  config_file >> config;

  // setup variables
  joint_num = config["joint_num"];
  past_points = config["past_points"];
  future_points = config["future_points"];
  pose_token_dim = config["pose_token_dim"];
  diffusion_steps = config["diffusion_steps"];
  joint_names = config["joint_names"].get<std::vector<std::string>>();
  input_names = config["input_names"].get<std::vector<std::string>>();
  output_names = config["output_names"].get<std::vector<std::string>>();
  // variables used for ddpm sampling
  posterior_log_variance_clipped =
      config["posterior_log_variance_clipped"].get<std::vector<float>>();
  posterior_mean_coef1 =
      config["posterior_mean_coef1"].get<std::vector<float>>();
  posterior_mean_coef2 =
      config["posterior_mean_coef2"].get<std::vector<float>>();
  data_std = config["data_std"].get<std::vector<float>>();
  data_mean = config["data_mean"].get<std::vector<float>>();

  // initialize the model
  Ort::SessionOptions session_options;
  session_options.SetIntraOpNumThreads(1);
  session_options.SetGraphOptimizationLevel(
      GraphOptimizationLevel::ORT_ENABLE_ALL);
  try {
    session =
        Ort::Session(env, toolkit::string_to_wstring(onnx_filepath).c_str(),
                     session_options);
  } catch (const Ort::Exception &e) {
    std::cout << "Error loading model " << e.what() << std::endl;
    return;
  }
  print_onnx_node_details(session, allocator, true);  // Print input details
  print_onnx_node_details(session, allocator, false); // Print output details

  // manually specified shapes
  x_t_shape = {1, pose_token_dim, future_points};
  past_motion_shape = {1, pose_token_dim, past_points};
  style_idx_shape = {1};
  traj_pos_shape = {1, 10};
  traj_facing_shape = {1, 10};

  // initialize the conditioning inputs
  x_t_data = std::vector<float>(__shape_to_size(x_t_shape));
  past_motion_data =
      std::vector<float>(__shape_to_size(past_motion_shape), 0.0f);
  traj_facing_data =
      std::vector<float>(__shape_to_size(traj_facing_shape), 0.0f);
  traj_pos_data = std::vector<float>(__shape_to_size(traj_pos_shape), 0.0f);
  style_idx_data = std::vector<int64_t>(__shape_to_size(style_idx_shape), 0);

  // Start background worker for async inference
  stop_worker.store(false);
  if (!worker_thread.joinable())
    worker_thread = std::thread(&diffusion::worker_loop, this);
}

std::vector<float> run_model_inference(diffusion &self) {
  auto &x_t_data = self.x_t_data;
  static thread_local std::mt19937 gen{std::random_device{}()};
  std::normal_distribution<float> normal_dist(0.0f, 1.0f);
  for (float &v : x_t_data)
    v = normal_dist(gen);

  Ort::MemoryInfo memory_info =
      Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

  std::vector<int64_t> timestep_data(1, 0);
  const std::vector<int64_t> timestep_shape{1};
  std::vector<const char *> input_names;
  for (const auto &s : self.input_names)
    input_names.push_back(s.c_str());
  std::vector<const char *> output_names;
  for (const auto &s : self.output_names)
    output_names.push_back(s.c_str());

  std::vector<Ort::Value> inputs;
  // noisey future motion sequence
  inputs.push_back(std::move(Ort::Value::CreateTensor<float>(
      memory_info, x_t_data.data(), x_t_data.size(), self.x_t_shape.data(),
      self.x_t_shape.size())));
  // timestep
  inputs.push_back(std::move(Ort::Value::CreateTensor<int64_t>(
      memory_info, timestep_data.data(), timestep_data.size(),
      timestep_shape.data(), timestep_shape.size())));
  // past motion sequence condition
  inputs.push_back(std::move(Ort::Value::CreateTensor<float>(
      memory_info, self.past_motion_data.data(), self.past_motion_data.size(),
      self.past_motion_shape.data(), self.past_motion_shape.size())));
  // trajectory position condition
  inputs.push_back(std::move(Ort::Value::CreateTensor<float>(
      memory_info, self.traj_pos_data.data(), self.traj_pos_data.size(),
      self.traj_pos_shape.data(), self.traj_pos_shape.size())));
  // trajectory facing direction condition
  inputs.push_back(std::move(Ort::Value::CreateTensor<float>(
      memory_info, self.traj_facing_data.data(), self.traj_facing_data.size(),
      self.traj_facing_shape.data(), self.traj_facing_shape.size())));
  // motion style discrete label condition
  inputs.push_back(std::move(Ort::Value::CreateTensor<int64_t>(
      memory_info, self.style_idx_data.data(), self.style_idx_data.size(),
      self.style_idx_shape.data(), self.style_idx_shape.size())));

  /* ---------------- Diffusion loop ---------------- */
  for (int t = self.diffusion_steps - 1; t >= 0; --t) {
    // Update the value in the buffer; the tensor wrapper sees this change
    // automatically
    *(inputs[1].GetTensorMutableData<int64_t>()) = static_cast<int64_t>(t);

    auto outputs = self.session.Run(
        Ort::RunOptions{nullptr}, input_names.data(), inputs.data(),
        inputs.size(), output_names.data(), output_names.size());
    // batch_size (1), pose_token_dim, past_points + future_points
    const float *pred_noise = outputs[0].GetTensorData<float>();
    float *x_t_ptr = inputs[0].GetTensorMutableData<float>();
    const float coef1 = self.posterior_mean_coef1[t];
    const float coef2 = self.posterior_mean_coef2[t];
    const float std_dev =
        std::sqrt(std::exp(self.posterior_log_variance_clipped[t]));

    // Update x_t_data directly
    for (int p = 0; p < self.pose_token_dim; p++) {
      for (int f = 0; f < self.future_points; f++) {
        int i = 1 * p * self.future_points + f;
        int j = 1 * p * (self.future_points + self.past_points) + f +
                self.past_points;
        float posterior_mean = coef1 * pred_noise[j] + coef2 * x_t_ptr[i];
        float noise = (t > 0) ? normal_dist(gen) : 0.0f;
        x_t_ptr[i] = posterior_mean + std_dev * noise;
      }
    }
  }

  return x_t_data;
}

void diffusion::submit_inference(std::function<void(std::vector<float>)> cb) {
  {
    std::lock_guard<std::mutex> lk(queue_mutex);
    task_queue.push([this, cb = std::move(cb)]() mutable {
      toolkit::stopwatch timer;
      timer.reset();
      std::vector<float> result = run_model_inference(*this);
      float inference_time = timer.elapse_ms();
      printf("Inference complete, takes %.3f ms\n", inference_time);
      std::lock_guard<std::mutex> lk2(completion_mutex);
      completion_queue.push(
          [cb = std::move(cb), result = std::move(result)]() mutable {
            cb(std::move(result));
          });
    });
  }
  queue_cv.notify_one();
}

void diffusion::process_completions() {
  // Should be called from main thread periodically (e.g. each frame)
  std::function<void()> fn;
  while (true) {
    {
      std::lock_guard<std::mutex> lk(completion_mutex);
      if (completion_queue.empty())
        break;
      fn = std::move(completion_queue.front());
      completion_queue.pop();
    }
    try {
      fn();
    } catch (...) {
      // swallow callback exceptions
    }
  }
}

void diffusion::worker_loop() {
  while (!stop_worker.load()) {
    std::function<void()> task;
    {
      std::unique_lock<std::mutex> lk(queue_mutex);
      queue_cv.wait(
          lk, [this] { return stop_worker.load() || !task_queue.empty(); });
      if (stop_worker.load() && task_queue.empty())
        break;
      task = std::move(task_queue.front());
      task_queue.pop();
    }
    try {
      task();
    } catch (...) {
      // ignore task errors
    }
  }
}

diffusion::~diffusion() {
  stop_worker.store(true);
  queue_cv.notify_all();
  if (worker_thread.joinable())
    worker_thread.join();
  // clear queues
  {
    std::lock_guard<std::mutex> lk(queue_mutex);
    while (!task_queue.empty())
      task_queue.pop();
  }
  {
    std::lock_guard<std::mutex> lk(completion_mutex);
    while (!completion_queue.empty())
      completion_queue.pop();
  }
}