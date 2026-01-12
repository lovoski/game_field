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
  joint_parents = config["joint_parents"].get<std::vector<int>>();
  input_names = config["input_names"].get<std::vector<std::string>>();
  output_names = config["output_names"].get<std::vector<std::string>>();
  auto joint_offsets_data = config["joint_offsets"].get<std::vector<float>>();
  joint_offsets.resize(joint_num, toolkit::math::vector3::Zero());
  for (int i = 0; i < joint_num; i++) {
    joint_offsets[i].x() = joint_offsets_data[3 * i + 0];
    joint_offsets[i].y() = joint_offsets_data[3 * i + 1];
    joint_offsets[i].z() = joint_offsets_data[3 * i + 2];
  }
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
  // IntraOp threads: threads used to parallelize the work inside a single
  // operator (a heavy kernel like MatMul, Conv, GEMM). When a single op is the
  // bottleneck, more IntraOp threads let that op use more CPU cores and finish
  // faster.
  session_options.SetIntraOpNumThreads(
      std::min(8u, std::thread::hardware_concurrency()));
  // InterOp threads: threads used to run independent graph nodes (different
  // ops) concurrently. If the graph has many independent small ops, InterOp>1
  // can run multiple nodes in parallel.
  session_options.SetInterOpNumThreads(1);
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

  // setup random number generator
  ziggurat::r4_nor_setup(ziggurat_kn, ziggurat_fn, ziggurat_wn);

  // Start background worker for async inference
  stop_worker.store(false);
  if (!worker_thread.joinable())
    worker_thread = std::thread(&diffusion::worker_loop, this);
}

std::vector<float> diffusion::run_model_inference() {
  for (float &v : x_t_data)
    v = ziggurat::r4_nor(ziggurat_jsr, ziggurat_kn, ziggurat_fn, ziggurat_wn);

  std::vector<int64_t> timestep_data(1, 0);
  const std::vector<int64_t> timestep_shape{1};
  std::vector<const char *> _input_names;
  for (const auto &s : input_names)
    _input_names.push_back(s.c_str());
  std::vector<const char *> _output_names;
  for (const auto &s : output_names)
    _output_names.push_back(s.c_str());

  std::vector<Ort::Value> inputs;
  // noisey future motion sequence
  inputs.push_back(std::move(Ort::Value::CreateTensor<float>(
      memory_info, x_t_data.data(), x_t_data.size(), x_t_shape.data(),
      x_t_shape.size())));
  // timestep
  inputs.push_back(std::move(Ort::Value::CreateTensor<int64_t>(
      memory_info, timestep_data.data(), timestep_data.size(),
      timestep_shape.data(), timestep_shape.size())));
  // past motion sequence condition
  inputs.push_back(std::move(Ort::Value::CreateTensor<float>(
      memory_info, past_motion_data.data(), past_motion_data.size(),
      past_motion_shape.data(), past_motion_shape.size())));
  // trajectory position condition
  inputs.push_back(std::move(Ort::Value::CreateTensor<float>(
      memory_info, traj_pos_data.data(), traj_pos_data.size(),
      traj_pos_shape.data(), traj_pos_shape.size())));
  // trajectory facing direction condition
  inputs.push_back(std::move(Ort::Value::CreateTensor<float>(
      memory_info, traj_facing_data.data(), traj_facing_data.size(),
      traj_facing_shape.data(), traj_facing_shape.size())));
  // motion style discrete label condition
  inputs.push_back(std::move(Ort::Value::CreateTensor<int64_t>(
      memory_info, style_idx_data.data(), style_idx_data.size(),
      style_idx_shape.data(), style_idx_shape.size())));

  /* ---------------- Diffusion loop ---------------- */
  for (int t = diffusion_steps - 1; t >= 0; --t) {
    // Update the value in the buffer; the tensor wrapper sees this change
    // automatically
    *(inputs[1].GetTensorMutableData<int64_t>()) = static_cast<int64_t>(t);

    auto outputs = session.Run(Ort::RunOptions{nullptr}, _input_names.data(),
                               inputs.data(), inputs.size(),
                               _output_names.data(), output_names.size());
    // batch_size (1), pose_token_dim, past_points + future_points
    const float *pred_noise = outputs[0].GetTensorData<float>();
    float *x_t_ptr = inputs[0].GetTensorMutableData<float>();
    const float coef1 = posterior_mean_coef1[t];
    const float coef2 = posterior_mean_coef2[t];
    const float std_dev =
        std::sqrt(std::exp(posterior_log_variance_clipped[t]));

    // Update x_t_data directly
    for (int p = 0; p < pose_token_dim; p++) {
      for (int f = 0; f < future_points; f++) {
        int i = 1 * p * future_points + f;
        int j = 1 * p * (future_points + past_points) + f + past_points;
        float posterior_mean = coef1 * pred_noise[j] + coef2 * x_t_ptr[i];
        float noise = (t > 0) ? ziggurat::r4_nor(ziggurat_jsr, ziggurat_kn,
                                                 ziggurat_fn, ziggurat_wn)
                              : 0.0f;
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
      std::vector<float> result = run_model_inference();
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