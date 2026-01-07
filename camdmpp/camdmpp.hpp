#pragma once

#include "diffusion.hpp"

#include "toolkit/opengl3d/engine.hpp"

namespace toolkit::opengl3d {

class camdmpp : public engine3d {
public:
  void handle_custom_initialization() override;
  void handle_game_logic_tick(float dt) override;
  void handle_engine_gui() override;

private:
  // application variables
  bool hide_mouse = false, debug_draw_trajectory = true,
       debug_draw_skeleton = true;
  float cam_move_speed = 100.0f;
  float cam_angle_horizontal = 0.0f, cam_angle_vertical = 30.0f;
  std::int64_t __cur_exec_fixed = 0;
  double __cur_time = 0.0f, fixed_interval = 1.0f / 60.0f;
  entt::entity player_entity = entt::null;

  diffusion model;
  // How many frames of motion have been applied to the character
  unsigned int applied_frames = 0;
  // Make a new prediction when these frames are applied
  unsigned int submit_prediction_interval = 200;
  std::atomic_bool waiting_for_model_output{false};

  // Caches for character's state, converted from model_output
  std::array<std::vector<math::quat>, 100> joint_rotation_cache;
  std::array<math::vector3, 100> root_rel_pos_cache;
  std::array<math::quat, 100> root_rel_rot_cache;

  // Used for trajectory update and root position update
  math::vector3 char_root_world_pos = math::vector3::Zero(),
                char_root_world_vel = math::vector3::Zero(),
                char_root_world_acc = math::vector3::Zero(),
                char_root_world_ang = math::vector3::Zero();
  math::quat char_root_world_rot = math::quat::Identity();

  // Caches for network input
  std::vector<float> i_past_motion; // (1, pose_token_dim, past_points)
  std::vector<float> i_traj_facing; // (1, 10)
  std::vector<float> i_traj_pos;    // (1, 10)
  std::vector<int64_t> i_style_idx; // (1)

  // Trajectory predicted from user input with spring damper heuristics
  float vel_halflife = 0.2f, rot_halflife = 0.2f,
        traj_sample_time = 1.0f / 5.0f, velocity_scale = 5.0f;
  std::array<math::vector3, 5> _traj_world_vel, _traj_world_pos,
      _traj_world_dir;
  math::vector3 desired_vel = math::vector3::Zero(),
                desired_dir = math::vector3(0, 0, 1);

  /**
   * Three things are done inside this function:
   *   1. make new predictions to the trajectory based on user input
   *   2. apply pose to the character
   *   3. submit a new prediction when counter reaches a threashold
   */
  void fixed_interval_logic();

  void debug_draw();
};

}; // namespace toolkit::opengl3d