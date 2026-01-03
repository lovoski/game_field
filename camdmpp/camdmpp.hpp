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
  bool hide_mouse = true;
  std::int64_t __cur_exec_fixed = 0;
  double __cur_time = 0.0f, fixed_interval = 1.0f / 60.0f;

  diffusion model;
  // How many frames of motion have been applied to the character
  unsigned int applied_frames = 0;
  // Make a new prediction when these frames are applied
  unsigned int submit_prediction_interval = 20;
  std::atomic_bool waiting_for_model_output{false};

  // Caches for character's state
  std::array<std::vector<math::quat>, 100>
      joint_rotation_cache; // converted from model_output
  std::array<std::vector<math::vector3>, 100>
      root_rel_pos_cache; // converted from model_output
  std::array<std::vector<math::quat>, 100>
      root_rel_rot_cache; // converted from model_output

  math::vector3 char_root_world_pos = math::vector3::Zero();
  math::quat char_root_world_rot = math::quat::Identity();

  // Trajectory predicted from user input with spring damper heuristics
  std::array<math::vector3, 5> synthetic_traj_pos, synthetic_traj_facing;

  /**
   * Three things are done inside this function:
   *   1. make new predictions to the trajectory based on user input
   *   2. apply pose to the character
   *   3. submit a new prediction when counter reaches a threashold
   */
  void fixed_interval_logic();
};

}; // namespace toolkit::opengl3d