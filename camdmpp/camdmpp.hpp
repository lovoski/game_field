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
  bool debug_draw_trajectory = true, debug_draw_skeleton = true,
       draw_ground_mesh = true;
  float cam_move_speed = 60.0f, cam_stick_speed = 120.0f,
        cam_distance = 6.0f;
  float cam_angle_horizontal = 0.0f, cam_angle_vertical = 15.0f;
  float camera_follow_halflife = 0.25f;
  float cam_focus_height = 0.9f, cam_look_ahead = 0.1f;
  float move_stick_deadzone = 0.18f, camera_turn_deadzone = 0.15f;
  math::vector3 camera_follow_vel = math::vector3::Zero(),
                camera_follow_ang = math::vector3::Zero();

  std::int64_t __cur_exec_fixed = 0;
  double __cur_time = 0.0f, fixed_interval = 1.0f / 60.0f;
  entt::entity player_entity = entt::null, ground_entity = entt::null;
  float display_inference_time = 0.0f;

  // blending parameters
  bool enable_inertia_blending = true;
  int inertia_blend_wnd = 15;
  float inertia_halflife = 0.06f, inertia_lambda;

  diffusion model;
  // How many frames of motion have been applied to the character
  unsigned int applied_frames = 0;
  // Make a new prediction when these frames are applied
  unsigned int submit_prediction_interval = 15;
  // In each prediction, how many frames are used
  unsigned int switch_prediction_interval = 20;

  // Caches for character's state, converted from model_output
  static const int cache_size = 100;
  std::array<std::vector<math::quat>, cache_size> joint_rotation_cache;
  std::array<math::vector3, cache_size> root_rel_pos_cache;
  std::array<math::quat, cache_size> root_rel_rot_cache;
  std::array<float, cache_size> root_height_cache;
  // The update of model output is performed in a "double buffer" way, the async
  // inference is dispatched when "submit_prediction_interval" is reached, after
  // the inference finishes, the newly predicted output will be store in the
  // cache that is not currently being used, so the update can go smoothly from
  // the last prediction to the next when "switch_prediction_interval" is
  // reached.
  bool use_front_buffer = true;

  // Used for trajectory update and root position update
  math::vector3 char_root_world_pos = math::vector3::Zero(),
                char_root_world_vel = math::vector3::Zero(),
                char_root_world_acc = math::vector3::Zero(),
                char_root_world_ang = math::vector3::Zero();
  math::quat proj_char_root_world_rot = math::quat::Identity();
  std::vector<math::quat> char_repair_c;
  std::vector<int> char_joint_parents;
  std::map<int, int> char_data_to_actor;

  // Caches for network input
  std::vector<float> i_past_motion; // (1, pose_token_dim, past_points)
  // std::vector<float> i_traj_facing; // (1, 10)
  // std::vector<float> i_traj_pos;    // (1, 10)
  // std::vector<float> i_style_idx;   // (1, 1)
  std::vector<float> i_traj; // (1, 5, 7), [pos_x, pos_z, height_c, height_l,
                             // height_r, fac_x, fac_z]

  // Trajectory predicted from user input with spring damper heuristics
  float vel_halflife = 0.2f, rot_halflife = 0.2f,
        traj_sample_time = 1.0f / 5.0f, velocity_scale = 15.0f;
  std::array<math::vector3, 5> _traj_world_vel, _traj_world_pos,
      _traj_world_dir, _traj_world_height;
  math::vector3 desired_vel = math::vector3::Zero(),
                desired_dir = math::vector3(0, 0, 1);

  struct terrain_triangle {
    math::vector3 p0 = math::vector3::Zero(), p1 = math::vector3::Zero(),
                  p2 = math::vector3::Zero();
    math::vector2 xz0 = math::vector2::Zero(), xz1 = math::vector2::Zero(),
                  xz2 = math::vector2::Zero();
  };
  float terrain_probe_half_width = 0.2f;
  float terrain_default_height = 0.0f;
  math::vector2 terrain_grid_min = math::vector2::Zero(),
                terrain_grid_max = math::vector2::Zero(),
                terrain_grid_cell_size = math::vector2::Ones();
  int terrain_grid_width = 0, terrain_grid_depth = 0;
  std::vector<terrain_triangle> terrain_triangles;
  std::vector<std::vector<int>> terrain_grid_cells;

  /**
   * Three things are done inside this function:
   *   1. make new predictions to the trajectory based on user input
   *   2. apply pose to the character, fill in caches for network input
   *   3. submit a new prediction when counter reaches a threashold
   */
  void fixed_interval_logic();

  void build_terrain_sampler();
  float sample_terrain_height(const math::vector2 &xz,
                              float fallback_height) const;
  static bool sample_terrain_triangle(const terrain_triangle &triangle,
                                      const math::vector2 &xz, float &height);

  void predict_trajectory();
  void apply_pose_and_refill();
  void predict_new_tokens();

  void debug_draw();
};

}; // namespace toolkit::opengl3d