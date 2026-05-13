#pragma once

#include "diffusion.hpp"

#include "toolkit/opengl3d/components/actor.hpp"
#include "toolkit/opengl3d/engine.hpp"

namespace toolkit::opengl3d {

class camdmpp : public engine3d {
public:
  void handle_custom_initialization() override;
  void handle_game_logic_tick(float dt) override;
  void handle_engine_gui() override;

private:
  bool hide_mouse = true;
  bool camera_as_facing_direction = true;

  // motion terrain adjustment
  bool enable_motion_terrain_adjustment = true;

  // ik related values
  bool enable_foot_locking = true;
  float ik_value_left = 0.0f, ik_value_right = 0.0f;

  // character locomotion states
  bool char_running = false, char_crouching = false, char_idle = false;
  void update_character_states(float dt);

  // camera related
  float camera_horizontal_angle = 180.0f, camera_vertical_angle = 30.0f,
        camera_distance = 5.0f, camera_height = 0.0f, mouse_sensitivity = 0.3f,
        camera_follow_speed = 8.0f;
  math::vector3 camera_forward;
  float min_vertical_angle = -20.0f, max_vertical_angle = 80.0f;
  void update_camera(float dt);

  // user input and trajectory prediction related
  math::vector3 move_input = math::vector3::Zero();
  math::vector3 player_last_pos = math::vector3::Zero(),
                player_curr_pos = math::vector3::Zero();
  math::quat player_last_rot = math::quat::Identity(),
             player_curr_rot = math::quat::Identity();
  // use a velocity spring to track the simulation body movement predicted from
  // player velocity and user input
  float sim_acceleration = 5.0f, sim_deceleration = 5.0f,
        sim_move_speed_walk = 1.0f, sim_move_speed_run = 1.7f;
  math::vector3 player_vel = math::vector3::Zero(),
                player_ang = math::vector3::Zero();
  float vel_halflife = 0.2f, rot_halflife = 0.1f,
        traj_sample_time = 1.0f / 5.0f;
  std::vector<math::vector3> _traj_world_vel, _traj_world_pos, _traj_world_dir;
  std::vector<std::vector<float>> _traj_world_height;
  math::vector3 desired_vel = math::vector3::Zero(),
                desired_dir = math::vector3(0, 0, 1);
  math::quat network_root_rot = math::quat::Identity();

  // system setup related
  std::int64_t __cur_exec_fixed = 0;
  double __cur_time = 0.0f, fixed_interval = 1.0f / 60.0f;
  entt::entity player_entity = entt::null, ground_entity = entt::null;
  entt::entity left_thigh_entity = entt::null, right_thigh_entity = entt::null;
  float display_inference_time = 0.0f;

  // blending parameters
  bool enable_inertia_blending = true;
  int inertia_blend_wnd = 15;
  float inertia_halflife = 0.06f, inertia_lambda;

  diffusion model;
  // How many frames of motion have been applied to the character
  unsigned int applied_frames = 0;
  // Make a new prediction when these frames are applied
  unsigned int submit_prediction_interval = 6;
  // In each prediction, how many frames are used
  unsigned int switch_prediction_interval = 12;

  // Caches for character's state, converted from model_output
  static const int cache_size = 220;
  std::array<std::vector<math::quat>, cache_size> joint_rotation_cache;
  std::array<math::vector3, cache_size> root_rel_pos_cache;
  std::array<math::quat, cache_size> root_rel_rot_cache;
  std::array<float, cache_size> root_height_cache;
  std::array<float, cache_size> ik_left_cache, ik_right_cache;
  // The update of model output is performed in a "double buffer" way, the async
  // inference is dispatched when "submit_prediction_interval" is reached, after
  // the inference finishes, the newly predicted output will be store in the
  // cache that is not currently being used, so the update can go smoothly from
  // the last prediction to the next when "switch_prediction_interval" is
  // reached.
  bool use_front_buffer = true;

  std::vector<math::quat> char_repair_c;
  std::vector<int> char_joint_parents;
  std::map<int, int> char_data_to_actor, tpose_to_data;

  // Caches for network input
  std::vector<float> i_past_motion; // (1, pose_token_dim, past_points)
  // std::vector<float> i_traj_facing; // (1, 10)
  // std::vector<float> i_traj_pos;    // (1, 10)
  // std::vector<float> i_style_idx;   // (1, 1)
  std::vector<float> i_traj; // (1, 40, x), [pos_x, pos_z, height_c, height_l,
                             // height_r, fac_x, fac_z]

  float terrain_default_height = 0.0f;
  math::vector2 terrain_grid_min = math::vector2::Zero(),
                terrain_grid_max = math::vector2::Zero(),
                terrain_grid_cell_size = math::vector2::Ones(),
                terrain_grid_inv_cell_size = math::vector2::Ones();
  int terrain_grid_width = 0, terrain_grid_depth = 0;
  std::vector<float> terrain_height_map;

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
  float sample_terrain_path_length(const math::vector2 &start_xz,
                                   float start_height,
                                   const math::vector2 &direction,
                                   float planar_distance) const;

  void resample_trajectory_on_terrain(const math::vector2 &start_xz,
                                      float start_height);
  void predict_trajectory();
  void apply_pose_and_refill();
  void predict_new_tokens();

  void debug_draw();

  // post processing
  std::array<entt::entity, 4> biped_chain_left, biped_chain_right;
  // Per-foot IK state that has to survive across buffer swaps and async
  // re-predictions. Without this the IK target/strength would re-derive
  // from scratch each prediction, which causes the foot to pop at the
  // boundary even though the cached pose itself is continuous (inertia
  // blending only smooths the network's joint rotations, not the IK
  // residual on top). `anchor_pos` is the world-space target the IK was
  // driving the foot toward; `lock_weight` is the IK strength applied at
  // the last frame of the previous buffer; `active` flags whether that
  // last frame was inside a contact span.
  struct ik_foot_carry {
    math::vector3 anchor_pos = math::vector3::Zero();
    float         lock_weight = 0.0f;
    bool          active = false;
  };
  ik_foot_carry ik_left_carry, ik_right_carry;
  void postprocessing_ik(int buffer_start_idx, int transition_from_idx,
                         int frame_count);
};

}; // namespace toolkit::opengl3d