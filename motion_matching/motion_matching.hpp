/**
 * Algorithm reference:
 * https://github.com/orangeduck/GenoViewPython-MotionMatching
 * https://www.theorangeduck.com/page/code-vs-data-driven-displacement
 */
#pragma once

#include "toolkit/opengl3d/engine.hpp"

namespace toolkit::opengl3d {

#define MM_FEATURE_DIM 27

struct mm_context {
  math::vector3 root_world_pos = math::vector3::Zero(),
                root_world_vel = math::vector3::Zero(),
                root_world_acc = math::vector3::Zero(),
                root_world_ang = math::vector3::Zero();
  math::quat root_world_rot = math::quat::Identity();
  std::array<math::vector3, 3> traj_world_pos, traj_world_dir;
};

class motion_matching_app : public engine3d {
public:
  void handle_custom_initialization() override;
  void handle_game_logic_tick(float dt) override;
  void handle_engine_gui() override;

  void animate_player(float dt);

private:
  bool mouse_hidden = false;
  entt::entity player_entity = entt::null;
  float cam_move_speed = 100.0f;
  float cam_angle_horizontal = 0.0f, cam_angle_vertical = 30.0f;
  float joystick_deadzone = 0.2f;

  std::int64_t __cur_exec_fixed = 0;
  double __cur_time = 0.0f, fixed_interval = 1.0f / 60.0f;

  std::map<std::string, int> joint_name_to_idx;

  float current_bias = 0.01, approx_bias = 0.01;
  float vel_halflife = 0.2f, rot_halflife = 0.2f;
  float search_time = 0.25f, search_timer = search_time;

  float traj_sample_time = 0.33f;
  mm_context context;

  math::quat desired_rot = math::quat::Identity();
  math::vector3 desired_vel = math::vector3::Zero(),
                desired_dir = math::vector3(0, 0, 1);
  math::quat db_start_rot = math::quat::Identity(),
             ent_start_rot = math::quat::Identity();

  std::vector<math::vector3> data_joints_world_pos;

  int anim_range = 0, anim_frame = 0;
  int best_range = 0, best_frame = 0;

  std::vector<math::quat> off_rot;
  std::vector<math::vector3> off_pos, off_vel, off_ang;

  std::vector<std::array<float, MM_FEATURE_DIM>> X;
  std::array<float, MM_FEATURE_DIM> Xoffset, Xscale;
  std::vector<std::vector<math::vector3>> Ypos, Yvel, Yang;
  std::vector<std::vector<math::quat>> Yrot;
  std::vector<int> YrangeStarts, YrangeStops, parents;
  std::vector<std::string> names;
  std::array<float, MM_FEATURE_DIM>
  compute_runtime_feature(int frame, const mm_context &ctx);
  float feature_dist(std::array<float, MM_FEATURE_DIM> &feat0,
                     std::array<float, MM_FEATURE_DIM> &feat1);
};

}; // namespace toolkit::opengl3d