/**
 * Algorithm reference:
 * https://github.com/orangeduck/GenoViewPython-MotionMatching
 * https://www.theorangeduck.com/page/code-vs-data-driven-displacement
 */
#pragma once

#include "toolkit/anim/components/actor.hpp"
#include "toolkit/opengl/base.hpp"
#include "toolkit/opengl/draw.hpp"
#include "toolkit/scriptable.hpp"
#include <cnpy.h>

namespace toolkit::anim {

#define MM_FEATURE_DIM 27

struct mm_context {
  math::vector3 root_world_pos = math::vector3::Zero(),
                root_world_vel = math::vector3::Zero(),
                root_world_acc = math::vector3::Zero(),
                root_world_ang = math::vector3::Zero();
  math::quat root_world_rot = math::quat::Identity();
  std::array<math::vector3, 3> traj_world_pos, traj_world_dir;
};

class motion_matching : public sub_system {
public:
  void start(entt::registry &registry) override;
  void destroy(entt::registry &registry) override;

  void draw_to_scene(entt::registry &registry, transform &cam_trans,
                     camera &cam_comp) override;
  void draw_gui(entt::registry &registry, entt::entity entity) override;

  void update(entt::registry &registry, float dt) override;
  void fixedupdate(entt::registry &registry, float dt) override;

private:
  bool db_loaded, mapping_loaded = false;
  std::string db_filepath = "", mapping_filepath = "";
  std::map<std::string, int> joint_name_to_idx;

  float joystick_deadzone = 0.2f;
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

  // std::vector<math::quat> actor_bind_rot;
  // std::vector<math::vector3> actor_bind_pos;
  // std::vector<math::matrix4> actor_bind_mat;
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
DECLARE_SUB_SYSTEM(motion_matching, animation)

}; // namespace toolkit::anim