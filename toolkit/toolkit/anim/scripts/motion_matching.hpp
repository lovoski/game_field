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

std::tuple<math::vector3, math::vector3>
query_left_right_joystick(float deadzone = 0.1);

void inertialize_transition_position(std::vector<math::vector3> &off_pos,
                                     std::vector<math::vector3> &off_vel,
                                     std::vector<math::vector3> src_pos,
                                     std::vector<math::vector3> src_vel,
                                     std::vector<math::vector3> target_pos,
                                     std::vector<math::vector3> target_vel);

void inertialize_transition_rotation(std::vector<math::quat> &off_rot,
                                     std::vector<math::vector3> &off_ang,
                                     std::vector<math::quat> src_rot,
                                     std::vector<math::vector3> src_ang,
                                     std::vector<math::quat> target_rot,
                                     std::vector<math::vector3> target_ang);

class motion_matching : public scriptable {
public:
  void start() override;
  void destroy() override;

  void draw_to_scene(iapp *app) override;
  void draw_gui(iapp *app) override;

  void update(iapp *app, float dt) override;
  void fixedupdate(iapp *app, float dt);

private:
  bool db_loaded, mapping_loaded = false;
  std::string db_filepath = "", mapping_filepath = "";
  std::map<std::string, int> joint_name_to_idx;

  int cur_exec_fixed = 0;
  float joystick_deadzone = 0.2f;
  float current_bias = 0.01, approx_bias = 0.01;
  float vel_halflife = 0.2f, rot_halflife = 0.2f;
  float cur_time = 0.0f, fixed_interval = 1.0f / 60.0f;
  float search_time = 0.25f, search_timer = search_time;
  math::vector3 root_pos = math::vector3::Zero(),
                root_vel = math::vector3::Zero(),
                root_acc = math::vector3::Zero(),
                root_ang = math::vector3::Zero();
  // math::vector3 chara_pos = math::vector3::Zero(),
  //               chara_vel = math::vector3::Zero();
  math::quat root_rot = math::quat::Identity(),
             desired_rot = math::quat::Identity();
  math::vector3 desired_vel = math::vector3::Zero(),
                desired_dir = math::vector3(0, 0, 1);

  std::vector<math::quat> actor_bind_rot;
  std::vector<math::vector3> actor_bind_pos, data_joints_world_pos;
  std::vector<math::matrix4> actor_bind_mat;

  std::vector<float> t_times = {20.0f / 60.0f, 40.0f / 60.0f, 60.0f / 60.0f};
  std::vector<math::vector3> t_pos =
      std::vector<math::vector3>(3, math::vector3::Zero());
  std::vector<math::vector3> t_vel =
      std::vector<math::vector3>(3, math::vector3::Zero());
  std::vector<math::vector3> t_dir =
      std::vector<math::vector3>(3, math::vector3(0, 0, 1));
  std::vector<math::quat> t_rot =
      std::vector<math::quat>(3, math::quat::Identity());

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
  std::array<float, MM_FEATURE_DIM> compute_runtime_feature(int frame);
  float feature_dist(std::array<float, MM_FEATURE_DIM> &feat0,
                     std::array<float, MM_FEATURE_DIM> &feat1);
};
DECLARE_SCRIPT(motion_matching, animation)

}; // namespace toolkit::anim