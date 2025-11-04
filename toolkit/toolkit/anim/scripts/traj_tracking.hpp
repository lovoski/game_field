#pragma once

#include "toolkit/anim/components/actor.hpp"
#include "toolkit/anim/scripts/motion_matching.hpp"
#include "toolkit/opengl/base.hpp"
#include "toolkit/opengl/draw.hpp"
#include "toolkit/scriptable.hpp"
#include <cnpy.h>

namespace toolkit::anim {

struct mm_context {
  math::vector3 root_world_pos = math::vector3::Zero(),
                root_world_vel = math::vector3::Zero(),
                root_world_acc = math::vector3::Zero(),
                root_world_ang = math::vector3::Zero();
  math::quat root_world_rot = math::quat::Identity();
  std::array<math::vector3, 3> traj_world_pos, traj_world_dir;
};

class traj_tracking : public scriptable {
public:
  void start() override;
  void destroy() override;

  void draw_to_scene(iapp *app, transform &cam_trans, camera &cam_comp) override;
  void draw_gui(iapp *app) override;

  void update(iapp *app, float dt) override;
  void fixedupdate(iapp *app, float dt);

private:
  // if trajectory loaded, then follow the trajectory instead of user input
  int applied_traj_frame = 0;
  std::vector<math::vector3> traj_points, traj_facing;
  bool db_loaded, mapping_loaded = false, trajectory_loaded = false;
  std::string db_filepath = "", mapping_filepath = "", traj_filepath = "";
  std::map<std::string, int> joint_name_to_idx;

  math::quat desired_rot = math::quat::Identity();

  math::quat db_start_rot = math::quat::Identity(),
             ent_start_rot = math::quat::Identity();

  mm_context cur_context;
  std::tuple<float, int, int> lhmm(mm_context context, int cur_frame,
                                   int cur_range, int k, int l);

  void animate_character_with_context(float dt);

  float joystick_deadzone = 0.2f;
  float current_bias = 0.01, approx_bias = 0.01;
  float vel_halflife = 0.2f, rot_halflife = 0.2f;
  float search_time = 0.5f, search_timer = search_time;

  bool inertialize = false;

  std::vector<math::quat> actor_bind_rot;
  std::vector<math::vector3> actor_bind_pos, data_joints_world_pos;
  std::vector<math::matrix4> actor_bind_mat;

  std::vector<float> t_times = {20.0f / 60.0f, 40.0f / 60.0f, 60.0f / 60.0f};

  std::vector<math::vector3> root_pos_history;

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
  compute_runtime_feature(int frame, const mm_context &context);
  float feature_dist(std::array<float, MM_FEATURE_DIM> &feat0,
                     std::array<float, MM_FEATURE_DIM> &feat1);
};
DECLARE_SCRIPT(traj_tracking, animation)

}; // namespace toolkit::anim