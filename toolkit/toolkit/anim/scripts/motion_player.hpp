#pragma once

#include "toolkit/anim/components/actor.hpp"
#include "toolkit/loaders/bvh.hpp"
#include "toolkit/opengl/base.hpp"
#include "toolkit/opengl/draw.hpp"
#include "toolkit/scriptable.hpp"
#include <cnpy.h>

namespace toolkit::anim {

class bvh_motion_player : public scriptable {
public:
  void start() override {}
  void destroy() override {}

  void update(iapp *app, float dt) override;
  void lateupdate(iapp *app, float dt) override;
  void draw_gui(iapp *app) override;

  entt::entity load_motion(std::string filepath);

  static inline float current_time = 0.0f;
  static inline float play_speed = 1.0f;
  static inline bool auto_play = false;

  bool apply_motion = true;

private:
  bool motion_loaded = false;
  assets::bvh_data motion;
};
DECLARE_SCRIPT(bvh_motion_player, animation)

void import_all_bvh_motion(entt::registry &registry, std::string dirpath);

}; // namespace toolkit::anim
