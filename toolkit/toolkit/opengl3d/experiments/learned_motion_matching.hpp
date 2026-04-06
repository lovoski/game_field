#pragma once

#include "toolkit/opengl3d/components/actor.hpp"
#include "toolkit/opengl3d/engine.hpp"
#include "toolkit/opengl3d/native_subsys.hpp"
#include "toolkit/system.hpp"

class learned_motion_matching : public toolkit::opengl3d::sub_system {
public:
  void start(entt::registry &registry) override {
  }
  void update(entt::registry &registry, float dt) override {
  }

  void draw_to_scene(entt::registry &registry, toolkit::transform &cam_trans,
                     toolkit::opengl3d::camera &cam_comp) override {
  }
  void draw_gui(entt::registry &registry, entt::entity entity) override {
  }
};
DECLARE_SUB_SYSTEM(learned_motion_matching)
