#pragma once

#include "toolkit/anim/components/actor.hpp"
#include "toolkit/opengl/base.hpp"
#include "toolkit/opengl/draw.hpp"
#include "toolkit/opengl/editor.hpp"
#include "toolkit/scriptable.hpp"

namespace toolkit::anim {

class tps_cam_controller : public sub_system {
public:
  void start() override {}
  void destroy() override {}
  void draw_gui(entt::registry &registry, entt::entity entity) override {}
  void lateupdate(entt::registry &registry, float dt) override {}

  float distance = 5.0f, sensitivity_x = 5.0f, sensitivity_y = 5.0f;
  float min_angle_y = -20.0f, max_angle_y = 60.0f;
  float current_rotation_x = 0.0f, current_rotation_y = 0.0f;
  math::vector3 offset = math::vector3(0, 2, 0);
};
DECLARE_SUB_SYSTEM(tps_cam_controller, animation)

}; // namespace toolkit::anim