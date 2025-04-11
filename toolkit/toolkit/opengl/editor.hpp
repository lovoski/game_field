#pragma once

#include "toolkit/opengl/base.hpp"
#include "toolkit/scriptable.hpp"
#include "toolkit/system.hpp"
#include "toolkit/utils.hpp"

#include "toolkit/transform.hpp"

#include "toolkit/opengl/components/camera.hpp"
#include "toolkit/opengl/rasterize/mixed.hpp"

namespace toolkit::opengl {

class editor;

class editor : public iapp {
public:
  void init() {
    context::get_instance().init();
    dm_sys = add_sys<default_mixed>();
    transform_sys = add_sys<transform_system>();
    script_sys = add_sys<script_system>();
  }

  void run();

  ImGuiIO *io = nullptr;
  stopwatch timer;

  bool with_translate = false, with_rotate = false, with_scale = false;
  ImGuizmo::OPERATION current_gizmo_operation() {
    return (ImGuizmo::OPERATION)(
        (with_translate ? (int)ImGuizmo::OPERATION::TRANSLATE : 0) |
        (with_rotate ? (int)ImGuizmo::OPERATION::ROTATE : 0) |
        (with_scale ? (int)ImGuizmo::OPERATION::SCALE : 0));
  };
  ImGuizmo::MODE current_gizmo_mode = ImGuizmo::MODE::WORLD;

  entt::entity active_camera = entt::null, selected_entity = entt::null;

  default_mixed *dm_sys = nullptr;
  transform_system *transform_sys = nullptr;
  script_system *script_sys = nullptr;

  void draw_gizmos(bool enable = true);
};

// entt::entity CreateCube(entt::registry &registry);
// entt::entity CreateSphere(entt::registry &registry);
// entt::entity CreateCylinder(entt::registry &registry);
// entt::entity CreateCone(entt::registry &registry);

}; // namespace toolkit::opengl