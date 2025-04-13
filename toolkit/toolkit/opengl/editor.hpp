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
  void init();
  void run();

  ImGuiIO *imgui_io = nullptr;
  stopwatch timer;

  bool with_translate = false, with_rotate = false, with_scale = false;
  ImGuizmo::OPERATION current_gizmo_operation() {
    return (ImGuizmo::OPERATION)(
        (with_translate ? (int)ImGuizmo::OPERATION::TRANSLATE : 0) |
        (with_rotate ? (int)ImGuizmo::OPERATION::ROTATE : 0) |
        (with_scale ? (int)ImGuizmo::OPERATION::SCALE : 0));
  };
  ImGuizmo::MODE current_gizmo_mode = ImGuizmo::MODE::WORLD;

  entt::entity selected_entity = entt::null;

  defered_forward_mixed *dm_sys = nullptr;
  transform_system *transform_sys = nullptr;
  script_system *script_sys = nullptr;

  void late_deserialize(nlohmann::json &j) override;

  void draw_main_menubar();
  void draw_entity_hierarchy();
  void draw_entity_components();
  void draw_gizmos(bool enable = true);
};

}; // namespace toolkit::opengl