#pragma once

#include "toolkit/opengl/base.hpp"
#include "toolkit/opengl/gui/utils.hpp"
#include "toolkit/system.hpp"
#include "toolkit/scriptable.hpp"
#include "toolkit/opengl/draw.hpp"

namespace toolkit::opengl {

struct point_light : public scriptable {
  math::vector3 color = White;

  void draw_gui(iapp *app) override {
    ImGui::Checkbox("Enable", &enabled);
    if (!enabled)
      ImGui::BeginDisabled();
    gui::color_edit_3("Color", color);
    if (!enabled)
      ImGui::EndDisabled();
  }

  void draw_to_scene(iapp *app) override;

  bool enabled = true;
};
DECLARE_SCRIPT(point_light, graphics, color, enabled)

}; // namespace toolkit::opengl