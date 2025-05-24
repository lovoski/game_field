#pragma once

#include "toolkit/opengl/base.hpp"
#include "toolkit/opengl/gui/utils.hpp"
#include "toolkit/system.hpp"

namespace toolkit::opengl {

struct dir_light : public icomponent {
  math::vector3 dir = math::world_forward;
  math::vector3 color = White;

  void draw_gui(iapp *app) override {
    ImGui::Checkbox("Enable", &enabled);
    if (!enabled)
      ImGui::BeginDisabled();
    gui::color_edit_3("Color", color);
    if (!enabled)
      ImGui::EndDisabled();
  }

  bool enabled = true;
};
DECLARE_COMPONENT(dir_light, graphics, dir, color, enabled)

struct point_light : public icomponent {
  math::vector3 color = White;

  void draw_gui(iapp *app) override {
    ImGui::Checkbox("Enable", &enabled);
    if (!enabled)
      ImGui::BeginDisabled();
    gui::color_edit_3("Color", color);
    if (!enabled)
      ImGui::EndDisabled();
  }

  bool enabled = true;
};
DECLARE_COMPONENT(point_light, graphics, color, enabled)

}; // namespace toolkit::opengl