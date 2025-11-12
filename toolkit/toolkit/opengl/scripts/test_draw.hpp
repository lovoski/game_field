#pragma once

#include "toolkit/opengl/base.hpp"
#include "toolkit/opengl/editor.hpp"
#include "toolkit/scriptable.hpp"
#include "toolkit/system.hpp"

namespace toolkit::opengl {

struct test_draw : public sub_system {
  float text_width = 1.0, text_height = 1.0, text_scale = 1.0,
        text_spacing = 0.0, text_line_height = 1.0, text_thickness = 0.0f;

  void start(entt::registry &registry) override {
    spdlog::info("test_draw start called, entity={0}",
                 entt::to_integral(entity));
  }
  void destroy(entt::registry &registry) override {
    spdlog::info("test_draw destroy called, entity={0}",
                 entt::to_integral(entity));
  }
  void init1() override {
    spdlog::info("test_draw init1 called, entity={0}",
                 entt::to_integral(entity));
  }

  void draw_gui(entt::registry &registry, entt::entity entity) override;
  void draw_to_scene(entt::registry &registry, transform &cam_trans,
                     camera &cam_comp) override;
};
DECLARE_SUB_SYSTEM(test_draw, debug, text_width, text_height, text_scale,
                   text_spacing, text_line_height, text_thickness)

}; // namespace toolkit::opengl