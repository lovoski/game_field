#pragma once

#include "toolkit/opengl/base.hpp"
#include "toolkit/opengl/editor.hpp"
#include "toolkit/scriptable.hpp"
#include "toolkit/system.hpp"

namespace toolkit::opengl {

struct test_draw : public scriptable {
  float text_width = 1.0, text_height = 1.0, text_scale = 1.0,
        text_spacing = 0.0, text_line_height = 1.0;

  void start() override {
    spdlog::info("test_draw start called, entity={0}",
                 entt::to_integral(entity));
  }
  void destroy() override {
    spdlog::info("test_draw destroy called, entity={0}",
                 entt::to_integral(entity));
  }
  void init1() override {
    spdlog::info("test_draw init1 called, entity={0}",
                 entt::to_integral(entity));
  }

  void draw_gui(iapp *app) override;
  void draw_to_scene(iapp *app) override;
};
DECLARE_SCRIPT(test_draw, debug)

}; // namespace toolkit::opengl