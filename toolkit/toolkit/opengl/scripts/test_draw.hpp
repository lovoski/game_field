#pragma once

#include "toolkit/opengl/base.hpp"
#include "toolkit/opengl/editor.hpp"
#include "toolkit/scriptable.hpp"
#include "toolkit/system.hpp"

namespace toolkit::opengl {

struct test_draw : public scriptable {
  float text_width = 1.0, text_height = 1.0, text_scale = 1.0,
        text_spacing = 0.0, text_line_height = 1.0;

  void draw_gui(iapp *app) override;
  void draw_to_scene(iapp *app) override;
};
DECLARE_SCRIPT(test_draw, debug)

}; // namespace toolkit::opengl