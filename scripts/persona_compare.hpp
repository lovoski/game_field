#pragma once

#include "toolkit/scriptable.hpp"
#include "toolkit/system.hpp"
#include "toolkit/opengl/editor.hpp"

class persona_compare : public toolkit::scriptable {
public:
  void draw_to_scene(toolkit::iapp *app) override {}
  void draw_gui(toolkit::iapp *app) override {}
};
DECLARE_SCRIPT(persona_compare, persona)