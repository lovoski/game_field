#pragma once

#include "toolkit/anim/components/actor.hpp"
#include "toolkit/opengl/editor.hpp"
#include "toolkit/scriptable.hpp"
#include "toolkit/system.hpp"

class mesh_modify : public toolkit::scriptable {
public:
  void draw_gui(toolkit::iapp *app) override;
  void draw_to_scene(toolkit::iapp *app) override;
};
DECLARE_SCRIPT(mesh_modify, modify)