#pragma once

#include "toolkit/opengl/base.hpp"
#include "toolkit/opengl/draw.hpp"
#include "toolkit/scriptable.hpp"

namespace toolkit::anim {

class vis_skeleton : public scriptable {
public:
  void draw_to_scene(iapp *app) override;
  void draw_gui(iapp *app) override;
};
DECLARE_SCRIPT(vis_skeleton, animation)

}; // namespace toolkit::anim