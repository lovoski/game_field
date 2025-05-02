#include "toolkit/anim/scripts/vis.hpp"
#include "toolkit/opengl/editor.hpp"

namespace toolkit::anim {

void vis_anim::draw_to_scene(iapp *app) {
  opengl::script_draw_to_scene_proxy(app, [&](opengl::editor *eptr,
                                              transform &cam_trans,
                                              opengl::camera &cam_comp) {});
}
void vis_anim::draw_gui(iapp *app) {}

}; // namespace toolkit::anim