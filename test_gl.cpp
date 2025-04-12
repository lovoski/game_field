#include "toolkit/opengl/editor.hpp"

using namespace toolkit;

int main() {
  toolkit::opengl::editor editor;
  editor.init();

  opengl::create_cube(editor.registry);

  auto cam = editor.registry.create();
  auto &cam_comp = editor.registry.emplace<opengl::camera>(cam);
  auto &trans = editor.registry.emplace<transform>(cam);
  trans.name = "camera";
  trans.set_global_position(math::vector3(0, 0, 5));

  opengl::context::get_instance().active_camera = cam;

  for (int i = 0; i < 10; i++) {
    auto ent = editor.registry.create();
    auto &trans = editor.registry.emplace<transform>(ent);
    trans.name = str_format("entity %d", i);
  }

  editor.run();
  return 0;
}