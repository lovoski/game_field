#include "toolkit/opengl/editor.hpp"

using namespace toolkit;

int main() {
  toolkit::opengl::editor editor;
  editor.init();

  for (int i = 0; i < 10; i++) {
    auto ent = editor.registry.create();
    auto &trans = editor.registry.emplace<transform>(ent);
    trans.name = str_format("entity %d", i);
  }

  editor.run();
  return 0;
}