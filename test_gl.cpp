#include "toolkit/opengl/editor.hpp"
#include "scripts/persona_compare.hpp"

using namespace toolkit;

int main() {
  toolkit::opengl::editor editor;
  editor.init();
  editor.run();
  return 0;
}