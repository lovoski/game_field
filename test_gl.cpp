#include "toolkit/opengl/editor.hpp"

using namespace toolkit;

int main() {
  toolkit::opengl::editor editor;
  editor.init();
  editor.run();
  return 0;
}