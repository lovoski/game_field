#include "scripts/mesh_modify.hpp"
#include "scripts/mixamo_manipulate.hpp"
#include "scripts/spring_damper.hpp"
#include "toolkit/opengl/editor.hpp"

using namespace toolkit;

#ifdef _WIN32
#include <Windows.h>
#include <locale>
#endif

int main() {
#ifdef _WIN32
  SetConsoleOutputCP(CP_UTF8);
  std::locale::global(std::locale("zh_CN.UTF-8"));
#endif
  toolkit::opengl::editor editor;
  editor.init();
  editor.run();
  return 0;
}