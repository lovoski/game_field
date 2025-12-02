#include "toolkit/opengl/editor.hpp"
#include "toolkit/opengl/sdl_context.hpp"

using namespace toolkit;

#ifdef _WIN32
#include <Windows.h>
#include <locale>
#endif

int main(int argc, char **argv) {
#ifdef _WIN32
  SetProcessDPIAware();
#ifdef _MSC_VER
  SetConsoleOutputCP(CP_UTF8);
  std::locale::global(std::locale("zh_CN.UTF-8"));
#endif
#endif
  toolkit::opengl::editor editor;
  editor.init();
  editor.run();
  return 0;
}