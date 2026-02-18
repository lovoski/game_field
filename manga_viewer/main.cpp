#include "app.hpp"

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
  manga_viewer app;
  app.init();
  app.run();
  app.shutdown();
  return 0;
}