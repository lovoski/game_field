#include "camdmpp.hpp"

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
  // toolkit::opengl3d::camdmpp app;
  // app.init(1920, 1080, "CAMDMPP DEMO");
  // app.run();
  // app.shutdown();

  diffusion model;

  model.setup("camdmpp/model.onnx", "camdmpp/config.json");
  model.submit_inference([&](std::vector<float> &&results) {
    std::ofstream output("pred.txt");
    for (int i = 0; i < model.pose_token_dim; i++) {
      for (int j = 0; j < model.future_points; j++) {
        output << results[i*model.future_points+j] << " ";
      }
      output << "\n";
    }
    output.close();
    exit(0);
  });
  while (true) {
    model.process_completions();
  }

  return 0;
}