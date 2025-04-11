#include "toolkit/opengl/base.hpp"
#include "toolkit/opengl/editor.hpp"
#include <iostream>

int main() {
  auto &instance = toolkit::opengl::context::get_instance();
  instance.init();

  unsigned int frameCounter = 0;

  instance.run([&]() {
    instance.set_window_title(toolkit::str_format("%d", frameCounter++));

    instance.begin_imgui();

    if (ImGui::Button("This is a button")) {
      std::string filepath;
      if (toolkit::open_file_dialog("select a .bvh file", {"*.bvh", "*.BVH"},
                                    "*.bvh", filepath)) {
        std::cout << filepath << std::endl;
      }
    }

    instance.end_imgui();
    instance.swap_buffer();
  });

  return 0;
}