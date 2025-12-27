#include "craft.hpp"

namespace toolkit::opengl3d {

void craft::handle_custom_initialization() {}

void craft::handle_custom_cleanup() {}

void craft::handle_engine_gui() {}

void craft::reset() {
  registry.clear();
  systems.clear();
  transform_hierarchy_sys = register_sys<transform_system>();
}

void craft::engine_fixed_update(float dt) {
  transform_hierarchy_sys->update_transform(registry);
  
  static int w_counter = 1;
  if (is_key_triggered(SDLK_w))
    std::cout << w_counter++ << ": key W triggered" << std::endl;
}

void craft::run() {
  timer.reset();
  while (app_running) {
    float dt = timer.elapse_s();
    timer.reset();

    // the game handles inputs at fixed interval
    double residual = __cur_time - __cur_exec_fixed * fixed_interval;
    while (residual > fixed_interval) {
      handle_input_events();
      if (!window) {
        app_running = false;
        break;
      }

      engine_fixed_update(fixed_interval);

      residual -= fixed_interval;
      __cur_exec_fixed += 1;
    }
    __cur_time += dt;

    // opengl rendering
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glViewport(0, 0, wnd_width, wnd_height);

    // render the main scene
    quad_program.use();
    quad_program.set_texture2d("scene_tex", checkerboard_tex.get_handle(), 0);
    quad_draw_call();

    // render the gui
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();
    handle_engine_gui();
    ImGui::EndFrame();
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    if (window)
      SDL_GL_SwapWindow(window);
  }
}

}; // namespace toolkit::opengl3d