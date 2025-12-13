#include "toolkit/sdl2d/engine.hpp"

namespace toolkit::sdl2d {

void engine2d::handle_game_logic_tick() {
  // Update high-resolution delta time at the start of the frame
  Uint64 now_counter = SDL_GetPerformanceCounter();
  delta_time = static_cast<float>(now_counter - last_counter) /
               static_cast<float>(perf_frequency);
  last_counter = now_counter;

  // box2d_solver->update(registry, delta_time);
  avbd_solver.update(delta_time);

  if (!engine_play_mode) {
    // editor exclusive logic
    if (is_mouse_button_pressed(SDL_BUTTON_MIDDLE)) {
      // move the camera when mouse middle button pressed
      camera_position =
          screen_to_world(0.5f * math::vector2(screen_width, screen_height) -
                          mouse_screen_delta);
    }
  }
}

}; // namespace toolkit::sdl2d