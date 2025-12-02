/**
 * SDL Input Keycodes / Buttons Reference
 *
 * Keyboard (SDL_Keycode / SDLK_*):
 *   Letters:        SDLK_a ... SDLK_z
 *   Numbers:        SDLK_0 ... SDLK_9
 *   Arrow keys:     SDLK_UP, SDLK_DOWN, SDLK_LEFT, SDLK_RIGHT
 *   Enter / Return: SDLK_RETURN
 *   Escape:         SDLK_ESCAPE
 *   Space:          SDLK_SPACE
 *   Function keys:  SDLK_F1 ... SDLK_F12
 *   Modifiers:      SDLK_LSHIFT, SDLK_RSHIFT, SDLK_LCTRL, SDLK_RCTRL,
 * SDLK_LALT, SDLK_RALT
 *
 * Mouse buttons (SDL_BUTTON_*):
 *   SDL_BUTTON_LEFT   = 1
 *   SDL_BUTTON_MIDDLE = 2
 *   SDL_BUTTON_RIGHT  = 3
 *   SDL_BUTTON_X1     = 4
 *   SDL_BUTTON_X2     = 5
 *   Use SDL_BUTTON(n) macro with SDL_GetMouseState()
 *
 * Joystick / Gamepad buttons (SDL_CONTROLLER_BUTTON_*):
 *   SDL_CONTROLLER_BUTTON_A
 *   SDL_CONTROLLER_BUTTON_B
 *   SDL_CONTROLLER_BUTTON_X
 *   SDL_CONTROLLER_BUTTON_Y
 *   SDL_CONTROLLER_BUTTON_BACK
 *   SDL_CONTROLLER_BUTTON_GUIDE
 *   SDL_CONTROLLER_BUTTON_START
 *
 * Joystick axes:
 *   Use SDL_JoystickGetAxis()
 *   Range: -32768 ... 32767
 *
 * Mouse wheel:
 *   SDL_MOUSEWHEEL event: event.wheel.x (horizontal), event.wheel.y (vertical)
 */

#pragma once

#include "toolkit/math.hpp"
#include "toolkit/system.hpp"
#include "toolkit/utils.hpp"

#include "toolkit/opengl/base.hpp"

#include <SDL.h>
#include <SDL_opengl.h>
#include <glad/glad.h>

#include "backends/imgui_impl_opengl3.h"
#include "backends/imgui_impl_sdl2.h"
#include <imgui.h>
#include <implot.h>

#include <cstdio>
#include <functional>
#include <set>
#include <string>
#include <unordered_map>

namespace toolkit::opengl {

class sdl_context {
public:
  // Get the singleton instance
  static sdl_context &get_instance() {
    static sdl_context instance;
    return instance;
  }

  sdl_context(const sdl_context &) = delete;
  sdl_context &operator=(const sdl_context &) = delete;

  void init(unsigned int width = 1920, unsigned int height = 1080,
            const char *title = "App", int majorVersion = 4,
            int minorVersion = 3);
  void run(std::function<void(void)> mainLoop);
  void shutdown();

  void begin_imgui();
  void end_imgui();

  void swap_buffer() {
    if (window) {
      SDL_GL_SwapWindow(window);
    }
  }

  void poll();

  // If the specified `key` at pressed state at this frame
  bool is_key_pressed(int key) const;
  // The specified `key` went from unpressed to pressed at this frame
  bool is_key_triggered(int key) const;
  // The specified `key` went from pressed to unpressed at this frame
  bool is_key_untriggered(int key) const;

  // The specified `key` went from unpressed to pressed at this frame
  bool is_mouse_button_triggered(int key) const;
  // The specified `key` went from pressed to unpressed at this frame
  bool is_mouse_button_untriggered(int key) const;
  bool is_mouse_button_pressed(int button) const;

  bool loop_cursor_in_window();

  void set_window_title(std::string title = "app") {
    if (window)
      SDL_SetWindowTitle(window, title.c_str());
  }

  math::vector2 get_mouse_position() const { return {mouse_x, mouse_y}; }

  math::vector2 get_scroll_offsets() { return scroll_offset; }
  math::vector2 get_mouse_offsets() {
    return {mouse_x - mouse_last_x, mouse_y - mouse_last_y};
  }

  math::vector2 get_window_size() const {
    return {static_cast<double>(wnd_width), static_cast<double>(wnd_height)};
  }

  std::vector<SDL_GameController *> get_game_controllers();
  std::tuple<math::vector2, math::vector2, float, float>
  get_game_controller_analog_inputs(SDL_GameController *controller);
  std::string get_game_controller_name(SDL_GameController *controller) {
    return std::string(SDL_GameControllerName(controller));
  }

  bool get_vsync_state() const { return should_vsync; }
  void set_vsync_state(bool enable);

  SDL_Window *window = nullptr;
  SDL_GLContext gl_context = nullptr;
  uint32_t wnd_width = 0, wnd_height = 0;

  bool caps_lock_on = false;
  bool wnd_resized = false;

  static inline std::set<unsigned int> buffer_handles, vertex_array_handles,
      texture_handles, program_handles, framebuffer_handles;

  texture white_tex, black_tex, checkerboard_tex;

private:
  sdl_context() : mouse_x(0.0), mouse_y(0.0) {}

  // due to DPI settings, the actual size of the window could differ from the
  // recorded window size, we need to setup the opengl render in the actual size
  void reset_wnd_drawable_size();

  // Member variables
  std::set<int> triggered_keys, untriggered_keys;
  std::unordered_map<int, bool> key_states;
  std::set<int> triggered_mouse_keys, untriggered_mouse_keys;
  std::unordered_map<int, bool> mouse_button_states;
  bool mouse_pos_init = true;
  double mouse_x, mouse_y, mouse_last_x, mouse_last_y;
  math::vector2 scroll_offset{0.0, 0.0};

  // It's possible that your drive override vsync operation for the program,
  // visit the driver panel, and make sure the vsync option is set to "let
  // program decides", otherwise this settings might not work
  bool should_vsync = false;

  ImFont *default_font = nullptr, *icon_font = nullptr;
};

}; // namespace toolkit::opengl
