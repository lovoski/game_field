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

#include "toolkit/system.hpp"
#include "toolkit/utils.hpp"

#include "toolkit/transform.hpp"

#include "toolkit/opengl3d/assets.hpp"

#include "toolkit/opengl3d/base.hpp"
#include "toolkit/opengl3d/bullet/system.hpp"
#include "toolkit/opengl3d/components/camera.hpp"
#include "toolkit/opengl3d/rasterize/system.hpp"

#include "toolkit/opengl3d/native_subsys.hpp"

#include <ImGuizmo.h>

namespace toolkit::opengl3d {

struct ray_query_data {
  float pdist, dist;
  entt::entity entity;
  const float weighted_dist() const { return pdist / dist; }
};
struct compare_ray_query_data {
  bool operator()(const ray_query_data &a, const ray_query_data &b) const {
    return a.weighted_dist() > b.weighted_dist();
  }
};
struct active_camera_manipulate_data {
  math::vector3 camera_pivot{0.0, 0.0, 0.0};
  // Some parameter related to camera control
  float initial_factor = 0.6f;
  float speed_pow = 1.5f;
  float max_speed = 8e2f;
  // fps-style camera parameters
  float fps_speed = 4;
  float fps_camera_speed = 3.0f;
};
REFLECT(active_camera_manipulate_data, camera_pivot, initial_factor, speed_pow,
        max_speed, fps_speed, fps_camera_speed)

class engine3d : public iapp {
public:
  void init(int width = 1920, int height = 1080, std::string title = "engine3d",
            int majorVersion = 4, int minorVersion = 6);
  void run();
  void shutdown();

  void reset();
  void add_default_objects();

  ImGuiIO *imgui_io = nullptr;
  stopwatch timer;

  bool with_translate = true, with_rotate = false, with_scale = false;
  ImGuizmo::OPERATION current_gizmo_operation() {
    return (ImGuizmo::OPERATION)(
        (with_translate ? (int)ImGuizmo::OPERATION::TRANSLATE : 0) |
        (with_rotate ? (int)ImGuizmo::OPERATION::ROTATE : 0) |
        (with_scale ? (int)ImGuizmo::OPERATION::SCALE : 0));
  };
  ImGuizmo::MODE current_gizmo_mode = ImGuizmo::MODE::WORLD;

  entt::entity selected_entity = entt::null;

  defered_render_system *default_render_sys = nullptr;
  transform_system *transform_hierarchy_sys = nullptr;
  sub_system_handler *ss_handler_system = nullptr;
  bullet::bullet_physics_system *physics_system = nullptr;

  float click_selection_max_sin = 2e-2f;
  std::vector<ray_query_data> selection_candidates;

  void late_deserialize(nlohmann::json &j) override;
  void late_serialize(nlohmann::json &j) override;

  void draw_main_menubar();
  void draw_hierarchy_window();
  void draw_components_window();
  void draw_gizmos(bool enable = true);

  // override this if there's more systems
  void draw_components_gui(entt::entity current_entity);
  // override this if there's more components
  void draw_systems_gui();

  // this function can be override to create custom initialization
  virtual void handle_custom_initialization() {}
  // this function can be override to create custom logic
  virtual void handle_game_logic_tick() {}
  virtual void handle_engine_gui();

  void set_game_mode(bool game_mode, bool hide_mouse = false) {
    in_game_mode = game_mode;
    SDL_SetRelativeMouseMode((in_game_mode && hide_mouse) ? SDL_TRUE
                                                          : SDL_FALSE);
  }
  void quit_app_running() {
    app_running = false;
    if (window) {
      SDL_DestroyWindow(window);
      window = nullptr;
    }
  }

  /**
   * Handle keyboard short cut inputs, modify editor states
   */
  void editor_shortkeys();

  /**
   * Unproject the 2d position of cursor to 3d space as a ray. `o` for origin,
   * `d` for direction. Returns whether the ray generation succeeded or not.
   */
  bool mouse_query_ray(math::vector3 &o, math::vector3 &d);
  /**
   * Screen pos ranges [0.0, 1.0]
   */
  bool screen_query_ray(math::vector2 screen_pos, math::vector3 &o,
                        math::vector3 &d);

  bool editor_manipulate_camera = true;
  entt::entity active_camera = entt::null;
  void active_camera_manipulate(float dt);

  bool app_in_game_mode() const { return in_game_mode; }

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

  std::vector<SDL_GameController *> get_game_controllers();
  std::tuple<math::vector2, math::vector2, float, float>
  get_game_controller_analog_inputs(SDL_GameController *controller);

  math::vector2 get_mouse_screen_pos() const { return mouse_screen_pos; }
  math::vector2 get_mouse_screen_delta() const { return mouse_screen_delta; }

  // default generated assets
  texture white_tex, black_tex, checkerboard_tex;

protected:
  int gizmo_mode_idx = 0;
  bool in_game_mode = false;
  bool app_running = true;
  bool should_vsync = false;

  // input handling
  std::set<int> triggered_keys, untriggered_keys;
  std::unordered_map<int, bool> key_states;
  std::set<int> triggered_mouse_keys, untriggered_mouse_keys;
  std::unordered_map<int, bool> mouse_button_states;
  math::vector2 mouse_screen_pos, mouse_screen_delta;
  math::vector2 scroll_offset{0.0, 0.0};

  SDL_Window *window = nullptr;
  SDL_GLContext gl_context = nullptr;
  uint32_t wnd_width = 0, wnd_height = 0;

  bool caps_lock_on = false;
  bool wnd_resized = false;

  shader quad_program;

  math::vector2 scene_wnd_size, scene_wnd_pos;

  active_camera_manipulate_data cam_manip_data;

  void set_vsync_state(bool enable);
  void reset_wnd_drawable_size();

  void handle_input_events();
};

}; // namespace toolkit::opengl3d