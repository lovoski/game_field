/**
 * Internal coordinate system:
 *          ^ y+
 *          |
 *          |
 * x- <---- o ----> x+
 *          |
 *          |
 *          v y-
 *
 * Screen space coordinate system:
 * o ----> x+
 * |
 * |
 * v y+
 */
#pragma once

#include "toolkit/sdl2d/header.hpp"

#include "toolkit/sdl2d/avbd/solver.h"

namespace toolkit::sdl2d {

class engine2d : public iapp {
public:
  void init(int width = 1920, int height = 1080);
  void run();
  void shutdown();

  void reset();

  void late_deserialize(nlohmann::json &j) override;
  void late_serialize(nlohmann::json &j) override;

private:
  SDL_Window *window = nullptr;
  SDL_Renderer *renderer = nullptr;
  SDL_Event event;

  bool engine_vsync_on = true;
  bool engine_running = true, engine_play_mode = false;

  int screen_width, screen_height;
  float camera_rotation, camera_zoom;
  math::vector2 camera_position;

  math::vector2 mouse_screen_delta = math::vector2::Zero();
  math::vector2 mouse_screen_position, mouse_scroll_offset;

  // High resolution timer for delta time
  Uint64 perf_frequency = 0;
  Uint64 last_counter = 0;
  double delta_time = 0.0; // seconds

  // box2d::box2d_lite_world *box2d_solver;
  avbd::Solver avbd_solver;

  void draw_editor_gui();

  void add_default_objects();

  bool caps_lock_on = false;
  std::set<int> triggered_keys, untriggered_keys;
  std::unordered_map<int, bool> key_states;
  std::set<int> triggered_mouse_keys, untriggered_mouse_keys;
  std::unordered_map<int, bool> mouse_button_states;
  void handle_event_states();

  void handle_game_logic_tick();
  void handle_game_render_tick();

public:
  math::vector2 world_to_screen(const math::vector2 &world_pos);
  math::vector2 screen_to_world(const math::vector2 &screen_pos);

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

  // Return last frame delta in seconds
  double get_delta_time() const { return delta_time; }

  void ss_draw_point(int x, int y,
                     const math::vector4 &color = math::vector4(1, 1, 1, 1));
  void ss_draw_line(int x1, int y1, int x2, int y2,
                    const math::vector4 &color = math::vector4(1, 1, 1, 1));
  // Additional helper drawing functions
  void ss_draw_points(const std::vector<math::vector2> &points,
                      const math::vector4 &color = math::vector4(1, 1, 1, 1));
  void ss_draw_lines(const std::vector<math::vector2> &points,
                     bool closed = false,
                     const math::vector4 &color = math::vector4(1, 1, 1, 1));
  void ss_draw_rectangle(int x, int y, int w, int h, bool filled = false,
                         const math::vector4 &color = math::vector4(1, 1, 1, 1));
  void ss_draw_circle(int x, int y, int radius, bool filled = false,
                      const math::vector4 &color = math::vector4(1, 1, 1, 1));
};

}; // namespace toolkit::sdl2d