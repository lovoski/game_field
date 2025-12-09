#pragma once

#include "toolkit/sdl2d/header.hpp"

#include "toolkit/sdl2d/sim2d/system.hpp"

namespace toolkit::sdl2d {

class engine2d : public iapp {
public:
  void init(int width = 1280, int height = 720);
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

  // High resolution timer for delta time
  Uint64 perf_frequency = 0;
  Uint64 last_counter = 0;
  double delta_time = 0.0; // seconds

  void draw_editor_gui();
  void draw_game_content();

  void add_default_objects();

public:
  entt::registry registry;

  // Return last frame delta in seconds
  double get_delta_time() const { return delta_time; }

  void ss_draw_point(int x, int y,
                     const math::vector3 &color = math::vector3(1, 1, 1));
  void ss_draw_line(int x1, int y1, int x2, int y2,
                    const math::vector3 &color = math::vector3(1, 1, 1));
  // Additional helper drawing functions
  void ss_draw_points(const std::vector<math::vector2> &points,
                      const math::vector3 &color = math::vector3(1, 1, 1));
  void ss_draw_lines(const std::vector<math::vector2> &points,
                     bool closed = false,
                     const math::vector3 &color = math::vector3(1, 1, 1));
  void ss_draw_rectangle(int x, int y, int w, int h, bool filled = false,
                         const math::vector3 &color = math::vector3(1, 1, 1));
  void ss_draw_circle(int x, int y, int radius, bool filled = false,
                      const math::vector3 &color = math::vector3(1, 1, 1));
};

}; // namespace toolkit::sdl2d