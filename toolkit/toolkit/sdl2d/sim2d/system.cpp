#include "toolkit/sdl2d/sim2d/system.hpp"

namespace toolkit::sdl2d {

void sim_sys_2d::fixedupdate(entt::registry &registry, float dt) {}

void sim_sys_2d::draw_gui(entt::registry &registry, entt::entity entity) {}
void sim_sys_2d::draw_menu_gui() {}

void sim_sys_2d::update(entt::registry &registry, float dt) {
  float fixed_interval = 1.0f / sim_fps;
  float residual = cur_time - cur_exec_fixed * fixed_interval;
  while (residual > fixed_interval) {
    residual -= fixed_interval;
    fixedupdate(registry, fixed_interval);
    cur_exec_fixed += 1;
  }
  cur_time += dt;
}

}; // namespace toolkit::sdl2d