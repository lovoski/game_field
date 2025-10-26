#include "toolkit/sim/components/colliders.hpp"
#include "toolkit/sim/systems.hpp"

namespace toolkit::sim {

void xpbd_system::update(entt::registry &registry, float dt) {
  float residual = cur_time - cur_exec_fixed * fixed_interval;
  while (residual > fixed_interval) {
    residual -= fixed_interval;
    fixedupdate(registry, fixed_interval);
    cur_exec_fixed += 1;
  }
  cur_time += dt;
}

void xpbd_system::fixedupdate(entt::registry &registry, float dt) {}

void xpbd_system::draw_gui(entt::registry &registry, entt::entity entity) {}

void xpbd_system::draw_menu_gui() {}

void xpbd_system::draw_to_scene(entt::registry &registry, iapp *app) {}

}; // namespace toolkit::sim