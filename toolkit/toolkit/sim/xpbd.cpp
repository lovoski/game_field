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

void xpbd_system::fixedupdate(entt::registry &registry, float dt) {
  // spdlog::info("xpbd system fixed update");
}

}; // namespace toolkit::sim