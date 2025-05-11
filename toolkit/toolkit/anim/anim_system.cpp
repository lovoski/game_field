#include "toolkit/anim/anim_system.hpp"
#include "toolkit/anim/components/actor.hpp"

namespace toolkit::anim {

void anim_system::draw_gui(entt::registry &registry, entt::entity entity) {
  if (auto ptr = registry.try_get<actor>(entity)) {
    if (ImGui::CollapsingHeader("Actor")) {
    }
  }
}

};