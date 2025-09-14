#pragma once

#include "toolkit/anim/components/actor.hpp"
#include "toolkit/system.hpp"

namespace toolkit::anim {

class anim_system : public isystem {
public:
  anim_system() {}
  ~anim_system() {}

  void draw_gui(entt::registry &registry, entt::entity entity) override;
};
DECLARE_SYSTEM(anim_system)

void draw_skeleton_gui(entt::registry &registry, entt::entity entity);

}; // namespace toolkit::anim