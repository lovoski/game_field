#pragma once

#include "toolkit/system.hpp"

namespace toolkit::anim {

class anim_system : public isystem {
public:
  anim_system() {}
  ~anim_system() {}

  void draw_gui(entt::registry &registry, entt::entity entity) override;

private:
};
DECLARE_SYSTEM(anim_system)

}; // namespace toolkit::anim