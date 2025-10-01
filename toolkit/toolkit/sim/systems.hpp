#pragma once

#include "toolkit/system.hpp"
#include "toolkit/transform.hpp"

namespace toolkit::sim {

class xpbd_system : public isystem {
public:
  void update(entt::registry &registry, float dt) override;

  void fixedupdate(entt::registry &registry, float dt);

private:
  int cur_exec_fixed = 0;
  float cur_time = 0.0f, fixed_interval = 1.0f / 60.0f;

  math::vector3 gravity = math::vector3(0.0, -9.8, 0.0);

  REFLECT_PRIVATE(xpbd_system)
};
DECLARE_SYSTEM(xpbd_system, gravity)

}; // namespace toolkit::sim
