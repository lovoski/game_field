#pragma once

#include "toolkit/sdl2d/header.hpp"

namespace toolkit::sdl2d {

struct transform {};

class sim_sys_2d : public isystem {
public:
  void update(entt::registry &registry, float dt) override;
  void fixedupdate(entt::registry &registry, float dt);

  void draw_gui(entt::registry &registry, entt::entity entity) override;
  void draw_menu_gui() override;

private:
  int cur_exec_fixed = 0;
  float cur_time = 0.0f;

  int num_sub_steps = 20, sim_fps = 60;

  REFLECT_PRIVATE(sim_sys_2d)
};
DECLARE_SYSTEM(sim_sys_2d, num_sub_steps, sim_fps);

}; // namespace toolkit::sdl2d