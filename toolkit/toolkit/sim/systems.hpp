/**
 * PBD simulation adapted from: https://github.com/felipeek/raw-physics
 */
#pragma once

#include "toolkit/sim/components/colliders.hpp"
#include "toolkit/system.hpp"
#include "toolkit/transform.hpp"

namespace toolkit::sim {

struct sim_obj_data {
  entt::entity entity;
  transform *trans;
  rigid_sim_object *sim_obj;
};

class phy_system : public isystem {
public:
  void update(entt::registry &registry, float dt) override;
  void fixedupdate(entt::registry &registry, float dt);

  void draw_gui(entt::registry &registry, entt::entity entity) override;
  void draw_menu_gui() override;

  void draw_to_scene(entt::registry &registry, transform &cam_trans, camera &cam_comp);

  void update_collider_properties(entt::registry &registry,
                                  std::vector<sim_obj_data> &obj_data);
  std::vector<std::pair<entt::entity, entt::entity>>
  get_broadphase_collision_pairs(entt::registry &registry,
                                 std::vector<sim_obj_data> &obj_data);

private:
  int cur_exec_fixed = 0;
  float cur_time = 0.0f;

  int num_sub_steps = 20, sim_fps = 60;

  math::vector3 gravity = math::vector3(0.0, -9.8, 0.0);

  REFLECT_PRIVATE(phy_system)
};
DECLARE_SYSTEM(phy_system, gravity, num_sub_steps)

}; // namespace toolkit::sim
