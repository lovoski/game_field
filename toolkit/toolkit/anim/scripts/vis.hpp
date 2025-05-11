#pragma once

#include "toolkit/anim/components/actor.hpp"
#include "toolkit/opengl/base.hpp"
#include "toolkit/opengl/draw.hpp"
#include "toolkit/scriptable.hpp"

namespace toolkit::anim {

class vis_skeleton : public scriptable {
public:
  void draw_to_scene(iapp *app) override;
  void draw_gui(iapp *app) override;

  bool draw_axes = false;
  float axes_length = 1.0f;
  math::vector3 bone_color = opengl::Green;

private:
  std::set<entt::entity> active_joint_entities;
  std::vector<std::pair<math::vector3, math::vector3>> draw_queue;
};
DECLARE_SCRIPT(vis_skeleton, animation, draw_axes, axes_length, bone_color)

void collect_skeleton_draw_queue(
    entt::registry &registry, actor &actor_comp,
    std::vector<std::pair<math::vector3, math::vector3>> &draw_queue,
    std::set<entt::entity> &active_joint_entities);

}; // namespace toolkit::anim