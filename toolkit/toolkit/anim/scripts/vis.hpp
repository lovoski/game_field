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

  void start() override;
  void destroy() override;

  bool draw_axes = false, draw_spheres = true;
  float axes_length = 1.0f;
  math::vector3 bone_color = opengl::Green;

  void collect_skeleton_draw_queue(actor &actor_comp);

private:
  std::set<entt::entity> active_joint_entities;
  std::vector<math::vector3> joint_positions;
  std::vector<std::pair<math::vector3, math::vector3>> draw_queue, x_dir, y_dir,
      z_dir;
};
DECLARE_SCRIPT(vis_skeleton, animation, draw_axes, draw_spheres, axes_length,
               bone_color)

}; // namespace toolkit::anim