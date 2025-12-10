// #pragma once

// #include "toolkit/anim/components/actor.hpp"
// #include "toolkit/opengl/base.hpp"
// #include "toolkit/opengl/draw.hpp"
// #include "toolkit/scriptable.hpp"

// namespace toolkit::anim {

// class vis_skeleton : public sub_system {
// public:
//   void draw_to_scene(entt::registry &registry, transform &cam_trans,
//                      camera &cam_comp) override;
//   void draw_gui(entt::registry &registry, entt::entity entity) override;

//   void start(entt::registry &registry) override;
//   void destroy(entt::registry &registry) override;

//   bool draw_axes = false, draw_spheres = true, draw_names = false;
//   float axes_length = 1.0f;
//   math::vector3 bone_color = opengl::Green;

//   void collect_skeleton_draw_queue(entt::registry &registry, actor &actor_comp);

// private:
//   std::set<entt::entity> active_joint_entities;
//   std::vector<math::vector3> joint_positions;
//   std::vector<std::pair<math::vector3, math::vector3>> draw_queue, x_dir, y_dir,
//       z_dir;
// };
// DECLARE_SUB_SYSTEM(vis_skeleton, animation, draw_axes, draw_spheres, draw_names,
//                    axes_length, bone_color)

// }; // namespace toolkit::anim