// #pragma once

// #include "toolkit/anim/components/actor.hpp"
// #include "toolkit/opengl/base.hpp"
// #include "toolkit/opengl/draw.hpp"
// #include "toolkit/scriptable.hpp"

// namespace toolkit::anim {

// class character_controller : public sub_system {
// public:
//   void start(entt::registry &registry) override;
//   void destroy(entt::registry &registry) override;

//   void draw_to_scene(entt::registry &registry, transform &cam_trans,
//                      camera &cam_comp) override;
//   void draw_gui(entt::registry &registry, entt::entity entity) override;

//   void update(entt::registry &registry, float dt) override;
//   void fixedupdate(entt::registry &registry, float dt) override;

// private:
//   int input_ticks_index = 0;

//   float cam_move_speed = 100.0f;
//   float cam_angle_horizontal = 0.0f, cam_angle_vertical = 30.0f;

//   math::vector3 character_pos = math::vector3::Zero();
//   math::quat character_rot = math::quat::Identity();
// };
// DECLARE_SUB_SYSTEM(character_controller, animation)

// }; // namespace toolkit::anim