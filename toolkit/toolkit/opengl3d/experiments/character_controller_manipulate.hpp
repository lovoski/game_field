#pragma once

#include "toolkit/opengl3d/components/actor.hpp"
#include "toolkit/opengl3d/components/character_controller.hpp"
#include "toolkit/opengl3d/components/physics_world.hpp"
#include "toolkit/opengl3d/engine.hpp"
#include "toolkit/opengl3d/native_subsys.hpp"
#include "toolkit/system.hpp"

class cc_manipulate : public toolkit::opengl3d::sub_system {
public:
  void start(entt::registry &registry) override {
    // Ensure entity has a character_controller component
    if (!registry.any_of<toolkit::opengl3d::character_controller>(entity)) {
      registry.emplace<toolkit::opengl3d::character_controller>(entity);
    }

    // Create a target entity for the CC to follow
    if (target == entt::null) {
      target = registry.create();
      auto &trans = registry.emplace<toolkit::transform>(target);
      auto &self_trans = registry.get<toolkit::transform>(entity);
      trans.name = self_trans.name + " cc target";
      trans.set_world_pos(self_trans.world_pos());
      trans.set_world_rot(self_trans.world_rot());
    }
  }

  void update(entt::registry &registry, float dt) override {
    auto *cc =
        registry.try_get<toolkit::opengl3d::character_controller>(entity);
    if (!cc)
      return;

    auto *app = registry.ctx().get<toolkit::iapp *>();
    auto *pw = app->get_sys<toolkit::opengl3d::physics_world>();
    if (!pw)
      return;

    auto &self_trans = registry.get<toolkit::transform>(entity);
    auto &target_trans = registry.get<toolkit::transform>(target);

    // Spring-damper toward target position
    const float lambda = logf(2.0f) / damper_half_life;
    toolkit::math::vector3 x0 = self_trans.world_pos(),
                           xt = target_trans.world_pos();
    toolkit::math::vector3 x = x0 - xt;
    auto x_prev = x;
    x = (x_prev + (velocity + lambda * x_prev) * dt) * expf(-lambda * dt);
    velocity = (velocity + lambda * x_prev) * expf(-lambda * dt) - lambda * x;

    // Drive character controller with the spring displacement
    toolkit::math::vector3 displacement = (x + xt) - x0;
    pw->cc_move(*cc, displacement);

    // Spring-damper toward target rotation (applied directly since CC is
    // kinematic and doesn't handle rotation)
    auto q0 = self_trans.world_rot();
    auto qt = target_trans.world_rot();
    if (q0.dot(qt) < 0.0f)
      qt = toolkit::math::quat(-qt.w(), -qt.x(), -qt.y(), -qt.z());
    toolkit::math::vector3 q =
        toolkit::math::quat_to_rot_vec(q0 * qt.inverse());
    auto q_prev = q;
    q = (q_prev + (angular_velocity + lambda * q_prev) * dt) *
        expf(-lambda * dt);
    angular_velocity =
        (angular_velocity + lambda * q_prev) * expf(-lambda * dt) - lambda * q;
    self_trans.set_world_rot(toolkit::math::rot_vec_to_quat(q) * qt);

    // check collision events
    for (auto &e : cc->events) {
      if (e.type == toolkit::opengl3d::collision_event_type::COLLISION_ENTER) {
        printf(
            "Collision Enter with entity %d, name %s\n",
            static_cast<int>(e.other_entity),
            registry.try_get<toolkit::transform>(e.other_entity)
                ? registry.get<toolkit::transform>(e.other_entity).name.c_str()
                : "N/A");
      } else if (e.type ==
                 toolkit::opengl3d::collision_event_type::COLLISION_STAY) {
        printf(
            "Collision Stay with entity %d, name %s\n",
            static_cast<int>(e.other_entity),
            registry.try_get<toolkit::transform>(e.other_entity)
                ? registry.get<toolkit::transform>(e.other_entity).name.c_str()
                : "N/A");
      } else if (e.type ==
                 toolkit::opengl3d::collision_event_type::COLLISION_EXIT) {
        printf(
            "Collision Exit with entity %d, name %s\n",
            static_cast<int>(e.other_entity),
            registry.try_get<toolkit::transform>(e.other_entity)
                ? registry.get<toolkit::transform>(e.other_entity).name.c_str()
                : "N/A");
      }
    }
  }

  void draw_to_scene(entt::registry &registry, toolkit::transform &cam_trans,
                     toolkit::opengl3d::camera &cam_comp) override {
    if (target == entt::null)
      return;

    toolkit::opengl3d::draw_sphere(
        registry.get<toolkit::transform>(target).world_pos(), cam_comp.vp, 0.1f,
        toolkit::math::vector3(1, 1, 1), false, 0.5f, false);

    auto &object_trans = registry.get<toolkit::transform>(entity);
    toolkit::opengl3d::draw_sphere(object_trans.world_pos(), cam_comp.vp, 0.1f,
                                   toolkit::math::vector3(1, 0, 0), false, 0.5f,
                                   false);
    toolkit::opengl3d::draw_arrow(
        object_trans.world_pos(),
        object_trans.world_pos() + object_trans.local_right(), cam_comp.vp,
        toolkit::math::vector3(1, 0, 0), 0.2f, 0.5f, false);
    toolkit::opengl3d::draw_arrow(
        object_trans.world_pos(),
        object_trans.world_pos() + object_trans.local_up(), cam_comp.vp,
        toolkit::math::vector3(0, 1, 0), 0.2f, 0.5f, false);
    toolkit::opengl3d::draw_arrow(
        object_trans.world_pos(),
        object_trans.world_pos() + object_trans.local_forward(), cam_comp.vp,
        toolkit::math::vector3(0, 0, 1), 0.2f, 0.5f, false);
  }

  void draw_gui(entt::registry &registry, entt::entity entity) override {
    ImGui::Text("Velocity x=%.2f,y=%.2f,z=%.2f", velocity.x(), velocity.y(),
                velocity.z());
    ImGui::Text("Angular Velocity x=%.2f,y=%.2f,z=%.2f", angular_velocity.x(),
                angular_velocity.y(), angular_velocity.z());
    ImGui::DragFloat("Half Life", &damper_half_life, 0.01f, 0.01f, 10.0f);
    ImGui::DragFloat("Move Speed", &move_speed, 0.1f, 0.0f, 50.0f);
    auto *cc =
        registry.try_get<toolkit::opengl3d::character_controller>(entity);
    if (cc)
      ImGui::Text("Grounded: %s", cc->grounded ? "true" : "false");
    if (ImGui::Button("Random target transform", {-1, 30})) {
      auto &target_trans = registry.get<toolkit::transform>(target);
      target_trans.set_world_pos(toolkit::math::vector3::Random() * 10.0f);
      target_trans.set_world_rot(toolkit::math::quat::UnitRandom());
    }
  }

  toolkit::math::vector3 velocity = toolkit::math::vector3::Zero(),
                         angular_velocity = toolkit::math::vector3::Zero();
  float damper_half_life = 0.5f;
  float move_speed = 5.0f;
  entt::entity target = entt::null;
};
DECLARE_SUB_SYSTEM(cc_manipulate, target, damper_half_life, move_speed)
