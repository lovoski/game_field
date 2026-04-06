#pragma once

#include "toolkit/opengl3d/components/actor.hpp"
#include "toolkit/opengl3d/engine.hpp"
#include "toolkit/opengl3d/native_subsys.hpp"
#include "toolkit/system.hpp"

float critical_spring_damper(float x0, float v0, float xt, float t,
                             float half_life = 1.0f) {
  const float step = 1e-1f;
  const float e = 2.71828f;
  float lambda = log(2) / (half_life * log(e));
  float x = xt - x0, v = -v0, x_prev;
  float time = 0.0f;
  while (time < t) {
    x_prev = x;
    x = (x_prev + (v + lambda * x_prev) * step) * exp(-lambda * step);
    v = (v + lambda * x_prev) * exp(-lambda * step) - lambda * x;
    time += step;
  }
  return xt - x;
}

class spring_damper : public toolkit::opengl3d::sub_system {
public:
  void start(entt::registry &registry) override {
    if (target == entt::null) {
      target = registry.create();
      auto &trans = registry.emplace<toolkit::transform>(target);
      auto &self_trans = registry.get<toolkit::transform>(entity);
      trans.name = self_trans.name + " spring damper target";
      trans.set_world_pos(self_trans.world_pos());
      trans.set_world_rot(self_trans.world_rot());
    }
  }
  void update(entt::registry &registry, float dt) override {
    // update the transform of attached entity given target position and dt
    auto &self_trans = registry.get<toolkit::transform>(entity);
    auto &target_trans = registry.get<toolkit::transform>(target);
    const float e = 2.71828f;
    float lambda = log(2) / (damper_half_life * log(e));
    toolkit::math::vector3 x0 = self_trans.world_pos(),
                           xt = target_trans.world_pos();
    toolkit::math::vector3 x = x0 - xt;
    auto x_prev = x;
    x = (x_prev + (velocity + lambda * x_prev) * dt) * exp(-lambda * dt);
    velocity = (velocity + lambda * x_prev) * exp(-lambda * dt) - lambda * x;
    self_trans.set_world_pos(x + xt);

    // update the rotation of attached entity given target rotation and dt
    auto q0 = self_trans.world_rot();
    auto qt = target_trans.world_rot();
    if (q0.dot(qt) < 0.0f)
      qt = toolkit::math::quat(-qt.w(), -qt.x(), -qt.y(), -qt.z());
    toolkit::math::vector3 q =
        toolkit::math::quat_to_rot_vec(q0 * qt.inverse());
    auto q_prev = q;
    q = (q_prev + (angular_velocity + lambda * q_prev) * dt) *
        exp(-lambda * dt);
    angular_velocity =
        (angular_velocity + lambda * q_prev) * exp(-lambda * dt) - lambda * q;
    self_trans.set_world_rot(toolkit::math::rot_vec_to_quat(q) * qt);
  }

  void draw_to_scene(entt::registry &registry, toolkit::transform &cam_trans,
                     toolkit::opengl3d::camera &cam_comp) override {
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
    ImGui::DragFloat("Half Life", &damper_half_life, 0.01f, 0.0f, 10.0f);
    if (ImGui::Button("Random target transform", {-1, 30})) {
      auto &target_trans = registry.get<toolkit::transform>(target);
      target_trans.set_world_pos(toolkit::math::vector3::Random() * 10.0f);
      target_trans.set_world_rot(toolkit::math::quat::UnitRandom());
    }
  }

  toolkit::math::vector3 velocity = toolkit::math::vector3::Zero(),
                         angular_velocity = toolkit::math::vector3::Zero();
  float damper_half_life = 0.5f;
  entt::entity target = entt::null;
};
DECLARE_SUB_SYSTEM(spring_damper, target, damper_half_life)
