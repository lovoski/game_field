#pragma once

#include "toolkit/anim/components/actor.hpp"
#include "toolkit/opengl/editor.hpp"
#include "toolkit/scriptable.hpp"
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

toolkit::math::vector3 quat_to_so3(toolkit::math::quat q) {
  float half_theta = acos(q.w());
  float sin_half_theta = sin(half_theta);
  if (abs(sin_half_theta) < 1e-5f)
    return toolkit::math::vector3::Zero();
  return half_theta * 2 / sin_half_theta *
         toolkit::math::vector3(q.x(), q.y(), q.z());
}
toolkit::math::quat so3_to_quat(toolkit::math::vector3 a) {
  float theta = a.norm();
  if (abs(theta) < 1e-5f)
    return toolkit::math::quat::Identity();
  float sin_half_theta = sin(theta / 2);
  return toolkit::math::quat(cos(theta / 2), sin_half_theta / theta * a.x(),
                             sin_half_theta / theta * a.y(),
                             sin_half_theta / theta * a.z());
}

class spring_damper : public toolkit::scriptable {
public:
  void start() override {
    if (target == entt::null) {
      target = registry->create();
      auto &trans = registry->emplace<toolkit::transform>(target);
      auto &self_trans = registry->get<toolkit::transform>(entity);
      trans.name = self_trans.name + " spring damper target";
      trans.set_world_pos(self_trans.position());
      trans.set_world_rot(self_trans.rotation());
    }
  }
  void lateupdate(toolkit::iapp *app, float dt) override {
    // update the transform of attached entity given target position and dt
    auto &self_trans = registry->get<toolkit::transform>(entity);
    auto &target_trans = registry->get<toolkit::transform>(target);
    const float e = 2.71828f;
    float lambda = log(2) / (damper_half_life * log(e));
    toolkit::math::vector3 x0 = self_trans.position(),
                           xt = target_trans.position();
    toolkit::math::vector3 x = x0 - xt;
    auto x_prev = x;
    x = (x_prev + (velocity + lambda * x_prev) * dt) * exp(-lambda * dt);
    velocity = (velocity + lambda * x_prev) * exp(-lambda * dt) - lambda * x;
    self_trans.set_world_pos(x + xt);

    // update the rotation of attached entity given target rotation and dt
    auto q0 = self_trans.rotation();
    auto qt = target_trans.rotation();
    if (q0.dot(qt) < 0.0f)
      qt = toolkit::math::quat(-qt.w(), -qt.x(), -qt.y(), -qt.z());
    toolkit::math::vector3 q = quat_to_so3(q0 * qt.inverse());
    auto q_prev = q;
    q = (q_prev + (angular_velocity + lambda * q_prev) * dt) *
        exp(-lambda * dt);
    angular_velocity =
        (angular_velocity + lambda * q_prev) * exp(-lambda * dt) - lambda * q;
    self_trans.set_world_rot(so3_to_quat(q) * qt);
  }

  void draw_to_scene(toolkit::iapp *app) override {
    toolkit::opengl::script_draw_to_scene_proxy(
        app, [&](toolkit::opengl::editor *editor, toolkit::transform &trans,
                 toolkit::opengl::camera &cam_comp) {
          toolkit::opengl::draw_wire_sphere(
              registry->get<toolkit::transform>(target).position(), cam_comp.vp,
              0.1f);
        });
  }
  void draw_gui(toolkit::iapp *app) override {
    ImGui::Text("Velocity x=%.2f,y=%.2f,z=%.2f", velocity.x(), velocity.y(),
                velocity.z());
    ImGui::Text("Angular Velocity x=%.2f,y=%.2f,z=%.2f", angular_velocity.x(),
                angular_velocity.y(), angular_velocity.z());
    ImGui::DragFloat("Half Life", &damper_half_life, 0.01f, 0.0f, 10.0f);
  }

  toolkit::math::vector3 velocity = toolkit::math::vector3::Zero(),
                         angular_velocity = toolkit::math::vector3::Zero();
  float damper_half_life = 0.5f;
  entt::entity target = entt::null;
};
DECLARE_SCRIPT(spring_damper, animation, target, damper_half_life)
