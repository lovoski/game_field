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
    toolkit::math::vector3 x0 = self_trans.position(), xt = target_trans.position();
    for (int i = 0; i < 3; i++) {
      float x = x0(i)-xt(i);
      float x_prev = x;
      x = (x_prev + (velocity(i) + lambda * x_prev) * dt) * exp(-lambda * dt);
      velocity(i) = (velocity(i) + lambda * x_prev) * exp(-lambda * dt) - lambda * x;
      x0(i) = x + xt(i);
    }
    self_trans.set_world_pos(x0);
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
    ImGui::DragFloat("Half Life", &damper_half_life, 0.01f, 0.0f, 10.0f);
  }

  toolkit::math::vector3 velocity = toolkit::math::vector3::Zero();
  float damper_half_life = 0.5f;
  entt::entity target = entt::null;
};
DECLARE_SCRIPT(spring_damper, animation, target, damper_half_life)
