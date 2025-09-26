#pragma once

#include "toolkit/anim/components/actor.hpp"
#include "toolkit/opengl/editor.hpp"
#include "toolkit/scriptable.hpp"
#include "toolkit/system.hpp"

namespace toolkit {

class task_verify : public scriptable {
public:
  void start() override {
    math::quat q0 = math::quat::UnitRandom();
    math::quat q1 = math::quat::UnitRandom();
    R0 << q0 * math::world_right, q0 * math::world_up, q0 * math::world_forward;
    R1 << q1 * math::world_right, q1 * math::world_up, q1 * math::world_forward;

    x = math::vector3::Random().normalized() * 3.0;
    y = math::vector3::Random().normalized() * 2.0;
    z = math::vector3::Random().normalized() * 1.0;
  }

  void update(iapp *app, float dt) override {
    auto r = math::mat_log(R1 * (R0.inverse()));
    math::matrix3 Rt = math::mat_exp(t * r) * R0;
    xt = Rt * x;
    yt = Rt * y;
    zt = Rt * z;
    x_traj.push_back(xt);
    y_traj.push_back(yt);
    z_traj.push_back(zt);

    if (opengl::g_instance.is_key_pressed(GLFW_KEY_LEFT))
      t -= dt * 1.0f;
    if (opengl::g_instance.is_key_pressed(GLFW_KEY_RIGHT))
      t += dt * 1.0f;
    t = std::clamp(t, 0.0f, 1.0f);
  }

  void draw_to_scene(iapp *app) override {
    opengl::script_draw_to_scene_proxy(app, [&](opengl::editor *editor,
                                                transform &cam_trans,
                                                opengl::camera &cam_comp) {
      opengl::draw_linestrip(x_traj, cam_comp.vp, opengl::Red);
      opengl::draw_linestrip(y_traj, cam_comp.vp, opengl::Green);
      opengl::draw_linestrip(z_traj, cam_comp.vp, opengl::Blue);
      opengl::draw_wire_sphere(xt, cam_comp.vp, 0.05f, opengl::Red);
      opengl::draw_wire_sphere(yt, cam_comp.vp, 0.05f, opengl::Green);
      opengl::draw_wire_sphere(zt, cam_comp.vp, 0.05f, opengl::Blue);

      opengl::draw_arrow(math::vector3::Zero(), R0 * math::world_right,
                         cam_comp.vp, opengl::Red);
      opengl::draw_arrow(math::vector3::Zero(), R0 * math::world_up,
                         cam_comp.vp, opengl::Green);
      opengl::draw_arrow(math::vector3::Zero(), R0 * math::world_forward,
                         cam_comp.vp, opengl::Blue);

      opengl::draw_arrow(math::vector3::Zero(), R1 * math::world_right,
                         cam_comp.vp, opengl::Red * 0.5f);
      opengl::draw_arrow(math::vector3::Zero(), R1 * math::world_up,
                         cam_comp.vp, opengl::Green * 0.5f);
      opengl::draw_arrow(math::vector3::Zero(), R1 * math::world_forward,
                         cam_comp.vp, opengl::Blue * 0.5f);
    });
  }

  void draw_gui(iapp *app) override {
    ImGui::DragFloat("t", &t, 0.01f, 0.0f, 1.0f);
  }

private:
  float t = 0.0f;
  math::vector3 x, y, z;
  math::vector3 xt, yt, zt;
  std::vector<math::vector3> x_traj, y_traj, z_traj;
  math::matrix3 R0, R1;
};
DECLARE_SCRIPT(task_verify, utils)

}; // namespace toolkit