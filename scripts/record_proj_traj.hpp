#pragma once

#include "toolkit/anim/components/actor.hpp"
#include "toolkit/opengl/editor.hpp"
#include "toolkit/scriptable.hpp"
#include "toolkit/system.hpp"

class record_proj_traj : public toolkit::scriptable {
public:
  void update(toolkit::iapp *app, float dt) override {
    float residual = cur_time - cur_exec_fixed * fixed_interval;
    while (residual > fixed_interval) {
      residual -= fixed_interval;
      fixedupdate(app, fixed_interval);
      cur_exec_fixed += 1;
    }
    cur_time += dt;
  }
  void fixedupdate(toolkit::iapp *app, float dt) {
    // left ctl + left mouse to draw trajectory
    if (toolkit::opengl::g_instance.is_mouse_button_pressed(
            GLFW_MOUSE_BUTTON_LEFT) &&
        toolkit::opengl::g_instance.is_key_pressed(GLFW_KEY_LEFT_CONTROL)) {
      // get the mouse position projected to ground
      auto editor = static_cast<toolkit::opengl::editor *>(app);
      toolkit::math::vector3 mouse_o, mouse_d;
      if (editor->mouse_query_ray(mouse_o, mouse_d)) {
        // hit ground plane with the mouse ray
        float t = -mouse_o.y() / mouse_d.y();
        record_points.push_back(mouse_o + t * mouse_d);
      }
    }
  }

  void draw_gui(toolkit::iapp *app) override {
    if (ImGui::Button("export trajectory", {-1, 30})) {
      std::string filepath;
      if (toolkit::save_file_dialog("Save Trajectory", {"*.txt"}, filepath)) {
        std::ofstream output(filepath);
        if (output.is_open()) {
          for (int i = 0; i < record_points.size(); i++)
            output << toolkit::str_format(
                "%.4f %.4f %.4f\n", record_points[i].x(), record_points[i].y(),
                record_points[i].z());
          output.close();
          spdlog::info("Save trajectory to {0}", filepath);
        }
      }
    }
  }

  void draw_to_scene(toolkit::iapp *app, toolkit::transform &cam_trans,
                     toolkit::camera &cam_comp) override {
    toolkit::opengl::draw_wire_spheres(record_points, cam_comp.vp, 0.005f,
                                       toolkit::opengl::Red);
    toolkit::opengl::draw_linestrip(record_points, cam_comp.vp,
                                    toolkit::opengl::Red);
  }

private:
  int cur_exec_fixed = 0;
  float cur_time = 0.0f, fixed_interval = 1.0f / 60.0f;
  std::vector<toolkit::math::vector3> record_points;
};
DECLARE_SCRIPT(record_proj_traj, utils)