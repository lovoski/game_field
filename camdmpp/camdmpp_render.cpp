#include "camdmpp.hpp"

namespace toolkit::opengl3d {

void camdmpp::handle_engine_gui() {
  ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(450, 500), ImGuiCond_Always);
  ImGui::SetNextWindowBgAlpha(1.0f);
  ImGui::Begin("Settings", nullptr,
               ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

  ImGui::SeparatorText("Commands");
  // ImGui::TextColored({0, 1, 0, 1}, "LCTRL+LMB");
  // ImGui::SameLine();
  // ImGui::Text(": Rotate View");
  // ImGui::TextColored({0, 1, 0, 1}, "J");
  // ImGui::SameLine();
  // ImGui::Text(": Previous Style");
  // ImGui::TextColored({0, 1, 0, 1}, "K");
  // ImGui::SameLine();
  // ImGui::Text(": Next Style");

  ImGui::SeparatorText("Draws");
  ImGui::DragFloat("Camera Offset", &cam_distance, 0.01f, 0.0f, 10.0f);
  ImGui::Checkbox("Draw Trajectory", &debug_draw_trajectory);
  if (ImGui::Checkbox("Draw Ground", &draw_ground_mesh)) {
    registry.get<mesh_data>(ground_entity).should_render_mesh =
        draw_ground_mesh;
  }

  ImGui::SeparatorText("Camera Follow");
  ImGui::DragFloat("Camera Half Life", &camera_follow_halflife, 0.0001f,
                   0.0f, 1.0f);

  ImGui::SeparatorText("Inertia Blending");
  ImGui::Checkbox("Enable", &enable_inertia_blending);
  ImGui::DragFloat("Half Life", &inertia_halflife, 0.0001f, 0.0f, 1.0f);
  ImGui::InputInt("Blend Window", &inertia_blend_wnd);

  ImGui::SeparatorText("Statistics");
  // int cur_style_idx = static_cast<int>(model.style_idx_data[0]);
  // ImGui::Text("Current Style: %s (%d)",
  //             model.style_names[cur_style_idx].c_str(), cur_style_idx);
  ImGui::Text("Buffer In Use: %s", use_front_buffer ? "Front" : "Back");
  ImGui::Text("Inference Time: %.3f ms", display_inference_time);

  ImGui::End();
}

void camdmpp::debug_draw() {
  auto &cam_comp = registry.get<camera>(active_camera);
  auto &cam_trans = registry.get<transform>(active_camera);

  if (debug_draw_trajectory) {
    std::vector<math::vector3> traj_points_pos;
    std::vector<std::pair<math::vector3, math::vector3>> traj_points_dir;
    for (int i = 0; i < 5; i++) {
      traj_points_pos.push_back(_traj_world_pos[i] +
                                math::vector3(0.0f, 0.01f, 0.0f));
      traj_points_dir.push_back(
          std::make_pair(_traj_world_pos[i] + math::vector3(0.0f, 0.01f, 0.0f),
                         _traj_world_pos[i] + 0.5f * _traj_world_dir[i] +
                             math::vector3(0.0f, 0.01f, 0.0f)));
    }
    draw_spheres(traj_points_pos, cam_comp.vp, 0.03f, Green, false, 1.0f, true);
    draw_arrows(traj_points_dir, cam_comp.vp, Green, 0.1f, 1.0f, true);
    // draw_wire_sphere(char_root_world_pos, cam_comp.vp, 0.03f, Purple, 10, 1.0f,
    //                  false);

    math::vector2 proj_char_pos_xz =
        math::vector2(char_root_world_pos.x(), char_root_world_pos.z());
    draw_sphere(
        math::vector3(proj_char_pos_xz.x(),
                      sample_terrain_height(proj_char_pos_xz, 0.0f),
                      proj_char_pos_xz.y()),
        cam_comp.vp, 0.03f, Purple, false, 1.0f, true);

    // draw_arrow(char_root_world_pos,
    //            char_root_world_pos +
    //                proj_char_root_world_rot * math::vector3(0, 0, 0.5f),
    //            cam_comp.vp, Purple, 0.1f, 1.0f, false);
  }
}

}; // namespace toolkit::opengl3d