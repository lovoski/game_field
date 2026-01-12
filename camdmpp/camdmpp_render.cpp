#include "camdmpp.hpp"

namespace toolkit::opengl3d {

void camdmpp::handle_engine_gui() {
  ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(400, 400), ImGuiCond_Always);
  ImGui::SetNextWindowBgAlpha(1.0f);
  ImGui::Begin("Settings", nullptr,
               ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

  ImGui::SeparatorText("Commands");
  ImGui::TextColored({0, 1, 0, 1}, "LCTRL+LMB");
  ImGui::SameLine();
  ImGui::Text(": Rotate View");

  ImGui::SeparatorText("Draws");
  ImGui::Checkbox("Draw Trajectory", &debug_draw_trajectory);
  if (ImGui::Checkbox("Draw Ground", &draw_ground_mesh)) {
    registry.get<mesh_data>(ground_entity).should_render_mesh =
        draw_ground_mesh;
  }

  ImGui::SeparatorText("Inertia Blending");
  ImGui::Checkbox("Enable", &enable_inertia_blending);
  ImGui::DragFloat("Half Life", &inertia_halflife, 0.0001f, 0.0f, 1.0f);
  ImGui::InputInt("Blend Window", &inertia_blend_wnd);
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
    draw_wire_spheres(traj_points_pos, cam_comp.vp, 0.05f, Green, 1.0f, true);
    draw_arrows(traj_points_dir, cam_comp.vp, Green, 0.1f, 1.0f, true);
    draw_wire_sphere(char_root_world_pos, cam_comp.vp, 0.05f, Purple, 10, 1.0f,
                     false);
    draw_arrow(char_root_world_pos,
               char_root_world_pos +
                   char_root_world_rot * math::vector3(0, 0, 0.5f),
               cam_comp.vp, Purple, 0.1f, 1.0f, false);
  }
}

}; // namespace toolkit::opengl3d