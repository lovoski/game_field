#include "controller.hpp"

#include "toolkit/opengl3d/draw.hpp"

namespace toolkit::opengl3d {

void controller::handle_engine_gui() {
  ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(450, 500), ImGuiCond_Always);
  ImGui::SetNextWindowBgAlpha(0.4f);

  ImGui::Begin("Controller");
  ImGui::Text("Player Entity: %s (%u)",
              registry.get<transform>(player_entity).name.c_str(),
              static_cast<uint32_t>(player_entity));
  ImGui::Text("Active Camera: %s (%u)",
              registry.get<transform>(active_camera).name.c_str(),
              static_cast<uint32_t>(active_camera));

  if (registry.valid(player_entity)) {
    if (auto *cc = registry.try_get<character_controller>(player_entity)) {
      ImGui::Text("Grounded: %s", cc->grounded ? "true" : "false");
    }
  }

  ImGui::DragFloat("Camera Height", &camera_height, 0.01f, 0.0f, 5.0f);
  ImGui::DragFloat("Camera Distance", &camera_distance, 0.01f, 0.0f, 20.0f);
  ImGui::DragFloat("Camera Sensitivity", &mouse_sensitivity, 0.01f, 0.01f,
                   1.0f);

  ImGui::End();
}

void controller::handle_scene_draw() {
  auto &player_trans = registry.get<transform>(player_entity);
  auto &cam_comp = registry.get<camera>(active_camera);
  auto &cam_trans = registry.get<transform>(active_camera);

  math::vector3 player_trans_proj = player_trans.world_pos();
  auto ground_hit = physics_world_sys->raycast(
      player_trans.world_pos(), math::vector3(0.0f, -1.0f, 0.0f));
  if (ground_hit.hit)
    player_trans_proj = ground_hit.point;

  draw_arrow(player_trans_proj, player_trans_proj + camera_forward, cam_comp.vp,
             Blue, 0.05, 1.0f, false);
  draw_sphere(player_trans_proj, cam_comp.vp, 0.05f, Blue, false, 1.0f, false);

  for (int i = 0; i < 21; i++) {
    draw_sphere(debug_traj_points[i], cam_comp.vp, 0.01f, Green, false, 1.0f,
                false);
    draw_arrow(debug_traj_points[i],
               debug_traj_points[i] + 0.15f * debug_traj_facing[i], cam_comp.vp,
               Green, 0.02, 1.0f, false);
  }
}

}; // namespace toolkit::opengl3d