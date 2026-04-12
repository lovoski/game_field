#include "controller.hpp"

#include "toolkit/opengl3d/draw.hpp"

namespace toolkit::opengl3d {

void controller::handle_engine_gui() {
  ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(450, 500), ImGuiCond_Always);
  ImGui::SetNextWindowBgAlpha(0.2f);

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
      ImGui::Text("Velocity: (%.2f, %.2f, %.2f)", player_velocity.x(),
                  player_velocity.y(), player_velocity.z());
    }
  }

  ImGui::End();
}

void controller::handle_scene_draw() {
  auto &player_trans = registry.get<transform>(player_entity);
  auto &cam_comp = registry.get<camera>(active_camera);
  auto &cam_trans = registry.get<transform>(active_camera);

  math::vector3 player_trans_proj = math::vector3(
      player_trans.world_pos().x(), 0.0f, player_trans.world_pos().z());

  draw_arrow(player_trans_proj, player_trans_proj + camera_forward, cam_comp.vp,
             Blue, 0.05, 1.0f, false);

  for (int i = 0; i < 21; i++) {
    draw_arrow(debug_traj_points[i],
               debug_traj_points[i] + 0.1f * debug_traj_facing[i], cam_comp.vp,
               Green, 0.01, 1.0f, false);
  }
}

}; // namespace toolkit::opengl3d