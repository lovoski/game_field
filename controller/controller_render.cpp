#include "controller.hpp"

#include "toolkit/opengl3d/draw.hpp"

namespace toolkit::opengl3d {

namespace {
constexpr int kSpeedHistorySize = 180;
}

void controller::handle_engine_gui() {
  ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(500, 650), ImGuiCond_Always);
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
      ImGui::Text("Events: %d", static_cast<int>(cc->events.size()));
    }
    ImGui::Text("Velocity: (%.2f, %.2f, %.2f)", player_velocity.x(),
                player_velocity.y(), player_velocity.z());

    static std::array<float, kSpeedHistorySize> speed_mag_history{};
    static std::array<float, kSpeedHistorySize> ordered_speed_mag{};
    static int history_write_idx = 0;
    const float speed_mag = player_velocity.norm();
    speed_mag_history[history_write_idx] = speed_mag;
    history_write_idx = (history_write_idx + 1) % kSpeedHistorySize;

    float speed_max = 0.0f;
    for (int i = 0; i < kSpeedHistorySize; ++i) {
      const int src_idx = (history_write_idx + i) % kSpeedHistorySize;
      const float sample = speed_mag_history[src_idx];
      ordered_speed_mag[i] = sample;
      speed_max = std::max(speed_max, sample);
    }

    const ImVec2 plot_size(-1.0f, 140.0f);
    if (ImPlot::BeginPlot("Speed Magnitude", plot_size,
                          ImPlotFlags_NoMouseText)) {
      ImPlot::SetupAxes("Samples", "m/s", ImPlotAxisFlags_NoTickLabels,
                        ImPlotAxisFlags_AutoFit);
      ImPlot::SetupAxisLimits(ImAxis_X1, 0.0, kSpeedHistorySize - 1.0,
                              ImPlotCond_Always);
      ImPlot::SetupAxisLimits(ImAxis_Y1, 0.0, std::max(1.0f, speed_max * 1.1f),
                              ImPlotCond_Always);
      ImPlot::PlotLine("|v|", ordered_speed_mag.data(), kSpeedHistorySize);
      ImPlot::EndPlot();
    }
  }

  ImGui::SeparatorText("Camera Settings");
  ImGui::DragFloat("Height", &camera_height, 0.01f, 0.0f, 5.0f);
  ImGui::DragFloat("Distance", &camera_distance, 0.01f, 0.0f, 20.0f);
  ImGui::DragFloat("Sensitivity", &mouse_sensitivity, 0.01f, 0.01f, 1.0f);

  ImGui::SeparatorText("Movement Settings");
  ImGui::DragFloat("Move Speed", &move_speed, 0.1f, 0.0f, 100.0f);
  ImGui::DragFloat("Jump Speed", &jump_speed, 0.1f, 0.0f, 100.0f);
  ImGui::DragFloat("Acceleration", &acceleration, 0.1f, 0.0f, 100.0f);
  ImGui::DragFloat("Deceleration", &deceleration, 0.1f, 0.0f, 100.0f);
  ImGui::DragFloat("Directional Acceleration", &directional_acceleration, 0.01f,
                   0.0f, 1.0f);

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

  // draw_arrow(player_trans_proj, player_trans_proj + camera_forward, cam_comp.vp,
  //            Blue, 0.05, 1.0f, false);
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