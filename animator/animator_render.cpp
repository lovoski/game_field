#include "animator.hpp"

namespace toolkit::opengl3d {

void animator::debug_draw() {}

void animator::handle_engine_gui() {
  ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(550, 500), ImGuiCond_Always);
  ImGui::SetNextWindowBgAlpha(0.7f);
  ImGui::Begin("Settings", nullptr,
               ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

  ImGui::SeparatorText("Statistics");
  ImGui::DragFloat("Motion Time Scale", &motion_time_scale, 0.01f, -10.0f, 10.0f);
  ImGui::Text("Motion Frames: %d", motion_data.local_rot.size());
  ImGui::Text("Motion Time: %.3f s / %.3f s", motion_time,
              motion_data.frametime * motion_data.local_rot.size());

  ImGui::SeparatorText("Camera");
  ImGui::DragFloat("Camera Distance", &camera_distance, 0.01f, 0.0f, 10.0f);
  ImGui::Text("Camera Height: %.3f", camera_height);
  ImGui::Text("Camera Horizontal Angle: %.3f", camera_horizontal_angle);
  ImGui::Text("Camera Vertical Angle: %.3f", camera_vertical_angle);

  ImGui::End();
}

}; // namespace toolkit::opengl3d