#include "animator.hpp"

namespace toolkit::opengl3d {

void animator::debug_draw() {}

void animator::handle_engine_gui() {
  ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(550, 500), ImGuiCond_Always);
  ImGui::SetNextWindowBgAlpha(0.7f);
  ImGui::Begin("Settings", nullptr,
               ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

  ImGui::DragFloat("Motion Time Scale", &motion_time_scale, 0.01f, 0.0f, 10.0f);

  ImGui::End();
}

}; // namespace toolkit::opengl3d