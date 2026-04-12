#include "toolkit/opengl3d/components/character_controller.hpp"

namespace toolkit::opengl3d {

void character_controller::draw_gui(entt::registry &registry,
                                    entt::entity entity) {
  bool changed = false;

  ImGui::Text("Shape");
  if (ImGui::DragFloat("Height", &height, 0.01f, 0.1f, 10.0f))
    changed = true;
  if (ImGui::DragFloat("Radius", &radius, 0.01f, 0.01f, 5.0f))
    changed = true;
  if (ImGui::DragFloat("Step Height", &step_height, 0.01f, 0.0f, 2.0f))
    changed = true;
  if (ImGui::DragFloat("Max Slope", &max_slope, 0.5f, 0.0f, 89.0f))
    changed = true;

  ImGui::Separator();
  ImGui::Text("Runtime");
  ImGui::Text("Grounded: %s", grounded ? "true" : "false");
  ImGui::Text("Events: %d", static_cast<int>(events.size()));

  if (changed)
    dirty = true;
}

}; // namespace toolkit::opengl3d
