#include "toolkit/opengl3d/components/rigidbody.hpp"

namespace toolkit::opengl3d {

void rigidbody::draw_gui(entt::registry &registry, entt::entity entity) {
  const char *type_names[] = {"Dynamic", "Kinematic", "Static"};
  int type_idx = static_cast<int>(body_type);
  if (ImGui::Combo("Body Type", &type_idx, type_names,
                    IM_ARRAYSIZE(type_names))) {
    body_type = static_cast<rigidbody_type>(type_idx);
    dirty = true;
  }

  const char *interp_names[] = {"None", "Interpolate", "Extrapolate"};
  int interp_idx = static_cast<int>(interpolation);
  if (ImGui::Combo("Interpolation", &interp_idx, interp_names,
                    IM_ARRAYSIZE(interp_names))) {
    interpolation = static_cast<rigidbody_interpolation>(interp_idx);
    dirty = true;
  }

  if (body_type == rigidbody_type::DYNAMIC) {
    if (ImGui::DragFloat("Mass", &mass, 0.01f, 0.001f, 10000.0f))
      dirty = true;
    if (ImGui::DragFloat("Drag", &drag, 0.01f, 0.0f, 100.0f))
      dirty = true;
    if (ImGui::DragFloat("Angular Drag", &angular_drag, 0.01f, 0.0f, 100.0f))
      dirty = true;
    if (ImGui::Checkbox("Use Gravity", &use_gravity))
      dirty = true;
  }

  if (ImGui::TreeNode("Constraints")) {
    if (ImGui::Checkbox("Freeze Pos X", &constraints.freeze_pos_x))
      dirty = true;
    ImGui::SameLine();
    if (ImGui::Checkbox("Freeze Pos Y", &constraints.freeze_pos_y))
      dirty = true;
    ImGui::SameLine();
    if (ImGui::Checkbox("Freeze Pos Z", &constraints.freeze_pos_z))
      dirty = true;
    if (ImGui::Checkbox("Freeze Rot X", &constraints.freeze_rot_x))
      dirty = true;
    ImGui::SameLine();
    if (ImGui::Checkbox("Freeze Rot Y", &constraints.freeze_rot_y))
      dirty = true;
    ImGui::SameLine();
    if (ImGui::Checkbox("Freeze Rot Z", &constraints.freeze_rot_z))
      dirty = true;
    ImGui::TreePop();
  }

  ImGui::Separator();
  ImGui::Text("Velocity: (%.2f, %.2f, %.2f)", velocity.x(), velocity.y(),
              velocity.z());
  ImGui::Text("Angular Vel: (%.2f, %.2f, %.2f)", angular_velocity.x(),
              angular_velocity.y(), angular_velocity.z());
}

}; // namespace toolkit::opengl3d
