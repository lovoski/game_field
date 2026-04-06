#include "toolkit/opengl3d/components/collider.hpp"

namespace toolkit::opengl3d {

void collider::draw_gui(entt::registry &registry, entt::entity entity) {
  const char *shape_names[] = {"Box", "Sphere", "Capsule", "Cylinder",
                               "Mesh", "Convex", "Compound"};
  int shape_idx = static_cast<int>(shape);
  if (ImGui::Combo("Shape", &shape_idx, shape_names, IM_ARRAYSIZE(shape_names))) {
    shape = static_cast<collider_shape>(shape_idx);
    dirty = true;
  }

  if (ImGui::DragFloat3("Center", center.data(), 0.01f))
    dirty = true;

  switch (shape) {
  case collider_shape::BOX:
    if (ImGui::DragFloat3("Half Extents", size.data(), 0.01f, 0.001f, 100.0f))
      dirty = true;
    break;
  case collider_shape::SPHERE:
    if (ImGui::DragFloat("Radius", &size.x(), 0.01f, 0.001f, 100.0f))
      dirty = true;
    break;
  case collider_shape::CAPSULE:
    if (ImGui::DragFloat("Radius", &size.x(), 0.01f, 0.001f, 100.0f))
      dirty = true;
    if (ImGui::DragFloat("Height", &size.y(), 0.01f, 0.001f, 100.0f))
      dirty = true;
    break;
  case collider_shape::CYLINDER:
    if (ImGui::DragFloat("Radius", &size.x(), 0.01f, 0.001f, 100.0f))
      dirty = true;
    if (ImGui::DragFloat("Half Height", &size.y(), 0.01f, 0.001f, 100.0f))
      dirty = true;
    break;
  default:
    break;
  }

  if (ImGui::Checkbox("Is Trigger", &is_trigger))
    dirty = true;
  if (ImGui::DragFloat("Friction", &friction, 0.01f, 0.0f, 2.0f))
    dirty = true;
  if (ImGui::DragFloat("Restitution", &restitution, 0.01f, 0.0f, 2.0f))
    dirty = true;
}

}; // namespace toolkit::opengl3d
