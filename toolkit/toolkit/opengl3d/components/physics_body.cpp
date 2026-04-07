#include "toolkit/opengl3d/components/physics_body.hpp"

namespace toolkit::opengl3d {

void physics_body::draw_gui(entt::registry &registry, entt::entity entity) {
  bool changed = false;

  // ── Body type ──
  const char *type_names[] = {"Static", "Dynamic", "Kinematic"};
  int type_idx = static_cast<int>(type);
  if (ImGui::Combo("Body Type", &type_idx, type_names,
                    IM_ARRAYSIZE(type_names))) {
    type = static_cast<body_type>(type_idx);
    changed = true;
  }

  // ── Shape ──
  ImGui::Separator();
  const char *shape_names[] = {"Box", "Sphere", "Capsule",
                               "Cylinder", "Mesh", "Convex"};
  int shape_idx = static_cast<int>(shape);
  if (ImGui::Combo("Shape", &shape_idx, shape_names,
                    IM_ARRAYSIZE(shape_names))) {
    shape = static_cast<body_shape>(shape_idx);
    changed = true;
  }
  if (ImGui::DragFloat3("Center", center.data(), 0.01f))
    changed = true;

  switch (shape) {
  case body_shape::BOX:
    if (ImGui::DragFloat3("Half Extents", size.data(), 0.01f, 0.001f, 100.0f))
      changed = true;
    break;
  case body_shape::SPHERE:
    if (ImGui::DragFloat("Radius", &size.x(), 0.01f, 0.001f, 100.0f))
      changed = true;
    break;
  case body_shape::CAPSULE:
    if (ImGui::DragFloat("Radius", &size.x(), 0.01f, 0.001f, 100.0f))
      changed = true;
    if (ImGui::DragFloat("Height", &size.y(), 0.01f, 0.001f, 100.0f))
      changed = true;
    break;
  case body_shape::CYLINDER:
    if (ImGui::DragFloat("Radius", &size.x(), 0.01f, 0.001f, 100.0f))
      changed = true;
    if (ImGui::DragFloat("Half Height", &size.y(), 0.01f, 0.001f, 100.0f))
      changed = true;
    break;
  default:
    break;
  }

  if (ImGui::Checkbox("Is Trigger", &is_trigger))
    changed = true;
  if (ImGui::DragFloat("Friction", &friction, 0.01f, 0.0f, 2.0f))
    changed = true;
  if (ImGui::DragFloat("Restitution", &restitution, 0.01f, 0.0f, 2.0f))
    changed = true;

  // ── Dynamics (only for non-static) ──
  if (type == body_type::DYNAMIC) {
    ImGui::Separator();
    if (ImGui::DragFloat("Mass", &mass, 0.01f, 0.001f, 10000.0f))
      changed = true;
    if (ImGui::DragFloat("Drag", &drag, 0.01f, 0.0f, 100.0f))
      changed = true;
    if (ImGui::DragFloat("Angular Drag", &angular_drag, 0.01f, 0.0f, 100.0f))
      changed = true;
    if (ImGui::Checkbox("Use Gravity", &use_gravity))
      changed = true;
  }

  if (type != body_type::STATIC && ImGui::TreeNode("Constraints")) {
    if (ImGui::Checkbox("Freeze Pos X", &constraints.freeze_pos_x))
      changed = true;
    ImGui::SameLine();
    if (ImGui::Checkbox("Freeze Pos Y", &constraints.freeze_pos_y))
      changed = true;
    ImGui::SameLine();
    if (ImGui::Checkbox("Freeze Pos Z", &constraints.freeze_pos_z))
      changed = true;
    if (ImGui::Checkbox("Freeze Rot X", &constraints.freeze_rot_x))
      changed = true;
    ImGui::SameLine();
    if (ImGui::Checkbox("Freeze Rot Y", &constraints.freeze_rot_y))
      changed = true;
    ImGui::SameLine();
    if (ImGui::Checkbox("Freeze Rot Z", &constraints.freeze_rot_z))
      changed = true;
    ImGui::TreePop();
  }

  // ── Runtime readout ──
  ImGui::Separator();
  if (type == body_type::DYNAMIC) {
    ImGui::Text("Velocity: (%.2f, %.2f, %.2f)", velocity.x(), velocity.y(),
                velocity.z());
    ImGui::Text("Angular Vel: (%.2f, %.2f, %.2f)", angular_velocity.x(),
                angular_velocity.y(), angular_velocity.z());
  }

  // ── Collision events ──
  int collision_count = 0, trigger_count = 0;
  for (auto &ev : events) {
    if (ev.type == collision_event_type::COLLISION_ENTER ||
        ev.type == collision_event_type::COLLISION_STAY)
      collision_count++;
    if (ev.type == collision_event_type::TRIGGER_ENTER ||
        ev.type == collision_event_type::TRIGGER_STAY)
      trigger_count++;
  }
  ImGui::Text("Events: %d  (col %d / trig %d)",
              static_cast<int>(events.size()), collision_count, trigger_count);
  if (!events.empty() && ImGui::TreeNode("Events")) {
    const char *ev_names[] = {"CollisionEnter", "CollisionStay",
                              "CollisionExit",  "TriggerEnter",
                              "TriggerStay",    "TriggerExit"};
    for (size_t i = 0; i < events.size(); i++) {
      auto &ev = events[i];
      ImGui::Text("[%zu] %s  entity=%u", i,
                  ev_names[static_cast<int>(ev.type)],
                  static_cast<uint32_t>(ev.other_entity));
    }
    ImGui::TreePop();
  }

  if (changed)
    dirty = true;
}

}; // namespace toolkit::opengl3d
