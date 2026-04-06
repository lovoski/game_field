#include "toolkit/opengl3d/components/physics_constraint.hpp"

namespace toolkit::opengl3d {

static void draw_dof6_axis_gui(const char *label, dof6_axis_config &cfg) {
  if (ImGui::TreeNode(label)) {
    ImGui::DragFloat("Lower Limit", &cfg.lower_limit, 0.01f);
    ImGui::DragFloat("Upper Limit", &cfg.upper_limit, 0.01f);
    ImGui::Checkbox("Use Spring", &cfg.use_spring);
    if (cfg.use_spring) {
      ImGui::DragFloat("Stiffness", &cfg.stiffness, 0.1f, 0.0f, 10000.0f);
      ImGui::DragFloat("Damping", &cfg.damping, 0.01f, 0.0f, 100.0f);
    }
    ImGui::Checkbox("Use Motor", &cfg.use_motor);
    if (cfg.use_motor) {
      ImGui::DragFloat("Target Velocity", &cfg.motor_target_velocity, 0.01f);
      ImGui::DragFloat("Max Force", &cfg.motor_max_force, 0.1f, 0.0f,
                        10000.0f);
    }
    ImGui::TreePop();
  }
}

void physics_constraint::draw_gui(entt::registry &registry,
                                  entt::entity entity) {
  const char *type_names[] = {"Fixed",      "Hinge",    "Ball Socket",
                              "Cone Twist", "Slider",   "Generic 6DOF"};
  int type_idx = static_cast<int>(type);
  if (ImGui::Combo("Type", &type_idx, type_names, IM_ARRAYSIZE(type_names)))
    type = static_cast<constraint_type>(type_idx);

  // Connected entity display
  ImGui::Text("Connected Entity: %u",
              static_cast<uint32_t>(connected_entity));
  ImGui::DragFloat3("Anchor", anchor.data(), 0.01f);
  ImGui::DragFloat3("Connected Anchor", connected_anchor.data(), 0.01f);
  ImGui::DragFloat3("Axis", axis.data(), 0.01f);

  ImGui::Separator();

  switch (type) {
  case constraint_type::HINGE: {
    ImGui::Checkbox("Use Limits", &hinge_use_limits);
    if (hinge_use_limits) {
      float lo_deg = hinge_lower_limit * 180.0f / 3.14159f;
      float hi_deg = hinge_upper_limit * 180.0f / 3.14159f;
      if (ImGui::DragFloat("Lower (deg)", &lo_deg, 0.5f, -180.0f, 180.0f))
        hinge_lower_limit = lo_deg * 3.14159f / 180.0f;
      if (ImGui::DragFloat("Upper (deg)", &hi_deg, 0.5f, -180.0f, 180.0f))
        hinge_upper_limit = hi_deg * 3.14159f / 180.0f;
    }
    ImGui::Checkbox("Use Motor", &hinge_use_motor);
    if (hinge_use_motor) {
      ImGui::DragFloat("Target Velocity", &hinge_motor_target_velocity, 0.1f);
      ImGui::DragFloat("Max Impulse", &hinge_motor_max_impulse, 0.1f, 0.0f,
                        10000.0f);
    }
    break;
  }
  case constraint_type::CONE_TWIST: {
    float s1_deg = cone_swing_span1 * 180.0f / 3.14159f;
    float s2_deg = cone_swing_span2 * 180.0f / 3.14159f;
    float tw_deg = cone_twist_span * 180.0f / 3.14159f;
    if (ImGui::DragFloat("Swing Span 1 (deg)", &s1_deg, 0.5f, 0.0f, 180.0f))
      cone_swing_span1 = s1_deg * 3.14159f / 180.0f;
    if (ImGui::DragFloat("Swing Span 2 (deg)", &s2_deg, 0.5f, 0.0f, 180.0f))
      cone_swing_span2 = s2_deg * 3.14159f / 180.0f;
    if (ImGui::DragFloat("Twist Span (deg)", &tw_deg, 0.5f, 0.0f, 180.0f))
      cone_twist_span = tw_deg * 3.14159f / 180.0f;
    ImGui::DragFloat("Softness", &cone_softness, 0.01f, 0.0f, 1.0f);
    ImGui::DragFloat("Bias", &cone_bias, 0.01f, 0.0f, 1.0f);
    ImGui::DragFloat("Relaxation", &cone_relaxation, 0.01f, 0.0f, 1.0f);
    break;
  }
  case constraint_type::SLIDER: {
    ImGui::DragFloat("Lower Linear", &slider_lower_lin, 0.01f);
    ImGui::DragFloat("Upper Linear", &slider_upper_lin, 0.01f);
    float sla_deg = slider_lower_ang * 180.0f / 3.14159f;
    float sua_deg = slider_upper_ang * 180.0f / 3.14159f;
    if (ImGui::DragFloat("Lower Angular (deg)", &sla_deg, 0.5f))
      slider_lower_ang = sla_deg * 3.14159f / 180.0f;
    if (ImGui::DragFloat("Upper Angular (deg)", &sua_deg, 0.5f))
      slider_upper_ang = sua_deg * 3.14159f / 180.0f;
    break;
  }
  case constraint_type::GENERIC_6DOF: {
    const char *axis_names[] = {"Linear X",  "Linear Y",  "Linear Z",
                                "Angular X", "Angular Y", "Angular Z"};
    for (int i = 0; i < 6; i++)
      draw_dof6_axis_gui(axis_names[i], dof6_axes[i]);
    break;
  }
  default:
    break;
  }

  ImGui::Separator();
  ImGui::Checkbox("Collision Between Bodies",
                  &enable_collision_between_bodies);
  ImGui::DragFloat("Break Force (0=unbreakable)", &break_force, 0.1f, 0.0f,
                    100000.0f);
}

}; // namespace toolkit::opengl3d
