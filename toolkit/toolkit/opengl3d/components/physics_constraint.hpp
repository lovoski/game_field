#pragma once

#include "toolkit/math.hpp"
#include "toolkit/system.hpp"
#include "toolkit/transform.hpp"

namespace toolkit::opengl3d {

/**
 * Joint/constraint types, mirrors Unity's Joint family.
 *
 *  FIXED            – welds two bodies together (FixedJoint)
 *  HINGE            – single-axis rotation (HingeJoint) — knees, doors
 *  BALL_SOCKET      – point-to-point (no rotation limits) — simple pendulum
 *  CONE_TWIST       – ball-and-socket with swing/twist limits — hips, shoulders
 *  SLIDER           – single-axis translation (like a piston)
 *  GENERIC_6DOF     – full 6-DOF with per-axis limits and springs
 *                     (ConfigurableJoint in Unity)
 */
enum class constraint_type : int {
  FIXED = 0,
  HINGE = 1,
  BALL_SOCKET = 2,
  CONE_TWIST = 3,
  SLIDER = 4,
  GENERIC_6DOF = 5
};

/**
 * Per-axis spring/motor settings for GENERIC_6DOF joints.
 */
struct dof6_axis_config {
  float lower_limit = 0.0f;
  float upper_limit = 0.0f;
  float stiffness = 0.0f;
  float damping = 0.0f;
  bool use_spring = false;
  bool use_motor = false;
  float motor_target_velocity = 0.0f;
  float motor_max_force = 0.0f;
};
REFLECT(dof6_axis_config, lower_limit, upper_limit, stiffness, damping,
        use_spring, use_motor, motor_target_velocity, motor_max_force)

/**
 * Physics constraint (joint) component.
 *
 * Connects this entity's physics_body to another entity's physics_body.
 * The constraint is always placed on the "child" body; `connected_entity`
 * references the "parent" body (like Unity's connectedBody).
 *
 * If `connected_entity` is entt::null, the joint is attached to the world.
 *
 * Anchors and axes are specified in local space of each body.
 */
struct physics_constraint : public icomponent {
  constraint_type type = constraint_type::FIXED;
  entt::entity connected_entity = entt::null;

  // Anchor positions in local space
  math::vector3 anchor = math::vector3::Zero();
  math::vector3 connected_anchor = math::vector3::Zero();

  // Primary axis in local space (hinge axis, slider axis, twist axis)
  math::vector3 axis = math::vector3(1.0f, 0.0f, 0.0f);

  // --- Hinge parameters ---
  bool hinge_use_limits = true;
  float hinge_lower_limit = 0.0f;     // radians
  float hinge_upper_limit = 3.14159f; // radians
  bool hinge_use_motor = false;
  float hinge_motor_target_velocity = 0.0f;
  float hinge_motor_max_impulse = 1.0f;

  // --- Cone-twist parameters (for ragdoll joints) ---
  float cone_swing_span1 = 0.7f; // radians — swing limit around axis 1
  float cone_swing_span2 = 0.7f; // radians — swing limit around axis 2
  float cone_twist_span = 0.5f;  // radians — twist limit around the axis
  float cone_softness = 1.0f;    // [0,1] how easily the limit is penetrated
  float cone_bias = 0.3f;        // [0,1] constraint correction strength
  float cone_relaxation = 1.0f;  // [0,1] limit relaxation factor

  // --- Slider parameters ---
  float slider_lower_lin = -1.0f;
  float slider_upper_lin = 1.0f;
  float slider_lower_ang = 0.0f;
  float slider_upper_ang = 0.0f;

  // --- Generic 6DOF parameters ---
  // Indices: 0-2 = linear X,Y,Z; 3-5 = angular X,Y,Z
  std::array<dof6_axis_config, 6> dof6_axes = {};

  bool enable_collision_between_bodies = false;
  float break_force = 0.0f;  // 0 = unbreakable

  // Internal — managed by physics_world
  void *bt_constraint = nullptr;

  void draw_gui(entt::registry &registry, entt::entity entity) override;
};
DECLARE_COMPONENT(physics_constraint, physics, type, connected_entity, anchor,
                  connected_anchor, axis, hinge_use_limits, hinge_lower_limit,
                  hinge_upper_limit, hinge_use_motor,
                  hinge_motor_target_velocity, hinge_motor_max_impulse,
                  cone_swing_span1, cone_swing_span2, cone_twist_span,
                  cone_softness, cone_bias, cone_relaxation, slider_lower_lin,
                  slider_upper_lin, slider_lower_ang, slider_upper_ang,
                  dof6_axes, enable_collision_between_bodies, break_force)

}; // namespace toolkit::opengl3d
