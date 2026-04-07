#pragma once

#include "toolkit/math.hpp"
#include "toolkit/system.hpp"
#include "toolkit/transform.hpp"

namespace toolkit::opengl3d {

// ─── Enums & helpers ───────────────────────────────────────────────────────

enum class body_shape : int {
  BOX = 0,
  SPHERE = 1,
  CAPSULE = 2,
  CYLINDER = 3,
  MESH = 4,
  CONVEX = 5,
};

enum class body_type : int {
  STATIC = 0,
  DYNAMIC = 1,
  KINEMATIC = 2,
};

struct axis_lock {
  bool freeze_pos_x = false;
  bool freeze_pos_y = false;
  bool freeze_pos_z = false;
  bool freeze_rot_x = false;
  bool freeze_rot_y = false;
  bool freeze_rot_z = false;
};
REFLECT(axis_lock, freeze_pos_x, freeze_pos_y, freeze_pos_z, freeze_rot_x,
        freeze_rot_y, freeze_rot_z)

// ─── Collision events ──────────────────────────────────────────────────────

enum class collision_event_type : int {
  COLLISION_ENTER = 0,
  COLLISION_STAY = 1,
  COLLISION_EXIT = 2,
  TRIGGER_ENTER = 3,
  TRIGGER_STAY = 4,
  TRIGGER_EXIT = 5,
};

struct collision_event {
  entt::entity other_entity = entt::null;
  collision_event_type type = collision_event_type::COLLISION_ENTER;
  math::vector3 contact_point = math::vector3::Zero();
  math::vector3 contact_normal = math::vector3::Zero();
  float impulse = 0.0f;
};

// ─── physics_body — merged collider + rigidbody + collision state ──────────

/**
 * Unified physics component. Replaces the old collider + rigidbody +
 * collision_state trio with a single component.
 *
 * body_type controls the simulation mode:
 *   STATIC    — never moves (mass ignored). Walls, floors.
 *   DYNAMIC   — fully simulated. Crates, projectiles.
 *   KINEMATIC — moved by transform, pushes dynamic bodies.
 *
 * Collision events (enter/stay/exit) are written into `events` every frame
 * by physics_world. Read them in your sub_system update.
 */
struct physics_body : public icomponent {
  // ── Shape ──
  body_shape shape = body_shape::BOX;
  math::vector3 center = math::vector3::Zero();
  math::vector3 size = math::vector3(0.5f, 0.5f, 0.5f);
  bool is_trigger = false;
  float friction = 0.5f;
  float restitution = 0.3f;

  // ── Dynamics (ignored when STATIC) ──
  body_type type = body_type::STATIC;
  float mass = 1.0f;
  float drag = 0.0f;
  float angular_drag = 0.05f;
  bool use_gravity = true;
  axis_lock constraints;

  // ── Runtime state (read-only, written by physics_world) ──
  math::vector3 velocity = math::vector3::Zero();
  math::vector3 angular_velocity = math::vector3::Zero();
  std::vector<collision_event> events;

  // ── Internal — managed by physics_world ──
  void *bt_body = nullptr;
  void *bt_motion_state = nullptr;
  void *bt_shape = nullptr;
  bool dirty = true;

  void draw_gui(entt::registry &registry, entt::entity entity) override;
};
DECLARE_COMPONENT(physics_body, physics, shape, center, size, is_trigger,
                  friction, restitution, type, mass, drag, angular_drag,
                  use_gravity, constraints)

}; // namespace toolkit::opengl3d
