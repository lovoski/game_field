#pragma once

#include "toolkit/math.hpp"
#include "toolkit/system.hpp"
#include "toolkit/transform.hpp"

namespace toolkit::opengl3d {

/**
 * Body type, mirrors Unity's RigidbodyType / isKinematic.
 *
 *  DYNAMIC    – fully simulated (mass > 0, responds to forces/collisions)
 *  KINEMATIC  – moved via transform/animation, pushes dynamic bodies but
 *               is not affected by forces (like Unity's isKinematic = true)
 *  STATIC     – never moves (mass = 0, infinite inertia)
 */
enum class rigidbody_type : int { DYNAMIC = 0, KINEMATIC = 1, STATIC = 2 };

/**
 * Interpolation mode for smoothing between physics steps.
 */
enum class rigidbody_interpolation : int {
  NONE = 0,
  INTERPOLATE = 1,
  EXTRAPOLATE = 2
};

/**
 * Freeze flags for locking individual axes.
 */
struct rigidbody_constraints_flags {
  bool freeze_pos_x = false;
  bool freeze_pos_y = false;
  bool freeze_pos_z = false;
  bool freeze_rot_x = false;
  bool freeze_rot_y = false;
  bool freeze_rot_z = false;
};
REFLECT(rigidbody_constraints_flags, freeze_pos_x, freeze_pos_y, freeze_pos_z,
        freeze_rot_x, freeze_rot_y, freeze_rot_z)

/**
 * Rigidbody component — defines dynamics properties for an entity.
 *
 * Requires a collider component on the same entity to define the collision shape.
 * Similar to Unity's Rigidbody component.
 *
 * If body_type is DYNAMIC, the entity is fully physics-simulated.
 * If body_type is KINEMATIC, the physics world reads the transform each step
 * (useful for animation-driven characters that still push objects around).
 * If body_type is STATIC, the body never moves (optimized broadphase).
 */
struct rigidbody : public icomponent {
  rigidbody_type body_type = rigidbody_type::DYNAMIC;
  rigidbody_interpolation interpolation = rigidbody_interpolation::NONE;

  float mass = 1.0f;
  float drag = 0.0f;
  float angular_drag = 0.05f;
  bool use_gravity = true;

  rigidbody_constraints_flags constraints;

  // Runtime state — readable, not serialized
  math::vector3 velocity = math::vector3::Zero();
  math::vector3 angular_velocity = math::vector3::Zero();

  // Internal — managed by physics_world
  void *bt_body = nullptr;
  void *bt_motion_state = nullptr;
  bool dirty = true;

  void draw_gui(entt::registry &registry, entt::entity entity) override;
};
DECLARE_COMPONENT(rigidbody, physics, body_type, interpolation, mass, drag,
                  angular_drag, use_gravity, constraints)

}; // namespace toolkit::opengl3d
