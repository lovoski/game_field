#pragma once

#include "toolkit/math.hpp"
#include "toolkit/opengl3d/components/physics_body.hpp"
#include "toolkit/system.hpp"
#include "toolkit/transform.hpp"

namespace toolkit::opengl3d {

/**
 * Character controller — capsule-based kinematic character movement.
 *
 * Uses Bullet's btKinematicCharacterController internally.
 * Not a rigidbody — moves via displacement and interacts kinematically.
 *
 * Setup:
 *   1. Add this component (no physics_body needed).
 *   2. Call physics_world::cc_move() each frame with displacement.
 *   3. Check `grounded` after the step.
 *   4. Read `events` for collision enter/stay/exit.
 */
struct character_controller : public icomponent {
  // Shape
  float height = 1.8f;        // Total capsule height (including caps)
  float radius = 0.3f;        // Capsule radius
  float step_height = 0.35f;  // Max step-up height
  float max_slope = 45.0f;    // Maximum walkable slope (degrees)

  // Runtime state (read-only, updated by physics_world)
  bool grounded = false;
  std::vector<collision_event> events; // cleared each frame

  // Internal — managed by physics_world
  void *bt_ghost_object = nullptr;
  void *bt_controller = nullptr;
  void *bt_shape = nullptr;
  bool dirty = true;

  void draw_gui(entt::registry &registry, entt::entity entity) override;
};
DECLARE_COMPONENT(character_controller, physics, height, radius, step_height,
                  max_slope)

}; // namespace toolkit::opengl3d
