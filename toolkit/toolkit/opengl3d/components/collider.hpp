#pragma once

#include "toolkit/math.hpp"
#include "toolkit/system.hpp"
#include "toolkit/transform.hpp"

namespace toolkit::opengl3d {

/**
 * Collider shape types, analogous to Unity's collider components.
 *
 *  BOX       - axis-aligned box defined by half-extents
 *  SPHERE    - sphere defined by radius
 *  CAPSULE   - capsule defined by radius and height (along local Y)
 *  CYLINDER  - cylinder defined by half-extents (along local Y)
 *  MESH      - static triangle mesh (from mesh_data AABB when no mesh)
 *  CONVEX    - convex hull built from mesh vertices
 *  COMPOUND  - reserved for future compound shapes
 */
enum class collider_shape : int {
  BOX = 0,
  SPHERE = 1,
  CAPSULE = 2,
  CYLINDER = 3,
  MESH = 4,
  CONVEX = 5,
  COMPOUND = 6
};

/**
 * Collider component — defines the collision shape for an entity.
 *
 * Similar to Unity's BoxCollider / SphereCollider / CapsuleCollider / MeshCollider,
 * but combined into a single component with a shape enum.
 *
 * The collider can exist without a rigidbody; in that case it acts as a static
 * collider (like Unity). If a rigidbody is also present, the shape is used for
 * that rigidbody's dynamics.
 *
 * `center` is the local-space offset from the entity's transform origin.
 * `size` interpretation depends on shape:
 *   BOX:      half-extents (x, y, z)
 *   SPHERE:   x = radius (y, z ignored)
 *   CAPSULE:  x = radius, y = total height (including caps)
 *   CYLINDER: half-extents (x = radius, y = half-height)
 *   MESH/CONVEX: ignored (built from mesh vertices)
 */
struct collider : public icomponent {
  collider_shape shape = collider_shape::BOX;
  math::vector3 center = math::vector3::Zero();
  math::vector3 size = math::vector3(0.5f, 0.5f, 0.5f);

  bool is_trigger = false;

  // Physics material
  float friction = 0.5f;
  float restitution = 0.3f;

  // Internal — managed by physics_world, not serialized
  void *bt_shape = nullptr;
  bool dirty = true;

  void draw_gui(entt::registry &registry, entt::entity entity) override;
};
DECLARE_COMPONENT(collider, physics, shape, center, size, is_trigger, friction,
                  restitution)

}; // namespace toolkit::opengl3d
