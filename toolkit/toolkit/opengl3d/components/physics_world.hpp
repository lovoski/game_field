#pragma once

#include "toolkit/opengl3d/components/collider.hpp"
#include "toolkit/opengl3d/components/mesh.hpp"
#include "toolkit/opengl3d/components/physics_constraint.hpp"
#include "toolkit/opengl3d/components/rigidbody.hpp"
#include "toolkit/opengl3d/draw.hpp"
#include "toolkit/system.hpp"
#include "toolkit/transform.hpp"

// Bullet headers
#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>
#include <BulletDynamics/ConstraintSolver/btConeTwistConstraint.h>
#include <BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h>
#include <BulletDynamics/ConstraintSolver/btHingeConstraint.h>
#include <BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h>
#include <BulletDynamics/ConstraintSolver/btSliderConstraint.h>
#include <BulletDynamics/ConstraintSolver/btFixedConstraint.h>

namespace toolkit::opengl3d {

// ─── Bullet ↔ Engine conversion helpers ────────────────────────────────────

inline btVector3 to_bt(const math::vector3 &v) {
  return btVector3(v.x(), v.y(), v.z());
}
inline math::vector3 from_bt(const btVector3 &v) {
  return math::vector3(v.x(), v.y(), v.z());
}
inline btQuaternion to_bt(const math::quat &q) {
  return btQuaternion(q.x(), q.y(), q.z(), q.w());
}
inline math::quat from_bt(const btQuaternion &q) {
  return math::quat(q.w(), q.x(), q.y(), q.z());
}
inline btTransform to_bt_transform(const math::vector3 &pos,
                                   const math::quat &rot) {
  return btTransform(to_bt(rot), to_bt(pos));
}

// ─── Debug drawer using engine's draw utilities ────────────────────────────

class bullet_debug_drawer : public btIDebugDraw {
public:
  math::matrix4 vp = math::matrix4::Identity();

  void drawLine(const btVector3 &from, const btVector3 &to,
                const btVector3 &color) override {
    draw_line(from_bt(from), from_bt(to), vp,
              math::vector3(color.x(), color.y(), color.z()), 1.0f, false);
  }

  void drawContactPoint(const btVector3 &point, const btVector3 &normal,
                        btScalar distance, int /*lifeTime*/,
                        const btVector3 &color) override {
    btVector3 to = point + normal * distance;
    drawLine(point, to, color);
  }

  void reportErrorWarning(const char *warning) override {
    printf("[BulletPhysics] %s\n", warning);
  }

  void draw3dText(const btVector3 & /*location*/,
                  const char * /*text*/) override {}

  void setDebugMode(int mode) override { m_debug_mode = mode; }
  int getDebugMode() const override { return m_debug_mode; }

private:
  int m_debug_mode = btIDebugDraw::DBG_DrawWireframe |
                     btIDebugDraw::DBG_DrawConstraints |
                     btIDebugDraw::DBG_DrawConstraintLimits;
};

// ─── Collision callback info ───────────────────────────────────────────────

struct collision_info {
  entt::entity entity_a = entt::null;
  entt::entity entity_b = entt::null;
  math::vector3 point_on_a = math::vector3::Zero();
  math::vector3 point_on_b = math::vector3::Zero();
  math::vector3 normal = math::vector3::Zero();
  float impulse = 0.0f;
};

// ─── Physics World System ──────────────────────────────────────────────────

/**
 * Physics world system — manages Bullet dynamics world and synchronizes
 * transforms between the ECS and the physics simulation.
 *
 * Unity-like workflow:
 *   1. Add a `collider` component to define a shape.
 *   2. Optionally add a `rigidbody` to make it dynamic/kinematic.
 *      (collider-only = static collider, like Unity)
 *   3. Optionally add `physics_constraint` to connect two rigidbodies.
 *   4. The system handles creation, simulation, and sync automatically.
 *
 * Registration in engine:
 *   physics_world_sys = register_sys<physics_world>();
 *
 * Call each frame:
 *   physics_world_sys->step(registry, dt);
 *   physics_world_sys->debug_draw(registry, vp);  // optional
 */
class physics_world : public isystem {
public:
  // --- Configurable ---

  math::vector3 gravity = math::vector3(0.0f, -9.81f, 0.0f);
  float fixed_timestep = 1.0f / 60.0f;
  int max_substeps = 4;
  bool draw_debug = false;
  bool simulation_enabled = true;

  // Per-frame collision results (cleared each step)
  std::vector<collision_info> collisions;

  // --- Lifecycle ---

  void init0(entt::registry &registry) override;
  void init1(entt::registry &registry) override;

  void shutdown(entt::registry &registry);

  // --- Main update (call once per frame) ---

  void step(entt::registry &registry, float dt);

  // --- Debug visualization ---

  void debug_draw_world(entt::registry &registry, math::matrix4 vp);

  // --- Utility: apply forces/impulses to rigidbodies ---

  void add_force(rigidbody &rb, math::vector3 force);
  void add_impulse(rigidbody &rb, math::vector3 impulse);
  void add_torque(rigidbody &rb, math::vector3 torque);
  void add_force_at_position(rigidbody &rb, math::vector3 force,
                             math::vector3 world_pos);

  // --- Raycast ---

  struct raycast_hit {
    bool hit = false;
    entt::entity entity = entt::null;
    math::vector3 point = math::vector3::Zero();
    math::vector3 normal = math::vector3::Zero();
    float distance = 0.0f;
  };

  raycast_hit raycast(math::vector3 origin, math::vector3 direction,
                      float max_distance = 1000.0f);

  // --- Access to Bullet world for advanced usage ---

  btDiscreteDynamicsWorld *get_dynamics_world() { return dynamics_world; }

  void draw_menu_gui() override;

private:
  // Bullet infrastructure
  btDefaultCollisionConfiguration *collision_config = nullptr;
  btCollisionDispatcher *dispatcher = nullptr;
  btDbvtBroadphase *broadphase = nullptr;
  btSequentialImpulseConstraintSolver *solver = nullptr;
  btDiscreteDynamicsWorld *dynamics_world = nullptr;
  bullet_debug_drawer debug_drawer;

  // Track which entities have been registered
  std::set<entt::entity> registered_bodies;
  std::set<entt::entity> registered_constraints;

  // --- Internal helpers ---

  btCollisionShape *create_bt_shape(entt::registry &registry, entt::entity e,
                                    collider &col);

  void ensure_body_registered(entt::registry &registry, entt::entity e);
  void ensure_constraint_registered(entt::registry &registry, entt::entity e);

  void sync_kinematic_to_bullet(entt::registry &registry);
  void sync_bullet_to_transforms(entt::registry &registry);
  void read_back_velocities(entt::registry &registry);
  void collect_collisions();

  void remove_body(entt::entity e, rigidbody &rb, collider &col);
  void remove_static_body(entt::entity e, collider &col);
  void remove_constraint(entt::entity e, physics_constraint &pc);

  void apply_axis_locks(btRigidBody *body,
                        const rigidbody_constraints_flags &flags);
};
DECLARE_SYSTEM(physics_world, gravity, fixed_timestep, max_substeps,
               draw_debug, simulation_enabled)

}; // namespace toolkit::opengl3d
