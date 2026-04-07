#pragma once

#include "toolkit/opengl3d/components/character_controller.hpp"
#include "toolkit/opengl3d/components/mesh.hpp"
#include "toolkit/opengl3d/components/physics_body.hpp"
#include "toolkit/opengl3d/components/physics_constraint.hpp"
#include "toolkit/opengl3d/draw.hpp"
#include "toolkit/system.hpp"
#include "toolkit/transform.hpp"

// Bullet headers
#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>
#include <BulletDynamics/Character/btKinematicCharacterController.h>
#include <BulletDynamics/ConstraintSolver/btConeTwistConstraint.h>
#include <BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h>
#include <BulletDynamics/ConstraintSolver/btHingeConstraint.h>
#include <BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h>
#include <BulletDynamics/ConstraintSolver/btSliderConstraint.h>
#include <BulletDynamics/ConstraintSolver/btFixedConstraint.h>

namespace toolkit::opengl3d {

// ─── Bullet ↔ Engine conversion ────────────────────────────────────────────

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

// ─── Debug drawer ──────────────────────────────────────────────────────────

class bullet_debug_drawer : public btIDebugDraw {
public:
  math::matrix4 vp = math::matrix4::Identity();

  void drawLine(const btVector3 &from, const btVector3 &to,
                const btVector3 &color) override {
    draw_line(from_bt(from), from_bt(to), vp,
              math::vector3(color.x(), color.y(), color.z()), 1.0f, false);
  }
  void drawContactPoint(const btVector3 &point, const btVector3 &normal,
                        btScalar distance, int, const btVector3 &color) override {
    drawLine(point, point + normal * distance, color);
  }
  void reportErrorWarning(const char *w) override {
    printf("[BulletPhysics] %s\n", w);
  }
  void draw3dText(const btVector3 &, const char *) override {}
  void setDebugMode(int mode) override { m_debug_mode = mode; }
  int getDebugMode() const override { return m_debug_mode; }

private:
  int m_debug_mode = DBG_DrawWireframe | DBG_DrawConstraints |
                     DBG_DrawConstraintLimits;
};

// ─── Physics World System ──────────────────────────────────────────────────

class physics_world : public isystem {
public:
  // --- Config ---
  math::vector3 gravity = math::vector3(0.0f, -9.81f, 0.0f);
  float fixed_timestep = 1.0f / 60.0f;
  int max_substeps = 8;
  int solver_iterations = 20;  // higher = more precise contacts (default 10)
  float erp = 0.2f;           // error reduction parameter
  float erp2 = 0.8f;          // split-impulse ERP (penetration recovery)
  float ccd_threshold = 0.5f; // dynamic bodies moving faster than this per
                              // step get swept CCD (0 = disabled)
  bool draw_debug = false;
  bool simulation_enabled = true;

  // --- Lifecycle ---
  void init0(entt::registry &registry) override;
  void init1(entt::registry &registry) override;
  void shutdown(entt::registry &registry);

  // --- Per-frame ---
  void step(entt::registry &registry, float dt);
  void debug_draw_world(entt::registry &registry, math::matrix4 vp);

  // --- Forces (require body_type DYNAMIC) ---
  void add_force(physics_body &pb, math::vector3 force);
  void add_impulse(physics_body &pb, math::vector3 impulse);
  void add_torque(physics_body &pb, math::vector3 torque);
  void add_force_at_position(physics_body &pb, math::vector3 force,
                             math::vector3 world_pos);

  // --- Character controller ---
  void cc_move(character_controller &cc, math::vector3 displacement);
  void cc_set_position(character_controller &cc, math::vector3 world_pos);
  void cc_set_velocity(character_controller &cc, math::vector3 vel);

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

  // Registered entity tracking
  std::set<entt::entity> registered_bodies;
  std::set<entt::entity> registered_constraints;
  std::set<entt::entity> registered_controllers;

  // Previous-frame collision pairs for enter/stay/exit detection.
  // Key: ordered (min,max) entity pair. Value: cached contact + trigger flag.
  struct contact_cache {
    math::vector3 point = math::vector3::Zero();
    math::vector3 normal = math::vector3::Zero();
    float impulse = 0.0f;
    bool is_trigger = false;
  };
  std::map<std::pair<uint32_t, uint32_t>, contact_cache> prev_pairs;

  // --- Internal ---
  btCollisionShape *create_shape(entt::registry &registry, entt::entity e,
                                 physics_body &pb);
  void register_body(entt::registry &registry, entt::entity e);
  void register_constraint(entt::registry &registry, entt::entity e);
  void register_controller(entt::registry &registry, entt::entity e);
  void unregister_body(entt::registry &registry, entt::entity e);
  void unregister_constraint(entt::entity e, physics_constraint &pc);
  void unregister_controller(character_controller &cc);
  void update_collision_events(entt::registry &registry);
};
DECLARE_SYSTEM(physics_world, gravity, fixed_timestep, max_substeps,
               solver_iterations, erp, erp2, ccd_threshold,
               draw_debug, simulation_enabled)

}; // namespace toolkit::opengl3d
