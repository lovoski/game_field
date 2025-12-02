#pragma once
#include "toolkit/bullet/components/colliders.hpp"

namespace toolkit::bullet {

class bullet_physics : public isystem {
public:
  void init0(entt::registry &registry) override;

  void update(entt::registry &registry, float dt) override;
  void fixedupdate(entt::registry &registry, float dt);

  void draw_gui(entt::registry &registry, entt::entity entity) override;
  void draw_menu_gui() override;

  void draw_to_scene(entt::registry &registry, transform &cam_trans,
                     camera &cam_comp);

  void stepSimulation(float deltaTime);
  void updateTransforms(entt::registry &registry);
  void setRigidBodyTransform(entt::registry &registry, entt::entity entity,
                             const math::vector3 &position,
                             const math::quat &rotation);

  btDiscreteDynamicsWorld *getDynamicsWorld() { return dynamic_world; }

private:
  int cur_exec_fixed = 0;
  float cur_time = 0.0f;

  int sim_fps = 60;
  math::vector3 gravity = math::vector3(0.0, -9.8, 0.0);

  btDefaultCollisionConfiguration *collision_config;
  btCollisionDispatcher *dispatcher;
  btBroadphaseInterface *broadphase;
  btSequentialImpulseConstraintSolver *solver;
  btDiscreteDynamicsWorld *dynamic_world;

  void setupEventHandlers(entt::registry &registry);
  void onRigidBodyDestroyed(entt::registry &reg, entt::entity entity);

  void onColliderAdded(entt::registry &reg, entt::entity entity);

  void createSphereRigidBody(entt::registry &registry, entt::entity entity,
                             const transform &trans);
  void createCapsuleRigidBody(entt::registry &registry, entt::entity entity,
                              const transform &trans);
  void createConvexHullRigidBody(entt::registry &registry, entt::entity entity,
                                 const transform &trans);
  void setupRigidBody(entt::entity entity, rigid_body_component &rb,
                      const transform &trans);

  REFLECT_PRIVATE(bullet_physics)
};
DECLARE_SYSTEM(bullet_physics, gravity, sim_fps)

}; // namespace toolkit::bullet