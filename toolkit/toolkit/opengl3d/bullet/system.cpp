#include "toolkit/opengl3d/bullet/system.hpp"

namespace toolkit::bullet {

void bullet_physics::init0(entt::registry &registry) {}

void bullet_physics::fixedupdate(entt::registry &registry, float dt) {}

void bullet_physics::draw_gui(entt::registry &registry, entt::entity entity) {}

void bullet_physics::draw_menu_gui() {}

void bullet_physics::draw_to_scene(entt::registry &registry,
                                   transform &cam_trans,
                                   opengl3d::camera &cam_comp) {}

void bullet_physics::stepSimulation(float deltaTime) {
  dynamic_world->stepSimulation(deltaTime);
}

void bullet_physics::updateTransforms(entt::registry &registry) {
  registry.view<transform, rigid_body_component>().each(
      [&](entt::entity entity, transform &trans, rigid_body_component &rb) {
        if (rb.rigid_body && rb.rigid_body->getMotionState()) {
          btTransform physicsTransform;
          rb.rigid_body->getMotionState()->getWorldTransform(physicsTransform);

          // Update transform component
          btVector3 origin = physicsTransform.getOrigin();
          btQuaternion rotation = physicsTransform.getRotation();

          trans.set_world_pos(
              math::vector3(origin.x(), origin.y(), origin.z()));
          trans.set_world_rot(math::quat(rotation.w(), rotation.x(),
                                         rotation.y(), rotation.z()));
        }
      });
}

void bullet_physics::setRigidBodyTransform(entt::registry &registry,
                                           entt::entity entity,
                                           const math::vector3 &position,
                                           const math::quat &rotation) {
  if (auto rb = registry.try_get<rigid_body_component>(entity)) {
    if (rb->rigid_body) {
      btTransform transform;
      transform.setOrigin(btVector3(position.x(), position.y(), position.z()));
      transform.setRotation(
          btQuaternion(rotation.x(), rotation.y(), rotation.z(), rotation.w()));

      rb->rigid_body->setWorldTransform(transform);
      if (rb->rigid_body->getMotionState()) {
        rb->rigid_body->getMotionState()->setWorldTransform(transform);
      }
    }
  }
}

void bullet_physics::update(entt::registry &registry, float dt) {
  float fixed_interval = 1.0f / sim_fps;
  float residual = cur_time - cur_exec_fixed * fixed_interval;
  while (residual > fixed_interval) {
    residual -= fixed_interval;
    fixedupdate(registry, fixed_interval);
    cur_exec_fixed += 1;
  }
  cur_time += dt;
}

void bullet_physics::setupEventHandlers(entt::registry &registry) {
  // When an entity is destroyed, clean up its physics body
  registry.on_destroy<rigid_body_component>()
      .connect<&bullet_physics::onRigidBodyDestroyed>(this);
  // When components are added, set up physics bodies
  registry.on_construct<sphere_collider>()
      .connect<&bullet_physics::onColliderAdded>(this);
  registry.on_construct<capsule_collider>()
      .connect<&bullet_physics::onColliderAdded>(this);
  registry.on_construct<convex_hull_collider>()
      .connect<&bullet_physics::onColliderAdded>(this);
}

void bullet_physics::onRigidBodyDestroyed(entt::registry &reg,
                                          entt::entity entity) {
  auto &rb = reg.get<rigid_body_component>(entity);
  if (rb.rigid_body) {
    dynamic_world->removeRigidBody(rb.rigid_body);
  }
}

void bullet_physics::onColliderAdded(entt::registry &reg, entt::entity entity) {
  // Only create physics body if we have a transform and don't already have a
  // rigid body
  if (!reg.all_of<transform>(entity) ||
      reg.all_of<rigid_body_component>(entity)) {
    return;
  }

  auto &trans = reg.get<transform>(entity);

  // Create rigid body based on which collider type we have
  if (reg.all_of<sphere_collider>(entity)) {
    createSphereRigidBody(reg, entity, trans);
  } else if (reg.all_of<capsule_collider>(entity)) {
    createCapsuleRigidBody(reg, entity, trans);
  } else if (reg.all_of<convex_hull_collider>(entity)) {
    createConvexHullRigidBody(reg, entity, trans);
  }
}
void bullet_physics::createSphereRigidBody(entt::registry &registry,
                                           entt::entity entity,
                                           const transform &trans) {
  auto &sphere = registry.get<sphere_collider>(entity);

  auto &rb = registry.emplace<rigid_body_component>(entity);
  rb.mass = 1.0f; // Default mass

  // Create sphere shape
  rb.collision_shape = new btSphereShape(sphere.radius);

  setupRigidBody(entity, rb, trans);
}
void bullet_physics::createCapsuleRigidBody(entt::registry &registry,
                                            entt::entity entity,
                                            const transform &trans) {
  auto &capsule = registry.get<capsule_collider>(entity);

  auto &rb = registry.emplace<rigid_body_component>(entity);
  rb.mass = 1.0f; // Default mass

  // Create capsule shape (Y-axis aligned)
  rb.collision_shape = new btCapsuleShape(capsule.radius, capsule.height);

  setupRigidBody(entity, rb, trans);
}
void bullet_physics::createConvexHullRigidBody(entt::registry &registry,
                                               entt::entity entity,
                                               const transform &trans) {
  auto &convexHull = registry.get<convex_hull_collider>(entity);

  auto &rb = registry.emplace<rigid_body_component>(entity);
  rb.mass = 1.0f; // Default mass

  // Create convex hull shape
  btConvexHullShape *convexShape = new btConvexHullShape();
  for (const auto &vertex : convexHull.vertices) {
    convexShape->addPoint(btVector3(vertex.x(), vertex.y(), vertex.z()));
  }
  convexShape->optimizeConvexHull();

  rb.collision_shape = convexShape;

  setupRigidBody(entity, rb, trans);
}
void bullet_physics::setupRigidBody(entt::entity entity,
                                    rigid_body_component &rb,
                                    const transform &trans) {
  // Create initial transform
  btTransform startTransform;
  startTransform.setIdentity();
  startTransform.setOrigin(btVector3(
      trans.world_pos().x(), trans.world_pos().y(), trans.world_pos().z()));
  startTransform.setRotation(
      btQuaternion(trans.world_rot().x(), trans.world_rot().y(),
                   trans.world_rot().z(), trans.world_rot().w()));

  // Calculate inertia
  btVector3 localInertia(0, 0, 0);
  if (rb.mass != 0.0f) {
    rb.collision_shape->calculateLocalInertia(rb.mass, localInertia);
  }

  // Create motion state and rigid body
  btDefaultMotionState *motionState = new btDefaultMotionState(startTransform);
  btRigidBody::btRigidBodyConstructionInfo rbInfo(
      rb.mass, motionState, rb.collision_shape, localInertia);

  rb.rigid_body = new btRigidBody(rbInfo);
  dynamic_world->addRigidBody(rb.rigid_body);

  // Store entity reference in the rigid body for collision callbacks
  rb.rigid_body->setUserPointer((void *)entity);
}

}; // namespace toolkit::bullet