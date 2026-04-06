#include "toolkit/opengl3d/components/physics_world.hpp"

namespace toolkit::opengl3d {

// ─── Lifecycle ─────────────────────────────────────────────────────────────

void physics_world::init0(entt::registry &registry) {
  collision_config = new btDefaultCollisionConfiguration();
  dispatcher = new btCollisionDispatcher(collision_config);
  broadphase = new btDbvtBroadphase();
  broadphase->getOverlappingPairCache()->setInternalGhostPairCallback(
      new btGhostPairCallback());
  solver = new btSequentialImpulseConstraintSolver();
  dynamics_world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver,
                                              collision_config);
  dynamics_world->setGravity(to_bt(gravity));
  dynamics_world->setDebugDrawer(&debug_drawer);
}

void physics_world::init1(entt::registry &registry) {
  // Called after scene deserialization — reset Bullet world so stale objects
  // from the previous scene are cleaned up.  Fields like gravity etc. have
  // already been restored by from_json before this is called.
  if (dynamics_world) {
    shutdown(registry);
    init0(registry);
  }
}

void physics_world::shutdown(entt::registry &registry) {
  // Remove constraints first
  for (int i = dynamics_world->getNumConstraints() - 1; i >= 0; i--) {
    auto *c = dynamics_world->getConstraint(i);
    dynamics_world->removeConstraint(c);
    delete c;
  }
  // Remove bodies
  for (int i = dynamics_world->getNumCollisionObjects() - 1; i >= 0; i--) {
    btCollisionObject *obj = dynamics_world->getCollisionObjectArray()[i];
    btRigidBody *body = btRigidBody::upcast(obj);
    if (body && body->getMotionState())
      delete body->getMotionState();
    dynamics_world->removeCollisionObject(obj);
    delete obj->getCollisionShape();
    delete obj;
  }
  registered_bodies.clear();
  registered_constraints.clear();

  delete dynamics_world;
  delete solver;
  delete broadphase;
  delete dispatcher;
  delete collision_config;
  dynamics_world = nullptr;
  solver = nullptr;
  broadphase = nullptr;
  dispatcher = nullptr;
  collision_config = nullptr;
}

// ─── Shape creation ────────────────────────────────────────────────────────

btCollisionShape *physics_world::create_bt_shape(entt::registry &registry,
                                                 entt::entity e,
                                                 collider &col) {
  btCollisionShape *shape = nullptr;

  switch (col.shape) {
  case collider_shape::BOX:
    shape = new btBoxShape(to_bt(col.size));
    break;

  case collider_shape::SPHERE:
    shape = new btSphereShape(col.size.x());
    break;

  case collider_shape::CAPSULE:
    // Capsule total height = size.y, radius = size.x
    // btCapsuleShape(radius, cylinderHeight) — height excludes caps
    shape = new btCapsuleShape(col.size.x(),
                               std::max(0.f, col.size.y() - 2.f * col.size.x()));
    break;

  case collider_shape::CYLINDER:
    shape = new btCylinderShape(btVector3(col.size.x(), col.size.y(), col.size.x()));
    break;

  case collider_shape::MESH: {
    // Build static triangle mesh from mesh_data if available
    auto *md = registry.try_get<mesh_data>(e);
    if (md && md->indices.size() >= 3) {
      auto *mesh = new btTriangleMesh();
      for (size_t i = 0; i + 2 < md->indices.size(); i += 3) {
        auto &v0 = md->vertices[md->indices[i]].position;
        auto &v1 = md->vertices[md->indices[i + 1]].position;
        auto &v2 = md->vertices[md->indices[i + 2]].position;
        mesh->addTriangle(btVector3(v0.x(), v0.y(), v0.z()),
                          btVector3(v1.x(), v1.y(), v1.z()),
                          btVector3(v2.x(), v2.y(), v2.z()));
      }
      shape = new btBvhTriangleMeshShape(mesh, true);
    } else {
      // Fallback: use AABB as box
      if (md) {
        math::vector3 half = (md->bb_max - md->bb_min) * 0.5f;
        shape = new btBoxShape(to_bt(half));
      } else {
        shape = new btBoxShape(to_bt(col.size));
      }
    }
    break;
  }

  case collider_shape::CONVEX: {
    auto *md = registry.try_get<mesh_data>(e);
    if (md && !md->vertices.empty()) {
      auto *hull = new btConvexHullShape();
      for (auto &v : md->vertices)
        hull->addPoint(
            btVector3(v.position.x(), v.position.y(), v.position.z()), false);
      hull->recalcLocalAabb();
      hull->optimizeConvexHull();
      shape = hull;
    } else {
      shape = new btBoxShape(to_bt(col.size));
    }
    break;
  }

  case collider_shape::COMPOUND:
    // Placeholder — compound shapes require user to build manually
    shape = new btBoxShape(to_bt(col.size));
    break;
  }

  return shape;
}

// ─── Body registration ─────────────────────────────────────────────────────

void physics_world::ensure_body_registered(entt::registry &registry,
                                           entt::entity e) {
  if (registered_bodies.count(e))
    return;

  auto *col = registry.try_get<collider>(e);
  if (!col)
    return;
  auto *trans = registry.try_get<transform>(e);
  if (!trans)
    return;

  // Create collision shape
  btCollisionShape *bt_shape = create_bt_shape(registry, e, *col);
  col->bt_shape = bt_shape;

  // Apply center offset via compound shape if non-zero
  btCollisionShape *effective_shape = bt_shape;
  if (col->center.squaredNorm() > 1e-8f) {
    auto *compound = new btCompoundShape();
    btTransform local;
    local.setIdentity();
    local.setOrigin(to_bt(col->center));
    compound->addChildShape(local, bt_shape);
    effective_shape = compound;
  }

  // Determine mass
  auto *rb = registry.try_get<rigidbody>(e);
  float mass = 0.0f;
  if (rb) {
    if (rb->body_type == rigidbody_type::DYNAMIC)
      mass = rb->mass;
    // KINEMATIC and STATIC both have mass = 0 in Bullet
  }

  btVector3 local_inertia(0, 0, 0);
  if (mass > 0)
    effective_shape->calculateLocalInertia(mass, local_inertia);

  // Build initial transform
  btTransform bt_trans =
      to_bt_transform(trans->world_pos(), trans->world_rot());

  auto *motion_state = new btDefaultMotionState(bt_trans);
  btRigidBody::btRigidBodyConstructionInfo ci(mass, motion_state,
                                              effective_shape, local_inertia);
  ci.m_friction = col->friction;
  ci.m_restitution = col->restitution;

  auto *body = new btRigidBody(ci);

  // Store entity ID in user pointer for collision callbacks / raycasts
  body->setUserIndex(static_cast<int>(entt::to_integral(e)));

  // Configure body type
  if (rb) {
    rb->bt_body = body;
    rb->bt_motion_state = motion_state;

    if (rb->body_type == rigidbody_type::KINEMATIC) {
      body->setCollisionFlags(body->getCollisionFlags() |
                              btCollisionObject::CF_KINEMATIC_OBJECT);
      body->setActivationState(DISABLE_DEACTIVATION);
    }

    if (rb->body_type == rigidbody_type::STATIC) {
      body->setCollisionFlags(body->getCollisionFlags() |
                              btCollisionObject::CF_STATIC_OBJECT);
    }

    // Drag → damping
    body->setDamping(rb->drag, rb->angular_drag);

    // Gravity
    if (!rb->use_gravity)
      body->setGravity(btVector3(0, 0, 0));

    // Axis locks
    apply_axis_locks(body, rb->constraints);
  } else {
    // Collider only (no rigidbody) → static
    body->setCollisionFlags(body->getCollisionFlags() |
                            btCollisionObject::CF_STATIC_OBJECT);
  }

  // Triggers
  if (col->is_trigger) {
    body->setCollisionFlags(body->getCollisionFlags() |
                            btCollisionObject::CF_NO_CONTACT_RESPONSE);
  }

  dynamics_world->addRigidBody(body);
  registered_bodies.insert(e);
}

// ─── Constraint registration ───────────────────────────────────────────────

void physics_world::ensure_constraint_registered(entt::registry &registry,
                                                 entt::entity e) {
  if (registered_constraints.count(e))
    return;

  auto *pc = registry.try_get<physics_constraint>(e);
  if (!pc)
    return;

  auto *rb_a = registry.try_get<rigidbody>(e);
  if (!rb_a || !rb_a->bt_body)
    return;

  btRigidBody *body_a = static_cast<btRigidBody *>(rb_a->bt_body);
  btRigidBody *body_b = nullptr;

  if (pc->connected_entity != entt::null && registry.valid(pc->connected_entity)) {
    auto *rb_b = registry.try_get<rigidbody>(pc->connected_entity);
    if (rb_b && rb_b->bt_body)
      body_b = static_cast<btRigidBody *>(rb_b->bt_body);
  }

  btTypedConstraint *constraint = nullptr;

  switch (pc->type) {

  case constraint_type::FIXED: {
    btTransform frame_a, frame_b;
    frame_a.setIdentity();
    frame_a.setOrigin(to_bt(pc->anchor));
    frame_b.setIdentity();
    frame_b.setOrigin(to_bt(pc->connected_anchor));
    if (body_b)
      constraint = new btFixedConstraint(*body_a, *body_b, frame_a, frame_b);
    else {
      // Fixed to world — create a static anchor body
      static btRigidBody s_fixed_body(0.0f, nullptr, nullptr);
      s_fixed_body.setWorldTransform(frame_b);
      constraint = new btFixedConstraint(*body_a, s_fixed_body, frame_a, frame_b);
    }
    break;
  }

  case constraint_type::HINGE: {
    if (body_b) {
      constraint = new btHingeConstraint(
          *body_a, *body_b, to_bt(pc->anchor), to_bt(pc->connected_anchor),
          to_bt(pc->axis), to_bt(pc->axis));
    } else {
      constraint = new btHingeConstraint(*body_a, to_bt(pc->anchor),
                                         to_bt(pc->axis));
    }
    auto *hinge = static_cast<btHingeConstraint *>(constraint);
    if (pc->hinge_use_limits)
      hinge->setLimit(pc->hinge_lower_limit, pc->hinge_upper_limit);
    if (pc->hinge_use_motor)
      hinge->enableAngularMotor(true, pc->hinge_motor_target_velocity,
                                pc->hinge_motor_max_impulse);
    break;
  }

  case constraint_type::BALL_SOCKET: {
    if (body_b) {
      constraint = new btPoint2PointConstraint(*body_a, *body_b,
                                               to_bt(pc->anchor),
                                               to_bt(pc->connected_anchor));
    } else {
      constraint = new btPoint2PointConstraint(*body_a, to_bt(pc->anchor));
    }
    break;
  }

  case constraint_type::CONE_TWIST: {
    btTransform frame_a, frame_b;
    frame_a.setIdentity();
    frame_a.setOrigin(to_bt(pc->anchor));
    // Orient frame so that the twist axis aligns with pc->axis
    math::quat rot_a = math::from_to_rot(math::vector3(0, 0, 1), pc->axis);
    frame_a.setRotation(to_bt(rot_a));

    frame_b.setIdentity();
    frame_b.setOrigin(to_bt(pc->connected_anchor));
    frame_b.setRotation(to_bt(rot_a));

    if (body_b) {
      constraint = new btConeTwistConstraint(*body_a, *body_b, frame_a, frame_b);
    } else {
      constraint = new btConeTwistConstraint(*body_a, frame_a);
    }
    auto *ct = static_cast<btConeTwistConstraint *>(constraint);
    ct->setLimit(pc->cone_swing_span1, pc->cone_swing_span2,
                 pc->cone_twist_span, pc->cone_softness, pc->cone_bias,
                 pc->cone_relaxation);
    break;
  }

  case constraint_type::SLIDER: {
    btTransform frame_a, frame_b;
    frame_a.setIdentity();
    frame_a.setOrigin(to_bt(pc->anchor));
    frame_b.setIdentity();
    frame_b.setOrigin(to_bt(pc->connected_anchor));

    if (body_b) {
      constraint = new btSliderConstraint(*body_a, *body_b, frame_a, frame_b, true);
    } else {
      constraint = new btSliderConstraint(*body_a, frame_a, true);
    }
    auto *sl = static_cast<btSliderConstraint *>(constraint);
    sl->setLowerLinLimit(pc->slider_lower_lin);
    sl->setUpperLinLimit(pc->slider_upper_lin);
    sl->setLowerAngLimit(pc->slider_lower_ang);
    sl->setUpperAngLimit(pc->slider_upper_ang);
    break;
  }

  case constraint_type::GENERIC_6DOF: {
    btTransform frame_a, frame_b;
    frame_a.setIdentity();
    frame_a.setOrigin(to_bt(pc->anchor));
    frame_b.setIdentity();
    frame_b.setOrigin(to_bt(pc->connected_anchor));

    btGeneric6DofSpring2Constraint *dof6 = nullptr;
    if (body_b) {
      dof6 = new btGeneric6DofSpring2Constraint(*body_a, *body_b, frame_a,
                                                frame_b);
    } else {
      dof6 = new btGeneric6DofSpring2Constraint(*body_a, frame_a);
    }

    // Linear limits (axes 0-2)
    btVector3 lin_lower(pc->dof6_axes[0].lower_limit,
                        pc->dof6_axes[1].lower_limit,
                        pc->dof6_axes[2].lower_limit);
    btVector3 lin_upper(pc->dof6_axes[0].upper_limit,
                        pc->dof6_axes[1].upper_limit,
                        pc->dof6_axes[2].upper_limit);
    dof6->setLinearLowerLimit(lin_lower);
    dof6->setLinearUpperLimit(lin_upper);

    // Angular limits (axes 3-5)
    btVector3 ang_lower(pc->dof6_axes[3].lower_limit,
                        pc->dof6_axes[4].lower_limit,
                        pc->dof6_axes[5].lower_limit);
    btVector3 ang_upper(pc->dof6_axes[3].upper_limit,
                        pc->dof6_axes[4].upper_limit,
                        pc->dof6_axes[5].upper_limit);
    dof6->setAngularLowerLimit(ang_lower);
    dof6->setAngularUpperLimit(ang_upper);

    // Springs and motors
    for (int i = 0; i < 6; i++) {
      auto &cfg = pc->dof6_axes[i];
      if (cfg.use_spring) {
        dof6->enableSpring(i, true);
        dof6->setStiffness(i, cfg.stiffness);
        dof6->setDamping(i, cfg.damping);
      }
      if (cfg.use_motor) {
        dof6->enableMotor(i, true);
        dof6->setTargetVelocity(i, cfg.motor_target_velocity);
        dof6->setMaxMotorForce(i, cfg.motor_max_force);
      }
    }

    constraint = dof6;
    break;
  }
  } // switch

  if (!constraint)
    return;

  // Breaking threshold
  if (pc->break_force > 0)
    constraint->setBreakingImpulseThreshold(pc->break_force);

  dynamics_world->addConstraint(constraint, !pc->enable_collision_between_bodies);
  pc->bt_constraint = constraint;
  registered_constraints.insert(e);
}

// ─── Transform synchronization ─────────────────────────────────────────────

void physics_world::sync_kinematic_to_bullet(entt::registry &registry) {
  auto view = registry.view<rigidbody, collider, transform>();
  for (auto e : view) {
    auto &rb = view.get<rigidbody>(e);
    if (rb.body_type != rigidbody_type::KINEMATIC || !rb.bt_body)
      continue;
    auto &trans = view.get<transform>(e);
    auto *motion =
        static_cast<btDefaultMotionState *>(rb.bt_motion_state);
    btTransform bt_t = to_bt_transform(trans.world_pos(), trans.world_rot());
    motion->setWorldTransform(bt_t);
    static_cast<btRigidBody *>(rb.bt_body)
        ->setWorldTransform(bt_t);
  }
}

void physics_world::sync_bullet_to_transforms(entt::registry &registry) {
  auto view = registry.view<rigidbody, collider, transform>();
  for (auto e : view) {
    auto &rb = view.get<rigidbody>(e);
    if (rb.body_type != rigidbody_type::DYNAMIC || !rb.bt_body)
      continue;

    btTransform bt_t;
    static_cast<btDefaultMotionState *>(rb.bt_motion_state)
        ->getWorldTransform(bt_t);

    auto &trans = view.get<transform>(e);
    trans.set_world_pos(from_bt(bt_t.getOrigin()));
    trans.set_world_rot(from_bt(bt_t.getRotation()));
  }
}

void physics_world::read_back_velocities(entt::registry &registry) {
  auto view = registry.view<rigidbody>();
  for (auto e : view) {
    auto &rb = view.get<rigidbody>(e);
    if (!rb.bt_body)
      continue;
    auto *body = static_cast<btRigidBody *>(rb.bt_body);
    rb.velocity = from_bt(body->getLinearVelocity());
    rb.angular_velocity = from_bt(body->getAngularVelocity());
  }
}

// ─── Collision collection ──────────────────────────────────────────────────

void physics_world::collect_collisions() {
  collisions.clear();
  int num_manifolds = dispatcher->getNumManifolds();
  for (int i = 0; i < num_manifolds; i++) {
    btPersistentManifold *manifold = dispatcher->getManifoldByIndexInternal(i);
    int num_contacts = manifold->getNumContacts();
    if (num_contacts == 0)
      continue;

    const btCollisionObject *obj_a = manifold->getBody0();
    const btCollisionObject *obj_b = manifold->getBody1();
    entt::entity ea =
        static_cast<entt::entity>(static_cast<uint32_t>(obj_a->getUserIndex()));
    entt::entity eb =
        static_cast<entt::entity>(static_cast<uint32_t>(obj_b->getUserIndex()));

    for (int j = 0; j < num_contacts; j++) {
      btManifoldPoint &pt = manifold->getContactPoint(j);
      if (pt.getDistance() < 0.0f) {
        collision_info ci;
        ci.entity_a = ea;
        ci.entity_b = eb;
        ci.point_on_a = from_bt(pt.getPositionWorldOnA());
        ci.point_on_b = from_bt(pt.getPositionWorldOnB());
        ci.normal = from_bt(pt.m_normalWorldOnB);
        ci.impulse = pt.getAppliedImpulse();
        collisions.push_back(ci);
      }
    }
  }
}

// ─── Axis locks ────────────────────────────────────────────────────────────

void physics_world::apply_axis_locks(
    btRigidBody *body, const rigidbody_constraints_flags &flags) {
  btVector3 lin_factor(flags.freeze_pos_x ? 0.f : 1.f,
                       flags.freeze_pos_y ? 0.f : 1.f,
                       flags.freeze_pos_z ? 0.f : 1.f);
  btVector3 ang_factor(flags.freeze_rot_x ? 0.f : 1.f,
                       flags.freeze_rot_y ? 0.f : 1.f,
                       flags.freeze_rot_z ? 0.f : 1.f);
  body->setLinearFactor(lin_factor);
  body->setAngularFactor(ang_factor);
}

// ─── Main step ─────────────────────────────────────────────────────────────

void physics_world::step(entt::registry &registry, float dt) {
  if (!dynamics_world)
    return;

  // Update gravity in case it changed
  dynamics_world->setGravity(to_bt(gravity));

  // Clean up stale registrations (deleted entities / removed colliders)
  {
    std::vector<entt::entity> stale;
    for (auto e : registered_bodies) {
      if (!registry.valid(e) || !registry.try_get<collider>(e))
        stale.push_back(e);
    }
    for (auto e : stale) {
      for (int i = dynamics_world->getNumCollisionObjects() - 1; i >= 0; i--) {
        btCollisionObject *obj = dynamics_world->getCollisionObjectArray()[i];
        if (obj->getUserIndex() == static_cast<int>(entt::to_integral(e))) {
          btRigidBody *body = btRigidBody::upcast(obj);
          if (body && body->getMotionState())
            delete body->getMotionState();
          auto *shape = obj->getCollisionShape();
          dynamics_world->removeCollisionObject(obj);
          delete shape;
          delete obj;
          break;
        }
      }
      registered_bodies.erase(e);
    }
  }

  // Rebuild dirty bodies (GUI modifications or newly added components)
  {
    auto col_view = registry.view<collider, transform>();
    for (auto e : col_view) {
      auto &col = col_view.get<collider>(e);
      auto *rb = registry.try_get<rigidbody>(e);
      bool needs_rebuild = col.dirty || (rb && rb->dirty);
      if (needs_rebuild && registered_bodies.count(e)) {
        // Use remove_body only when the rigidbody owns the Bullet body;
        // otherwise fall back to remove_static_body which searches by index.
        if (rb && rb->bt_body)
          remove_body(e, *rb, col);
        else
          remove_static_body(e, col);
        col.dirty = false;
        if (rb)
          rb->dirty = false;
      }
    }
  }

  // Register any new bodies/constraints
  {
    auto col_view = registry.view<collider, transform>();
    for (auto e : col_view) {
      ensure_body_registered(registry, e);
      // Clear dirty after registration so it doesn't re-trigger
      auto &col = col_view.get<collider>(e);
      col.dirty = false;
      if (auto *rb = registry.try_get<rigidbody>(e))
        rb->dirty = false;
    }
  }
  {
    auto con_view = registry.view<physics_constraint>();
    for (auto e : con_view)
      ensure_constraint_registered(registry, e);
  }

  if (!simulation_enabled)
    return;

  // Push animated transforms into Bullet for kinematic bodies
  sync_kinematic_to_bullet(registry);

  // Step simulation
  dynamics_world->stepSimulation(dt, max_substeps, fixed_timestep);

  // Pull results back into engine transforms
  sync_bullet_to_transforms(registry);

  // Readback velocities
  read_back_velocities(registry);

  // Gather collisions
  collect_collisions();
}

// ─── Debug draw ────────────────────────────────────────────────────────────

void physics_world::debug_draw_world(entt::registry &registry,
                                     math::matrix4 vp) {
  if (!dynamics_world || !draw_debug)
    return;

  debug_drawer.vp = vp;
  dynamics_world->debugDrawWorld();
}

// ─── Force / impulse helpers ───────────────────────────────────────────────

void physics_world::add_force(rigidbody &rb, math::vector3 force) {
  if (!rb.bt_body)
    return;
  auto *body = static_cast<btRigidBody *>(rb.bt_body);
  body->activate(true);
  body->applyCentralForce(to_bt(force));
}

void physics_world::add_impulse(rigidbody &rb, math::vector3 impulse) {
  if (!rb.bt_body)
    return;
  auto *body = static_cast<btRigidBody *>(rb.bt_body);
  body->activate(true);
  body->applyCentralImpulse(to_bt(impulse));
}

void physics_world::add_torque(rigidbody &rb, math::vector3 torque) {
  if (!rb.bt_body)
    return;
  auto *body = static_cast<btRigidBody *>(rb.bt_body);
  body->activate(true);
  body->applyTorque(to_bt(torque));
}

void physics_world::add_force_at_position(rigidbody &rb, math::vector3 force,
                                          math::vector3 world_pos) {
  if (!rb.bt_body)
    return;
  auto *body = static_cast<btRigidBody *>(rb.bt_body);
  body->activate(true);
  btVector3 rel = to_bt(world_pos) - body->getCenterOfMassPosition();
  body->applyForce(to_bt(force), rel);
}

// ─── Raycast ───────────────────────────────────────────────────────────────

physics_world::raycast_hit physics_world::raycast(math::vector3 origin,
                                                  math::vector3 direction,
                                                  float max_distance) {
  raycast_hit result;
  if (!dynamics_world)
    return result;

  btVector3 from = to_bt(origin);
  btVector3 to = to_bt(origin + direction.normalized() * max_distance);

  btCollisionWorld::ClosestRayResultCallback callback(from, to);
  dynamics_world->rayTest(from, to, callback);

  if (callback.hasHit()) {
    result.hit = true;
    result.point = from_bt(callback.m_hitPointWorld);
    result.normal = from_bt(callback.m_hitNormalWorld);
    result.distance = (result.point - origin).norm();
    result.entity = static_cast<entt::entity>(
        static_cast<uint32_t>(callback.m_collisionObject->getUserIndex()));
  }
  return result;
}

// ─── Body / constraint removal ─────────────────────────────────────────────

void physics_world::remove_body(entt::entity e, rigidbody &rb, collider &col) {
  if (rb.bt_body) {
    auto *body = static_cast<btRigidBody *>(rb.bt_body);
    if (body->getMotionState())
      delete body->getMotionState();
    dynamics_world->removeRigidBody(body);
    // If a compound was created for center offset, shape is the compound
    auto *body_shape = body->getCollisionShape();
    if (body_shape != col.bt_shape) {
      // body_shape is a compound wrapper; delete it, the child (bt_shape)
      // will be deleted below
      delete body_shape;
    }
    delete body;
    rb.bt_body = nullptr;
    rb.bt_motion_state = nullptr;
  }
  if (col.bt_shape) {
    delete static_cast<btCollisionShape *>(col.bt_shape);
    col.bt_shape = nullptr;
  }
  registered_bodies.erase(e);
}

void physics_world::remove_static_body(entt::entity e, collider &col) {
  // For entities that have a collider but no rigidbody (static colliders)
  // We need to find and remove the collision object from the world
  for (int i = dynamics_world->getNumCollisionObjects() - 1; i >= 0; i--) {
    btCollisionObject *obj = dynamics_world->getCollisionObjectArray()[i];
    if (obj->getUserIndex() == static_cast<int>(entt::to_integral(e))) {
      btRigidBody *body = btRigidBody::upcast(obj);
      if (body && body->getMotionState())
        delete body->getMotionState();
      auto *body_shape = obj->getCollisionShape();
      dynamics_world->removeCollisionObject(obj);
      // Delete compound wrapper if used
      if (body_shape != col.bt_shape)
        delete body_shape;
      delete obj;
      break;
    }
  }
  if (col.bt_shape) {
    delete static_cast<btCollisionShape *>(col.bt_shape);
    col.bt_shape = nullptr;
  }
  registered_bodies.erase(e);
}

void physics_world::remove_constraint(entt::entity e,
                                      physics_constraint &pc) {
  if (pc.bt_constraint) {
    dynamics_world->removeConstraint(
        static_cast<btTypedConstraint *>(pc.bt_constraint));
    delete static_cast<btTypedConstraint *>(pc.bt_constraint);
    pc.bt_constraint = nullptr;
  }
  registered_constraints.erase(e);
}

// ─── GUI ───────────────────────────────────────────────────────────────────

void physics_world::draw_menu_gui() {
  ImGui::Checkbox("Simulation Enabled", &simulation_enabled);
  ImGui::Checkbox("Debug Draw", &draw_debug);
  ImGui::DragFloat3("Gravity", gravity.data(), 0.01f);
  ImGui::DragFloat("Fixed Timestep", &fixed_timestep, 0.001f, 0.001f, 0.1f);
  ImGui::DragInt("Max Substeps", &max_substeps, 1, 1, 16);
  if (dynamics_world) {
    ImGui::Text("Rigid Bodies: %d",
                dynamics_world->getNumCollisionObjects());
    ImGui::Text("Constraints: %d", dynamics_world->getNumConstraints());
    ImGui::Text("Collisions this frame: %zu", collisions.size());
  }
}

}; // namespace toolkit::opengl3d
