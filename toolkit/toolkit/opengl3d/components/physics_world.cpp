#include "toolkit/opengl3d/components/physics_world.hpp"

namespace toolkit::opengl3d {

// ─── Helpers (file-local) ──────────────────────────────────────────────────

static entt::entity entity_from_bt(const btCollisionObject *obj) {
  return static_cast<entt::entity>(static_cast<uint32_t>(obj->getUserIndex()));
}

static void set_bt_entity(btCollisionObject *obj, entt::entity e) {
  obj->setUserIndex(static_cast<int>(entt::to_integral(e)));
}

// ─── Lifecycle ─────────────────────────────────────────────────────────────

void physics_world::init0(entt::registry &registry) {
  // Use larger manifold/contact-pool for better precision under load.
  btDefaultCollisionConstructionInfo cci;
  cci.m_defaultMaxPersistentManifoldPoolSize = 4096;
  cci.m_defaultMaxCollisionAlgorithmPoolSize = 4096;
  collision_config = new btDefaultCollisionConfiguration(cci);

  dispatcher = new btCollisionDispatcher(collision_config);
  broadphase = new btDbvtBroadphase();
  broadphase->getOverlappingPairCache()->setInternalGhostPairCallback(
      new btGhostPairCallback());
  solver = new btSequentialImpulseConstraintSolver();
  dynamics_world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver,
                                              collision_config);
  dynamics_world->setGravity(to_bt(gravity));
  dynamics_world->setDebugDrawer(&debug_drawer);

  // Solver precision
  auto &si = dynamics_world->getSolverInfo();
  si.m_numIterations = solver_iterations;
  si.m_erp = erp;
  si.m_erp2 = erp2;
  si.m_splitImpulse = true;       // reduces jitter on deep penetrations
  si.m_splitImpulsePenetrationThreshold = -0.04f;
  si.m_solverMode |= SOLVER_SIMD | SOLVER_USE_WARMSTARTING;
}

void physics_world::init1(entt::registry &registry) {
  if (dynamics_world) {
    shutdown(registry);
    init0(registry);
  }
}

void physics_world::shutdown(entt::registry &registry) {
  // Controllers
  auto cc_view = registry.view<character_controller>();
  for (auto e : cc_view)
    unregister_controller(cc_view.get<character_controller>(e));
  registered_controllers.clear();

  // Constraints
  for (int i = dynamics_world->getNumConstraints() - 1; i >= 0; i--) {
    auto *c = dynamics_world->getConstraint(i);
    dynamics_world->removeConstraint(c);
    delete c;
  }
  registered_constraints.clear();

  // Bodies
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
  prev_pairs.clear();

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

btCollisionShape *physics_world::create_shape(entt::registry &registry,
                                              entt::entity e,
                                              physics_body &pb) {
  switch (pb.shape) {
  case body_shape::BOX:
    return new btBoxShape(to_bt(pb.size));

  case body_shape::SPHERE:
    return new btSphereShape(pb.size.x());

  case body_shape::CAPSULE:
    return new btCapsuleShape(
        pb.size.x(), std::max(0.f, pb.size.y() - 2.f * pb.size.x()));

  case body_shape::CYLINDER:
    return new btCylinderShape(
        btVector3(pb.size.x(), pb.size.y(), pb.size.x()));

  case body_shape::MESH: {
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
      return new btBvhTriangleMeshShape(mesh, true);
    }
    if (md) {
      math::vector3 half = (md->bb_max - md->bb_min) * 0.5f;
      return new btBoxShape(to_bt(half));
    }
    return new btBoxShape(to_bt(pb.size));
  }

  case body_shape::CONVEX: {
    auto *md = registry.try_get<mesh_data>(e);
    if (md && !md->vertices.empty()) {
      auto *hull = new btConvexHullShape();
      for (auto &v : md->vertices)
        hull->addPoint(
            btVector3(v.position.x(), v.position.y(), v.position.z()), false);
      hull->recalcLocalAabb();
      hull->optimizeConvexHull();
      return hull;
    }
    return new btBoxShape(to_bt(pb.size));
  }

  default:
    return new btBoxShape(to_bt(pb.size));
  }
}

// ─── Body registration ─────────────────────────────────────────────────────

void physics_world::register_body(entt::registry &registry, entt::entity e) {
  if (registered_bodies.count(e))
    return;
  auto *pb = registry.try_get<physics_body>(e);
  auto *trans = registry.try_get<transform>(e);
  if (!pb || !trans)
    return;

  btCollisionShape *shape = create_shape(registry, e, *pb);
  pb->bt_shape = shape;

  // Wrap in compound if center offset is non-zero
  btCollisionShape *effective = shape;
  if (pb->center.squaredNorm() > 1e-8f) {
    auto *compound = new btCompoundShape();
    btTransform local;
    local.setIdentity();
    local.setOrigin(to_bt(pb->center));
    compound->addChildShape(local, shape);
    effective = compound;
  }

  float mass = (pb->type == body_type::DYNAMIC) ? pb->mass : 0.f;

  btVector3 inertia(0, 0, 0);
  if (mass > 0)
    effective->calculateLocalInertia(mass, inertia);

  btTransform bt_t = to_bt_transform(trans->world_pos(), trans->world_rot());
  auto *ms = new btDefaultMotionState(bt_t);
  btRigidBody::btRigidBodyConstructionInfo ci(mass, ms, effective, inertia);
  ci.m_friction = pb->friction;
  ci.m_restitution = pb->restitution;
  auto *body = new btRigidBody(ci);
  set_bt_entity(body, e);

  pb->bt_body = body;
  pb->bt_motion_state = ms;

  if (pb->type == body_type::KINEMATIC) {
    body->setCollisionFlags(body->getCollisionFlags() |
                            btCollisionObject::CF_KINEMATIC_OBJECT);
    body->setActivationState(DISABLE_DEACTIVATION);
  }
  if (pb->type == body_type::STATIC)
    body->setCollisionFlags(body->getCollisionFlags() |
                            btCollisionObject::CF_STATIC_OBJECT);

  if (pb->type == body_type::DYNAMIC) {
    body->setDamping(pb->drag, pb->angular_drag);
    if (!pb->use_gravity)
      body->setGravity(btVector3(0, 0, 0));

    // CCD prevents fast-moving bodies tunnelling through thin geometry
    if (ccd_threshold > 0.f) {
      body->setCcdMotionThreshold(ccd_threshold);
      body->setCcdSweptSphereRadius(0.05f);
    }

    btVector3 lf(pb->constraints.freeze_pos_x ? 0.f : 1.f,
                 pb->constraints.freeze_pos_y ? 0.f : 1.f,
                 pb->constraints.freeze_pos_z ? 0.f : 1.f);
    btVector3 af(pb->constraints.freeze_rot_x ? 0.f : 1.f,
                 pb->constraints.freeze_rot_y ? 0.f : 1.f,
                 pb->constraints.freeze_rot_z ? 0.f : 1.f);
    body->setLinearFactor(lf);
    body->setAngularFactor(af);
  }

  if (pb->is_trigger)
    body->setCollisionFlags(body->getCollisionFlags() |
                            btCollisionObject::CF_NO_CONTACT_RESPONSE);

  dynamics_world->addRigidBody(body);
  registered_bodies.insert(e);
  pb->dirty = false;
}

void physics_world::unregister_body(entt::registry &registry, entt::entity e) {
  if (!registered_bodies.count(e))
    return;

  uint32_t idx = static_cast<uint32_t>(entt::to_integral(e));
  for (int i = dynamics_world->getNumCollisionObjects() - 1; i >= 0; i--) {
    btCollisionObject *obj = dynamics_world->getCollisionObjectArray()[i];
    if (static_cast<uint32_t>(obj->getUserIndex()) != idx)
      continue;

    btRigidBody *body = btRigidBody::upcast(obj);
    if (body && body->getMotionState())
      delete body->getMotionState();

    dynamics_world->removeCollisionObject(obj);

    // Handle compound wrapper
    btCollisionShape *obj_shape = obj->getCollisionShape();
    auto *pb = registry.try_get<physics_body>(e);
    if (pb && pb->bt_shape && obj_shape != pb->bt_shape) {
      delete obj_shape; // compound wrapper
      delete static_cast<btCollisionShape *>(pb->bt_shape);
    } else {
      delete obj_shape;
    }
    delete obj;

    if (pb) {
      pb->bt_shape = nullptr;
      pb->bt_body = nullptr;
      pb->bt_motion_state = nullptr;
    }
    break;
  }
  registered_bodies.erase(e);
}

// ─── Constraint registration ───────────────────────────────────────────────

void physics_world::register_constraint(entt::registry &registry,
                                        entt::entity e) {
  if (registered_constraints.count(e))
    return;
  auto *pc = registry.try_get<physics_constraint>(e);
  if (!pc)
    return;
  auto *pb = registry.try_get<physics_body>(e);
  if (!pb || !pb->bt_body)
    return;

  btRigidBody *body_a = static_cast<btRigidBody *>(pb->bt_body);
  btRigidBody *body_b = nullptr;
  if (pc->connected_entity != entt::null &&
      registry.valid(pc->connected_entity)) {
    auto *pb_b = registry.try_get<physics_body>(pc->connected_entity);
    if (pb_b && pb_b->bt_body)
      body_b = static_cast<btRigidBody *>(pb_b->bt_body);
  }

  btTypedConstraint *constraint = nullptr;

  switch (pc->type) {
  case constraint_type::FIXED: {
    btTransform fa, fb;
    fa.setIdentity();
    fa.setOrigin(to_bt(pc->anchor));
    fb.setIdentity();
    fb.setOrigin(to_bt(pc->connected_anchor));
    if (body_b) {
      constraint = new btFixedConstraint(*body_a, *body_b, fa, fb);
    } else {
      static btRigidBody s_fixed(0.0f, nullptr, nullptr);
      s_fixed.setWorldTransform(fb);
      constraint = new btFixedConstraint(*body_a, s_fixed, fa, fb);
    }
    break;
  }
  case constraint_type::HINGE: {
    if (body_b)
      constraint = new btHingeConstraint(
          *body_a, *body_b, to_bt(pc->anchor), to_bt(pc->connected_anchor),
          to_bt(pc->axis), to_bt(pc->axis));
    else
      constraint =
          new btHingeConstraint(*body_a, to_bt(pc->anchor), to_bt(pc->axis));
    auto *h = static_cast<btHingeConstraint *>(constraint);
    if (pc->hinge_use_limits)
      h->setLimit(pc->hinge_lower_limit, pc->hinge_upper_limit);
    if (pc->hinge_use_motor)
      h->enableAngularMotor(true, pc->hinge_motor_target_velocity,
                            pc->hinge_motor_max_impulse);
    break;
  }
  case constraint_type::BALL_SOCKET: {
    if (body_b)
      constraint = new btPoint2PointConstraint(
          *body_a, *body_b, to_bt(pc->anchor), to_bt(pc->connected_anchor));
    else
      constraint = new btPoint2PointConstraint(*body_a, to_bt(pc->anchor));
    break;
  }
  case constraint_type::CONE_TWIST: {
    math::quat rot = math::from_to_rot(math::vector3(0, 0, 1), pc->axis);
    btTransform fa, fb;
    fa.setIdentity();
    fa.setOrigin(to_bt(pc->anchor));
    fa.setRotation(to_bt(rot));
    fb.setIdentity();
    fb.setOrigin(to_bt(pc->connected_anchor));
    fb.setRotation(to_bt(rot));
    if (body_b)
      constraint = new btConeTwistConstraint(*body_a, *body_b, fa, fb);
    else
      constraint = new btConeTwistConstraint(*body_a, fa);
    static_cast<btConeTwistConstraint *>(constraint)
        ->setLimit(pc->cone_swing_span1, pc->cone_swing_span2,
                   pc->cone_twist_span, pc->cone_softness, pc->cone_bias,
                   pc->cone_relaxation);
    break;
  }
  case constraint_type::SLIDER: {
    btTransform fa, fb;
    fa.setIdentity();
    fa.setOrigin(to_bt(pc->anchor));
    fb.setIdentity();
    fb.setOrigin(to_bt(pc->connected_anchor));
    if (body_b)
      constraint = new btSliderConstraint(*body_a, *body_b, fa, fb, true);
    else
      constraint = new btSliderConstraint(*body_a, fa, true);
    auto *sl = static_cast<btSliderConstraint *>(constraint);
    sl->setLowerLinLimit(pc->slider_lower_lin);
    sl->setUpperLinLimit(pc->slider_upper_lin);
    sl->setLowerAngLimit(pc->slider_lower_ang);
    sl->setUpperAngLimit(pc->slider_upper_ang);
    break;
  }
  case constraint_type::GENERIC_6DOF: {
    btTransform fa, fb;
    fa.setIdentity();
    fa.setOrigin(to_bt(pc->anchor));
    fb.setIdentity();
    fb.setOrigin(to_bt(pc->connected_anchor));
    btGeneric6DofSpring2Constraint *dof6 =
        body_b ? new btGeneric6DofSpring2Constraint(*body_a, *body_b, fa, fb)
               : new btGeneric6DofSpring2Constraint(*body_a, fa);
    dof6->setLinearLowerLimit(btVector3(pc->dof6_axes[0].lower_limit,
                                        pc->dof6_axes[1].lower_limit,
                                        pc->dof6_axes[2].lower_limit));
    dof6->setLinearUpperLimit(btVector3(pc->dof6_axes[0].upper_limit,
                                        pc->dof6_axes[1].upper_limit,
                                        pc->dof6_axes[2].upper_limit));
    dof6->setAngularLowerLimit(btVector3(pc->dof6_axes[3].lower_limit,
                                         pc->dof6_axes[4].lower_limit,
                                         pc->dof6_axes[5].lower_limit));
    dof6->setAngularUpperLimit(btVector3(pc->dof6_axes[3].upper_limit,
                                         pc->dof6_axes[4].upper_limit,
                                         pc->dof6_axes[5].upper_limit));
    for (int i = 0; i < 6; i++) {
      auto &ax = pc->dof6_axes[i];
      if (ax.use_spring) {
        dof6->enableSpring(i, true);
        dof6->setStiffness(i, ax.stiffness);
        dof6->setDamping(i, ax.damping);
      }
      if (ax.use_motor) {
        dof6->enableMotor(i, true);
        dof6->setTargetVelocity(i, ax.motor_target_velocity);
        dof6->setMaxMotorForce(i, ax.motor_max_force);
      }
    }
    constraint = dof6;
    break;
  }
  } // switch

  if (!constraint)
    return;
  if (pc->break_force > 0)
    constraint->setBreakingImpulseThreshold(pc->break_force);
  dynamics_world->addConstraint(constraint,
                                !pc->enable_collision_between_bodies);
  pc->bt_constraint = constraint;
  registered_constraints.insert(e);
}

void physics_world::unregister_constraint(entt::entity e,
                                          physics_constraint &pc) {
  if (pc.bt_constraint) {
    dynamics_world->removeConstraint(
        static_cast<btTypedConstraint *>(pc.bt_constraint));
    delete static_cast<btTypedConstraint *>(pc.bt_constraint);
    pc.bt_constraint = nullptr;
  }
  registered_constraints.erase(e);
}

// ─── Character controller registration ─────────────────────────────────────

void physics_world::register_controller(entt::registry &registry,
                                        entt::entity e) {
  if (registered_controllers.count(e))
    return;
  auto *cc = registry.try_get<character_controller>(e);
  auto *trans = registry.try_get<transform>(e);
  if (!cc || !trans)
    return;

  float cyl_h = std::max(0.f, cc->height - 2.0f * cc->radius);
  auto *capsule = new btCapsuleShape(cc->radius, cyl_h);
  cc->bt_shape = capsule;

  auto *ghost = new btPairCachingGhostObject();
  ghost->setCollisionShape(capsule);
  ghost->setCollisionFlags(btCollisionObject::CF_CHARACTER_OBJECT);
  ghost->setWorldTransform(
      to_bt_transform(trans->world_pos(), trans->world_rot()));
  set_bt_entity(ghost, e);
  cc->bt_ghost_object = ghost;

  auto *ctrl =
      new btKinematicCharacterController(ghost, capsule, cc->step_height);
  ctrl->setMaxSlope(btRadians(cc->max_slope));
  ctrl->setGravity(btVector3(0, 0, 0));
  ctrl->setMaxPenetrationDepth(0.03f);    // tighter penetration recovery
  ctrl->setUseGhostSweepTest(true);       // sweep-based test for smoother movement
  cc->bt_controller = ctrl;

  dynamics_world->addCollisionObject(
      ghost, btBroadphaseProxy::CharacterFilter,
      btBroadphaseProxy::StaticFilter | btBroadphaseProxy::DefaultFilter |
          btBroadphaseProxy::KinematicFilter);
  dynamics_world->addAction(ctrl);
  registered_controllers.insert(e);
  cc->dirty = false;
}

void physics_world::unregister_controller(character_controller &cc) {
  if (cc.bt_controller) {
    dynamics_world->removeAction(
        static_cast<btKinematicCharacterController *>(cc.bt_controller));
    delete static_cast<btKinematicCharacterController *>(cc.bt_controller);
    cc.bt_controller = nullptr;
  }
  if (cc.bt_ghost_object) {
    dynamics_world->removeCollisionObject(
        static_cast<btPairCachingGhostObject *>(cc.bt_ghost_object));
    delete static_cast<btPairCachingGhostObject *>(cc.bt_ghost_object);
    cc.bt_ghost_object = nullptr;
  }
  if (cc.bt_shape) {
    delete static_cast<btCapsuleShape *>(cc.bt_shape);
    cc.bt_shape = nullptr;
  }
}

// ─── Character controller API ──────────────────────────────────────────────

void physics_world::cc_move(character_controller &cc,
                            math::vector3 displacement) {
  if (!cc.bt_controller)
    return;
  static_cast<btKinematicCharacterController *>(cc.bt_controller)
      ->setWalkDirection(to_bt(displacement));
}

void physics_world::cc_set_position(character_controller &cc,
                                    math::vector3 world_pos) {
  if (!cc.bt_ghost_object)
    return;
  auto *ghost = static_cast<btPairCachingGhostObject *>(cc.bt_ghost_object);
  btTransform t = ghost->getWorldTransform();
  t.setOrigin(to_bt(world_pos));
  ghost->setWorldTransform(t);
  if (cc.bt_controller)
    static_cast<btKinematicCharacterController *>(cc.bt_controller)
        ->warp(to_bt(world_pos));
}

void physics_world::cc_set_velocity(character_controller &cc,
                                    math::vector3 vel) {
  if (!cc.bt_controller)
    return;
  static_cast<btKinematicCharacterController *>(cc.bt_controller)
      ->setLinearVelocity(to_bt(vel));
  cc.velocity = vel;
}

// ─── Main step ─────────────────────────────────────────────────────────────

void physics_world::step(entt::registry &registry, float dt) {
  if (!dynamics_world)
    return;
  dynamics_world->setGravity(to_bt(gravity));

  // ── Sync registrations: physics bodies ───────────────────────────────
  {
    std::set<entt::entity> live;
    registry.view<physics_body, transform>().each(
        [&](entt::entity e, physics_body &pb, transform &) {
          live.insert(e);
          if (pb.dirty && registered_bodies.count(e))
            unregister_body(registry, e);
          register_body(registry, e);
        });
    std::vector<entt::entity> stale;
    for (auto e : registered_bodies)
      if (!live.count(e))
        stale.push_back(e);
    for (auto e : stale)
      unregister_body(registry, e);
  }

  // ── Sync registrations: constraints ──────────────────────────────────
  {
    auto con_view = registry.view<physics_constraint>();
    for (auto e : con_view)
      register_constraint(registry, e);
  }

  // ── Sync registrations: character controllers ────────────────────────
  {
    std::set<entt::entity> live;
    registry.view<character_controller, transform>().each(
        [&](entt::entity e, character_controller &cc, transform &) {
          live.insert(e);
          if (cc.dirty && registered_controllers.count(e)) {
            unregister_controller(cc);
            registered_controllers.erase(e);
          }
          register_controller(registry, e);
        });
    std::vector<entt::entity> stale;
    for (auto e : registered_controllers)
      if (!live.count(e))
        stale.push_back(e);
    for (auto e : stale) {
      if (auto *cc = registry.try_get<character_controller>(e))
        unregister_controller(*cc);
      registered_controllers.erase(e);
    }
  }

  if (!simulation_enabled)
    return;

  // ── Push kinematic transforms to Bullet ──────────────────────────────
  registry.view<physics_body, transform>().each(
      [&](entt::entity e, physics_body &pb, transform &t) {
        if (pb.type != body_type::KINEMATIC || !pb.bt_body)
          return;
        btTransform bt_t = to_bt_transform(t.world_pos(), t.world_rot());
        static_cast<btDefaultMotionState *>(pb.bt_motion_state)
            ->setWorldTransform(bt_t);
        static_cast<btRigidBody *>(pb.bt_body)->setWorldTransform(bt_t);
      });

  // ── Step simulation ──────────────────────────────────────────────────
  dynamics_world->getSolverInfo().m_numIterations = solver_iterations;
  dynamics_world->getSolverInfo().m_erp = erp;
  dynamics_world->getSolverInfo().m_erp2 = erp2;
  dynamics_world->stepSimulation(dt, max_substeps, fixed_timestep);

  // ── Pull dynamic transforms from Bullet ──────────────────────────────
  registry.view<physics_body, transform>().each(
      [&](entt::entity e, physics_body &pb, transform &t) {
        if (pb.type != body_type::DYNAMIC || !pb.bt_body)
          return;
        btTransform bt_t;
        static_cast<btDefaultMotionState *>(pb.bt_motion_state)
            ->getWorldTransform(bt_t);
        t.set_world_pos(from_bt(bt_t.getOrigin()));
        t.set_world_rot(from_bt(bt_t.getRotation()));
      });

  // ── Sync controller transforms + grounded state ──────────────────────
  registry.view<character_controller, transform>().each(
      [&](entt::entity e, character_controller &cc, transform &t) {
        if (!cc.bt_ghost_object)
          return;
        auto *ghost =
            static_cast<btPairCachingGhostObject *>(cc.bt_ghost_object);
        auto *ctrl =
            static_cast<btKinematicCharacterController *>(cc.bt_controller);
        t.set_world_pos(from_bt(ghost->getWorldTransform().getOrigin()));
        cc.grounded = ctrl->onGround();
      });

  // ── Readback velocities ──────────────────────────────────────────────
  {
    auto view = registry.view<physics_body>();
    for (auto e : view) {
      auto &pb = view.get<physics_body>(e);
      if (!pb.bt_body)
        continue;
      auto *body = static_cast<btRigidBody *>(pb.bt_body);
      pb.velocity = from_bt(body->getLinearVelocity());
      pb.angular_velocity = from_bt(body->getAngularVelocity());
    }
  }

  // ── Collision event tracking ─────────────────────────────────────────
  update_collision_events(registry);
}

// ─── Collision event tracking (enter/stay/exit) ────────────────────────────

void physics_world::update_collision_events(entt::registry &registry) {
  std::map<std::pair<uint32_t, uint32_t>, contact_cache> cur_pairs;

  auto record_pair = [&](entt::entity ea, entt::entity eb, math::vector3 pt,
                         math::vector3 nm, float impulse, bool trigger) {
    uint32_t a = static_cast<uint32_t>(entt::to_integral(ea));
    uint32_t b = static_cast<uint32_t>(entt::to_integral(eb));
    if (a > b) {
      std::swap(a, b);
      nm = -nm;
    }
    auto key = std::make_pair(a, b);
    auto it = cur_pairs.find(key);
    if (it == cur_pairs.end() || impulse > it->second.impulse)
      cur_pairs[key] = {pt, nm, impulse, trigger};
  };

  // Scan dispatcher manifolds (body-to-body contacts). Skip ghosts.
  int num_manifolds = dispatcher->getNumManifolds();
  for (int i = 0; i < num_manifolds; i++) {
    btPersistentManifold *m = dispatcher->getManifoldByIndexInternal(i);
    if (m->getNumContacts() == 0)
      continue;
    const btCollisionObject *oa = m->getBody0();
    const btCollisionObject *ob = m->getBody1();
    if (oa->getInternalType() == btCollisionObject::CO_GHOST_OBJECT ||
        ob->getInternalType() == btCollisionObject::CO_GHOST_OBJECT)
      continue;

    entt::entity ea = entity_from_bt(oa);
    entt::entity eb = entity_from_bt(ob);
    bool trigger = false;
    if (auto *pa = registry.try_get<physics_body>(ea))
      trigger |= pa->is_trigger;
    if (auto *pb_o = registry.try_get<physics_body>(eb))
      trigger |= pb_o->is_trigger;

    for (int j = 0; j < m->getNumContacts(); j++) {
      btManifoldPoint &pt = m->getContactPoint(j);
      if (pt.getDistance() < 0.0f)
        record_pair(ea, eb, from_bt(pt.getPositionWorldOnA()),
                    from_bt(pt.m_normalWorldOnB), pt.getAppliedImpulse(),
                    trigger);
    }
  }

  // Scan ghost object contacts (character controllers).
  {
    auto cc_view = registry.view<character_controller>();
    for (auto e : cc_view) {
      auto &cc = cc_view.get<character_controller>(e);
      if (!cc.bt_ghost_object)
        continue;
      auto *ghost =
          static_cast<btPairCachingGhostObject *>(cc.bt_ghost_object);
      for (int i = 0; i < ghost->getNumOverlappingObjects(); i++) {
        btCollisionObject *obj = ghost->getOverlappingObject(i);
        entt::entity other = entity_from_bt(obj);
        if (other == e)
          continue;

        btBroadphasePair *bp = dynamics_world->getPairCache()->findPair(
            ghost->getBroadphaseHandle(), obj->getBroadphaseHandle());
        if (!bp || !bp->m_algorithm)
          continue;

        btManifoldArray manifolds;
        bp->m_algorithm->getAllContactManifolds(manifolds);
        for (int mi = 0; mi < manifolds.size(); mi++) {
          btPersistentManifold *mf = manifolds[mi];
          for (int j = 0; j < mf->getNumContacts(); j++) {
            btManifoldPoint &pt = mf->getContactPoint(j);
            if (pt.getDistance() < 0.0f)
              record_pair(e, other, from_bt(pt.getPositionWorldOnA()),
                          from_bt(pt.m_normalWorldOnB),
                          pt.getAppliedImpulse(), false);
          }
        }
      }
    }
  }

  // Clear events on all physics entities
  {
    auto pb_view = registry.view<physics_body>();
    for (auto e : pb_view)
      pb_view.get<physics_body>(e).events.clear();
    auto cc_view = registry.view<character_controller>();
    for (auto e : cc_view)
      cc_view.get<character_controller>(e).events.clear();
  }

  // Helper to push event to the right component
  auto push = [&](entt::entity target, entt::entity other,
                  collision_event_type type, math::vector3 pt,
                  math::vector3 nm, float impulse) {
    collision_event ev{other, type, pt, nm, impulse};
    if (auto *pb = registry.try_get<physics_body>(target)) {
      pb->events.push_back(ev);
      return;
    }
    if (auto *cc = registry.try_get<character_controller>(target))
      cc->events.push_back(ev);
  };

  // ENTER / STAY
  for (auto &[key, data] : cur_pairs) {
    entt::entity ea = static_cast<entt::entity>(key.first);
    entt::entity eb = static_cast<entt::entity>(key.second);
    bool was_active = prev_pairs.count(key) > 0;
    collision_event_type t;
    if (data.is_trigger)
      t = was_active ? collision_event_type::TRIGGER_STAY
                     : collision_event_type::TRIGGER_ENTER;
    else
      t = was_active ? collision_event_type::COLLISION_STAY
                     : collision_event_type::COLLISION_ENTER;
    push(ea, eb, t, data.point, data.normal, data.impulse);
    push(eb, ea, t, data.point, -data.normal, data.impulse);
  }

  // EXIT
  for (auto &[key, data] : prev_pairs) {
    if (cur_pairs.count(key))
      continue;
    entt::entity ea = static_cast<entt::entity>(key.first);
    entt::entity eb = static_cast<entt::entity>(key.second);
    collision_event_type t = data.is_trigger
                                 ? collision_event_type::TRIGGER_EXIT
                                 : collision_event_type::COLLISION_EXIT;
    push(ea, eb, t, data.point, data.normal, 0.0f);
    push(eb, ea, t, data.point, -data.normal, 0.0f);
  }

  prev_pairs = std::move(cur_pairs);
}

// ─── Debug draw ────────────────────────────────────────────────────────────

void physics_world::debug_draw_world(entt::registry &registry,
                                     math::matrix4 vp) {
  if (!dynamics_world || !draw_debug)
    return;
  debug_drawer.vp = vp;
  dynamics_world->debugDrawWorld();
}

// ─── Forces ────────────────────────────────────────────────────────────────

void physics_world::add_force(physics_body &pb, math::vector3 force) {
  if (!pb.bt_body)
    return;
  auto *b = static_cast<btRigidBody *>(pb.bt_body);
  b->activate(true);
  b->applyCentralForce(to_bt(force));
}

void physics_world::add_impulse(physics_body &pb, math::vector3 impulse) {
  if (!pb.bt_body)
    return;
  auto *b = static_cast<btRigidBody *>(pb.bt_body);
  b->activate(true);
  b->applyCentralImpulse(to_bt(impulse));
}

void physics_world::add_torque(physics_body &pb, math::vector3 torque) {
  if (!pb.bt_body)
    return;
  auto *b = static_cast<btRigidBody *>(pb.bt_body);
  b->activate(true);
  b->applyTorque(to_bt(torque));
}

void physics_world::add_force_at_position(physics_body &pb, math::vector3 force,
                                          math::vector3 world_pos) {
  if (!pb.bt_body)
    return;
  auto *b = static_cast<btRigidBody *>(pb.bt_body);
  b->activate(true);
  b->applyForce(to_bt(force),
                to_bt(world_pos) - b->getCenterOfMassPosition());
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
  btCollisionWorld::ClosestRayResultCallback cb(from, to);
  dynamics_world->rayTest(from, to, cb);
  if (cb.hasHit()) {
    result.hit = true;
    result.point = from_bt(cb.m_hitPointWorld);
    result.normal = from_bt(cb.m_hitNormalWorld);
    result.distance = (result.point - origin).norm();
    result.entity = entity_from_bt(cb.m_collisionObject);
  }
  return result;
}

// ─── GUI ───────────────────────────────────────────────────────────────────

void physics_world::draw_menu_gui() {
  ImGui::Checkbox("Simulation Enabled", &simulation_enabled);
  ImGui::Checkbox("Debug Draw", &draw_debug);
  ImGui::DragFloat3("Gravity", gravity.data(), 0.01f);
  ImGui::DragFloat("Fixed Timestep", &fixed_timestep, 0.001f, 0.001f, 0.1f);
  ImGui::DragInt("Max Substeps", &max_substeps, 1, 1, 16);
  ImGui::DragInt("Solver Iterations", &solver_iterations, 1, 1, 100);
  ImGui::DragFloat("ERP", &erp, 0.01f, 0.0f, 1.0f);
  ImGui::DragFloat("ERP2", &erp2, 0.01f, 0.0f, 1.0f);
  ImGui::DragFloat("CCD Threshold", &ccd_threshold, 0.01f, 0.0f, 10.0f);
  if (dynamics_world) {
    ImGui::Text("Bodies: %d  Constraints: %d  Controllers: %zu",
                dynamics_world->getNumCollisionObjects(),
                dynamics_world->getNumConstraints(),
                registered_controllers.size());
  }
}

}; // namespace toolkit::opengl3d
