/*
 * Copyright (c) 2025 Chris Giles
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Chris Giles makes no representations about the suitability
 * of this software for any purpose.
 * It is provided "as is" without express or implied warranty.
 */

#pragma once

#include "toolkit/sdl2d/avbd/utils.hpp"
#include "toolkit/sdl2d/header.hpp"

namespace toolkit::sdl2d {
class engine2d;
};

namespace toolkit::avbd {

struct Rigid;
struct Force;
struct Manifold;
struct Solver;

// Core solver class which holds all the rigid bodies and forces, and has logic
// to step the simulation forward in time
struct Solver {
  // Most number of rows an individual constraint can have
  inline static const int MAX_ROWS = 4;
  // Minimum penalty parameter
  inline static const float PENALTY_MIN = 1.0f;
  // Maximum penalty parameter
  inline static const float PENALTY_MAX = 1000000000.0f;
  // Margin for collision detection to avoid flickering contacts
  inline static const float COLLISION_MARGIN = 0.0005f;
  // Position threshold for sticking contacts (ie static friction)
  inline static const float STICK_THRESH = 0.01f;
  // Whether to show contacts in the debug draw
  bool SHOW_CONTACTS = true;

  float dt;       // Timestep
  float gravity;  // Gravity
  int iterations; // Solver iterations

  float alpha; // Stabilization parameter
  float beta;  // Penalty ramping parameter
  float gamma; // Warmstarting decay parameter

  bool postStabilize; // Whether to apply post-stabilization to the system

  Rigid *bodies;
  Force *forces;

  Solver();
  ~Solver();

  int cur_exec_fixed = 0;
  float cur_time = 0.0f;
  void update(float timestep);

  Rigid *pick(math::vector2 at, math::vector2 &local);
  void clear();
  void defaultParams();

  // One step call consists of one collision_step and one simulation_step
  void step();

  void collision_step();
  void simulation_step();

  void debug_draw(sdl2d::engine2d *engine);
};

enum ColliderType { CIRCLE, CONVEX };

class ColliderShape {
public:
  ColliderType type;
  virtual void debug_draw(sdl2d::engine2d *engine) {}
};

class CircleColliderShape : public ColliderShape {
public:
  CircleColliderShape() { type = ColliderType::CIRCLE; }
  float radius = 1.0f;
  math::vector2 center = math::vector2(0, 0);
};

class ConvexColliderShape : public ColliderShape {
public:
  ConvexColliderShape() { type = ColliderType::CONVEX; }
  // points on the convex shape in CCW ordering
  std::vector<math::vector2> points;
};

// Holds all the state for a single rigid body that is needed by AVBD
struct Rigid {
  Solver *solver;
  Force *forces;
  Rigid *next;
  math::vector3 position;
  math::vector3 initial;
  math::vector3 inertial;
  math::vector3 velocity;
  math::vector3 prevVelocity;
  math::vector2 size;
  float mass;
  float moment;
  float friction;
  float radius;

  std::unique_ptr<ColliderShape> shape;

  Rigid(Solver *solver, math::vector2 size, float density, float friction,
        math::vector3 position,
        math::vector3 velocity = math::vector3{0, 0, 0});
  ~Rigid();

  bool constrainedTo(Rigid *other) const;
  void debug_draw(sdl2d::engine2d *engine);
};

// Holds all user defined and derived constraint parameters, and provides a
// common interface for all forces.
struct Force {
  Solver *solver;
  Rigid *bodyA;
  Rigid *bodyB;
  Force *nextA;
  Force *nextB;
  Force *next;

  math::vector3 J[Solver::MAX_ROWS];
  math::matrix3 H[Solver::MAX_ROWS];
  float C[Solver::MAX_ROWS];
  float fmin[Solver::MAX_ROWS];
  float fmax[Solver::MAX_ROWS];
  float stiffness[Solver::MAX_ROWS];
  float fracture[Solver::MAX_ROWS];
  float penalty[Solver::MAX_ROWS];
  float lambda[Solver::MAX_ROWS];

  Force(Solver *solver, Rigid *bodyA, Rigid *bodyB);
  virtual ~Force();

  void disable();

  virtual int rows() const = 0;
  virtual bool initialize() = 0;
  virtual void computeConstraint(float alpha) = 0;
  virtual void computeDerivatives(Rigid *body) = 0;
  virtual void debug_draw(sdl2d::engine2d *engine) const {}
};

// Revolute joint + angle constraint between two rigid bodies, with optional
// fracture
struct Joint : Force {
  math::vector2 rA, rB;
  math::vector3 C0;
  float torqueArm;
  float restAngle;

  Joint(Solver *solver, Rigid *bodyA, Rigid *bodyB, math::vector2 rA,
        math::vector2 rB,
        math::vector3 stiffness = math::vector3{INFINITY, INFINITY, INFINITY},
        float fracture = INFINITY);

  int rows() const override { return 3; }

  bool initialize() override;
  void computeConstraint(float alpha) override;
  void computeDerivatives(Rigid *body) override;
  void debug_draw(sdl2d::engine2d *engine) const override;
};

// Standard spring force
struct Spring : Force {
  math::vector2 rA, rB;
  float rest;

  Spring(Solver *solver, Rigid *bodyA, Rigid *bodyB, math::vector2 rA,
         math::vector2 rB, float stiffness, float rest = -1);

  int rows() const override { return 1; }

  bool initialize() override { return true; }
  void computeConstraint(float alpha) override;
  void computeDerivatives(Rigid *body) override;
  void debug_draw(sdl2d::engine2d *engine) const override;
};

// Force which has no physical effect, but is used to ignore collisions between
// two bodies
struct IgnoreCollision : Force {
  IgnoreCollision(Solver *solver, Rigid *bodyA, Rigid *bodyB)
      : Force(solver, bodyA, bodyB) {}

  int rows() const override { return 0; }

  bool initialize() override { return true; }
  void computeConstraint(float alpha) override {}
  void computeDerivatives(Rigid *body) override {}
  void debug_draw(sdl2d::engine2d *engine) const override {}
};

// Motor force which applies a torque to two rigid bodies to achieve a desired
// angular speed
struct Motor : Force {
  float speed;

  Motor(Solver *solver, Rigid *bodyA, Rigid *bodyB, float speed,
        float maxTorque);

  int rows() const override { return 1; }

  bool initialize() override { return true; }
  void computeConstraint(float alpha) override;
  void computeDerivatives(Rigid *body) override;
  void debug_draw(sdl2d::engine2d *engine) const override {}
};

// Collision manifold between two rigid bodies, which contains up to two
// frictional contact points
struct Manifold : Force {
  // Used to track contact features between frames
  union FeaturePair {
    struct Edges {
      char inEdge1;
      char outEdge1;
      char inEdge2;
      char outEdge2;
    } e;
    int value;
  };

  // Contact point information for a single contact
  struct Contact {
    FeaturePair feature;
    math::vector2 rA;
    math::vector2 rB;
    math::vector2 normal;

    math::vector3 JAn, JBn, JAt, JBt;
    math::vector2 C0;
    bool stick;
  };

  Contact contacts[2];
  int numContacts;
  float friction;

  Manifold(Solver *solver, Rigid *bodyA, Rigid *bodyB);

  int rows() const override { return numContacts * 2; }

  bool initialize() override;
  void computeConstraint(float alpha) override;
  void computeDerivatives(Rigid *body) override;
  void debug_draw(sdl2d::engine2d *engine) const override;

  static int collide(Rigid *bodyA, Rigid *bodyB, Contact *contacts);
};

}; // namespace toolkit::avbd
