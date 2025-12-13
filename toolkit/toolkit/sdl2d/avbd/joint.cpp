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

#include "solver.h"

using namespace toolkit::math;

namespace toolkit::avbd {

Joint::Joint(Solver *solver, Rigid *bodyA, Rigid *bodyB, math::vector2 rA,
             math::vector2 rB, math::vector3 stiffness, float fracture)
    : Force(solver, bodyA, bodyB), rA(rA), rB(rB) {
  this->stiffness[0] = stiffness.x();
  this->stiffness[1] = stiffness.y();
  this->stiffness[2] = stiffness.z();
  this->fmax[2] = fracture;
  this->fmin[2] = -fracture;
  this->fracture[2] = fracture;
  this->restAngle = (bodyA ? bodyA->position.z() : 0.0f) - bodyB->position.z();
  this->torqueArm =
      ((bodyA ? bodyA->size : vector2{0, 0}) + bodyB->size).squaredNorm();
}

bool Joint::initialize() {
  // Store constraint function at beginnning of timestep C(x-)
  // Note: if bodyA is null, it is assumed that the joint connects a body to the
  // world space position rA
  C0.head<2>() =
      (bodyA ? (rotation(bodyA->position.z()) * rA + bodyA->position.head<2>())
             : rA) -
      (rotation(bodyB->position.z()) * rB + bodyB->position.head<2>());
  C0.z() =
      ((bodyA ? bodyA->position.z() : 0) - bodyB->position.z() - restAngle) *
      torqueArm;
  return stiffness[0] != 0 || stiffness[1] != 0 || stiffness[2] != 0;
}

void Joint::computeConstraint(float alpha) {
  // Compute constraint function at current state C(x)
  float3 Cn;
  Cn.head<2>() =
      (bodyA ? (rotation(bodyA->position.z()) * rA + bodyA->position.head<2>())
             : rA) -
      (rotation(bodyB->position.z()) * rB + bodyB->position.head<2>());
  Cn.z() =
      ((bodyA ? bodyA->position.z() : 0) - bodyB->position.z() - restAngle) *
      torqueArm;

  for (int i = 0; i < rows(); i++) {
    // Store stabilized constraint function, if a hard constraint (Eq. 18)
    if (isinf(stiffness[i]))
      C[i] = Cn[i] - C0[i] * alpha;
    else
      C[i] = Cn[i];
  }
}

void Joint::computeDerivatives(Rigid *body) {
  // Compute the first and second derivatives for the desired body
  if (body == bodyA) {
    float2 r = rotate(bodyA->position.z(), rA);
    J[0] = {1.0f, 0.0f, -r.y()};
    J[1] = {0.0f, 1.0f, r.x()};
    J[2] = {0.0f, 0.0f, torqueArm};
    H[0] = matrix3::Zero();
    H[0](2, 2) = -r.x();
    H[1] = matrix3::Zero();
    H[1](2, 2) = -r.y();
    H[2] = matrix3::Zero();
  } else {
    float2 r = rotate(bodyB->position.z(), rB);
    J[0] = {-1.0f, 0.0f, r.y()};
    J[1] = {0.0f, -1.0f, -r.x()};
    J[2] = {0.0f, 0.0f, -torqueArm};
    H[0] = matrix3::Zero();
    H[0](2, 2) = r.x();
    H[1] = matrix3::Zero();
    H[1](2, 2) = r.y();
    H[2] = matrix3::Zero();
  }
}

void Joint::draw() const {
  // float2 v0 = bodyA ? transform(bodyA->position, rA) : rA;
  // float2 v1 = transform(bodyB->position, rB);

  // glColor3f(0.75f, 0.0f, 0.0f);
  // glBegin(GL_LINES);
  // glVertex2f(v0.x, v0.y);
  // glVertex2f(v1.x, v1.y);
  // glEnd();
}

}; // namespace toolkit::avbd
