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

Spring::Spring(Solver *solver, Rigid *bodyA, Rigid *bodyB, float2 rA, float2 rB,
               float stiffness, float rest)
    : Force(solver, bodyA, bodyB), rA(rA), rB(rB), rest(rest) {
  this->stiffness[0] = stiffness;
  if (this->rest < 0)
    this->rest = (transform_position(bodyA->position, rA) -
                  transform_position(bodyB->position, rB))
                     .norm();
}

void Spring::computeConstraint(float alpha) {
  // Compute constraint function at current state C(x)
  C[0] = (transform_position(bodyA->position, rA) -
          transform_position(bodyB->position, rB))
             .norm() -
         rest;
}

void Spring::computeDerivatives(Rigid *body) {
  // Compute the first and second derivatives for the desired body
  matrix2 S;
  S(0, 0) = 0;
  S(0, 1) = -1;
  S(1, 0) = 1;
  S(1, 1) = 0;
  matrix2 I;
  I(0, 0) = 1;
  I(0, 1) = 0;
  I(1, 0) = 0;
  I(1, 1) = 1;

  vector2 d = transform_position(bodyA->position, rA) -
              transform_position(bodyB->position, rB);
  float dlen2 = d.dot(d);
  if (dlen2 == 0)
    return;

  float dlen = sqrtf(dlen2);
  vector2 n = d / dlen;
  matrix2 dxx = (I - outer(n, n)) / dlen;

  if (body == bodyA) {
    vector2 Sr = rotate(bodyA->position.z(), S * rA);
    vector2 r = rotate(bodyA->position.z(), rA);

    vector2 dxr = dxx * Sr;
    float drr = -n.dot(r) - n.dot(r);

    J[0].head<2>() = n;
    J[0].z() = n.dot(Sr);
    H[0] << dxx.row(0).x(), dxx.row(0).y(), dxr.x(), dxx.row(1).x(),
        dxx.row(1).y(), dxr.y(), dxr.x(), dxr.y(), drr;
  } else {
    vector2 Sr = rotate(bodyB->position.z(), S * rB);
    vector2 r = rotate(bodyB->position.z(), rB);
    vector2 dxr = dxx * Sr;
    float drr = n.dot(r) + n.dot(r);

    J[0].head<2>() = -n;
    J[0].z() = n.dot(-Sr);
    H[0] << dxx.row(0).x(), dxx.row(0).y(), dxr.x(), dxx.row(1).x(),
        dxx.row(1).y(), dxr.y(), dxr.x(), dxr.y(), drr;
  }
}

void Spring::draw() const {
  //   float2 v0 = transform(bodyA->position, rA);
  //   float2 v1 = transform(bodyB->position, rB);

  //   glColor3f(0.75f, 0.0f, 0.0f);
  //   glBegin(GL_LINES);
  //   glVertex2f(v0.x, v0.y);
  //   glVertex2f(v1.x, v1.y);
  //   glEnd();
}

}; // namespace toolkit::avbd
