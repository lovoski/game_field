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

#include "toolkit/sdl2d/engine.hpp"

using namespace toolkit::math;

namespace toolkit::avbd {

Rigid::Rigid(Solver *solver, vector2 size, float density, float friction,
             vector3 position, vector3 velocity)
    : solver(solver), forces(0), next(0), position(position),
      velocity(velocity), prevVelocity(velocity), size(size),
      friction(friction) {
  // Add to linked list
  next = solver->bodies;
  solver->bodies = this;

  // Compute mass properties and bounding radius
  mass = size.x() * size.y() * density;
  moment = mass * size.dot(size) / 12.0f;
  radius = (0.5f * size).norm();
}

Rigid::~Rigid() {
  // Remove from linked list
  Rigid **p = &solver->bodies;
  while (*p != this)
    p = &(*p)->next;
  *p = next;
  if (shape != nullptr)
    delete shape;
}

bool Rigid::constrainedTo(Rigid *other) const {
  // Check if this body is constrained to the other body
  for (Force *f = forces; f != 0; f = f->next)
    if ((f->bodyA == this && f->bodyB == other) ||
        (f->bodyA == other && f->bodyB == this))
      return true;
  return false;
}

void Rigid::debug_draw(sdl2d::engine2d *engine) {
  matrix2 R = from_angle(position.z());
  std::vector<vector2> points(4);
  points[0] =
      R * vector2{-size.x() * 0.5f, -size.y() * 0.5f} + position.head<2>();
  points[1] =
      R * vector2{size.x() * 0.5f, -size.y() * 0.5f} + position.head<2>();
  points[2] =
      R * vector2{size.x() * 0.5f, size.y() * 0.5f} + position.head<2>();
  points[3] =
      R * vector2{-size.x() * 0.5f, size.y() * 0.5f} + position.head<2>();

  for (int i = 0; i < 4; i++) {
    points[i] = engine->world_to_screen(points[i]);
  }

  engine->ss_draw_lines(points, true);
}

}; // namespace toolkit::avbd
