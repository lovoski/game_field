#include "toolkit/sim/algorithms/algo.hpp"

using namespace toolkit::math;

#define GJK_MAX_ITERATIONS 100

namespace toolkit::sim {

void add_to_simplex(gjk_simplex &simplex, vector3 point);

vector3 support_point_of_minkowski_difference(base_collider *c1,
                                              base_collider *c2,
                                              vector3 direction) {
  return c1->get_support(direction) - c2->get_support(-direction);
}

bool do_simplex_2(gjk_simplex &simplex, vector3 &direction) {
  vector3 a = simplex.a; // the last point added
  vector3 b = simplex.b;
  vector3 ao = -a; // 'a' to 'origin'
  vector3 ab = b - a;
  if (same_direction(ab, ao)) {
    simplex.dim = 2;
    direction = triple_cross(ab, ao, ab);
  } else {
    simplex.dim = 1;
    direction = ao;
  }
  return false;
}

bool do_simplex_3(gjk_simplex &simplex, vector3 &direction) {
  vector3 a = simplex.a; // the last point added
  vector3 b = simplex.b;
  vector3 c = simplex.c;
  vector3 ao = -a;
  vector3 ab = b - a;
  vector3 ac = c - a;
  vector3 abc = ab.cross(ac);
  if (same_direction(abc.cross(ac), ao)) {
    if (same_direction(ac, ao)) {
      // outside AC region
      simplex.a = a;
      simplex.b = c;
      simplex.dim = 2;
      direction = triple_cross(ac, ao, ac);
    } else {
      simplex.a = a;
      simplex.b = b;
      simplex.dim = 2;
      return do_simplex_2(simplex, direction);
    }
  } else {
    if (same_direction(ab.cross(abc), ao)) {
      simplex.a = a;
      simplex.b = b;
      simplex.dim = 2;
      return do_simplex_2(simplex, direction);
    } else {
      if (same_direction(abc, ao)) {
        // ABC region ("up")
        direction = abc;
      } else {
        // ABC region ("down")
        simplex.a = a;
        simplex.b = c;
        simplex.c = b;
        simplex.dim = 3;
        direction = -abc;
      }
    }
  }
  return false;
}

bool do_simplex_4(gjk_simplex &simplex, vector3 &direction) {
  vector3 a = simplex.a; // the last point added
  vector3 b = simplex.b;
  vector3 c = simplex.c;
  vector3 d = simplex.d;
  vector3 ao = -a;
  vector3 ab = b - a;
  vector3 ac = c - a;
  vector3 ad = d - a;
  vector3 abc = ab.cross(ac);
  vector3 acd = ac.cross(ad);
  vector3 adb = ad.cross(ab);

  if (same_direction(abc, ao)) {
    simplex.a = a;
    simplex.b = b;
    simplex.c = c;
    simplex.dim = 3;
    return do_simplex_3(simplex, direction);
  }

  if (same_direction(acd, ao)) {
    simplex.a = a;
    simplex.b = c;
    simplex.c = d;
    simplex.dim = 3;
    return do_simplex_3(simplex, direction);
  }

  if (same_direction(adb, ao)) {
    simplex.a = a;
    simplex.b = d;
    simplex.c = b;
    simplex.dim = 3;
    return do_simplex_3(simplex, direction);
  }

  return true;
}

bool do_simplex(gjk_simplex &simplex, vector3 &direction) {
  switch (simplex.dim) {
  case 2:
    return do_simplex_2(simplex, direction);
  case 3:
    return do_simplex_3(simplex, direction);
  case 4:
    return do_simplex_4(simplex, direction);
  }
  assert(0);
  return false;
}

/**
 * Reference:
 * https://winter.dev/articles/gjk-algorithm
 * https://www.youtube.com/watch?v=MDusDn8oTSE&t=92s
 * https://www.youtube.com/watch?v=Qupqu1xe7Io
 * https://github.com/felipeek/raw-physics
 *
 * By computing the minkowski difference support point of two arbitrary
 * colliders with support point implementation, we can get one vertex on the
 * convex minkowski difference shape. Then the problem is to determine whether
 * this minkowski difference contains the origin, if so we can say there's a
 * collision.
 *
 * The goal of the GJK algorithm is to determine if the origin is within the
 * Minkowski difference. This would be easy, but we've thrown out the complete
 * difference for the sake of performance. We only have the Support function
 * that gives us one vertex at a time. We need to iteratively search for and
 * build up what's referred to as a simplex around the origin.
 *
 * We get the vertices for the simplex from the Support function, so we need to
 * find the direction to the origin from the closest feature. Searching towards
 * the origin allows the algorithm to converge quickly. Let's look an example.
 * We'll start with an arbitrary vertex then add or remove vertices every
 * iteration until we surround the origin or find it's impossible.
 * TODO: What happens when direction is all zero?
 */
bool gjk_collides(base_collider *c1, base_collider *c2, gjk_simplex &simplex) {
  simplex.dim = 0;
  // Get initial support point in any direction, for now we only have one point
  // in the simplex
  simplex.a = support_point_of_minkowski_difference(
      c1, c2, vector3(0.01f, 0.02f, 1.03f));
  simplex.dim = 1;
  // Points the new direction towards the origin from one support point (since
  // we want our simplex as close to the origin as possible to accelerate
  // convergence)
  vector3 direction = -simplex.a;
  for (int i = 0; i < GJK_MAX_ITERATIONS; i++) {
    // Get a point on the minkowski difference convex given direction
    auto next_point = support_point_of_minkowski_difference(c1, c2, direction);
    if (next_point.dot(direction) <= 0.0f) {
      // no intersection
      return false;
    }
    add_to_simplex(simplex, next_point);
    // search for the next direction given the new simplex, we want to
    // determine in which regeion splited by the simplex the origin is, so we
    // can find a direction that points towards the origin
    if (do_simplex(simplex, direction)) {
      // intersection
      return true;
    }
  }
  // didn't converge
  spdlog::error("GJK did not converge");
  return false;
}

/**
 * Push the new point to the front of 3d simplex (tetrahedron)
 */
void add_to_simplex(gjk_simplex &simplex, vector3 point) {
  switch (simplex.dim) {
  case 1: {
    simplex.b = simplex.a;
    simplex.a = point;
  } break;
  case 2: {
    simplex.c = simplex.b;
    simplex.b = simplex.a;
    simplex.a = point;
  } break;
  case 3: {
    simplex.d = simplex.c;
    simplex.c = simplex.b;
    simplex.b = simplex.a;
    simplex.a = point;
  } break;
  default: {
    assert(0);
  } break;
  }
  ++simplex.dim;
}

}; // namespace toolkit::sim