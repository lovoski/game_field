#pragma once

#include "toolkit/sim/components/colliders.hpp"

namespace toolkit::sim {

std::pair<math::vector3, float>
welzl_bounding_sphere(const std::vector<math::vector3> &points,
                      bool shuffle = false);

struct gjk_simplex {
  math::vector3 a, b, c, d;
  std::uint32_t num;
};

math::vector3 support_point_of_minkowski_difference(base_collider *c1,
                                                    base_collider *c2,
                                                    math::vector3 direction);

bool gjk_collides(base_collider *c1, base_collider *c2, gjk_simplex &simplex);

bool epa(base_collider *c1, base_collider *c2, gjk_simplex &simplex,
         math::vector3 &normal, float &penetration);

void convex_convex_contact_manifold(convex_hull_collider *c1,
                                    convex_hull_collider *c2,
                                    math::vector3 normal,
                                    std::vector<collider_contact> &contacts);

}; // namespace toolkit::sim
