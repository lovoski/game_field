#pragma once

#include "toolkit/opengl/components/mesh.hpp"
#include "toolkit/system.hpp"
#include "toolkit/transform.hpp"

namespace toolkit::sim {

// struct collider_convex_hull : public icomponent {
//   collider_convex_hull(std::vector<assets::mesh_vertex> &vertices,
//                        std::vector<uint32_t> &indices);
// };
// DECLARE_COMPONENT(collider_convex_hull, simulation)

struct collider_sphere : public icomponent {
  float radius = 0.0f;
  math::vector3 offset = math::vector3::Zero();
};
DECLARE_COMPONENT(collider_sphere, simulation, radius, offset)

}; // namespace toolkit::sim