#pragma once

#include "toolkit/opengl/components/mesh.hpp"
#include "toolkit/system.hpp"
#include "toolkit/transform.hpp"

namespace toolkit::sim {

class collider_convex_hull : public icomponent {
public:
  void create_from_mesh_data(std::vector<assets::mesh_vertex> &vertices,
                             std::vector<uint32_t> &indices);
};
DECLARE_COMPONENT(collider_convex_hull, simulation)

struct collider_sphere : icomponent {
  float radius = 0.0f;
  math::vector3 center = math::vector3::Zero();
};
DECLARE_COMPONENT(collider_sphere, simulation, radius, center)

}; // namespace toolkit::sim