#pragma once

#include "toolkit/opengl/base.hpp"
#include "toolkit/system.hpp"
#include "toolkit/transform.hpp"

#include <array>

namespace toolkit::opengl {

struct camera {
  std::array<math::vector4, 6> planes;
  math::matrix4 view, proj, vp;

  float fovy_degree = 45.0f, z_near = 0.1f, z_far = 10000.0f;

  DECLARE_COMPONENT(camera, graphics, fovy_degree, z_near, z_far)
};

void compute_vp_matrix(entt::registry &registry, entt::entity entity,
                       float width, float height);

bool frustom_check(entt::registry &registry, entt::entity entity,
                   math::vector3 boxMin, math::vector3 boxMax);

}; // namespace toolkit::opengl