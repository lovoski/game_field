#pragma once

#include "toolkit/opengl/base.hpp"
#include "toolkit/system.hpp"
#include "toolkit/transform.hpp"

#include <array>

namespace toolkit::opengl {

struct camera {
  std::array<math::vector4, 6> planes;
  math::matrix4 view, proj, vp;

  float fovyDegree = 45.0f, zNear = 0.1f, zFar = 10000.0f;

  SERIALIZABLE_COMPONENT(camera, fovyDegree, zNear, zFar)
};

void compute_vp_matrix(entt::registry &registry, entt::entity entity,
                       float width, float height) {
  auto &trans = registry.get<transform>(entity);
  auto &cam = registry.get<camera>(entity);
  auto camPos = trans.position();

  cam.view =
      math::lookat(camPos, camPos - trans.local_forward(), trans.local_up());

  cam.proj = math::perspective(math::deg_to_rad(cam.fovyDegree), width / height,
                               cam.zNear, cam.zFar);

  cam.vp = cam.proj * cam.view;

  cam.planes[0] = cam.vp.row(3) + cam.vp.row(0); // Left
  cam.planes[1] = cam.vp.row(3) - cam.vp.row(0); // Right
  cam.planes[2] = cam.vp.row(3) + cam.vp.row(1); // Bottom
  cam.planes[3] = cam.vp.row(3) - cam.vp.row(1); // Top
  cam.planes[4] = cam.vp.row(3) + cam.vp.row(2); // Near
  cam.planes[5] = cam.vp.row(3) - cam.vp.row(2); // Far

  // Normalize planes
  for (auto &plane : cam.planes) {
    float length = plane.head<3>().norm();
    plane /= length;
  }
}

bool frustom_check(entt::registry &registry, entt::entity entity,
                   math::vector3 boxMin, math::vector3 boxMax) {
  // Check if bounding box is outside any plane
  auto &cam = registry.get<camera>(entity);
  for (const auto &plane : cam.planes) {
    math::vector3 positiveVertex(plane.x() >= 0 ? boxMax.x() : boxMin.x(),
                                 plane.y() >= 0 ? boxMax.y() : boxMin.y(),
                                 plane.z() >= 0 ? boxMax.z() : boxMin.z());

    // Compute distance from plane
    float distance = plane.head<3>().dot(positiveVertex) + plane.w();
    if (distance < 0) {
      return false; // Outside the frustum
    }
  }
  return true; // Inside or intersects the frustum
}

}; // namespace toolkit::opengl