#include "toolkit/opengl/components/camera.hpp"
#include "toolkit/opengl/components/lights.hpp"
#include "toolkit/opengl/editor.hpp"

namespace toolkit::opengl {

void compute_vp_matrix(entt::registry &registry, entt::entity entity,
                       float width, float height) {
  auto &trans = registry.get<transform>(entity);
  auto &cam = registry.get<camera>(entity);
  auto camPos = trans.position();

  cam.view =
      math::lookat(camPos, camPos - trans.local_forward(), trans.local_up());

  if (cam.perspective)
    cam.proj = math::perspective(math::deg_to_rad(cam.fovy_degree),
                                 width / height, cam.z_near, cam.z_far);
  else
    cam.proj = math::ortho(-cam.h_size / 2, cam.h_size / 2, cam.v_size / 2,
                           -cam.v_size / 2, cam.z_near, cam.z_far);

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

void point_light::draw_to_scene(iapp *app) {
  script_draw_to_scene_proxy(
      app, [&](editor *editor, transform &cam_trans, camera &cam_comp) {

      });
}

math::vector3 get_positive_vertex(const math::vector3 &box_min,
                                  const math::vector3 &box_max,
                                  const math::vector3 &normal) {
  math::vector3 p_vertex = box_min;
  if (normal.x() >= 0)
    p_vertex.x() = box_max.x();
  if (normal.y() >= 0)
    p_vertex.y() = box_max.y();
  if (normal.z() >= 0)
    p_vertex.z() = box_max.z();
  return p_vertex;
}

bool camera::visibility_check(math::vector3 &box_min, math::vector3 &box_max,
                              const math::matrix4 &trans) {
  std::array<math::vector3, 8> corners_local = {
      box_min,
      math::vector3(box_max.x(), box_min.y(), box_min.z()),
      math::vector3(box_min.x(), box_max.y(), box_min.z()),
      math::vector3(box_min.x(), box_min.y(), box_max.z()),
      math::vector3(box_max.x(), box_max.y(), box_min.z()),
      math::vector3(box_max.x(), box_min.y(), box_max.z()),
      math::vector3(box_min.x(), box_max.y(), box_max.z()),
      box_max};
  math::vector3 world_box_min = {std::numeric_limits<float>::max(),
                                 std::numeric_limits<float>::max(),
                                 std::numeric_limits<float>::max()};
  math::vector3 world_box_max = {std::numeric_limits<float>::lowest(),
                                 std::numeric_limits<float>::lowest(),
                                 std::numeric_limits<float>::lowest()};
  math::vector4 tmp_point;
  for (const auto &corner_local : corners_local) {
    tmp_point << corner_local, 1.0f;
    math::vector3 corner_world = (trans * tmp_point).head<3>();

    world_box_min.x() = std::min(world_box_min.x(), corner_world.x());
    world_box_min.y() = std::min(world_box_min.y(), corner_world.y());
    world_box_min.z() = std::min(world_box_min.z(), corner_world.z());

    world_box_max.x() = std::max(world_box_max.x(), corner_world.x());
    world_box_max.y() = std::max(world_box_max.y(), corner_world.y());
    world_box_max.z() = std::max(world_box_max.z(), corner_world.z());
  }
  for (int i = 0; i < 6; ++i) {
    const math::vector4 &plane = planes[i];
    math::vector3 normal = {plane.x(), plane.y(), plane.z()};
    float distance = plane.w();

    math::vector3 p_vertex =
        get_positive_vertex(world_box_min, world_box_max, normal);

    float dist_p = normal.dot(p_vertex) + distance;

    if (dist_p < 0) {
      return false;
    }
  }
  return true;
}

}; // namespace toolkit::opengl