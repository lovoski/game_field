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

}; // namespace toolkit::opengl