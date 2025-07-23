#pragma once

#include "toolkit/opengl/base.hpp"
#include "toolkit/system.hpp"
#include "toolkit/transform.hpp"

#include <array>

namespace toolkit::opengl {

struct camera : public icomponent {
  std::array<math::vector4, 6> planes;
  math::matrix4 view, proj, vp;

  bool perspective = true;
  float h_size = 10.0f, v_size = 10.0f;
  float fovy_degree = 45.0f, z_near = 0.1f, z_far = 200.0f;

  void draw_gui(iapp *app) override {
    ImGui::Checkbox("Perspective", &perspective);
    if (perspective) {
      ImGui::Text("Perspective Camera");
    } else {
      ImGui::Text("Orthogonal Camera");
    }
  }

  bool visibility_check(math::vector3 &box_min, math::vector3 &box_max,
                        const math::matrix4 &trans);
};
DECLARE_COMPONENT(camera, graphics, fovy_degree, z_near, z_far, perspective,
                  h_size, v_size)

void compute_vp_matrix(entt::registry &registry, entt::entity entity,
                       float width, float height);

}; // namespace toolkit::opengl