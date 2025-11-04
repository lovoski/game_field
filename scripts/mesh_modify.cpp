#include "mesh_modify.hpp"
using namespace toolkit;

void mesh_modify::draw_to_scene(iapp *app, transform &cam_trans,
                                camera &cam_comp) {}

void mesh_modify::draw_gui(iapp *app) {
  if (ImGui::Button("Recompute Normal")) {
    if (auto mesh_data = registry->try_get<opengl::mesh_data>(entity)) {
      smooth_normal(*mesh_data);
    }
  }
}
