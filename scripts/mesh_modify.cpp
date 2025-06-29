#include "mesh_modify.hpp"
using namespace toolkit;

void mesh_modify::draw_to_scene(iapp *app) {
  opengl::script_draw_to_scene_proxy(
      app,
      [&](opengl::editor *editor, transform &trans, opengl::camera &cam) {});
}

void mesh_modify::draw_gui(iapp *app) {
  if (ImGui::Button("Recompute Normal")) {
    if (auto mesh_data = registry->try_get<opengl::mesh_data>(entity)) {
      compute_normal(*mesh_data);
    }
  }
}

void compute_normal(opengl::mesh_data &data) {
  int num_triangles = data.indices.size() / 3;
  std::vector<math::vector3> normal_cache(data.vertices.size(),
                                          math::vector3::Zero());
  for (int i = 0; i < num_triangles; i++) {
    int vid0 = data.indices[3 * i], vid1 = data.indices[3 * i + 1],
        vid2 = data.indices[3 * i + 2];
    math::vector3 vp0 = data.vertices[vid0].position.head<3>(),
                  vp1 = data.vertices[vid1].position.head<3>(),
                  vp2 = data.vertices[vid2].position.head<3>();
    math::vector3 e01 = (vp1 - vp0).normalized(),
                  e12 = (vp2 - vp1).normalized();
    math::vector3 fn = (e01.cross(e12)).normalized();
    normal_cache[vid0] += fn;
    normal_cache[vid1] += fn;
    normal_cache[vid2] += fn;
  }
  for (int i = 0; i < data.vertices.size(); i++) {
    data.vertices[i].normal << (normal_cache[i] / 3), 0.0f;
  }
  data.update_buffers(false);
}
