#include "toolkit/opengl/components/mesh.hpp"

namespace toolkit::opengl {

void convex_hull(mesh_data &data, std::vector<assets::mesh_vertex> &vertices,
                 std::vector<uint32_t> &indices) {}

void smooth_normal(opengl::mesh_data &data) {
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

}; // namespace toolkit::opengl
