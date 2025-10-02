#include "QuickHull.hpp"
#include "toolkit/opengl/components/mesh.hpp"

namespace toolkit::opengl {

std::tuple<std::vector<assets::mesh_vertex>, std::vector<std::uint32_t>>
quickhull(std::vector<assets::mesh_vertex> &vertices,
          std::vector<std::uint32_t> &indices) {
  std::vector<assets::mesh_vertex> convex_vertices;
  std::vector<std::uint32_t> convex_indices;

  quickhull::QuickHull<float> qh;
  std::vector<quickhull::Vector3<float>> points(vertices.size());
  for (int i = 0; i < vertices.size(); i++) {
    points[i].x = vertices[i].position.x();
    points[i].y = vertices[i].position.y();
    points[i].z = vertices[i].position.z();
  }
  auto hull = qh.getConvexHull(points, true, false);
  auto &index_buffer = hull.getIndexBuffer();
  auto &vertex_buffer = hull.getVertexBuffer();

  convex_vertices.resize(vertex_buffer.size());
  convex_indices.resize(index_buffer.size());
  for (int i = 0; i < convex_vertices.size(); i++) {
    convex_vertices[i].position << vertex_buffer[i].x, vertex_buffer[i].y,
        vertex_buffer[i].z, 1.0;
  }
  for (int i = 0; i < convex_indices.size(); i++)
    convex_indices[i] = index_buffer[i];

  return {convex_vertices, convex_indices};
}

}; // namespace toolkit::opengl
