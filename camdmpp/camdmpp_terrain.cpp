#include "camdmpp.hpp"

#include "toolkit/opengl3d/components/actor.hpp"

#include <cmath>
#include <limits>

namespace toolkit::opengl3d {

namespace {

math::vector2 clamp_terrain_query(const math::vector2 &xz,
                                  const math::vector2 &bounds_min,
                                  const math::vector2 &bounds_max) {
  return math::vector2(std::clamp(xz.x(), bounds_min.x(), bounds_max.x()),
                       std::clamp(xz.y(), bounds_min.y(), bounds_max.y()));
}

int clamp_grid_coord(float value, float min_value, float cell_size,
                     int cell_count) {
  if (cell_count <= 1)
    return 0;
  return std::clamp(
      static_cast<int>(std::floor((value - min_value) / cell_size)), 0,
      cell_count - 1);
}

} // namespace

bool camdmpp::sample_terrain_triangle(const terrain_triangle &triangle,
                                      const math::vector2 &xz, float &height) {
  constexpr float barycentric_epsilon = 1e-4f;

  const float x0 = triangle.xz0.x(), z0 = triangle.xz0.y();
  const float x1 = triangle.xz1.x(), z1 = triangle.xz1.y();
  const float x2 = triangle.xz2.x(), z2 = triangle.xz2.y();
  const float px = xz.x(), pz = xz.y();

  const float denom = (z1 - z2) * (x0 - x2) + (x2 - x1) * (z0 - z2);
  if (std::abs(denom) < 1e-6f)
    return false;

  const float w0 = ((z1 - z2) * (px - x2) + (x2 - x1) * (pz - z2)) / denom;
  const float w1 = ((z2 - z0) * (px - x2) + (x0 - x2) * (pz - z2)) / denom;
  const float w2 = 1.0f - w0 - w1;
  if (w0 < -barycentric_epsilon || w1 < -barycentric_epsilon ||
      w2 < -barycentric_epsilon) {
    return false;
  }

  height = w0 * triangle.p0.y() + w1 * triangle.p1.y() + w2 * triangle.p2.y();
  return true;
}

void camdmpp::build_terrain_sampler() {
  terrain_triangles.clear();
  terrain_grid_cells.clear();
  terrain_grid_width = 0;
  terrain_grid_depth = 0;
  terrain_grid_min = math::vector2::Zero();
  terrain_grid_max = math::vector2::Zero();
  terrain_grid_cell_size = math::vector2::Ones();
  terrain_default_height = 0.0f;

  if (ground_entity == entt::null || !registry.valid(ground_entity) ||
      !registry.all_of<mesh_data, transform>(ground_entity)) {
    return;
  }

  auto &ground_mesh = registry.get<mesh_data>(ground_entity);
  auto &ground_trans = registry.get<transform>(ground_entity);
  ground_trans.force_update_hierarchy();
  terrain_default_height = ground_trans.world_pos().y();

  if (ground_mesh.vertices.empty()) {
    return;
  }

  std::vector<math::vector3> world_vertices(ground_mesh.vertices.size(),
                                            math::vector3::Zero());
  const auto world_matrix = ground_trans.matrix();
  for (std::size_t i = 0; i < ground_mesh.vertices.size(); i++) {
    math::vector4 local_vertex;
    local_vertex << ground_mesh.vertices[i].position.x(),
        ground_mesh.vertices[i].position.y(),
        ground_mesh.vertices[i].position.z(), 1.0f;
    world_vertices[i] = (world_matrix * local_vertex).head<3>();
  }

  bool has_bounds = false;
  auto append_triangle = [&](std::uint32_t i0, std::uint32_t i1,
                             std::uint32_t i2) {
    if (i0 >= world_vertices.size() || i1 >= world_vertices.size() ||
        i2 >= world_vertices.size()) {
      return;
    }

    terrain_triangle triangle;
    triangle.p0 = world_vertices[i0];
    triangle.p1 = world_vertices[i1];
    triangle.p2 = world_vertices[i2];
    triangle.xz0 = math::vector2(triangle.p0.x(), triangle.p0.z());
    triangle.xz1 = math::vector2(triangle.p1.x(), triangle.p1.z());
    triangle.xz2 = math::vector2(triangle.p2.x(), triangle.p2.z());

    float projected_height;
    if (!sample_terrain_triangle(triangle, triangle.xz0, projected_height)) {
      return;
    }

    const math::vector2 tri_min(
        std::min({triangle.xz0.x(), triangle.xz1.x(), triangle.xz2.x()}),
        std::min({triangle.xz0.y(), triangle.xz1.y(), triangle.xz2.y()}));
    const math::vector2 tri_max(
        std::max({triangle.xz0.x(), triangle.xz1.x(), triangle.xz2.x()}),
        std::max({triangle.xz0.y(), triangle.xz1.y(), triangle.xz2.y()}));

    if (!has_bounds) {
      terrain_grid_min = tri_min;
      terrain_grid_max = tri_max;
      has_bounds = true;
    } else {
      terrain_grid_min.x() = std::min(terrain_grid_min.x(), tri_min.x());
      terrain_grid_min.y() = std::min(terrain_grid_min.y(), tri_min.y());
      terrain_grid_max.x() = std::max(terrain_grid_max.x(), tri_max.x());
      terrain_grid_max.y() = std::max(terrain_grid_max.y(), tri_max.y());
    }

    terrain_triangles.push_back(std::move(triangle));
  };

  if (!ground_mesh.indices.empty()) {
    for (std::size_t tri_idx = 0; tri_idx + 2 < ground_mesh.indices.size();
         tri_idx += 3) {
      append_triangle(ground_mesh.indices[tri_idx + 0],
                      ground_mesh.indices[tri_idx + 1],
                      ground_mesh.indices[tri_idx + 2]);
    }
  } else {
    for (std::uint32_t tri_idx = 0; tri_idx + 2 < ground_mesh.vertices.size();
         tri_idx += 3) {
      append_triangle(tri_idx + 0, tri_idx + 1, tri_idx + 2);
    }
  }

  if (terrain_triangles.empty()) {
    return;
  }

  const float extent_x =
      std::max(terrain_grid_max.x() - terrain_grid_min.x(), 1e-3f);
  const float extent_z =
      std::max(terrain_grid_max.y() - terrain_grid_min.y(), 1e-3f);
  const int target_cell_count =
      std::clamp(static_cast<int>(terrain_triangles.size() / 2), 16, 4096);
  const float aspect = extent_x / extent_z;

  terrain_grid_width = std::max(
      1, static_cast<int>(std::round(std::sqrt(target_cell_count * aspect))));
  terrain_grid_depth = std::max(
      1, static_cast<int>(std::round(std::sqrt(target_cell_count / aspect))));
  terrain_grid_cell_size = math::vector2(extent_x / terrain_grid_width,
                                         extent_z / terrain_grid_depth);
  terrain_grid_cells.resize(terrain_grid_width * terrain_grid_depth);

  auto clamp_cell_x = [&](float x) {
    return clamp_grid_coord(x, terrain_grid_min.x(), terrain_grid_cell_size.x(),
                            terrain_grid_width);
  };
  auto clamp_cell_z = [&](float z) {
    return clamp_grid_coord(z, terrain_grid_min.y(), terrain_grid_cell_size.y(),
                            terrain_grid_depth);
  };

  for (int tri_idx = 0; tri_idx < terrain_triangles.size(); tri_idx++) {
    const auto &triangle = terrain_triangles[tri_idx];
    const int min_x = clamp_cell_x(
        std::min({triangle.xz0.x(), triangle.xz1.x(), triangle.xz2.x()}));
    const int max_x = clamp_cell_x(
        std::max({triangle.xz0.x(), triangle.xz1.x(), triangle.xz2.x()}));
    const int min_z = clamp_cell_z(
        std::min({triangle.xz0.y(), triangle.xz1.y(), triangle.xz2.y()}));
    const int max_z = clamp_cell_z(
        std::max({triangle.xz0.y(), triangle.xz1.y(), triangle.xz2.y()}));

    for (int cell_z = min_z; cell_z <= max_z; cell_z++) {
      for (int cell_x = min_x; cell_x <= max_x; cell_x++) {
        terrain_grid_cells[cell_z * terrain_grid_width + cell_x].push_back(
            tri_idx);
      }
    }
  }
}

float camdmpp::sample_terrain_height(const math::vector2 &xz,
                                     float fallback_height) const {
  if (terrain_triangles.empty() || terrain_grid_cells.empty() ||
      terrain_grid_width <= 0 || terrain_grid_depth <= 0) {
    return fallback_height;
  }

  const math::vector2 query_xz =
      clamp_terrain_query(xz, terrain_grid_min, terrain_grid_max);

  const auto clamp_cell_x = [&](float x) {
    return clamp_grid_coord(x, terrain_grid_min.x(), terrain_grid_cell_size.x(),
                            terrain_grid_width);
  };
  const auto clamp_cell_z = [&](float z) {
    return clamp_grid_coord(z, terrain_grid_min.y(), terrain_grid_cell_size.y(),
                            terrain_grid_depth);
  };

  auto sample_cell = [&](int cell_x, int cell_z, float &best_height) {
    bool hit = false;
    for (int tri_idx :
         terrain_grid_cells[cell_z * terrain_grid_width + cell_x]) {
      float sampled_height = 0.0f;
      if (sample_terrain_triangle(terrain_triangles[tri_idx], query_xz,
                                  sampled_height)) {
        best_height = std::max(best_height, sampled_height);
        hit = true;
      }
    }
    return hit;
  };

  float best_height = -std::numeric_limits<float>::infinity();
  const int base_x = clamp_cell_x(query_xz.x());
  const int base_z = clamp_cell_z(query_xz.y());
  if (sample_cell(base_x, base_z, best_height)) {
    return best_height;
  }

  // A triangle containing the query should land in the query cell, but sample
  // the immediate ring as a cheap guard against cell-boundary precision issues.
  for (int dz = -1; dz <= 1; dz++) {
    for (int dx = -1; dx <= 1; dx++) {
      if (dx == 0 && dz == 0)
        continue;
      const int cell_x = base_x + dx;
      const int cell_z = base_z + dz;
      if (cell_x < 0 || cell_x >= terrain_grid_width || cell_z < 0 ||
          cell_z >= terrain_grid_depth) {
        continue;
      }
      sample_cell(cell_x, cell_z, best_height);
    }
  }

  if (best_height > -std::numeric_limits<float>::infinity()) {
    return best_height;
  }

  return fallback_height;
}

}; // namespace toolkit::opengl3d