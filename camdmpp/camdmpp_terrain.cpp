#include "camdmpp.hpp"

#include "toolkit/opengl3d/components/actor.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>

namespace toolkit::opengl3d {

namespace {

struct terrain_triangle {
  math::vector3 p0 = math::vector3::Zero(), p1 = math::vector3::Zero(),
                p2 = math::vector3::Zero();
  math::vector2 xz0 = math::vector2::Zero(), xz1 = math::vector2::Zero(),
                xz2 = math::vector2::Zero();
};

constexpr float terrain_min_extent = 1e-3f;
constexpr int terrain_min_height_map_axis_resolution = 16;
constexpr int terrain_max_height_map_axis_resolution = 512;
// Tune this value and rebuild the app to trade build time for heightfield
// fidelity.
constexpr int terrain_height_map_max_axis_resolution = 1024;

bool sample_terrain_triangle(const terrain_triangle &triangle,
                             const math::vector2 &xz, float &height);

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

int terrain_height_sample_index(int sample_x, int sample_z, int sample_width) {
  return sample_z * sample_width + sample_x;
}

void choose_terrain_height_map_resolution(float extent_x, float extent_z,
                                          int &grid_width,
                                          int &grid_depth) {
  const int clamped_resolution =
    std::clamp(terrain_height_map_max_axis_resolution,
         terrain_min_height_map_axis_resolution,
         terrain_max_height_map_axis_resolution);
  if (extent_x >= extent_z) {
    grid_width = clamped_resolution;
    grid_depth = std::max(
        1, static_cast<int>(std::round(grid_width * extent_z / extent_x)));
  } else {
    grid_depth = clamped_resolution;
    grid_width = std::max(
        1, static_cast<int>(std::round(grid_depth * extent_x / extent_z)));
  }
}

void rasterize_triangle_to_height_map(
    const terrain_triangle &triangle, const math::vector2 &terrain_grid_min,
    const math::vector2 &terrain_grid_inv_cell_size, int terrain_grid_width,
    int terrain_grid_depth, const std::vector<float> &sample_x_positions,
    const std::vector<float> &sample_z_positions,
    std::vector<float> &terrain_height_map, std::vector<char> &height_valid) {
  const float min_x =
      std::min({triangle.xz0.x(), triangle.xz1.x(), triangle.xz2.x()});
  const float max_x =
      std::max({triangle.xz0.x(), triangle.xz1.x(), triangle.xz2.x()});
  const float min_z =
      std::min({triangle.xz0.y(), triangle.xz1.y(), triangle.xz2.y()});
  const float max_z =
      std::max({triangle.xz0.y(), triangle.xz1.y(), triangle.xz2.y()});

  const int min_sample_x = std::clamp(
      static_cast<int>(std::floor((min_x - terrain_grid_min.x()) *
                                  terrain_grid_inv_cell_size.x() - 1e-4f)),
      0, terrain_grid_width);
  const int max_sample_x = std::clamp(
      static_cast<int>(std::ceil((max_x - terrain_grid_min.x()) *
                                 terrain_grid_inv_cell_size.x() + 1e-4f)),
      0, terrain_grid_width);
  const int min_sample_z = std::clamp(
      static_cast<int>(std::floor((min_z - terrain_grid_min.y()) *
                                  terrain_grid_inv_cell_size.y() - 1e-4f)),
      0, terrain_grid_depth);
  const int max_sample_z = std::clamp(
      static_cast<int>(std::ceil((max_z - terrain_grid_min.y()) *
                                 terrain_grid_inv_cell_size.y() + 1e-4f)),
      0, terrain_grid_depth);

  const int sample_width = terrain_grid_width + 1;
  for (int sample_z = min_sample_z; sample_z <= max_sample_z; sample_z++) {
    const float z = sample_z_positions[sample_z];
    for (int sample_x = min_sample_x; sample_x <= max_sample_x; sample_x++) {
      const math::vector2 sample_xz(sample_x_positions[sample_x], z);
      float sampled_height = 0.0f;
      if (!sample_terrain_triangle(triangle, sample_xz, sampled_height)) {
        continue;
      }

      const int sample_idx =
          terrain_height_sample_index(sample_x, sample_z, sample_width);
      if (!height_valid[sample_idx] ||
          sampled_height > terrain_height_map[sample_idx]) {
        terrain_height_map[sample_idx] = sampled_height;
        height_valid[sample_idx] = 1;
      }
    }
  }
}

void fill_missing_terrain_height_samples(std::vector<float> &terrain_height_map,
                                         std::vector<char> &height_valid,
                                         int sample_width,
                                         int sample_depth) {
  std::deque<int> fill_queue;
  for (int sample_idx = 0; sample_idx < static_cast<int>(height_valid.size());
       sample_idx++) {
    if (height_valid[sample_idx]) {
      fill_queue.push_back(sample_idx);
    }
  }

  constexpr int neighbor_offsets[4][2] = {
      {-1, 0},
      {1, 0},
      {0, -1},
      {0, 1},
  };
  while (!fill_queue.empty()) {
    const int sample_idx = fill_queue.front();
    fill_queue.pop_front();

    const int sample_z = sample_idx / sample_width;
    const int sample_x = sample_idx % sample_width;
    for (const auto &offset : neighbor_offsets) {
      const int neighbor_x = sample_x + offset[0];
      const int neighbor_z = sample_z + offset[1];
      if (neighbor_x < 0 || neighbor_x >= sample_width || neighbor_z < 0 ||
          neighbor_z >= sample_depth) {
        continue;
      }

      const int neighbor_idx =
          terrain_height_sample_index(neighbor_x, neighbor_z, sample_width);
      if (height_valid[neighbor_idx]) {
        continue;
      }

      terrain_height_map[neighbor_idx] = terrain_height_map[sample_idx];
      height_valid[neighbor_idx] = 1;
      fill_queue.push_back(neighbor_idx);
    }
  }
}

bool sample_terrain_triangle(const terrain_triangle &triangle,
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

} // namespace

void camdmpp::build_terrain_sampler() {
  auto reset_terrain_sampler = [&]() {
    terrain_height_map.clear();
    terrain_grid_width = 0;
    terrain_grid_depth = 0;
    terrain_grid_min = math::vector2::Zero();
    terrain_grid_max = math::vector2::Zero();
    terrain_grid_cell_size = math::vector2::Ones();
    terrain_grid_inv_cell_size = math::vector2::Ones();
    terrain_default_height = 0.0f;
  };
  reset_terrain_sampler();

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

  std::vector<terrain_triangle> terrain_triangles;
  terrain_triangles.reserve(ground_mesh.indices.empty()
                                ? ground_mesh.vertices.size() / 3
                                : ground_mesh.indices.size() / 3);
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

  const float extent_x = std::max(terrain_grid_max.x() - terrain_grid_min.x(),
                                  terrain_min_extent);
  const float extent_z = std::max(terrain_grid_max.y() - terrain_grid_min.y(),
                                  terrain_min_extent);
  choose_terrain_height_map_resolution(extent_x, extent_z, terrain_grid_width,
                                       terrain_grid_depth);
  terrain_grid_cell_size = math::vector2(extent_x / terrain_grid_width,
                                         extent_z / terrain_grid_depth);
  terrain_grid_inv_cell_size = math::vector2(1.0f / terrain_grid_cell_size.x(),
                                             1.0f / terrain_grid_cell_size.y());

  const int sample_width = terrain_grid_width + 1;
  const int sample_depth = terrain_grid_depth + 1;
  terrain_height_map.assign(sample_width * sample_depth,
                            terrain_default_height);
  std::vector<char> height_valid(terrain_height_map.size(), 0);
  std::vector<float> sample_x_positions(sample_width, terrain_grid_min.x());
  std::vector<float> sample_z_positions(sample_depth, terrain_grid_min.y());

  for (int sample_x = 0; sample_x < sample_width; sample_x++) {
    sample_x_positions[sample_x] = terrain_grid_min.x() +
                                   terrain_grid_cell_size.x() * sample_x;
  }
  sample_x_positions.back() = terrain_grid_max.x();
  for (int sample_z = 0; sample_z < sample_depth; sample_z++) {
    sample_z_positions[sample_z] = terrain_grid_min.y() +
                                   terrain_grid_cell_size.y() * sample_z;
  }
  sample_z_positions.back() = terrain_grid_max.y();

  for (const auto &triangle : terrain_triangles) {
    rasterize_triangle_to_height_map(
        triangle, terrain_grid_min, terrain_grid_inv_cell_size,
        terrain_grid_width, terrain_grid_depth, sample_x_positions,
        sample_z_positions, terrain_height_map, height_valid);
  }

  const bool has_valid_sample =
      std::any_of(height_valid.begin(), height_valid.end(),
                  [](char is_valid) { return is_valid != 0; });
  if (!has_valid_sample) {
    reset_terrain_sampler();
    return;
  }

  fill_missing_terrain_height_samples(terrain_height_map, height_valid,
                                      sample_width, sample_depth);
}

float camdmpp::sample_terrain_height(const math::vector2 &xz,
                                     float fallback_height) const {
  if (terrain_height_map.empty() || terrain_grid_width <= 0 ||
      terrain_grid_depth <= 0) {
    return fallback_height;
  }

  const math::vector2 query_xz =
      clamp_terrain_query(xz, terrain_grid_min, terrain_grid_max);

    const float grid_x =
      (query_xz.x() - terrain_grid_min.x()) * terrain_grid_inv_cell_size.x();
    const float grid_z =
      (query_xz.y() - terrain_grid_min.y()) * terrain_grid_inv_cell_size.y();
  const int cell_x =
      clamp_grid_coord(query_xz.x(), terrain_grid_min.x(),
                       terrain_grid_cell_size.x(), terrain_grid_width);
  const int cell_z =
      clamp_grid_coord(query_xz.y(), terrain_grid_min.y(),
                       terrain_grid_cell_size.y(), terrain_grid_depth);
  const float tx = std::clamp(grid_x - static_cast<float>(cell_x), 0.0f, 1.0f);
  const float tz = std::clamp(grid_z - static_cast<float>(cell_z), 0.0f, 1.0f);

  const int sample_width = terrain_grid_width + 1;
  const float h00 = terrain_height_map[terrain_height_sample_index(
      cell_x, cell_z, sample_width)];
  const float h10 = terrain_height_map[terrain_height_sample_index(
      cell_x + 1, cell_z, sample_width)];
  const float h01 = terrain_height_map[terrain_height_sample_index(
      cell_x, cell_z + 1, sample_width)];
  const float h11 = terrain_height_map[terrain_height_sample_index(
      cell_x + 1, cell_z + 1, sample_width)];

  const float hx0 = h00 + (h10 - h00) * tx;
  const float hx1 = h01 + (h11 - h01) * tx;
  return hx0 + (hx1 - hx0) * tz;
}

float camdmpp::sample_terrain_path_length(const math::vector2 &start_xz,
                                          float start_height,
                                          const math::vector2 &direction,
                                          float planar_distance) const {
  if (planar_distance <= 1e-6f || direction.squaredNorm() <= 1e-6f)
    return 0.0f;

  constexpr int terrain_path_substeps = 4;
  float path_length = 0.0f;
  math::vector2 last_xz = start_xz;
  float last_height = start_height;

  for (int step = 1; step <= terrain_path_substeps; step++) {
    const float t = static_cast<float>(step) / terrain_path_substeps;
    const math::vector2 sample_xz =
        start_xz + direction * (planar_distance * t);
    const float sample_height = sample_terrain_height(sample_xz, last_height);
    const math::vector3 segment(sample_xz.x() - last_xz.x(),
                                sample_height - last_height,
                                sample_xz.y() - last_xz.y());
    path_length += segment.norm();
    last_xz = sample_xz;
    last_height = sample_height;
  }

  return path_length;
}

void camdmpp::resample_trajectory_on_terrain(const math::vector2 &start_xz,
                                             float start_height) {
  math::vector2 prev_flat_xz = start_xz;
  math::vector2 prev_resampled_xz = start_xz;
  float prev_resampled_height = start_height;

  for (int i = 0; i < model.future_points; i++) {
    const math::vector2 flat_center_xz(_traj_world_pos[i].x(),
                                       _traj_world_pos[i].z());
    const math::vector2 flat_step = flat_center_xz - prev_flat_xz;
    const float target_path_length = flat_step.norm();

    math::vector2 move_dir = math::vector2::Zero();
    if (target_path_length > 1e-6f) {
      move_dir = flat_step / target_path_length;
    } else {
      move_dir = math::vector2(_traj_world_dir[i].x(), _traj_world_dir[i].z());
      if (move_dir.squaredNorm() > 1e-6f)
        move_dir.normalize();
      else
        move_dir = math::vector2(0.0f, 1.0f);
    }

    float planar_step = target_path_length;
    if (target_path_length > 1e-6f) {
      float low = 0.0f;
      float high = target_path_length;
      for (int iter = 0; iter < 10; iter++) {
        const float mid = 0.5f * (low + high);
        const float mid_path_length = sample_terrain_path_length(
            prev_resampled_xz, prev_resampled_height, move_dir, mid);
        if (mid_path_length < target_path_length)
          low = mid;
        else
          high = mid;
      }
      planar_step = high;
    }

    const math::vector2 center_xz = prev_resampled_xz + move_dir * planar_step;
    math::vector3 terrain_left3 = math::world_up.cross(_traj_world_dir[i]);
    if (terrain_left3.squaredNorm() > 1e-6f)
      terrain_left3.normalize();
    else
      terrain_left3 = math::vector3(1, 0, 0);
    const math::vector2 terrain_left(terrain_left3.x(), terrain_left3.z());

    for (int j = 0; j < model.lateral_offsets_m.size(); j++) {
      const math::vector2 offset_xz = terrain_left * model.lateral_offsets_m[j];
      const math::vector2 sample_xz = center_xz + offset_xz;
      const float sample_height =
          sample_terrain_height(sample_xz, prev_resampled_height);
      _traj_world_height[i][j] = sample_height;
    }

    _traj_world_pos[i] = math::vector3(
        center_xz.x(), _traj_world_height[i][model.terrain_center_idx],
        center_xz.y());

    prev_flat_xz = flat_center_xz;
    prev_resampled_xz = center_xz;
    prev_resampled_height = _traj_world_height[i][model.terrain_center_idx];
  }
}

}; // namespace toolkit::opengl3d