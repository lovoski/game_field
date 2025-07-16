#pragma once

#include "toolkit/opengl/base.hpp"
#include "toolkit/system.hpp"

#include "toolkit/loaders/mesh.hpp"

namespace toolkit::opengl {

struct mesh_data : public icomponent {
  std::string mesh_name, model_name;
  std::vector<assets::mesh_vertex> vertices;
  std::vector<uint32_t> indices;
  std::vector<assets::blend_shape> blend_shapes;

  vao vertex_array;
  buffer vertex_buffer, index_buffer;
  std::vector<buffer> blendshape_targets;

  bool should_render_mesh = true;
  int64_t scene_vertex_offset = 0, scene_index_offset = 0;

  // axis aligned bounding box with identity transform
  math::vector3 bb_min = math::vector3::Zero(), bb_max = math::vector3::Zero();

  bool skinned = false;

  void draw_gui(iapp *app) override;

  void draw(GLenum mode = GL_TRIANGLES);

  void init1() override;

  bool force_update_flag = false;
  void update_buffers(bool save_assets = false);
};
DECLARE_COMPONENT(mesh_data, data, mesh_name, model_name, should_render_mesh)

struct skinned_mesh_bundle : public icomponent {
  std::vector<entt::entity> bone_entities, mesh_entities;
};
DECLARE_COMPONENT(skinned_mesh_bundle, data, bone_entities, mesh_entities)

void init_opengl_buffers(mesh_data &data, bool save_asset = true);

void draw_mesh_data(mesh_data &data, GLenum mode = GL_TRIANGLES);

entt::entity create_cube(entt::registry &registry,
                         math::matrix4 t = math::matrix4::Identity());
entt::entity create_plane(entt::registry &registry,
                          math::matrix4 t = math::matrix4::Identity());
entt::entity create_sphere(entt::registry &registry,
                           math::matrix4 t = math::matrix4::Identity());
entt::entity create_cylinder(entt::registry &registry,
                             math::matrix4 t = math::matrix4::Identity());

/**
 * Get convex hull of given mesh as triangulated mesh.
 */
void convex_hull(mesh_data &data, std::vector<assets::mesh_vertex> &vertices,
                 std::vector<uint32_t> &indices);

/**
 * Recompute smoothed normal given mesh data.
 */
void smooth_normal(toolkit::opengl::mesh_data &data);

}; // namespace toolkit::opengl
