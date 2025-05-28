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
  math::vector3 bb_min = math::vector3::Zero(), bb_max = math::vector3::Zero();

  entt::entity actor_entity = entt::null;

  void draw_gui(iapp *app) override;

  void draw(GLenum mode = GL_TRIANGLES);

  void init1() override;
};
DECLARE_COMPONENT(mesh_data, data, mesh_name, model_name, should_render_mesh,
                  actor_entity)

void draw_mesh_data(mesh_data &data, GLenum mode = GL_TRIANGLES);

entt::entity create_cube(entt::registry &registry,
                         math::matrix4 t = math::matrix4::Identity());
entt::entity create_plane(entt::registry &registry,
                          math::matrix4 t = math::matrix4::Identity());
entt::entity create_sphere(entt::registry &registry,
                           math::matrix4 t = math::matrix4::Identity());
entt::entity create_cylinder(entt::registry &registry,
                             math::matrix4 t = math::matrix4::Identity());

}; // namespace toolkit::opengl
