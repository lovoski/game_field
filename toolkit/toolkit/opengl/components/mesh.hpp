#pragma once

#include "toolkit/opengl/base.hpp"
#include "toolkit/system.hpp"

#include "toolkit/loaders/mesh.hpp"

namespace toolkit::opengl {

struct mesh_data : public icomponent {
  std::string mesh_name, model_path;
  std::vector<assets::mesh_vertex> vertices;
  std::vector<uint32_t> indices;
  std::vector<assets::blend_shape> blend_shapes;

  vao vertex_array;
  buffer vertex_buffer, index_buffer;
  buffer blend_shape_buffer, blend_shape_weights_buffer;

  void draw_gui(iapp *app) override;

  void init1() override;
};
DECLARE_COMPONENT(mesh_data, data, mesh_name, model_path)

struct scene_mesh_data : public icomponent {
  bool should_render_mesh = true;
  math::vector3 bb_min = math::vector3::Zero(), bb_max = math::vector3::Zero();
  int scene_vertex_offset = 0, scene_index_offset = 0;
};
DECLARE_COMPONENT(scene_mesh_data, data, should_render_mesh, bb_min, bb_max,
                  scene_vertex_offset, scene_index_offset)

void draw_mesh_data(mesh_data &data, GLenum mode = GL_TRIANGLES);

entt::entity create_cube(entt::registry &registry,
                         math::matrix4 t = math::matrix4::Identity());
entt::entity create_plane(entt::registry &registry,
                          math::matrix4 t = math::matrix4::Identity());
entt::entity create_sphere(entt::registry &registry,
                           math::matrix4 t = math::matrix4::Identity());
entt::entity create_cylinder(entt::registry &registry,
                             math::matrix4 t = math::matrix4::Identity());

void open_model(entt::registry &registry, std::string filepath);

}; // namespace toolkit::opengl
