#pragma once

#include "toolkit/opengl/base.hpp"
#include "toolkit/system.hpp"

#include "toolkit/loaders/mesh.hpp"

namespace toolkit::opengl {

struct mesh_data {
  std::string mesh_name, model_path;
  std::vector<assets::mesh_vertex> vertices;
  std::vector<uint32_t> indices;
  std::vector<assets::blend_shape> blend_shapes;

  vao vertex_array;
  buffer vertex_buffer, index_buffer;
  buffer blend_shape_buffer, blend_shape_weights_buffer;
};
DECLARE_COMPONENT(mesh_data, data, mesh_name, model_path)

void draw_mesh_data(mesh_data &data, GLenum mode = GL_TRIANGLES);

entt::entity create_cube(entt::registry &registry,
                         math::matrix4 t = math::matrix4::Identity());
// entt::entity create_plane(entt::registry &registry,
//                           math::matrix4 t = math::matrix4::Identity());
// entt::entity create_sphere(entt::registry &registry,
//                            math::matrix4 t = math::matrix4::Identity());
// entt::entity create_cylinder(entt::registry &registry,
//                              math::matrix4 t = math::matrix4::Identity());
// entt::entity create_cone(entt::registry &registry,
//                          math::matrix4 t = math::matrix4::Identity());

// entt::entity create_model(entt::registry &registry, std::string filepath,
//                           math::matrix4 t = math::matrix4::Identity());

}; // namespace toolkit::opengl
