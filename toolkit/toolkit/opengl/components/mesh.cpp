#include "toolkit/opengl/components/mesh.hpp"
#include "toolkit/assets/primitives.hpp"
#include "toolkit/transform.hpp"

namespace toolkit::opengl {

struct _render_vertex {
  float position[4];
  float normal[4];
  float tex_coords[4];
  float color[4];
  int bone_ids[assets::MAX_BONES_PER_MESH];
  float bone_weights[assets::MAX_BONES_PER_MESH];
};

struct _blendshape_data {
  math::vector4 offset_pos[assets::MAX_BLEND_SHAPES_PER_MESH];
  math::vector4 offset_normal[assets::MAX_BLEND_SHAPES_PER_MESH];
};

void draw_mesh_data(mesh_data &data, GLenum mode) {
  data.vertex_array.bind();
  glDrawElements(mode, data.indices.size(), GL_UNSIGNED_INT, 0);
  data.vertex_array.unbind();
}

void init_opengl_buffers(mesh_data &data) {
  std::vector<_render_vertex> vertices(data.vertices.size());
  for (int i = 0; i < data.vertices.size(); i++) {
    std::memcpy(vertices[i].position, data.vertices[i].position.data(),
                4 * sizeof(float));
    std::memcpy(vertices[i].normal, data.vertices[i].normal.data(),
                4 * sizeof(float));
    std::memcpy(vertices[i].tex_coords, data.vertices[i].tex_coords.data(),
                4 * sizeof(float));
    std::memcpy(vertices[i].color, data.vertices[i].color.data(),
                4 * sizeof(float));
    std::memcpy(vertices[i].bone_ids, data.vertices[i].bond_indices,
                assets::MAX_BONES_PER_MESH * sizeof(int));
    std::memcpy(vertices[i].bone_weights, data.vertices[i].bone_weights,
                assets::MAX_BONES_PER_MESH * sizeof(float));
  }

  data.vertex_array.create();
  data.vertex_array.bind();
  data.vertex_buffer.create();
  data.vertex_buffer.set_data_as(GL_ARRAY_BUFFER, vertices);
  data.index_buffer.create();
  data.index_buffer.set_data_as(GL_ELEMENT_ARRAY_BUFFER, data.indices);

  data.vertex_array.link_attribute(data.vertex_buffer, 0, 4, GL_FLOAT,
                                   sizeof(_render_vertex), (void *)0);
  data.vertex_array.link_attribute(data.vertex_buffer, 1, 4, GL_FLOAT,
                                   sizeof(_render_vertex),
                                   (void *)(offsetof(_render_vertex, normal)));
  data.vertex_array.link_attribute(
      data.vertex_buffer, 2, 4, GL_FLOAT, sizeof(_render_vertex),
      (void *)(offsetof(_render_vertex, tex_coords)));
  data.vertex_array.link_attribute(data.vertex_buffer, 3, 4, GL_FLOAT,
                                   sizeof(_render_vertex),
                                   (void *)(offsetof(_render_vertex, color)));

  data.vertex_array.unbind();
  data.vertex_buffer.unbind_as(GL_ARRAY_BUFFER);
  data.index_buffer.unbind_as(GL_ELEMENT_ARRAY_BUFFER);

  // setup blend shapes
  uint32_t nvertices = data.vertices.size();
  int nindices = data.indices.size();
  uint32_t blendShapeCount =
      data.blend_shapes.size() > assets::MAX_BLEND_SHAPES_PER_MESH
          ? assets::MAX_BLEND_SHAPES_PER_MESH
          : data.blend_shapes.size();
  std::vector<_blendshape_data> bs_data(nvertices);
  for (int i = 0; i < nvertices; i++) {
    for (int j = 0; j < blendShapeCount; j++) {
      for (int k = 0; k < 4; k++) {
        bs_data[i].offset_pos[j] = data.blend_shapes[j].data[i].offset_pos;
        bs_data[i].offset_normal[j] =
            data.blend_shapes[j].data[i].offset_normal;
      }
    }
  }
  data.blend_shape_buffer.create();
  data.blend_shape_weights_buffer.create();
  data.blend_shape_buffer.set_data_ssbo(bs_data);
  std::vector<float> _tmp_weights(blendShapeCount, 0.0f);
  data.blend_shape_weights_buffer.set_data_ssbo(_tmp_weights);
}

entt::entity create_cube(entt::registry &registry, math::matrix4 t) {
  auto ent = registry.create();
  auto &trans = registry.emplace<transform>(ent);
  auto &mesh = registry.emplace<mesh_data>(ent);
  trans.set_transform_matrix(t);
  trans.name = "Cube";
  mesh.indices.resize(cube_nindicies);
  for (int i = 0; i < cube_nindicies; i++)
    mesh.indices[i] = cube_indices[i];
  mesh.mesh_name = "Cube Primitive";
  mesh.model_path = "::";
  mesh.vertices.resize(cube_nvertices);
  for (int i = 0; i < cube_nvertices; i++) {
    mesh.vertices[i].position << cube_positions[i * 3],
        cube_positions[i * 3 + 1], cube_positions[i * 3 + 2], 1.0;
    mesh.vertices[i].normal << cube_normals[i * 3], cube_normals[i * 3 + 1],
        cube_normals[i * 3 + 2], 0.0;
  }
  init_opengl_buffers(mesh);
  return ent;
}
entt::entity create_plane(entt::registry &registry, math::matrix4 t) {
  auto ent = registry.create();
  auto &trans = registry.emplace<transform>(ent);
  auto &mesh = registry.emplace<mesh_data>(ent);
  trans.set_transform_matrix(t);
  trans.name = "Plane";
  mesh.indices.resize(plane_nindicies);
  for (int i = 0; i < plane_nindicies; i++)
    mesh.indices[i] = plane_indices[i];
  mesh.mesh_name = "Plane Primitive";
  mesh.model_path = "::";
  mesh.vertices.resize(plane_nvertices);
  for (int i = 0; i < plane_nvertices; i++) {
    mesh.vertices[i].position << plane_positions[i * 3],
        plane_positions[i * 3 + 1], plane_positions[i * 3 + 2], 1.0;
    mesh.vertices[i].normal << plane_normals[i * 3], plane_normals[i * 3 + 1],
        plane_normals[i * 3 + 2], 0.0;
  }
  init_opengl_buffers(mesh);
  return ent;
}
entt::entity create_sphere(entt::registry &registry, math::matrix4 t) {
  auto ent = registry.create();
  auto &trans = registry.emplace<transform>(ent);
  auto &mesh = registry.emplace<mesh_data>(ent);
  trans.set_transform_matrix(t);
  trans.name = "Sphere";
  mesh.indices.resize(sphere_nindicies);
  for (int i = 0; i < sphere_nindicies; i++)
    mesh.indices[i] = sphere_indices[i];
  mesh.mesh_name = "Sphere Primitive";
  mesh.model_path = "::";
  mesh.vertices.resize(sphere_nvertices);
  for (int i = 0; i < sphere_nvertices; i++) {
    mesh.vertices[i].position << sphere_positions[i * 3],
        sphere_positions[i * 3 + 1], sphere_positions[i * 3 + 2], 1.0;
    mesh.vertices[i].normal << sphere_normals[i * 3], sphere_normals[i * 3 + 1],
        sphere_normals[i * 3 + 2], 0.0;
  }
  init_opengl_buffers(mesh);
  return ent;
}
entt::entity create_cylinder(entt::registry &registry, math::matrix4 t) {
  auto ent = registry.create();
  auto &trans = registry.emplace<transform>(ent);
  auto &mesh = registry.emplace<mesh_data>(ent);
  trans.set_transform_matrix(t);
  trans.name = "Cylinder";
  mesh.indices.resize(cylinder_nindicies);
  for (int i = 0; i < cylinder_nindicies; i++)
    mesh.indices[i] = cylinder_indices[i];
  mesh.mesh_name = "Cylinder Primitive";
  mesh.model_path = "::";
  mesh.vertices.resize(cylinder_nvertices);
  for (int i = 0; i < cylinder_nvertices; i++) {
    mesh.vertices[i].position << cylinder_positions[i * 3],
        cylinder_positions[i * 3 + 1], cylinder_positions[i * 3 + 2], 1.0;
    mesh.vertices[i].normal << cylinder_normals[i * 3],
        cylinder_normals[i * 3 + 1], cylinder_normals[i * 3 + 2], 0.0;
  }
  init_opengl_buffers(mesh);
  return ent;
}

entt::entity create_model(entt::registry &registry, std::string filepath,
                          math::matrix4 t) {
  auto root = registry.create();
  auto &trans = registry.emplace<transform>(root);
  trans.name =
      std::filesystem::path(filepath).filename().replace_extension("").string();
  trans.set_transform_matrix(t);
  if (endswith(filepath, ".fbx") || endswith(filepath, "FBX")) {
    auto package = assets::load_fbx(filepath);
    for (auto &mesh : package.meshes) {
      auto mesh_ent = registry.create();
      auto &mesh_trans_comp = registry.emplace<transform>(mesh_ent);
      auto &mesh_data_comp = registry.emplace<mesh_data>(mesh_ent);
      mesh_trans_comp.name = mesh.mesh_name;
      mesh_trans_comp.set_transform_matrix(t);

      mesh_data_comp.model_path = filepath;
      mesh_data_comp.mesh_name = mesh.mesh_name;
      mesh_data_comp.indices = mesh.indices;
      mesh_data_comp.vertices = mesh.vertices;
      mesh_data_comp.blend_shapes = mesh.blend_shapes;

      init_opengl_buffers(mesh_data_comp);

      trans.add_children(mesh_ent);
    }
  } else if (endswith(filepath, ".OBJ") || endswith(filepath, ".obj")) {
    auto package = assets::load_obj(filepath);
    for (auto &mesh : package.meshes) {
      auto mesh_ent = registry.create();
      auto &mesh_trans_comp = registry.emplace<transform>(mesh_ent);
      auto &mesh_data_comp = registry.emplace<mesh_data>(mesh_ent);
      mesh_trans_comp.name = mesh.mesh_name;
      mesh_trans_comp.set_transform_matrix(t);

      mesh_data_comp.model_path = filepath;
      mesh_data_comp.mesh_name = mesh.mesh_name;
      mesh_data_comp.indices = mesh.indices;
      mesh_data_comp.vertices = mesh.vertices;

      init_opengl_buffers(mesh_data_comp);

      trans.add_children(mesh_ent);
    }
  }
  return root;
}

}; // namespace toolkit::opengl