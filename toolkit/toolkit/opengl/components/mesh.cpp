#include "toolkit/opengl/components/mesh.hpp"
#include "toolkit/anim/components/actor.hpp"
#include "toolkit/anim/scripts/vis.hpp"
#include "toolkit/assets/primitives.hpp"
#include "toolkit/transform.hpp"

namespace toolkit::opengl {

void mesh_data::draw_gui(iapp *app) {
  if (ImGui::BeginTable(("##mesh" + mesh_name).c_str(), 3,
                        ImGuiTableFlags_Resizable | ImGuiTableFlags_Borders)) {
    ImGui::TableSetupColumn("Mesh Name");
    ImGui::TableSetupColumn("Num Faces");
    ImGui::TableSetupColumn("Num Vertices");
    ImGui::TableHeadersRow();
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::Text("%s", mesh_name.c_str());
    ImGui::TableSetColumnIndex(1);
    ImGui::Text("%d", (int)(indices.size() / 3));
    ImGui::TableSetColumnIndex(2);
    ImGui::Text("%d", (int)(vertices.size()));
    ImGui::EndTable();
  }
  if (blend_shapes.size() > 0) {
    if (ImGui::TreeNode("Blend Shapes")) {
      for (auto &blend : blend_shapes) {
        ImGui::SliderFloat(blend.name.c_str(), &(blend.weight), -10.0f, 10.0f);
      }
      ImGui::TreePop();
    }
  }
}

void mesh_data::init1() {
  spdlog::warn("Mesh {0} init1 gets called", mesh_name);
}

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
    std::memcpy(vertices[i].bone_ids, data.vertices[i].bone_indices.data(),
                assets::MAX_BONES_PER_MESH * sizeof(int));
    std::memcpy(vertices[i].bone_weights, data.vertices[i].bone_weights.data(),
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

#ifdef _WIN32
#include <Windows.h>
std::string WStringToString(const std::wstring &wstr) {
  int buffer_size = WideCharToMultiByte(
      CP_UTF8,         // UTF-8 encoding for Chinese support
      0,               // No flags
      wstr.c_str(),    // Input wide string
      -1,              // Auto-detect length
      nullptr, 0,      // Null to calculate required buffer size
      nullptr, nullptr // Optional parameters (not needed here)
  );

  if (buffer_size == 0)
    return ""; // Handle error if needed

  std::string str(buffer_size, 0);
  WideCharToMultiByte(CP_UTF8, 0, wstr.c_str(), -1, &str[0], buffer_size,
                      nullptr, nullptr);
  str.pop_back(); // Remove null terminator added by -1
  return str;
}
#endif

void open_model(entt::registry &registry, std::string filepath) {
  auto models = assets::open_model(filepath);
  auto model_base = registry.create();
  auto &model_base_trans = registry.emplace<transform>(model_base);
#ifdef _WIN32
  model_base_trans.name =
      WStringToString(std::filesystem::u8path(filepath).filename().wstring());
#else
  model_base_trans.name = std::filesystem::path(filepath).filename().string();
#endif
  for (auto &model : models) {
    auto model_container = registry.create();
    auto &container_trans = registry.emplace<transform>(model_container);
    container_trans.name = model.name;
    for (auto &mesh : model.meshes) {
      auto mesh_container = registry.create();
      auto &mesh_trans = registry.emplace<transform>(mesh_container);
      auto &mesh_comp = registry.emplace<mesh_data>(mesh_container);
      mesh_trans.name = model.has_skeleton
                            ? model.skeleton.name + ":" + mesh.name
                            : mesh.name;
      mesh_comp.indices = mesh.indices;
      mesh_comp.vertices = mesh.vertices;
      mesh_comp.blend_shapes = mesh.blendshapes;
      mesh_comp.mesh_name = mesh.name;
      mesh_comp.model_path = filepath;
      init_opengl_buffers(mesh_comp);
      model_base_trans.add_children(mesh_container);
    }
    if (model.has_skeleton) {
      anim::create_actor_with_skeleton(registry, model_container,
                                       model.skeleton);
      auto &vis_script = registry.emplace<anim::vis_skeleton>(model_container);
    }
    model_base_trans.add_children(model_container);
  }
}

}; // namespace toolkit::opengl