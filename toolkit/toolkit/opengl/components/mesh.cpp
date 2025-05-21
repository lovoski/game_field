#include "toolkit/opengl/components/mesh.hpp"
#include "toolkit/anim/components/actor.hpp"
#include "toolkit/anim/scripts/vis.hpp"
#include "toolkit/assets/primitives.hpp"
#include "toolkit/transform.hpp"

namespace toolkit::opengl {

struct _render_vertex {
  float position[4];
  float normal[4];
  float tex_coords[4];
  float color[4];
  int bone_ids[4];
  float bone_weights[4];
};

struct _blendshape_vertex {
  float offset_pos[4];
  float offset_normal[4];
};

struct _blendshape_data {
  char name[1024];
  float weight;
  std::vector<_blendshape_vertex> vertices;
};

template <typename T> void read_vector(std::ifstream &in, std::vector<T> &vec) {
  uint64_t size = 0;
  in.read(reinterpret_cast<char *>(&size), sizeof(size));
  if (size > 0) {
    vec.resize(size);
    in.read(reinterpret_cast<char *>(vec.data()), size * sizeof(T));
  } else {
    vec.clear();
  }
}
template <typename T>
void write_vector(std::ofstream &out, const std::vector<T> &vec) {
  uint64_t size = vec.size(); // Use uint64_t for size to be safe
  out.write(reinterpret_cast<const char *>(&size), sizeof(size));
  if (size > 0) {
    out.write(reinterpret_cast<const char *>(vec.data()), size * sizeof(T));
  }
}
void read_blendshape_data(std::ifstream &in,
                          std::vector<_blendshape_data> &vec) {
  uint64_t size = 0;
  in.read(reinterpret_cast<char *>(&size), sizeof(size));
  vec.resize(size);
  for (int i = 0; i < size; i++) {
    in.read(vec[i].name, sizeof(vec[i].name));
    in.read(reinterpret_cast<char *>(&vec[i].weight), sizeof(vec[i].weight));
    read_vector(in, vec[i].vertices);
  }
}
void write_blendshape_data(std::ofstream &out,
                           std::vector<_blendshape_data> &vec) {
  uint64_t size = vec.size();
  out.write(reinterpret_cast<const char *>(&size), sizeof(size));
  for (int i = 0; i < size; i++) {
    out.write(vec[i].name, sizeof(vec[i].name));
    out.write(reinterpret_cast<const char *>(&vec[i].weight),
              sizeof(vec[i].weight));
    write_vector(out, vec[i].vertices);
  }
}

void prepare_plain_data(mesh_data &data, std::vector<_render_vertex> &vertices,
                        std::vector<_blendshape_data> &blendshapes) {
  vertices.resize(data.vertices.size());
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
                4 * sizeof(int));
    std::memcpy(vertices[i].bone_weights, data.vertices[i].bone_weights.data(),
                4 * sizeof(float));
  }
  blendshapes.resize(data.blend_shapes.size());
  for (int i = 0; i < data.blend_shapes.size(); i++) {
    blendshapes[i].vertices.resize(data.blend_shapes[i].data.size());
    for (int j = 0; j < data.blend_shapes[i].data.size(); j++) {
      std::memcpy(blendshapes[i].vertices[j].offset_pos,
                  data.blend_shapes[i].data[j].offset_pos.data(),
                  4 * sizeof(float));
      std::memcpy(blendshapes[i].vertices[j].offset_normal,
                  data.blend_shapes[i].data[j].offset_normal.data(),
                  4 * sizeof(float));
    }
    blendshapes[i].weight = data.blend_shapes[i].weight;
    std::strcpy(blendshapes[i].name, data.blend_shapes[i].name.c_str());
  }
}

void init_opengl_buffers(mesh_data &data, bool save_asset = true);

void init_opengl_buffers_internal(mesh_data &data,
                                  std::vector<_render_vertex> &vertices,
                                  std::vector<_blendshape_data> &blendshapes,
                                  bool save_asset);

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
  std::string asset_path = join_path(
      ".", "mesh_assets",
      str_format("%s_%s.mesh", model_name.c_str(), mesh_name.c_str()));
  asset_path = replace(replace(asset_path, ":", ""), " ", "");
  std::ifstream input(asset_path, std::ios::binary);
  if (input.is_open()) {
    spdlog::info("Load mesh {0} from path {1}", mesh_name, asset_path);
    std::vector<_render_vertex> data_vertices;
    std::vector<_blendshape_data> data_blendshapes;
    read_vector(input, data_vertices);
    read_vector(input, indices);
    read_blendshape_data(input, data_blendshapes);

    vertices.resize(data_vertices.size());
    for (int i = 0; i < data_vertices.size(); i++) {
      std::memcpy(vertices[i].position.data(), data_vertices[i].position,
                  4 * sizeof(float));
      std::memcpy(vertices[i].normal.data(), data_vertices[i].normal,
                  4 * sizeof(float));
      std::memcpy(vertices[i].tex_coords.data(), data_vertices[i].tex_coords,
                  4 * sizeof(float));
      std::memcpy(vertices[i].color.data(), data_vertices[i].color,
                  4 * sizeof(float));
      std::memcpy(vertices[i].bone_indices.data(), data_vertices[i].bone_ids,
                  4 * sizeof(int));
      std::memcpy(vertices[i].bone_weights.data(),
                  data_vertices[i].bone_weights, 4 * sizeof(float));
    }
    blend_shapes.resize(data_blendshapes.size());
    for (int i = 0; i < data_blendshapes.size(); i++) {
      blend_shapes[i].weight = data_blendshapes[i].weight;
      blend_shapes[i].name = data_blendshapes[i].name;
      blend_shapes[i].data.resize(data_blendshapes[i].vertices.size());
      for (int j = 0; j < data_blendshapes[i].vertices.size(); j++) {
        blend_shapes[i].data[j].offset_pos
            << data_blendshapes[i].vertices[j].offset_pos[0],
            data_blendshapes[i].vertices[j].offset_pos[1],
            data_blendshapes[i].vertices[j].offset_pos[2],
            data_blendshapes[i].vertices[j].offset_pos[3];
        blend_shapes[i].data[j].offset_normal
            << data_blendshapes[i].vertices[j].offset_normal[0],
            data_blendshapes[i].vertices[j].offset_normal[1],
            data_blendshapes[i].vertices[j].offset_normal[2],
            data_blendshapes[i].vertices[j].offset_normal[3];
      }
    }

    init_opengl_buffers_internal(*this, data_vertices, data_blendshapes, false);
  } else {
    spdlog::error("Failed to load mesh {0} from path {1}", mesh_name,
                  asset_path);
  }
  input.close();
}

void draw_mesh_data(mesh_data &data, GLenum mode) {
  data.vertex_array.bind();
  glDrawElements(mode, data.indices.size(), GL_UNSIGNED_INT, 0);
  data.vertex_array.unbind();
}

void init_opengl_buffers_internal(mesh_data &data,
                                  std::vector<_render_vertex> &vertices,
                                  std::vector<_blendshape_data> &blendshapes,
                                  bool save_asset) {
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

  if (save_asset) {
    // save data into text file relative to binary file
    mkdir(join_path(".", "mesh_assets"));
    std::string asset_path =
        join_path(".", "mesh_assets",
                  str_format("%s_%s.mesh", data.model_name.c_str(),
                             data.mesh_name.c_str()));
    asset_path = replace(replace(asset_path, ":", ""), " ", "");
    std::ofstream output(asset_path, std::ios::binary);
    if (output.is_open()) {
      write_vector(output, vertices);
      write_vector(output, data.indices);
      write_blendshape_data(output, blendshapes);
      spdlog::info("Save mesh asset at path {0}", asset_path);
    } else {
      spdlog::error("Failed to create mesh asset at path {0}", asset_path);
    }
    output.close();
  }
}

/**
 * Setup opengl buffer for a mesh, serialize the data into a .mesh text file
 * relative to output binary.
 */
void init_opengl_buffers(mesh_data &data, bool save_asset) {
  std::vector<_render_vertex> vertices;
  std::vector<_blendshape_data> blendshapes;
  prepare_plain_data(data, vertices, blendshapes);
  init_opengl_buffers_internal(data, vertices, blendshapes, save_asset);
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
  mesh.model_name = "__internal__";
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
  mesh.model_name = "__internal__";
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
  mesh.model_name = "__internal__";
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
  mesh.model_name = "__internal__";
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
      mesh_comp.model_name = model.name;
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