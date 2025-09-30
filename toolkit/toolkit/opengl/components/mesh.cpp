#include "toolkit/opengl/components/mesh.hpp"
#include "toolkit/anim/components/actor.hpp"
#include "toolkit/anim/scripts/vis.hpp"
#include "toolkit/assets/primitives.hpp"
#include "toolkit/opengl/gui/utils.hpp"
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

template <typename T>
void read_vector(std::istringstream &in, std::vector<T> &vec) {
  std::size_t size = 0;
  in.read(reinterpret_cast<char *>(&size), sizeof(size));
  if (size > 0) {
    vec.resize(size);
    in.read(reinterpret_cast<char *>(vec.data()), size * sizeof(T));
  } else {
    vec.clear();
  }
}
template <typename T>
std::size_t write_vector(std::ostringstream &out, const std::vector<T> &vec) {
  std::size_t size = vec.size(),
              bytes_count = 0; // Use uint64_t for size to be safe
  out.write(reinterpret_cast<const char *>(&size), sizeof(size));
  bytes_count += sizeof(size);
  if (size > 0) {
    out.write(reinterpret_cast<const char *>(vec.data()), size * sizeof(T));
    bytes_count += (size * sizeof(T));
  }
  return bytes_count;
}
void read_blendshape_data(std::vector<char> &data,
                          std::vector<_blendshape_data> &vec) {
  std::size_t size = 0;
  std::istringstream ss(
      std::string(reinterpret_cast<const char *>(data.data()), data.size()),
      std::ios::binary);
  ss.read(reinterpret_cast<char *>(&size), sizeof(size));
  vec.resize(size);
  for (int i = 0; i < size; i++) {
    ss.read(vec[i].name, sizeof(vec[i].name));
    ss.read(reinterpret_cast<char *>(&vec[i].weight), sizeof(vec[i].weight));
    read_vector(ss, vec[i].vertices);
  }
}
std::tuple<std::string, std::size_t>
write_blendshape_data(std::vector<_blendshape_data> &vec) {
  std::ostringstream ss(std::ios::binary);
  std::size_t size = vec.size(), bytes_count = 0;
  ss.write(reinterpret_cast<const char *>(&size), sizeof(size));
  bytes_count += sizeof(size);
  for (int i = 0; i < size; i++) {
    ss.write(vec[i].name, sizeof(vec[i].name));
    ss.write(reinterpret_cast<const char *>(&vec[i].weight),
             sizeof(vec[i].weight));
    bytes_count += write_vector(ss, vec[i].vertices);
  }
  return {ss.str(), bytes_count};
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
  blendshapes.resize(data.blendshapes.size());
  for (int i = 0; i < data.blendshapes.size(); i++) {
    blendshapes[i].vertices.resize(data.blendshapes[i].verts.size());
    for (int j = 0; j < data.blendshapes[i].verts.size(); j++) {
      std::memcpy(blendshapes[i].vertices[j].offset_pos,
                  data.blendshapes[i].verts[j].offset_pos.data(),
                  4 * sizeof(float));
      std::memcpy(blendshapes[i].vertices[j].offset_normal,
                  data.blendshapes[i].verts[j].offset_normal.data(),
                  4 * sizeof(float));
    }
    blendshapes[i].weight = data.blendshapes[i].weight;
    std::strcpy(blendshapes[i].name, data.blendshapes[i].name.c_str());
  }
}

void init_opengl_buffers_internal(mesh_data &data,
                                  std::vector<_render_vertex> &vertices,
                                  std::vector<_blendshape_data> &blendshapes);

void mesh_data::draw_gui(iapp *app) {
  if (ImGui::BeginTable(("##mesh" + mesh_name).c_str(), 2,
                        ImGuiTableFlags_Resizable | ImGuiTableFlags_Borders)) {
    ImGui::TableSetupColumn("Property");
    ImGui::TableSetupColumn("Value");
    ImGui::TableHeadersRow();

    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::Text("Mesh Name");
    ImGui::TableSetColumnIndex(1);
    ImGui::Text("%s", mesh_name.c_str());

    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::Text("Num Vertices");
    ImGui::TableSetColumnIndex(1);
    ImGui::Text("%d", vertices.size());

    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::Text("Num Indices");
    ImGui::TableSetColumnIndex(1);
    ImGui::Text("%d", indices.size());

    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::Text("Num Faces");
    ImGui::TableSetColumnIndex(1);
    ImGui::Text("%d", indices.size() / 3);

    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::Text("Skinned Mesh");
    ImGui::TableSetColumnIndex(1);
    ImGui::Text(skinned ? "True" : "False");

    ImGui::EndTable();
  }
  if (blendshapes.size() > 0) {
    if (ImGui::TreeNode("Blend Shapes")) {
      for (auto &blend : blendshapes) {
        ImGui::DragFloat(blend.name.c_str(), &(blend.weight), 0.001f, -1e5,
                         1e5);
      }
      ImGui::TreePop();
    }
  }
  if (ImGui::TreeNode("Default Material")) {
    gui::color_edit_3("albedo", material.albedo);
    ImGui::Checkbox("wireframe", &material.wireframe);
    ImGui::DragFloat("wireframe width", &material.wireframe_width, 0.01f, 0.0f,
                     1.0);
    ImGui::DragFloat("wireframe smooth", &material.wireframe_smooth, 0.01f,
                     0.0f, 1.0);
    ImGui::TreePop();
  }
}

nlohmann::json mesh_data::late_serialize() {
  // save vertices, indices and blendshapes as compressed base64 string
  nlohmann::json data;

  std::vector<_render_vertex> save_vertices;
  std::vector<_blendshape_data> save_blendshapes;
  prepare_plain_data(*this, save_vertices, save_blendshapes);

  std::ostringstream ss0(std::ios::binary);
  auto ss0_size = write_vector(ss0, save_vertices);
  auto ss0_str = ss0.str();

  std::ostringstream ss1(std::ios::binary);
  auto ss1_size = write_vector(ss1, indices);
  auto ss1_str = ss1.str();

  auto [ss2_str, ss2_size] = write_blendshape_data(save_blendshapes);

  auto ss0_compressed = compress_bytes(ss0_str.data(), ss0_size);
  auto ss1_compressed = compress_bytes(ss1_str.data(), ss1_size);
  auto ss2_compressed = compress_bytes(ss2_str.data(), ss2_size);

  auto ss0_base64 = base64_encode(ss0_compressed.data(), ss0_compressed.size());
  auto ss1_base64 = base64_encode(ss1_compressed.data(), ss1_compressed.size());
  auto ss2_base64 = base64_encode(ss2_compressed.data(), ss2_compressed.size());

  data["vertices"] = ss0_base64;
  data["indices"] = ss1_base64;
  data["blendshapes"] = ss2_base64;

  return data;
}

void mesh_data::late_deserialize(nlohmann::json &data) {
  if (!data.is_null()) {
    std::string ss0_base64 = data["vertices"];
    std::string ss1_base64 = data["indices"];
    std::string ss2_base64 = data["blendshapes"];

    auto ss0_base64_decode = base64_decode(ss0_base64);
    auto ss1_base64_decode = base64_decode(ss1_base64);
    auto ss2_base64_decode = base64_decode(ss2_base64);

    auto ss0_decompressed =
        decompress_bytes(ss0_base64_decode.data(), ss0_base64_decode.size());
    auto ss1_decompressed =
        decompress_bytes(ss1_base64_decode.data(), ss1_base64_decode.size());
    auto ss2_decompressed =
        decompress_bytes(ss2_base64_decode.data(), ss2_base64_decode.size());

    std::vector<_render_vertex> data_vertices;
    std::vector<_blendshape_data> data_blendshapes;

    std::string ss0_str(ss0_decompressed.begin(), ss0_decompressed.end());
    std::istringstream ss0(ss0_str, std::ios::binary);
    read_vector(ss0, data_vertices);

    std::string ss1_str(ss1_decompressed.begin(), ss1_decompressed.end());
    std::istringstream ss1(ss1_str, std::ios::binary);
    read_vector(ss1, indices);

    read_blendshape_data(ss2_decompressed, data_blendshapes);

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
    blendshapes.resize(data_blendshapes.size());
    for (int i = 0; i < data_blendshapes.size(); i++) {
      blendshapes[i].weight = data_blendshapes[i].weight;
      blendshapes[i].name = data_blendshapes[i].name;
      blendshapes[i].verts.resize(data_blendshapes[i].vertices.size());
      for (int j = 0; j < data_blendshapes[i].vertices.size(); j++) {
        blendshapes[i].verts[j].offset_pos
            << data_blendshapes[i].vertices[j].offset_pos[0],
            data_blendshapes[i].vertices[j].offset_pos[1],
            data_blendshapes[i].vertices[j].offset_pos[2],
            data_blendshapes[i].vertices[j].offset_pos[3];
        blendshapes[i].verts[j].offset_normal
            << data_blendshapes[i].vertices[j].offset_normal[0],
            data_blendshapes[i].vertices[j].offset_normal[1],
            data_blendshapes[i].vertices[j].offset_normal[2],
            data_blendshapes[i].vertices[j].offset_normal[3];
      }
    }

    init_opengl_buffers_internal(*this, data_vertices, data_blendshapes);
  } else {
    spdlog::error("Can't load mesh data from null late_deserialize");
  }
}

void mesh_data::draw(GLenum mode) { draw_mesh_data(*this, mode); }

void draw_mesh_data(mesh_data &data, GLenum mode) {
  data.vertex_array.bind();
  glDrawElements(mode, data.indices.size(), GL_UNSIGNED_INT, 0);
  data.vertex_array.unbind();
}

void mesh_data::update_buffers() {
  force_update_flag = true;
  init_opengl_buffers(*this);
}

void init_opengl_buffers_internal(mesh_data &data,
                                  std::vector<_render_vertex> &vertices,
                                  std::vector<_blendshape_data> &blendshapes) {
  // prepare default bounding box
  for (int i = 0; i < data.vertices.size(); i++) {
    data.bb_max = math::max3(data.bb_max, data.vertices[i].position.head<3>());
    data.bb_min = math::min3(data.bb_min, data.vertices[i].position.head<3>());
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

  if (blendshapes.size() > 0) {
    data.blendshape_targets.resize(blendshapes.size());
    for (int i = 0; i < blendshapes.size(); i++) {
      data.blendshape_targets[i].create();
      data.blendshape_targets[i].set_data_ssbo(blendshapes[i].vertices,
                                               GL_STATIC_DRAW);
    }
  }
}

/**
 * Setup opengl buffer for a mesh, serialize the data into a .mesh text file
 * relative to output binary.
 */
void init_opengl_buffers(mesh_data &data) {
  std::vector<_render_vertex> vertices;
  std::vector<_blendshape_data> blendshapes;
  prepare_plain_data(data, vertices, blendshapes);
  init_opengl_buffers_internal(data, vertices, blendshapes);
}

entt::entity create_cube(entt::registry &registry, math::matrix4 t) {
  auto ent = registry.create();
  auto &trans = registry.emplace<transform>(ent);
  auto &mesh = registry.emplace<mesh_data>(ent);
  trans.set_world_transform(t);
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
  trans.set_world_transform(t);
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
  trans.set_world_transform(t);
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
  trans.set_world_transform(t);
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

void skinned_mesh_bundle::try_setup() {
  if (!gl_initialized) {
    shadowmap_fb.create();
    shadowmap_fb.bind();
    shadowmap_depth.create(GL_TEXTURE_2D);
    shadowmap_depth.set_data(4096, 4096, GL_DEPTH_COMPONENT24,
                             GL_DEPTH_COMPONENT, GL_FLOAT);
    shadowmap_depth.set_parameters(
        {{GL_TEXTURE_MIN_FILTER, GL_LINEAR},
         {GL_TEXTURE_MAG_FILTER, GL_LINEAR},
         {GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE},
         {GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL},
         {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
         {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});
    shadowmap_fb.attach_depth_buffer(shadowmap_depth);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    if (!shadowmap_fb.check_status())
      spdlog::error("skinned mesh bundle shadow buffer not complete!");
    shadowmap_fb.unbind();
    gl_initialized = true;
  }
}

}; // namespace toolkit::opengl