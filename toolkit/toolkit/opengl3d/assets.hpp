#pragma once

#include "toolkit/math.hpp"
#include "toolkit/reflect.hpp"

#include "toolkit/system.hpp"
#include "toolkit/utils.hpp"

namespace toolkit::assets {

struct blend_shape_vertex {
  toolkit::math::vector4 offset_pos;
  toolkit::math::vector4 offset_normal;
};
struct blend_shape {
  float weight;
  std::string name;
  std::vector<blend_shape_vertex> verts;
};
struct mesh_vertex {
  toolkit::math::vector4 position;
  toolkit::math::vector4 normal;
  toolkit::math::vector4 tex_coords;
  toolkit::math::vector4 color;
  std::array<int, 4> bone_indices;
  std::array<float, 4> bone_weights;
};
struct mesh {
  std::string name;
  std::vector<unsigned int> indices;
  std::vector<mesh_vertex> vertices;
  std::vector<blend_shape> blendshapes;
};

// Load meshes from OBJ file using tinyobj loader (each shape becomes a separate mesh)
bool load_obj_mesh(const std::string& filepath, std::vector<mesh>& out_meshes);

// Save mesh to OBJ file
bool save_obj_mesh(const std::string& filepath, const mesh& in_mesh);

#ifdef _WIN32
std::string wstring_to_string(const std::wstring &wstr);
#endif

entt::entity open_model_ufbx(entt::registry &registry, std::string filepath);
entt::entity open_model_assimp(entt::registry &registry, std::string filepath);

}; // namespace toolkit::assets
