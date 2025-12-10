#include "toolkit/opengl3d/assets.hpp"
#include "toolkit/loaders/imp.hpp"
#include <fstream>
#include <iostream>
#include <sstream>

namespace toolkit::assets {

#ifdef _WIN32
#include <Windows.h>
std::string wstring_to_string(const std::wstring &wstr) {
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

bool load_obj_mesh(const std::string &filepath, std::vector<mesh> &out_meshes) {
  tinyobj::attrib_t attrib;
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;
  std::string warn, err;

  // Load OBJ file
  if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err,
                        filepath.c_str())) {
    if (!err.empty()) {
      std::cerr << "TinyOBJ error: " << err << std::endl;
    }
    return false;
  }

  if (!warn.empty()) {
    std::cout << "TinyOBJ warning: " << warn << std::endl;
  }

  // Clear output meshes
  out_meshes.clear();

  // Extract filename base as prefix for mesh names
  size_t last_slash = filepath.find_last_of("/\\");
  size_t last_dot = filepath.find_last_of(".");
  std::string base_name;
  if (last_slash != std::string::npos) {
    base_name = filepath.substr(last_slash + 1, last_dot - last_slash - 1);
  } else {
    base_name = filepath.substr(0, last_dot);
  }

  // Process each shape as a separate mesh
  for (size_t shape_idx = 0; shape_idx < shapes.size(); shape_idx++) {
    const auto &shape = shapes[shape_idx];
    mesh out_mesh;

    // Set mesh name from shape name or generate from base name
    if (!shape.name.empty()) {
      out_mesh.name = shape.name;
    } else {
      out_mesh.name = base_name + "_shape_" + std::to_string(shape_idx);
    }

    // Create vertices and collect indices for this shape
    std::unordered_map<std::string, uint32_t> vertex_cache;

    for (const auto &index : shape.mesh.indices) {
      // Create a key for vertex deduplication (position + normal + texcoord)
      std::ostringstream key;
      key << index.vertex_index << "_" << index.normal_index << "_"
          << index.texcoord_index;
      std::string vertex_key = key.str();

      uint32_t vertex_idx;
      auto it = vertex_cache.find(vertex_key);

      if (it != vertex_cache.end()) {
        // Vertex already exists, reuse it
        vertex_idx = it->second;
      } else {
        // Create new vertex
        mesh_vertex vertex;
        vertex.position = math::vector4::Zero();
        vertex.normal = math::vector4::Zero();
        vertex.tex_coords = math::vector4::Zero();
        vertex.color = math::vector4::Ones();
        vertex.bone_indices = {-1, -1, -1, -1};
        vertex.bone_weights = {0.0f, 0.0f, 0.0f, 0.0f};

        // Position
        if (index.vertex_index >= 0) {
          int vi = index.vertex_index * 3;
          vertex.position.x() = attrib.vertices[vi];
          vertex.position.y() = attrib.vertices[vi + 1];
          vertex.position.z() = attrib.vertices[vi + 2];
          vertex.position.w() = 1.0f;
        }

        // Normal
        if (index.normal_index >= 0) {
          int ni = index.normal_index * 3;
          vertex.normal.x() = attrib.normals[ni];
          vertex.normal.y() = attrib.normals[ni + 1];
          vertex.normal.z() = attrib.normals[ni + 2];
          vertex.normal.w() = 0.0f;
        }

        // Texture coordinates
        if (index.texcoord_index >= 0) {
          int ti = index.texcoord_index * 2;
          vertex.tex_coords.x() = attrib.texcoords[ti];
          vertex.tex_coords.y() = attrib.texcoords[ti + 1];
        }

        // Colors (if available)
        if (index.vertex_index >= 0 && !attrib.colors.empty()) {
          int ci = index.vertex_index * 3;
          if (ci + 2 < attrib.colors.size()) {
            vertex.color.x() = attrib.colors[ci];
            vertex.color.y() = attrib.colors[ci + 1];
            vertex.color.z() = attrib.colors[ci + 2];
          }
        }

        vertex_idx = out_mesh.vertices.size();
        out_mesh.vertices.push_back(vertex);
        vertex_cache[vertex_key] = vertex_idx;
      }

      out_mesh.indices.push_back(vertex_idx);
    }

    int num_triangles = out_mesh.indices.size() / 3;
    std::vector<math::vector3> normal_cache(out_mesh.vertices.size(),
                                            math::vector3::Zero());
    std::vector<int> normal_count(out_mesh.vertices.size(), 0);
    for (int i = 0; i < num_triangles; i++) {
      int vid0 = out_mesh.indices[3 * i], vid1 = out_mesh.indices[3 * i + 1],
          vid2 = out_mesh.indices[3 * i + 2];
      math::vector3 vp0 = out_mesh.vertices[vid0].position.head<3>(),
                    vp1 = out_mesh.vertices[vid1].position.head<3>(),
                    vp2 = out_mesh.vertices[vid2].position.head<3>();
      math::vector3 e01 = (vp1 - vp0).normalized(),
                    e12 = (vp2 - vp1).normalized();
      math::vector3 fn = (e01.cross(e12)).normalized();
      normal_cache[vid0] += fn;
      normal_count[vid0] += 1;
      normal_cache[vid1] += fn;
      normal_count[vid1] += 1;
      normal_cache[vid2] += fn;
      normal_count[vid2] += 1;
    }
    // use smoothed normal if no normal specified in the file
    for (int i = 0; i < out_mesh.vertices.size(); i++) {
      if (out_mesh.vertices[i].normal.norm() == 0) {
        out_mesh.vertices[i].normal << (normal_cache[i] / normal_count[i]),
            0.0f;
      }
    }

    // Add completed mesh to output
    out_meshes.push_back(out_mesh);
  }

  return true;
}

bool save_obj_mesh(const std::string &filepath, const mesh &in_mesh) {
  std::ofstream file(filepath);
  if (!file.is_open()) {
    std::cerr << "Failed to open file for writing: " << filepath << std::endl;
    return false;
  }

  // Write header comment
  file << ("# Exported mesh: " + in_mesh.name + "\n");
  file << ("# Vertices: " + std::to_string(in_mesh.vertices.size()) + "\n");
  file << ("# Faces: " + std::to_string(in_mesh.indices.size() / 3) + "\n\n");

  // Write vertices
  for (size_t i = 0; i < in_mesh.vertices.size(); i++) {
    const auto &vertex = in_mesh.vertices[i];
    // Vertex position
    file << str_format("v %.6f %.6f %.6f\n", vertex.position.x(),
                       vertex.position.y(), vertex.position.z());
  }
  file << "\n";

  // Write texture coordinates (if non-zero)
  bool has_texcoords = false;
  for (const auto &vertex : in_mesh.vertices) {
    if (vertex.tex_coords.x() != 0.0f || vertex.tex_coords.y() != 0.0f) {
      has_texcoords = true;
      break;
    }
  }

  if (has_texcoords) {
    for (const auto &vertex : in_mesh.vertices) {
      file << str_format("vt %.6f %.6f\n", vertex.tex_coords.x(),
                         vertex.tex_coords.y());
    }
    file << "\n";
  }

  // Write normals
  for (const auto &vertex : in_mesh.vertices) {
    file << str_format("vn %.6f %.6f %.6f\n", vertex.normal.x(),
                       vertex.normal.y(), vertex.normal.z());
  }
  file << "\n";

  // Write faces (indices)
  for (size_t i = 0; i < in_mesh.indices.size(); i += 3) {
    if (i + 2 >= in_mesh.indices.size())
      break;

    uint32_t i1 = in_mesh.indices[i] + 1; // OBJ indices are 1-based
    uint32_t i2 = in_mesh.indices[i + 1] + 1;
    uint32_t i3 = in_mesh.indices[i + 2] + 1;

    if (has_texcoords) {
      file << str_format("f %d/%d/%d %d/%d/%d %d/%d/%d\n", i1, i1, i1, i2, i2,
                         i2, i3, i3, i3);
    } else {
      file << str_format("f %d//%d %d//%d %d//%d\n", i1, i1, i2, i2, i3, i3);
    }
  }

  file.close();
  return true;
}

}; // namespace toolkit::assets
