/**
 * This is the loading library for meshes, currently, it supports:
 * .obj format: commonly used for geometry processing models
 * .fbx format: commonly used in industry applications
 *
 * Blend shapes, skeleton bindings can be loaded from .fbx file if there's any.
 *
 * The vertices and indices are stored in a gpu-friendly format, it's also quite
 * straight forward to convert these data into a mesh processing friendly
 * version.
 */

#pragma once

#include "toolkit/math.hpp"
#include "toolkit/loaders/motion.hpp"

namespace toolkit::assets {

constexpr uint32_t MAX_BONES_PER_MESH = 4;
constexpr uint32_t MAX_BLEND_SHAPES_PER_MESH = 52;

struct blend_shape_vertex {
  toolkit::math::vector4 offset_pos;
  toolkit::math::vector4 offset_normal;
};
struct blend_shape {
  float weight;
  std::string name;
  std::vector<blend_shape_vertex> data;
};
struct mesh_vertex {
  toolkit::math::vector4 position;
  toolkit::math::vector4 normal;
  toolkit::math::vector4 tex_coords;
  toolkit::math::vector4 color;
  int bond_indices[MAX_BONES_PER_MESH];
  float bone_weights[MAX_BONES_PER_MESH];
};
struct mesh {
  std::string mesh_name, model_path;
  std::vector<mesh_vertex> vertices;
  std::vector<uint32_t> indices;
  std::vector<blend_shape> blend_shapes;
  std::shared_ptr<skeleton> skeleton = nullptr;
};

struct fbx_package {
  std::string model_path;
  std::vector<mesh> meshes;
  std::shared_ptr<skeleton> skeleton = nullptr;
  std::shared_ptr<motion> motion = nullptr;
};

struct obj_package {
  std::string model_path;
  std::vector<mesh> meshes;
};

fbx_package load_fbx(std::string path);
obj_package load_obj(std::string path);

}; // namespace Common::Assets