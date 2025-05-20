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
  std::array<int, MAX_BONES_PER_MESH> bone_indices;
  std::array<float, MAX_BONES_PER_MESH> bone_weights;
};
struct mesh {
  std::string name;
  std::vector<unsigned int> indices;
  std::vector<mesh_vertex> vertices;
  std::vector<blend_shape> blendshapes;
};
struct model {
  std::string name;
  skeleton skeleton;
  std::vector<mesh> meshes;
  bool has_skeleton = false;
};

std::vector<model> open_model(std::string filepath);

}; // namespace Common::Assets