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

struct BlendShapeVertex {
  toolkit::math::vector4 PosOffset;
  toolkit::math::vector4 NormalOffset;
};
struct BlendShape {
  float weight;
  std::string name;
  std::vector<BlendShapeVertex> data;
};
struct MeshVertex {
  toolkit::math::vector4 Position;
  toolkit::math::vector4 Normal;
  toolkit::math::vector4 TexCoords;
  toolkit::math::vector4 Color;
  int BoneIndices[MAX_BONES_PER_MESH];
  float BoneWeights[MAX_BONES_PER_MESH];
};
struct Mesh {
  std::string meshName, modelPath;
  std::vector<MeshVertex> vertices;
  std::vector<uint32_t> indices;
  std::vector<BlendShape> blendShapes;
  std::shared_ptr<skeleton> skeleton = nullptr;
};

struct FBXPackage {
  std::string modelPath;
  std::vector<Mesh> meshes;
  std::shared_ptr<skeleton> skeleton = nullptr;
  std::shared_ptr<motion> motion = nullptr;
};

struct OBJPackage {
  std::string modelPath;
  std::vector<Mesh> meshes;
};

FBXPackage LoadFBX(std::string path);
OBJPackage LoadOBJ(std::string path);

}; // namespace Common::Assets