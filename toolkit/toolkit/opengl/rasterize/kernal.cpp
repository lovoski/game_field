#include "toolkit/opengl/rasterize/kernal.hpp"

std::string collect_scene_buffer_program_source = R"(
#version 430 core
#define WORK_GROUP_SIZE %d
layout(local_size_x = WORK_GROUP_SIZE) in;
struct _render_vertex {
  vec4 position;
  vec4 normal;
  vec4 tex_coords;
  vec4 color;
  int bone_ids[4];
  float bone_weights[4];
};
struct _packed_vertex {
  vec4 position;
  vec4 normal;
};
layout(std430, binding = 0) buffer OriginalVerticesBuffer {
  _render_vertex gVertices[];
};
layout(std430, binding = 1) coherent buffer PackedVerticesBuffer {
  _packed_vertex gPackedVertices[];
};

uniform int gActualSize;
uniform int gVertexOffset;
uniform mat4 gModelToWorldPoint;
uniform mat3 gModelToWorldDir;

void main() {
  uint gid = gl_GlobalInvocationID.x;
  uint lid = gl_LocalInvocationID.x;
  uint groupId = gl_WorkGroupID.x;

  if (gid >= gActualSize) return;

  Vertex oldVertex = gVertices[gid];
  uint newIndex = gid + gVertexOffset;

  _packed_vertex newVertex;
  newVertex.position = gModelToWorldPoint * oldVertex.position;
  newVertex.normal = vec4(gModelToWorldDir * oldVertex.normal.xyz, 0.0);
  gPackedVertices[newIndex] = newVertex;
}
)";

std::string scene_buffer_apply_blendshape_program_source = R"(
)";

std::string scene_buffer_apply_mesh_skinning_program_source = R"(
#version 430 core
#define WORK_GROUP_SIZE %d
layout(local_size_x = WORK_GROUP_SIZE) in;
struct _render_vertex {
  vec4 position;
  vec4 normal;
  vec4 tex_coords;
  vec4 color;
  int bone_ids[4];
  float bone_weights[4];
};
struct _bone_matrix_block {
  mat4 model_mat;
  mat4 offset_mat;
};
struct _packed_vertex {
  vec4 position;
  vec4 normal;
};
layout(std430, binding = 0) buffer SceneVertexBuffer {
  _render_vertex gVertices[];
};
layout(std430, binding = 1) buffer BoneTransformsBuffer {
  _bone_matrix_block gBoneMatrices[];
};
layout(std430, binding = 2) coherent buffer PackedVerticesBuffer {
  _packed_vertex gPackedVertices[];
};

uniform int gActualSize;
uniform int gVertexOffset;
uniform mat4 gModelToWorldPoint;
uniform mat3 gModelToWorldDir;

void main() {
  uint gid = gl_GlobalInvocationID.x;
  uint lid = gl_LocalInvocationID.x;
  uint groupId = gl_WorkGroupID.x;

  if (gid >= gActualSize) return;

  Vertex oldVertex = gVertices[gid];
  uint newIndex = gid + gVertexOffset;

  _packed_vertex newVertex;
  newVertex.position = vec4(0.0);
  newVertex.normal = vec4(0.0);

  for (int i = 0; i < MAX_BONES; i++) {
    int boneId = oldVertex.BoneId[i];
    float weight = oldVertex.BoneWeight[i];
    if (weight > 0.0) {
      MatrixBlock boneMatrices = gBoneMatrices[boneId];
      mat4 boneMatrix = boneMatrices.BoneModelMatrix * boneMatrices.BoneOffsetMatrix;
      newVertex.position += boneMatrix * oldVertex.Position * weight;
      newVertex.normal += boneMatrix * oldVertex.Normal * weight;
    }
  }

  newVertex.position = newVertex.position;
  newVertex.normal = newVertex.normal;
  gPackedVertices[newIndex] = newVertex;
}
)";
