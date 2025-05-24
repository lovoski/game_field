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

void main() {
  uint gid = gl_GlobalInvocationID.x;
  uint lid = gl_LocalInvocationID.x;
  uint groupId = gl_WorkGroupID.x;

  if (gid >= gActualSize) return;

  _render_vertex oldVertex = gVertices[gid];
  uint newIndex = gid + gVertexOffset;

  _packed_vertex newVertex;
  newVertex.position = oldVertex.position;
  newVertex.normal = vec4(normalize(oldVertex.normal.xyz), 0.0);
  gPackedVertices[newIndex] = newVertex;
}
)";

std::string collect_scene_index_buffer_program_source = R"(
#version 430 core
#define WORK_GROUP_SIZE %d
layout(local_size_x = WORK_GROUP_SIZE) in;

layout(std430, binding = 0) buffer OriginalIndicesBuffer {
  uint gIndices[];
};
layout(std430, binding = 1) buffer ShiftedIndicesBuffer {
  uint gShiftedIndices[];
};

uniform int gActualSize;
uniform int gIndexOffset;
uniform int gVertexOffset;

void main() {
  uint gid = gl_GlobalInvocationID.x;
  uint lid = gl_LocalInvocationID.x;
  uint groupId = gl_WorkGroupID.x;

  if (gid >= gActualSize) return;

  gShiftedIndices[gid + gIndexOffset] = gIndices[gid] + gVertexOffset;
}
)";

std::string scene_buffer_apply_blendshape_program_source = R"(
#version 430 core
#define WORK_GROUP_SIZE %d
layout(local_size_x = WORK_GROUP_SIZE) in;
struct _blendshape_vertex {
  vec4 position;
  vec4 normal;
};
struct _packed_vertex {
  vec4 position;
  vec4 normal;
};
layout(std430, binding = 0) buffer BlendShapeDataBuffer {
  _blendshape_vertex gBlendShapeVertices[];
};
layout(std430, binding = 1) coherent buffer PackedVerticesBuffer {
  _packed_vertex gSceneVertices[];
};

uniform int gActualSize;
uniform int gVertexOffset;
uniform float gWeightValue;

void main() {
  uint gid = gl_GlobalInvocationID.x;
  uint lid = gl_LocalInvocationID.x;
  uint groupId = gl_WorkGroupID.x;
  if (gid >= gActualSize) return;

  _packed_vertex vertex = gSceneVertices[gVertexOffset+gid];

  vertex.position = gWeightValue * vec4((gBlendShapeVertices[gid].position).xyz, 0.0) + (1-gWeightValue) * vertex.position;
  vertex.normal = gWeightValue * vec4((gBlendShapeVertices[gid].normal).xyz, 0.0) +  (1-gWeightValue) * vertex.normal;

  gSceneVertices[gVertexOffset+gid] = vertex;
}
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
layout(std430, binding = 0) buffer OriginalVerticesBuffer {
  _render_vertex gOriginalVertices[];
};
layout(std430, binding = 1) buffer BoneTransformsBuffer {
  _bone_matrix_block gBoneMatrices[];
};
layout(std430, binding = 2) coherent buffer SceneVerticesBuffer {
  _packed_vertex gSceneVertices[];
};

uniform int gActualSize;
uniform int gVertexOffset;
uniform bool gBlended;

void main() {
  uint gid = gl_GlobalInvocationID.x;
  uint lid = gl_LocalInvocationID.x;
  uint groupId = gl_WorkGroupID.x;
  if (gid >= gActualSize) return;

  _render_vertex original_vertex = gOriginalVertices[gid];
  _packed_vertex scene_vertex = gSceneVertices[gid+gVertexOffset];
  _packed_vertex new_vertex;
  new_vertex.position = vec4(0.0);
  new_vertex.normal = vec4(0.0);

  vec4 bind_position = original_vertex.position, bind_normal = original_vertex.normal;
  if (gBlended) {
    bind_position = scene_vertex.position;
    bind_normal = scene_vertex.normal;
  }
  for (int i = 0; i < 4; i++) {
    int bone_id = original_vertex.bone_ids[i];
    float bone_weight = original_vertex.bone_weights[i];
    if (bone_weight > 0.0) {
      _bone_matrix_block bone_mat_block = gBoneMatrices[bone_id];
      mat4 bone_mat = bone_mat_block.model_mat * bone_mat_block.offset_mat;
      new_vertex.position += bone_mat * bind_position * bone_weight;
      mat3 normal_matrix = mat3(bone_mat);
      new_vertex.normal.xyz += normal_matrix * bind_normal.xyz * bone_weight;
    }
  }

  new_vertex.position.w = 1.0;
  new_vertex.normal.w = 0.0;
  new_vertex.normal = normalize(new_vertex.normal);

  gSceneVertices[gid+gVertexOffset] = new_vertex;
}
)";
