#include "toolkit/opengl3d/rasterize/kernal.hpp"

std::string collect_scene_vertex_buffer_program_source = R"(
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
  vec4 texcoords;
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
  newVertex.texcoords = oldVertex.tex_coords;
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
layout(std430, binding = 1) coherent buffer ShiftedIndicesBuffer {
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
  vec4 texcoords;
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

  vertex.position.xyz = gWeightValue * gBlendShapeVertices[gid].position.xyz + vertex.position.xyz;
  vertex.normal.xyz = gWeightValue * gBlendShapeVertices[gid].normal.xyz + vertex.normal.xyz;

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
  vec4 texcoords;
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
uniform int gSkinningAlgorithm;

const int SKINNING_LBS = 0;
const int SKINNING_DUAL_QUATERNION = 1;

struct _dual_quat {
  vec4 real;
  vec4 dual;
};

vec4 quat_mul(vec4 a, vec4 b) {
  return vec4(
    a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    a.w * b.w - dot(a.xyz, b.xyz));
}

vec4 quat_conj(vec4 q) {
  return vec4(-q.xyz, q.w);
}

vec3 quat_rotate(vec4 q, vec3 v) {
  return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

mat3 orthonormalize(mat3 m) {
  vec3 x = normalize(m[0]);
  vec3 y = normalize(m[1] - x * dot(x, m[1]));
  vec3 z = cross(x, y);
  return mat3(x, y, z);
}

vec4 quat_from_mat3(mat3 m) {
  float trace_value = m[0][0] + m[1][1] + m[2][2];
  vec4 q;

  if (trace_value > 0.0) {
    float s = sqrt(trace_value + 1.0) * 2.0;
    q.x = (m[1][2] - m[2][1]) / s;
    q.y = (m[2][0] - m[0][2]) / s;
    q.z = (m[0][1] - m[1][0]) / s;
    q.w = 0.25 * s;
  } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
    float s = sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0;
    q.x = 0.25 * s;
    q.y = (m[1][0] + m[0][1]) / s;
    q.z = (m[2][0] + m[0][2]) / s;
    q.w = (m[1][2] - m[2][1]) / s;
  } else if (m[1][1] > m[2][2]) {
    float s = sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0;
    q.x = (m[1][0] + m[0][1]) / s;
    q.y = 0.25 * s;
    q.z = (m[2][1] + m[1][2]) / s;
    q.w = (m[2][0] - m[0][2]) / s;
  } else {
    float s = sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0;
    q.x = (m[2][0] + m[0][2]) / s;
    q.y = (m[2][1] + m[1][2]) / s;
    q.z = 0.25 * s;
    q.w = (m[0][1] - m[1][0]) / s;
  }

  return normalize(q);
}

_dual_quat dual_quat_from_mat4(mat4 transform) {
  _dual_quat dq;
  dq.real = quat_from_mat3(orthonormalize(mat3(transform)));
  dq.dual = 0.5 * quat_mul(vec4(transform[3].xyz, 0.0), dq.real);
  return dq;
}

_dual_quat normalized_dual_quat(_dual_quat dq) {
  float inv_len = inversesqrt(max(dot(dq.real, dq.real), 1e-8));
  dq.real *= inv_len;
  dq.dual *= inv_len;
  dq.dual -= dq.real * dot(dq.real, dq.dual);
  return dq;
}

vec3 dual_quat_transform_point(_dual_quat dq, vec3 position) {
  vec3 translation = 2.0 * quat_mul(dq.dual, quat_conj(dq.real)).xyz;
  return quat_rotate(dq.real, position) + translation;
}

void main() {
  uint gid = gl_GlobalInvocationID.x;
  uint lid = gl_LocalInvocationID.x;
  uint groupId = gl_WorkGroupID.x;
  if (gid >= gActualSize) return;

  _render_vertex original_vertex = gOriginalVertices[gid];
  _packed_vertex scene_vertex = gSceneVertices[gid+gVertexOffset];
  _packed_vertex new_vertex;
  new_vertex.position = vec4(0.0, 0.0, 0.0, 1.0);
  new_vertex.normal = vec4(0.0);

  vec4 bind_position = original_vertex.position, bind_normal = original_vertex.normal;
  if (gBlended) {
    bind_position = scene_vertex.position;
    bind_normal = scene_vertex.normal;
  }

  if (gSkinningAlgorithm == SKINNING_DUAL_QUATERNION) {
    _dual_quat blended_dq;
    blended_dq.real = vec4(0.0);
    blended_dq.dual = vec4(0.0);
    vec4 reference_real = vec4(0.0);
    bool has_reference = false;

    for (int i = 0; i < 4; i++) {
      int bone_id = original_vertex.bone_ids[i];
      float bone_weight = original_vertex.bone_weights[i];
      if (bone_weight > 0.0) {
        _bone_matrix_block bone_mat_block = gBoneMatrices[bone_id];
        mat4 bone_mat = bone_mat_block.model_mat * bone_mat_block.offset_mat;
        _dual_quat bone_dq = dual_quat_from_mat4(bone_mat);
        if (!has_reference) {
          reference_real = bone_dq.real;
          has_reference = true;
        }
        if (dot(reference_real, bone_dq.real) < 0.0) {
          bone_dq.real = -bone_dq.real;
          bone_dq.dual = -bone_dq.dual;
        }
        blended_dq.real += bone_weight * bone_dq.real;
        blended_dq.dual += bone_weight * bone_dq.dual;
      }
    }

    if (has_reference) {
      blended_dq = normalized_dual_quat(blended_dq);
      new_vertex.position.xyz = dual_quat_transform_point(blended_dq, bind_position.xyz);
      new_vertex.normal.xyz = quat_rotate(blended_dq.real, bind_normal.xyz);
    }
  } else {
    for (int i = 0; i < 4; i++) {
      int bone_id = original_vertex.bone_ids[i];
      float bone_weight = original_vertex.bone_weights[i];
      if (bone_weight > 0.0) {
        _bone_matrix_block bone_mat_block = gBoneMatrices[bone_id];
        mat4 bone_mat = bone_mat_block.model_mat * bone_mat_block.offset_mat;
        new_vertex.position.xyz += (bone_mat * bind_position * bone_weight).xyz;
        mat3 normal_matrix = mat3(bone_mat);
        new_vertex.normal.xyz += normal_matrix * bind_normal.xyz * bone_weight;
      }
    }
  }

  new_vertex.position.w = 1.0;
  new_vertex.normal.w = 0.0;
  new_vertex.normal = normalize(new_vertex.normal);
  new_vertex.texcoords = original_vertex.tex_coords;

  gSceneVertices[gid+gVertexOffset] = new_vertex;
}
)";
