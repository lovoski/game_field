#include "toolkit/opengl/compute/tools.hpp"

namespace toolkit::opengl {

std::string ReduceGlobalAABB = R"(
#version 430
#define WORK_GROUP_SIZE %d
layout(local_size_x = WORK_GROUP_SIZE) in;

#define MAX_BONES 4
struct Vertex {
  vec4 Position;
  vec4 Normal;
  vec4 TexCoords;
  vec4 Color;
  int BoneId[MAX_BONES];
  float BoneWeight[MAX_BONES];
};
struct AABB {
  vec3 boxMin;
  float padding1;
  vec3 boxMax;
  float padding2;
};
void mergeAABB(inout AABB a, AABB b) {
  a.boxMin = min(a.boxMin, b.boxMin);
  a.boxMax = max(a.boxMax, b.boxMax);
}

layout(std430, binding = 0) buffer VertexInput {
  Vertex vIn[];
};
layout(std430, binding = 1) buffer IndicesInput {
  uint iIn[];
};
uniform int numTris;
uniform mat4 modelToWorldMatrix;

layout(std430, binding = 2) buffer AABBOutput {
  AABB aabbOut[];
};

shared AABB sharedData[WORK_GROUP_SIZE];
shared bool sharedDataValid[WORK_GROUP_SIZE];

void main() {
  uint gid = gl_GlobalInvocationID.x; // global id for current thread
  uint lid = gl_LocalInvocationID.x;  // local id for current thread within the work group
  uint groupId = gl_WorkGroupID.x;    // id for current work group

  if (gid < numTris) {
    vec3 v0 = (modelToWorldMatrix * vIn[iIn[3 * gid + 0]].Position).xyz;
    vec3 v1 = (modelToWorldMatrix * vIn[iIn[3 * gid + 1]].Position).xyz;
    vec3 v2 = (modelToWorldMatrix * vIn[iIn[3 * gid + 2]].Position).xyz;
    sharedData[lid].boxMin = min(min(v0, v1), v2);
    sharedData[lid].boxMax = max(max(v0, v1), v2);
    sharedDataValid[lid] = true;
  } else {
    sharedDataValid[lid] = false;
  }

  memoryBarrierShared();
  barrier();

  for (uint stride = gl_WorkGroupSize.x / 2; stride > 0; stride /= 2) {
    if (lid < stride && sharedDataValid[lid + stride]) {
      mergeAABB(sharedData[lid], sharedData[lid + stride]);
    }
    memoryBarrierShared();
    barrier();
  }

  if (lid == 0)
    aabbOut[groupId] = sharedData[lid];
}
)";

std::string MortonCodeForTris = R"(
#version 430
#define WORK_GROUP_SIZE %d
layout(local_size_x = WORK_GROUP_SIZE) in;

struct _packed_vertex {
  vec4 position;
  vec4 normal;
};
layout(std430, binding = 0) buffer SceneVertexBuffer {
  _packed_vertex gSceneVertexBuffer[];
};
layout(std430, binding = 1) buffer SceneIndexBuffer {
  uint gSceneIndexBuffer[];
};
layout(std430, binding = 2) buffer MortonCodeBuffer {
  uint gMortonCode[];
};
layout(std430, binding = 3) buffer PrimitiveIndexBuffer {
  uint gPrimIndex[];
};
uniform int gNumTriangles;
uniform vec3 gBoundingBoxMin;
uniform vec3 gBoundingBoxMax;
uniform int gVertexOffset;
uniform int gIndexOffset;

// Expands a 10-bit integer into 30 bits
// by inserting 2 zeros after each bit.
uint expandBits(uint v) {
  v = (v * 0x00010001u) & 0xFF0000FFu;
  v = (v * 0x00000101u) & 0x0F00F00Fu;
  v = (v * 0x00000011u) & 0xC30C30C3u;
  v = (v * 0x00000005u) & 0x49249249u;
  return v;
}

// Calculates a 30-bit Morton code for the
// given 3D point located within the unit cube [0,1].
uint morton3(float x, float y, float z) {
  x = min(max(x * 1024.0, 0.0), 1023.0);
  y = min(max(y * 1024.0, 0.0), 1023.0);
  z = min(max(z * 1024.0, 0.0), 1023.0);
  uint xx = expandBits(uint(x));
  uint yy = expandBits(uint(y));
  uint zz = expandBits(uint(z));
  return xx * 4 + yy * 2 + zz;
}

void main() {
  uint gid = gl_GlobalInvocationID.x;
  uint lid = gl_LocalInvocationID.x;
  uint groupId = gl_WorkGroupID.x;

  if (gid >= gNumTriangles) return;

  vec3 v0 = gSceneVertexBuffer[gSceneIndexBuffer[3 * gid + 0 + gIndexOffset]].position.xyz;
  vec3 v1 = gSceneVertexBuffer[gSceneIndexBuffer[3 * gid + 1 + gIndexOffset]].position.xyz;
  vec3 v2 = gSceneVertexBuffer[gSceneIndexBuffer[3 * gid + 2 + gIndexOffset]].position.xyz;

  vec3 barycenter = (v0 + v1 + v2) / 3.0;
  barycenter -= gBoundingBoxMin;
  // scale each axis to [0.0, 1.0]
  vec3 extent = gBoundingBoxMax - gBoundingBoxMin;
  barycenter = vec3(barycenter.x / extent.x, barycenter.y / extent.y, barycenter.z / extent.z);

  gMortonCode[gid] = morton3(barycenter.x, barycenter.y, barycenter.z);
  gPrimIndex[gid] = gid;
}
)";

}; // namespace toolkit::opengl