#pragma once

#include "toolkit/opengl/base.hpp"

namespace toolkit::opengl {

// Compute the global aabb for a mesh
extern std::string ReduceGlobalAABB;

/**
 * Compute morton code for each triangle primitive.
 * The actual coordinate for each triangle's bartcenter should be scaled
 * relative to the global bounding box.
 */
extern std::string MortonCodeForTris;

class prefix_sum {
public:
  prefix_sum() {
    workGroupScan.create((str_format(work_group_scan_source, m_workGroupSize)));
    globalPrefixSum.create(
        str_format(global_prefix_sum_source, m_workGroupSize));
  }
  ~prefix_sum() {}

  const char *work_group_scan_source = R"(
#version 430
#define THREAD_NUM %d
layout(local_size_x = THREAD_NUM) in;

layout(std430, binding = 0) buffer InputData {
  uint gInputData[];
};
layout(std430, binding = 1) buffer OutputData {
  uint gOutputData[];
};
layout(std430, binding = 2) buffer GroupSumChunk {
  uint gGroupSumChunk[];
};

uniform int actualSize;
shared uint localPrefix[THREAD_NUM];

// exclusive scan
void LocalScan(uint lid) {
  //up sweep
  uint d = 0;
  uint i = 0;
  uint offset = 1;
  uint totalNum = THREAD_NUM;
  for (d = totalNum>>1; d > 0; d >>= 1) {
    barrier();
    if (lid < d) {
      uint ai = offset * (2 * lid + 1) - 1;
      uint bi = offset * (2 * lid + 2) - 1;

      localPrefix[bi] += localPrefix[ai];
    }
    offset *= 2;
  }

  //clear the last element
  if (lid == 0) {
    localPrefix[totalNum-1] = 0;
  }
  barrier();

  //Down-sweep
  for (d = 1; d < totalNum; d *= 2) {
    offset >>= 1;
    barrier();

    if (lid < d) {
      uint ai = offset * (2 * lid + 1) - 1;
      uint bi = offset * (2 * lid + 2) - 1;

      uint tmp = localPrefix[ai];
      localPrefix[ai] = localPrefix[bi];
      localPrefix[bi] += tmp;
    }
  }
  barrier();
}

void main() {
  uint gid = gl_GlobalInvocationID.x;
  uint lid = gl_LocalInvocationID.x;
  uint groupId = gl_WorkGroupID.x;

  uint inputValue;
  if (gid >= actualSize) {
    inputValue = 0;
  } else {
    inputValue = gInputData[gid];
  }
  localPrefix[lid] = inputValue;
  barrier();

  LocalScan(lid);

  gOutputData[gid] = localPrefix[lid];
  if (lid == gl_WorkGroupSize.x-1)
    gGroupSumChunk[groupId] = localPrefix[lid] + inputValue;
}
)";

  const char *global_prefix_sum_source = R"(
#version 430
#define THREAD_NUM %d
layout(local_size_x = THREAD_NUM) in;

layout(std430, binding = 0) buffer OutputData {
  uint gOutputData[];
};
layout(std430, binding = 1) buffer GroupSumChunkPrefixSum {
  uint gGroupSumChunkPrefixSum[];
};

uniform int actualSize;

void main() {
  uint gid = gl_GlobalInvocationID.x;
  uint groupId = gl_WorkGroupID.x;

  if (gid >= actualSize) {
    return;
  }

  if (groupId >= 1)
    gOutputData[gid] += gGroupSumChunkPrefixSum[groupId];
}
)";

  void operator()(buffer &input, buffer &output, unsigned int size) {
    workGroupScan.use();
    prefixSumInternal(input, output, size);
  }

private:
  const int m_workGroupSize = 1024;
  compute_shader workGroupScan;
  compute_shader globalPrefixSum;

  void prefixSumInternal(buffer &input, buffer &output, unsigned int size) {
    int workGroupNum = (size + m_workGroupSize - 1) / m_workGroupSize;
    buffer workGroupChunkSum;
    workGroupChunkSum.create();
    workGroupScan.use();
    workGroupChunkSum.set_data_ssbo(sizeof(unsigned int) * workGroupNum);
    workGroupScan.set_int("actualSize", size);
    workGroupScan.bind_buffer(input, 0).bind_buffer(output, 1).bind_buffer(
        workGroupChunkSum, 2);
    workGroupScan.dispatch(workGroupNum, 1, 1);
    workGroupScan.barrier();

    if (size > m_workGroupSize) {
      // recursion
      buffer workGroupChunkPreixSum;
      workGroupChunkPreixSum.create();
      workGroupChunkPreixSum.set_data_ssbo(sizeof(unsigned int) * workGroupNum);

      prefixSumInternal(workGroupChunkSum, workGroupChunkPreixSum,
                        workGroupNum);

      globalPrefixSum.use();
      globalPrefixSum.set_int("actualSize", size);
      globalPrefixSum.bind_buffer(output, 0).bind_buffer(workGroupChunkPreixSum,
                                                         1);
      globalPrefixSum.dispatch(workGroupNum, 1, 1);
      globalPrefixSum.barrier();

      workGroupChunkPreixSum.del();
    }
  }
};

class radix_sort {
public:
  radix_sort() {
    keysBackup.create();
    valuesBackup.create();
    gRadixCountingBuffer.create();
    gRadixPrefixBuffer.create();
    gRadixCountingPrefixBuffer.create();
    radixSortCount.create(
        str_format(radix_sort_count_source, radixBits, m_workGroupSize));
    radixSortFinal.create(
        str_format(radix_sort_final_source, radixBits, m_workGroupSize));
  }
  ~radix_sort() {
    keysBackup.del();
    valuesBackup.del();
    gRadixCountingBuffer.del();
    gRadixPrefixBuffer.del();
    gRadixCountingPrefixBuffer.del();
  }

  const char *radix_sort_count_source = R"(
#version 430
#define RADIX_BITS %d
#define THREAD_NUM %d
layout(local_size_x = THREAD_NUM) in;

layout(std430, binding = 0) buffer KeysInput {
  uint gKeysInput[];
};
// shape: (1 << RADIX_BITS) * numWorkGroup
layout(std430, binding = 1) buffer RadixCountingBuffer {
  uint gRadixCountingBuffer[];
};
// shape: THREAD_NUM * numWorkGroup
layout(std430, binding = 2) buffer RadixPrefixBuffer {
  uint gRadixPrefixBuffer[];
};

uniform int gActualSize;
uniform int gNumWorkGroups;
uniform int gCurrentIteration;
shared uint localPrefix[THREAD_NUM];

// exclusive scan
void LocalScan(uint lid) {
  //up sweep
  uint d = 0;
  uint i = 0;
  uint offset = 1;
  uint totalNum = THREAD_NUM;
  for (d = totalNum>>1; d > 0; d >>= 1) {
    barrier();
    if (lid < d) {
      uint ai = offset * (2 * lid + 1) - 1;
      uint bi = offset * (2 * lid + 2) - 1;

      localPrefix[bi] += localPrefix[ai];
    }
    offset *= 2;
  }

  //clear the last element
  if (lid == 0) {
    localPrefix[totalNum-1] = 0;
  }
  barrier();

  //Down-sweep
  for (d = 1; d < totalNum; d *= 2) {
    offset >>= 1;
    barrier();

    if (lid < d) {
      uint ai = offset * (2 * lid + 1) - 1;
      uint bi = offset * (2 * lid + 2) - 1;

      uint tmp = localPrefix[ai];
      localPrefix[ai] = localPrefix[bi];
      localPrefix[bi] += tmp;
    }
  }
  barrier();
}

uint getDigit(uint gid) {
  uint data = gid >= gActualSize ? 4294967295u : gKeysInput[gid];
  return (data << ((32 - RADIX_BITS) - RADIX_BITS * gCurrentIteration)) >> (32 - RADIX_BITS);
}

void RadixSortCount(uint gid, uint lid, uint groupId) {
  uint digit = getDigit(gid);

  uint radixCategory = 1 << RADIX_BITS;
  for (uint r = 0; r < radixCategory; ++r) {
    //load to share memory
    localPrefix[lid] = (digit == r ? 1 : 0);
    barrier();

    // prefix sum according to r in this blocks
    LocalScan(lid);

    // ouput the total sum to global counter
    if (lid == THREAD_NUM - 1) {
      uint counterIndex = r * gNumWorkGroups + groupId;
      uint counter = localPrefix[lid];
      if (digit == r)
        counter++;
      gRadixCountingBuffer[counterIndex] = counter;
    }

    // output prefix sum according to r
    if(digit == r) {
      gRadixPrefixBuffer[gid] = localPrefix[lid];
    }

    barrier();
  }
}

void main() {
  uint gid = gl_GlobalInvocationID.x;
  uint lid = gl_LocalInvocationID.x;
  uint groupId = gl_WorkGroupID.x;

  RadixSortCount(gid, lid, groupId);
}
)";

  const char *radix_sort_final_source = R"(
#version 430
#define RADIX_BITS %d
#define THREAD_NUM %d
layout(local_size_x = THREAD_NUM) in;

layout(std430, binding = 0) buffer KeysInput {
  uint gKeysInput[];
};
layout(std430, binding = 1) coherent buffer KeysOutput {
  uint gKeysOutput[];
};
layout(std430, binding = 2) buffer ValsInput {
  uint gValsInput[];
};
layout(std430, binding = 3) coherent buffer ValsOutput {
  uint gValsOutput[];
};
// shape: (1 << RADIX_BITS) * numWorkGroup
layout(std430, binding = 4) buffer RadixCountingPrefixBuffer {
  uint gRadixCountingPrefixBuffer[];
};
// shape: THREAD_NUM * numWorkGroup
layout(std430, binding = 5) buffer RadixPrefixBuffer {
  uint gRadixPrefixBuffer[];
};

uniform int gActualSize;
uniform int gNumWorkGroups;
uniform int gCurrentIteration;

uint getDigit(uint gid) {
  uint data = gid >= gActualSize ? 4294967295u : gKeysInput[gid];
  return (data << ((32 - RADIX_BITS) - RADIX_BITS * gCurrentIteration)) >> (32 - RADIX_BITS);
}

void main() {
  uint gid = gl_GlobalInvocationID.x;
  uint lid = gl_LocalInvocationID.x;
  uint groupId = gl_WorkGroupID.x;
  if (gid >= gActualSize) return;

  // get curr digit 
  uint digit = getDigit(gid);
  // global dispatch
  uint counterIndex = digit * gNumWorkGroups + groupId;
  uint globalPos = gRadixPrefixBuffer[gid] + gRadixCountingPrefixBuffer[counterIndex];
  gKeysOutput[globalPos] = gKeysInput[gid];
  gValsOutput[globalPos] = gValsInput[gid];
}
)";

  void operator()(buffer &keys, buffer &values, unsigned int size) {
    int numWorkGroups = (size + m_workGroupSize - 1) / m_workGroupSize;
    int countingBufferSize = (1 << radixBits) * numWorkGroups;
    gRadixCountingBuffer.set_data_ssbo(sizeof(unsigned int) *
                                       countingBufferSize);
    gRadixCountingPrefixBuffer.set_data_ssbo(sizeof(unsigned int) *
                                             countingBufferSize);
    gRadixPrefixBuffer.set_data_ssbo(sizeof(unsigned int) * numWorkGroups *
                                     m_workGroupSize);
    keysBackup.set_data_ssbo(sizeof(unsigned int) * size);
    valuesBackup.set_data_ssbo(sizeof(unsigned int) * size);
    unsigned int keysId[2] = {keys.get_handle(), keysBackup.get_handle()};
    unsigned int valuesId[2] = {values.get_handle(), valuesBackup.get_handle()};
    for (int i = 0; i < 32 / radixBits; i++) {
      // i % 2 -> current input
      // (i + 1) % 2 -> current output

      radixSortCount.use();
      radixSortCount.bind_buffer(keysId[i % 2], 0)
          .bind_buffer(gRadixCountingBuffer, 1)
          .bind_buffer(gRadixPrefixBuffer, 2);
      radixSortCount.set_int("gActualSize", size);
      radixSortCount.set_int("gCurrentIteration", i);
      radixSortCount.set_int("gNumWorkGroups", numWorkGroups);
      radixSortCount.dispatch(numWorkGroups, 1, 1);
      radixSortCount.barrier();

      prefixSum(gRadixCountingBuffer, gRadixCountingPrefixBuffer,
                countingBufferSize);

      radixSortFinal.use();
      radixSortFinal.bind_buffer(keysId[i % 2], 0)
          .bind_buffer(keysId[(i + 1) % 2], 1)
          .bind_buffer(valuesId[i % 2], 2)
          .bind_buffer(valuesId[(i + 1) % 2], 3)
          .bind_buffer(gRadixCountingPrefixBuffer, 4)
          .bind_buffer(gRadixPrefixBuffer, 5);
      radixSortFinal.set_int("gActualSize", size);
      radixSortFinal.set_int("gNumWorkGroups", numWorkGroups);
      radixSortFinal.set_int("gCurrentIteration", i);
      radixSortFinal.dispatch(numWorkGroups, 1, 1);
      radixSortFinal.barrier();
    }
  }

private:
  const int radixBits = 4;
  const int m_workGroupSize = 256;
  prefix_sum prefixSum;
  compute_shader radixSortCount;
  compute_shader radixSortFinal;

  buffer keysBackup, valuesBackup;
  buffer gRadixCountingBuffer, gRadixPrefixBuffer, gRadixCountingPrefixBuffer;
};

struct _padded_aabb {
  math::vector3 boxMin;
  float padding1;
  math::vector3 boxMax;
  float padding2;
};

class per_mesh_global_aabb {
public:
  per_mesh_global_aabb() {
    frontBuffer.create();
    backBuffer.create();
    computeTriangleBoundingBoxes.create(
        str_format(aabbForEachTriangleSource, m_workGroupSize));
    workGroupBoundingBoxes.create(
        str_format(workGroupBoundingBoxesSource, m_workGroupSize));
  }
  ~per_mesh_global_aabb() {
    frontBuffer.del();
    backBuffer.del();
  }

  const char *aabbForEachTriangleSource = R"(
#version 430
#define WORK_GROUP_SIZE %d
layout(local_size_x = WORK_GROUP_SIZE) in;
struct AABB {
  vec3 boxMin;
  float padding1;
  vec3 boxMax;
  float padding2;
};
struct _packed_vertex {
  vec4 position;
  vec4 normal;
};
layout(std430, binding = 0) buffer WorldVertexBuffer {
  _packed_vertex gVertices[];
};
layout(std430, binding = 1) buffer WorldIndexBuffer {
  uint gIndices[];
};
layout(std430, binding = 2) buffer BoundingBoxBuffer {
  AABB gBoundingBoxes[];
};

uniform int gActualSize;
uniform int gIndexOffset;
uniform int gVertexOffset;

void main() {
  uint gid = gl_GlobalInvocationID.x;
  uint lid = gl_LocalInvocationID.x;
  uint groupId = gl_WorkGroupID.x;

  if (gid >= gActualSize) return;
  
  vec3 v0 = gVertices[gIndices[3 * gid + 0 + gIndexOffset]].position.xyz;
  vec3 v1 = gVertices[gIndices[3 * gid + 1 + gIndexOffset]].position.xyz;
  vec3 v2 = gVertices[gIndices[3 * gid + 2 + gIndexOffset]].position.xyz;

  AABB boundingBox;
  boundingBox.boxMin = min(min(v0, v1), v2);
  boundingBox.boxMax = max(max(v0, v1), v2);

  gBoundingBoxes[gid] = boundingBox;
}
)";

  const char *workGroupBoundingBoxesSource = R"(
#version 430
#define WORK_GROUP_SIZE %d
layout(local_size_x = WORK_GROUP_SIZE) in;

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
layout(std430, binding = 0) buffer WorldVertexBuffer {
  AABB gGlobalBoundingBoxes[];
};
layout(std430, binding = 1) buffer WorkGroupBuffer {
  AABB gGroupBoundingBoxes[];
};

uniform int gActualSize;
shared AABB sharedData[WORK_GROUP_SIZE];

// exclusive scan
void LocalScan(uint lid) {
  //up sweep
  uint d = 0;
  uint i = 0;
  uint offset = 1;
  uint totalNum = WORK_GROUP_SIZE;
  for (d = totalNum>>1; d > 0; d >>= 1) {
    barrier();
    if (lid < d) {
      uint ai = offset * (2 * lid + 1) - 1;
      uint bi = offset * (2 * lid + 2) - 1;
      
      mergeAABB(sharedData[bi], sharedData[ai]);
    }
    offset *= 2;
  }

  //clear the last element
  if (lid == 0) {
    sharedData[totalNum-1].boxMin = vec3(1e30);
    sharedData[totalNum-1].boxMax = vec3(-1e30);
  }
  barrier();

  //Down-sweep
  for (d = 1; d < totalNum; d *= 2) {
    offset >>= 1;
    barrier();

    if (lid < d) {
      uint ai = offset * (2 * lid + 1) - 1;
      uint bi = offset * (2 * lid + 2) - 1;

      AABB tmp = sharedData[ai];
      sharedData[ai] = sharedData[bi];
      mergeAABB(sharedData[bi], tmp);
    }
  }
  barrier();
}

void main() {
  uint gid = gl_GlobalInvocationID.x;
  uint lid = gl_LocalInvocationID.x;
  uint groupId = gl_WorkGroupID.x;

  if (gid < gActualSize) {
    sharedData[lid] = gGlobalBoundingBoxes[gid];
  } else {
    sharedData[lid].boxMin = vec3(1e30);
    sharedData[lid].boxMax = vec3(-1e30);
  }
  barrier();

  LocalScan(lid);

  if (lid == WORK_GROUP_SIZE - 1)
    gGroupBoundingBoxes[groupId] = sharedData[lid];
}
)";

  std::tuple<math::vector3, math::vector3>
  operator()(buffer &sceneVertexBuffer, buffer &sceneIndexBuffer,
             const unsigned int vertexOffset, const unsigned int indexOffset,
             const unsigned int numTriangles) {
    frontBuffer.set_data_ssbo(sizeof(_padded_aabb) * numTriangles);
    computeTriangleBoundingBoxes.use();
    computeTriangleBoundingBoxes.bind_buffer(sceneVertexBuffer, 0)
        .bind_buffer(sceneIndexBuffer, 1)
        .bind_buffer(frontBuffer, 2);
    computeTriangleBoundingBoxes.set_int("gActualSize", numTriangles);
    computeTriangleBoundingBoxes.set_int("gIndexOffset", indexOffset);
    computeTriangleBoundingBoxes.set_int("gVertexOffset", vertexOffset);
    computeTriangleBoundingBoxes.dispatch(
        (numTriangles + m_workGroupSize - 1) / m_workGroupSize, 1, 1);
    computeTriangleBoundingBoxes.barrier();

    unsigned int shiftBuffer[2] = {frontBuffer.get_handle(),
                                   backBuffer.get_handle()};
    unsigned int i = 0, size = numTriangles, numWorkGroups;
    workGroupBoundingBoxes.use();
    while (true) {
      numWorkGroups = (size + m_workGroupSize - 1) / m_workGroupSize;
      glBindBuffer(GL_SHADER_STORAGE_BUFFER, shiftBuffer[(i + 1) % 2]);
      glBufferData(GL_SHADER_STORAGE_BUFFER,
                   sizeof(_padded_aabb) * numWorkGroups, nullptr,
                   GL_STREAM_DRAW);
      workGroupBoundingBoxes.bind_buffer(shiftBuffer[i % 2], 0)
          .bind_buffer(shiftBuffer[(i + 1) % 2], 1);
      workGroupBoundingBoxes.set_int("gActualSize", size);
      workGroupBoundingBoxes.dispatch(numWorkGroups, 1, 1);
      workGroupBoundingBoxes.barrier();
      if (numWorkGroups == 1)
        break;
      i++;
      size = numWorkGroups;
    }
    math::vector3 boxMin, boxMax;
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, shiftBuffer[(i + 1) % 2]);
    _padded_aabb *aabb =
        (_padded_aabb *)glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_READ_ONLY);
    boxMin = aabb->boxMin;
    boxMax = aabb->boxMax;
    glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    return {boxMin, boxMax};
  }

private:
  const int m_workGroupSize = 256;
  buffer frontBuffer, backBuffer;
  compute_shader computeTriangleBoundingBoxes;
  compute_shader workGroupBoundingBoxes;
};

}; // namespace toolkit::opengl
