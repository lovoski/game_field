#pragma once

#include "toolkit/math.hpp"
#include "toolkit/system.hpp"
#include "toolkit/transform.hpp"
#include "toolkit/opengl3d/base.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

namespace craft {

using namespace toolkit;
using namespace toolkit::opengl3d;

// ---------------------------------------------------------------------------
// Block definitions – integer IDs with a data-driven registry.
// Add new blocks by calling BlockRegistry::register_block() at startup.
// ---------------------------------------------------------------------------

using BlockId = uint8_t;

namespace Block {
  constexpr BlockId Air   = 0;
  constexpr BlockId Grass = 1;
  constexpr BlockId Dirt  = 2;
  constexpr BlockId Stone = 3;
  constexpr BlockId Sand  = 4;
  constexpr BlockId Water = 5;
  constexpr BlockId Wood  = 6;
  constexpr BlockId Leaf  = 7;
  constexpr BlockId COUNT = 8; // bump when adding new blocks
}

// Per-block properties stored in a flat table indexed by BlockId.
struct BlockDef {
  const char *name = "unknown";
  bool solid       = true;   // collides / occludes faces
  bool transparent = false;  // adjacent faces are emitted
  // Face colors: [0]=+X [1]=-X [2]=+Y [3]=-Y [4]=+Z [5]=-Z
  std::array<math::vector3, 6> face_colors{};

  // Helper: set all 6 faces to the same color
  BlockDef &set_color(math::vector3 c) { face_colors.fill(c); return *this; }
  // Helper: set top/bottom/side colors
  BlockDef &set_color_tbs(math::vector3 top, math::vector3 bottom, math::vector3 side) {
    face_colors.fill(side);
    face_colors[2] = top;
    face_colors[3] = bottom;
    return *this;
  }
};

class BlockRegistry {
public:
  static BlockRegistry &instance();

  // Register / overwrite a block definition.  Returns the same id for chaining.
  BlockId register_block(BlockId id, BlockDef def);

  const BlockDef &get(BlockId id) const;
  bool  is_solid(BlockId id) const       { return get(id).solid; }
  bool  is_transparent(BlockId id) const { return get(id).transparent; }
  math::vector3 face_color(BlockId id, int face_dir) const {
    return get(id).face_colors[face_dir];
  }

private:
  BlockRegistry();
  std::array<BlockDef, 256> defs_{};
};

// Convenience free functions (delegate to singleton)
inline bool is_solid(BlockId b)       { return BlockRegistry::instance().is_solid(b); }
inline bool is_transparent(BlockId b) { return BlockRegistry::instance().is_transparent(b); }
inline math::vector3 block_color(BlockId b, int face_dir) {
  return BlockRegistry::instance().face_color(b, face_dir);
}

// ---------------------------------------------------------------------------
// Chunk constants
// ---------------------------------------------------------------------------

constexpr int CHUNK_X = 16;
constexpr int CHUNK_Y = 64;
constexpr int CHUNK_Z = 16;
constexpr int CHUNK_VOLUME = CHUNK_X * CHUNK_Y * CHUNK_Z;

// ---------------------------------------------------------------------------
// Chunk – a fixed-size 3-D slab of blocks
// ---------------------------------------------------------------------------

struct ChunkCoord {
  int x, z; // chunk-space (multiply by CHUNK_X/Z for world-space)
  bool operator==(const ChunkCoord &o) const { return x == o.x && z == o.z; }
};
struct ChunkCoordHash {
  std::size_t operator()(const ChunkCoord &c) const {
    // simple spatial hash
    return std::hash<int>()(c.x) ^ (std::hash<int>()(c.z) << 16);
  }
};

struct ChunkVertex {
  float px, py, pz;   // position
  float nx, ny, nz;   // normal
  float r, g, b;      // color
};

class Chunk {
public:
  ChunkCoord coord{0, 0};

  // Block data -  flat array, index = x + z*CHUNK_X + y*CHUNK_X*CHUNK_Z
  std::array<BlockId, CHUNK_VOLUME> blocks{};

  // GPU mesh (rebuilt when dirty)
  bool mesh_dirty = true;
  vao   mesh_vao;
  buffer mesh_vbo, mesh_ebo;
  uint32_t index_count = 0;

  // ---- accessors ----
  BlockId get(int x, int y, int z) const;
  void    set(int x, int y, int z, BlockId b);
  bool    in_bounds(int x, int y, int z) const;

  // Build the mesh from block data.  `neighbor` callback resolves cross-chunk
  // queries: neighbor(wx, wy, wz) returns the BlockId even if the coordinate
  // is outside this chunk's bounds.
  void build_mesh(std::function<BlockId(int, int, int)> neighbor);

private:
  std::vector<ChunkVertex> verts_;
  std::vector<uint32_t>    idxs_;
  void upload_mesh();
};

// ---------------------------------------------------------------------------
// World – infinite (horizontally) chunk map with lazy generation
// ---------------------------------------------------------------------------

class World {
public:
  // View distance in chunks (radius)
  int view_distance = 4;

  // Get / create chunk
  Chunk *get_chunk(ChunkCoord c);
  Chunk *get_chunk_at_block(int bx, int bz);

  // World-space block access (creates chunk if needed)
  BlockId get_block(int x, int y, int z);
  void    set_block(int x, int y, int z, BlockId b);

  // Call each frame with player position – loads/unloads chunks
  void update_around(math::vector3 center);

  // Iterate loaded chunks
  void for_each_chunk(std::function<void(Chunk &)> fn);

  // Terrain generation callback – override for custom terrain
  std::function<void(Chunk &)> generate = nullptr;

private:
  std::unordered_map<ChunkCoord, Chunk, ChunkCoordHash> chunks_;
  void ensure_chunk(ChunkCoord c);
  void rebuild_chunk_mesh(ChunkCoord c);
};

// Default simple terrain generator (perlin-ish heightmap)
void default_terrain_generate(Chunk &chunk);

} // namespace craft
