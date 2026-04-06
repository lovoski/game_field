#include "voxel.hpp"
#include <algorithm>
#include <cmath>

namespace craft {

// ---------------------------------------------------------------------------
// BlockRegistry – singleton that holds all block definitions
// ---------------------------------------------------------------------------

BlockRegistry &BlockRegistry::instance() {
  static BlockRegistry reg;
  return reg;
}

BlockRegistry::BlockRegistry() {
  // Default: every slot is Air-like
  for (auto &d : defs_) {
    d.name = "air";
    d.solid = false;
    d.transparent = true;
    d.face_colors.fill(math::vector3(1, 0, 1)); // magenta = unset
  }

  // Register built-in blocks
  register_block(Block::Air,   {.name = "air",   .solid = false, .transparent = true});
  register_block(Block::Grass, BlockDef{.name = "grass", .solid = true, .transparent = false}
    .set_color_tbs(
      math::vector3(0.30f, 0.70f, 0.20f),   // top
      math::vector3(0.55f, 0.37f, 0.24f),   // bottom
      math::vector3(0.40f, 0.55f, 0.25f))); // sides
  register_block(Block::Dirt,  BlockDef{.name = "dirt",  .solid = true}.set_color({0.55f, 0.37f, 0.24f}));
  register_block(Block::Stone, BlockDef{.name = "stone", .solid = true}.set_color({0.50f, 0.50f, 0.50f}));
  register_block(Block::Sand,  BlockDef{.name = "sand",  .solid = true}.set_color({0.85f, 0.80f, 0.55f}));
  register_block(Block::Water, BlockDef{.name = "water", .solid = false, .transparent = true}
    .set_color({0.20f, 0.40f, 0.80f}));
  register_block(Block::Wood,  BlockDef{.name = "wood",  .solid = true}.set_color({0.60f, 0.40f, 0.20f}));
  register_block(Block::Leaf,  BlockDef{.name = "leaf",  .solid = true}.set_color({0.15f, 0.55f, 0.10f}));
}

BlockId BlockRegistry::register_block(BlockId id, BlockDef def) {
  defs_[id] = def;
  return id;
}

const BlockDef &BlockRegistry::get(BlockId id) const {
  return defs_[id];
}

// ---------------------------------------------------------------------------
// Chunk block access
// ---------------------------------------------------------------------------

static int flat_index(int x, int y, int z) {
  return x + z * CHUNK_X + y * CHUNK_X * CHUNK_Z;
}

bool Chunk::in_bounds(int x, int y, int z) const {
  return x >= 0 && x < CHUNK_X && y >= 0 && y < CHUNK_Y && z >= 0 && z < CHUNK_Z;
}

BlockId Chunk::get(int x, int y, int z) const {
  if (!in_bounds(x, y, z)) return Block::Air;
  return blocks[flat_index(x, y, z)];
}

void Chunk::set(int x, int y, int z, BlockId b) {
  if (!in_bounds(x, y, z)) return;
  blocks[flat_index(x, y, z)] = b;
  mesh_dirty = true;
}

// ---------------------------------------------------------------------------
// Chunk mesh building – simple culled-face approach
// ---------------------------------------------------------------------------

// 6 face directions: +X -X +Y -Y +Z -Z
static const int face_dx[] = { 1, -1,  0,  0,  0,  0};
static const int face_dy[] = { 0,  0,  1, -1,  0,  0};
static const int face_dz[] = { 0,  0,  0,  0,  1, -1};
static const math::vector3 face_normals[] = {
  { 1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0,-1, 0}, {0, 0, 1}, {0, 0,-1}
};

// Quad vertices for each face direction (4 corners).
// Winding is CCW when viewed from outside the block, so that
// (v1-v0)×(v2-v0) equals the outward face normal.
// Triangles are emitted as (0,1,2) and (0,2,3).
static void face_quad(int dir, float bx, float by, float bz,
                      ChunkVertex out[4]) {
  float nx = face_normals[dir].x();
  float ny = face_normals[dir].y();
  float nz = face_normals[dir].z();
  auto v = [&](float ox, float oy, float oz) -> ChunkVertex {
    return {bx + ox, by + oy, bz + oz, nx, ny, nz, 1, 1, 1};
  };
  switch (dir) {
  case 0: // +X  cross: (0,1,0)×(0,1,1) = (+1,0,0) ✓
    out[0] = v(1,0,0); out[1] = v(1,1,0);
    out[2] = v(1,1,1); out[3] = v(1,0,1); break;
  case 1: // -X  cross: (0,1,0)×(0,1,-1) = (-1,0,0) ✓
    out[0] = v(0,0,1); out[1] = v(0,1,1);
    out[2] = v(0,1,0); out[3] = v(0,0,0); break;
  case 2: // +Y  cross: (0,0,1)×(1,0,1) = (0,+1,0) ✓
    out[0] = v(0,1,0); out[1] = v(0,1,1);
    out[2] = v(1,1,1); out[3] = v(1,1,0); break;
  case 3: // -Y  cross: (0,0,-1)×(1,0,-1) = (0,-1,0) ✓
    out[0] = v(0,0,1); out[1] = v(0,0,0);
    out[2] = v(1,0,0); out[3] = v(1,0,1); break;
  case 4: // +Z  cross: (1,0,0)×(1,1,0) = (0,0,+1) ✓
    out[0] = v(0,0,1); out[1] = v(1,0,1);
    out[2] = v(1,1,1); out[3] = v(0,1,1); break;
  case 5: // -Z  cross: (0,1,0)×(1,1,0) = (0,0,-1) ✓
    out[0] = v(0,0,0); out[1] = v(0,1,0);
    out[2] = v(1,1,0); out[3] = v(1,0,0); break;
  }
}

void Chunk::build_mesh(std::function<BlockId(int, int, int)> neighbor) {
  verts_.clear();
  idxs_.clear();

  int origin_x = coord.x * CHUNK_X;
  int origin_z = coord.z * CHUNK_Z;

  for (int y = 0; y < CHUNK_Y; ++y)
    for (int z = 0; z < CHUNK_Z; ++z)
      for (int x = 0; x < CHUNK_X; ++x) {
        BlockId blk = blocks[flat_index(x, y, z)];
        if (blk == Block::Air) continue;

        for (int dir = 0; dir < 6; ++dir) {
          int nx = x + face_dx[dir];
          int ny = y + face_dy[dir];
          int nz = z + face_dz[dir];

          // Resolve neighbor across chunks via callback
          BlockId adj;
          if (in_bounds(nx, ny, nz))
            adj = blocks[flat_index(nx, ny, nz)];
          else
            adj = neighbor(origin_x + nx, ny, origin_z + nz);

          if (!is_transparent(adj)) continue; // face hidden

          // Emit quad
          ChunkVertex quad[4];
          face_quad(dir, (float)(origin_x + x), (float)y, (float)(origin_z + z), quad);

          math::vector3 col = block_color(blk, dir);
          for (auto &v : quad) { v.r = col.x(); v.g = col.y(); v.b = col.z(); }

          uint32_t base = (uint32_t)verts_.size();
          for (int i = 0; i < 4; ++i) verts_.push_back(quad[i]);
          // Two triangles
          idxs_.push_back(base + 0);
          idxs_.push_back(base + 1);
          idxs_.push_back(base + 2);
          idxs_.push_back(base + 0);
          idxs_.push_back(base + 2);
          idxs_.push_back(base + 3);
        }
      }

  upload_mesh();
  mesh_dirty = false;
}

void Chunk::upload_mesh() {
  index_count = (uint32_t)idxs_.size();
  if (index_count == 0) return;

  if (!mesh_vao.get_handle()) {
    mesh_vao.create();
    mesh_vbo.create();
    mesh_ebo.create();
  }

  mesh_vao.bind();

  mesh_vbo.set_data_as(GL_ARRAY_BUFFER, verts_, GL_DYNAMIC_DRAW);
  mesh_ebo.set_data_as(GL_ELEMENT_ARRAY_BUFFER, idxs_, GL_DYNAMIC_DRAW);

  constexpr GLsizei stride = sizeof(ChunkVertex);
  // position  layout=0
  mesh_vao.link_attribute(mesh_vbo, 0, 3, GL_FLOAT, stride, (void *)offsetof(ChunkVertex, px));
  // normal    layout=1
  mesh_vao.link_attribute(mesh_vbo, 1, 3, GL_FLOAT, stride, (void *)offsetof(ChunkVertex, nx));
  // color     layout=2
  mesh_vao.link_attribute(mesh_vbo, 2, 3, GL_FLOAT, stride, (void *)offsetof(ChunkVertex, r));

  // Rebind EBO while VAO is bound so VAO records it
  mesh_ebo.bind_as(GL_ELEMENT_ARRAY_BUFFER);

  mesh_vao.unbind();
}

// ---------------------------------------------------------------------------
// World
// ---------------------------------------------------------------------------

Chunk *World::get_chunk(ChunkCoord c) {
  auto it = chunks_.find(c);
  return it == chunks_.end() ? nullptr : &it->second;
}

Chunk *World::get_chunk_at_block(int bx, int bz) {
  int cx = (bx >= 0) ? bx / CHUNK_X : (bx - CHUNK_X + 1) / CHUNK_X;
  int cz = (bz >= 0) ? bz / CHUNK_Z : (bz - CHUNK_Z + 1) / CHUNK_Z;
  return get_chunk({cx, cz});
}

BlockId World::get_block(int x, int y, int z) {
  if (y < 0 || y >= CHUNK_Y) return Block::Air;
  int cx = (x >= 0) ? x / CHUNK_X : (x - CHUNK_X + 1) / CHUNK_X;
  int cz = (z >= 0) ? z / CHUNK_Z : (z - CHUNK_Z + 1) / CHUNK_Z;
  auto *ch = get_chunk({cx, cz});
  if (!ch) return Block::Air;
  int lx = x - cx * CHUNK_X;
  int lz = z - cz * CHUNK_Z;
  return ch->get(lx, y, lz);
}

void World::set_block(int x, int y, int z, BlockId b) {
  if (y < 0 || y >= CHUNK_Y) return;
  int cx = (x >= 0) ? x / CHUNK_X : (x - CHUNK_X + 1) / CHUNK_X;
  int cz = (z >= 0) ? z / CHUNK_Z : (z - CHUNK_Z + 1) / CHUNK_Z;
  ensure_chunk({cx, cz});
  auto *ch = get_chunk({cx, cz});
  int lx = x - cx * CHUNK_X;
  int lz = z - cz * CHUNK_Z;
  ch->set(lx, y, lz, b);

  // Mark neighboring chunks dirty if block is on boundary
  if (lx == 0)            if (auto *n = get_chunk({cx - 1, cz})) n->mesh_dirty = true;
  if (lx == CHUNK_X - 1)  if (auto *n = get_chunk({cx + 1, cz})) n->mesh_dirty = true;
  if (lz == 0)            if (auto *n = get_chunk({cx, cz - 1})) n->mesh_dirty = true;
  if (lz == CHUNK_Z - 1)  if (auto *n = get_chunk({cx, cz + 1})) n->mesh_dirty = true;
}

void World::ensure_chunk(ChunkCoord c) {
  if (chunks_.count(c)) return;
  Chunk &ch = chunks_[c];
  ch.coord = c;
  ch.blocks.fill(Block::Air);
  if (generate)
    generate(ch);
  else
    default_terrain_generate(ch);
}

void World::rebuild_chunk_mesh(ChunkCoord c) {
  auto *ch = get_chunk(c);
  if (!ch) return;
  // Cross-chunk neighbor resolver
  auto neighbor = [this](int wx, int wy, int wz) -> BlockId {
    return get_block(wx, wy, wz);
  };
  ch->build_mesh(neighbor);
}

void World::update_around(math::vector3 center) {
  int cx = (int)std::floor(center.x() / CHUNK_X);
  int cz = (int)std::floor(center.z() / CHUNK_Z);

  // Ensure chunks within view distance
  for (int dz = -view_distance; dz <= view_distance; ++dz)
    for (int dx = -view_distance; dx <= view_distance; ++dx)
      ensure_chunk({cx + dx, cz + dz});

  // Rebuild dirty meshes (limit per frame to avoid hitches)
  int rebuilt = 0;
  constexpr int MAX_REBUILDS_PER_FRAME = 4;
  for (auto &[coord, chunk] : chunks_) {
    if (chunk.mesh_dirty && rebuilt < MAX_REBUILDS_PER_FRAME) {
      rebuild_chunk_mesh(coord);
      ++rebuilt;
    }
  }

  // Unload far chunks
  std::vector<ChunkCoord> to_remove;
  for (auto &[coord, chunk] : chunks_) {
    int dist = std::max(std::abs(coord.x - cx), std::abs(coord.z - cz));
    if (dist > view_distance + 2)
      to_remove.push_back(coord);
  }
  for (auto &c : to_remove) {
    auto &ch = chunks_[c];
    ch.mesh_vao.del();
    ch.mesh_vbo.del();
    ch.mesh_ebo.del();
    chunks_.erase(c);
  }
}

void World::for_each_chunk(std::function<void(Chunk &)> fn) {
  for (auto &[coord, chunk] : chunks_)
    fn(chunk);
}

// ---------------------------------------------------------------------------
// Default terrain generator – simple sine-based heightmap
// ---------------------------------------------------------------------------

static float hash_noise(int x, int z) {
  int n = x * 73856093 ^ z * 19349663;
  n = (n << 13) ^ n;
  return 1.0f - (float)((n * (n * n * 15731 + 789221) + 1376312589) & 0x7fffffff) / 1073741824.0f;
}

static float smooth_noise(float x, float z) {
  int ix = (int)std::floor(x);
  int iz = (int)std::floor(z);
  float fx = x - ix;
  float fz = z - iz;
  // smoothstep
  fx = fx * fx * (3.0f - 2.0f * fx);
  fz = fz * fz * (3.0f - 2.0f * fz);

  float a = hash_noise(ix, iz);
  float b = hash_noise(ix + 1, iz);
  float c = hash_noise(ix, iz + 1);
  float d = hash_noise(ix + 1, iz + 1);
  return a + (b - a) * fx + (c - a) * fz + (a - b - c + d) * fx * fz;
}

static float terrain_height(int wx, int wz) {
  float h = 0.0f;
  h += smooth_noise(wx * 0.02f, wz * 0.02f) * 16.0f;
  h += smooth_noise(wx * 0.05f, wz * 0.05f) * 6.0f;
  h += smooth_noise(wx * 0.1f,  wz * 0.1f)  * 2.0f;
  return h + 20.0f; // base height offset
}

void default_terrain_generate(Chunk &chunk) {
  int ox = chunk.coord.x * CHUNK_X;
  int oz = chunk.coord.z * CHUNK_Z;

  for (int x = 0; x < CHUNK_X; ++x)
    for (int z = 0; z < CHUNK_Z; ++z) {
      int h = std::clamp((int)terrain_height(ox + x, oz + z), 1, CHUNK_Y - 1);
      for (int y = 0; y < h; ++y) {
        BlockId b;
        if (y < h - 4)
          b = Block::Stone;
        else if (y < h - 1)
          b = Block::Dirt;
        else
          b = Block::Grass;
        chunk.blocks[flat_index(x, y, z)] = b;
      }
    }
}

} // namespace craft
