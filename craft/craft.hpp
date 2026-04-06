#pragma once

#include "toolkit/opengl3d/engine.hpp"
#include "voxel.hpp"
#include "voxel_render.hpp"

namespace toolkit::opengl3d {

// ---------------------------------------------------------------------------
// craft – Minecraft-like voxel game built on the toolkit ECS engine.
//
// Architecture overview:
//   World / Chunk / Block   – map management  (voxel.hpp/cpp)
//   VoxelRenderPipeline     – render pipeline  (voxel_render.hpp/cpp)
//   craft (this class)      – game loop, input, ECS wiring
//
// The ECS registry holds:
//   - A player entity with transform + camera components
//   - (extensible) additional entities via sub_systems
// ---------------------------------------------------------------------------

class craft : public engine3d {
public:
  void handle_custom_initialization() override;
  void handle_custom_cleanup() override;
  void handle_engine_gui() override;

  void engine_update(float dt);

  void run();
  void reset() override;

  // ---- Public game state ----
  ::craft::World world;
  ::craft::VoxelRenderPipeline voxel_pipeline;

  // Player entity (has transform + camera)
  entt::entity player_entity = entt::null;

  // Player movement
  float move_speed = 8.0f;
  float look_sensitivity = 0.15f;
  float pitch = 0.0f, yaw = 0.0f;

private:
  void handle_player_input(float dt);
  void handle_block_interaction();
};

}; // namespace toolkit::opengl3d