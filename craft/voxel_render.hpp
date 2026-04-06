#pragma once

#include "voxel.hpp"
#include "toolkit/opengl3d/base.hpp"
#include "toolkit/opengl3d/components/camera.hpp"
#include "toolkit/transform.hpp"

namespace craft {

using namespace toolkit;
using namespace toolkit::opengl3d;

// ---------------------------------------------------------------------------
// VoxelRenderPipeline – owns its own FBO, shaders, and draw logic.
// Easy to extend: add post-process passes, swap shaders, etc.
// ---------------------------------------------------------------------------

class VoxelRenderPipeline {
public:
  void init(int width, int height);
  void resize(int width, int height);
  void shutdown();

  // Render all chunks in `world` from the given camera.
  void render(World &world, transform &cam_trans, camera &cam,
              float screen_w, float screen_h);

  // The final color texture (sample this to display on screen quad)
  GLuint output_texture() const { return color_tex_.get_handle(); }

  // --- Tweakable parameters exposed to GUI ---
  math::vector3 sun_dir   = math::vector3(-0.4f, -0.8f, -0.3f).normalized();
  math::vector3 sun_color = math::vector3(1.0f, 0.95f, 0.85f);
  math::vector3 ambient   = math::vector3(0.35f, 0.40f, 0.50f);
  math::vector3 sky_color = math::vector3(0.53f, 0.81f, 0.92f);
  float fog_start = 60.0f, fog_end = 120.0f;

private:
  void create_shaders();
  void create_framebuffer(int w, int h);

  int width_ = 0, height_ = 0;

  // Main geometry pass
  shader chunk_shader_;

  // Framebuffer for off-screen rendering
  framebuffer fbo_;
  texture color_tex_, depth_tex_;
};

} // namespace craft
