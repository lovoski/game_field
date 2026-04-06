#include "voxel_render.hpp"

namespace craft {

// ---------------------------------------------------------------------------
// Inline GLSL shaders – kept here so everything is self-contained.
// Swap these strings to change the entire look of the game.
// ---------------------------------------------------------------------------

static const char *chunk_vs = R"(
#version 450 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec3 aColor;

uniform mat4 uVP;

out vec3 vWorldPos;
out vec3 vNormal;
out vec3 vColor;

void main() {
  vWorldPos = aPos;
  vNormal   = aNormal;
  vColor    = aColor;
  gl_Position = uVP * vec4(aPos, 1.0);
}
)";

static const char *chunk_fs = R"(
#version 450 core
in vec3 vWorldPos;
in vec3 vNormal;
in vec3 vColor;

uniform vec3 uSunDir;
uniform vec3 uSunColor;
uniform vec3 uAmbient;
uniform vec3 uSkyColor;
uniform vec3 uCamPos;
uniform float uFogStart;
uniform float uFogEnd;

out vec4 FragColor;

void main() {
  // Simple directional + ambient lighting
  vec3 N = normalize(vNormal);
  float NdotL = max(dot(N, -uSunDir), 0.0);
  vec3 diffuse = uSunColor * NdotL;
  vec3 lit = vColor * (uAmbient + diffuse);

  // Distance fog
  float dist = length(vWorldPos - uCamPos);
  float fog = clamp((dist - uFogStart) / (uFogEnd - uFogStart), 0.0, 1.0);
  lit = mix(lit, uSkyColor, fog);

  FragColor = vec4(lit, 1.0);
}
)";

// ---------------------------------------------------------------------------
// Pipeline implementation
// ---------------------------------------------------------------------------

void VoxelRenderPipeline::init(int width, int height) {
  create_shaders();
  create_framebuffer(width, height);
}

void VoxelRenderPipeline::create_shaders() {
  chunk_shader_.compile_shader_from_source(chunk_vs, chunk_fs);
}

void VoxelRenderPipeline::create_framebuffer(int w, int h) {
  width_ = w;
  height_ = h;

  color_tex_.create(GL_TEXTURE_2D);
  color_tex_.set_data(w, h, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
  color_tex_.set_parameters({{GL_TEXTURE_MIN_FILTER, GL_LINEAR},
                             {GL_TEXTURE_MAG_FILTER, GL_LINEAR}});

  depth_tex_.create(GL_TEXTURE_2D);
  depth_tex_.set_data(w, h, GL_DEPTH_COMPONENT24, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
  depth_tex_.set_parameters({{GL_TEXTURE_MIN_FILTER, GL_NEAREST},
                             {GL_TEXTURE_MAG_FILTER, GL_NEAREST}});

  fbo_.create();
  fbo_.bind();
  fbo_.begin_draw_buffers();
  fbo_.attach_color_buffer(color_tex_, GL_COLOR_ATTACHMENT0);
  fbo_.end_draw_buffers();
  fbo_.attach_depth_buffer(depth_tex_);
  if (!fbo_.check_status())
    std::cerr << "[VoxelRenderPipeline] framebuffer incomplete!\n";
  fbo_.unbind();
}

void VoxelRenderPipeline::resize(int width, int height) {
  if (width == width_ && height == height_) return;
  color_tex_.del();
  depth_tex_.del();
  fbo_.del();
  create_framebuffer(width, height);
}

void VoxelRenderPipeline::shutdown() {
  chunk_shader_.del();
  color_tex_.del();
  depth_tex_.del();
  fbo_.del();
}

void VoxelRenderPipeline::render(World &world, transform &cam_trans,
                                 camera &cam, float screen_w, float screen_h) {
  // Compute camera matrices
  math::matrix4 view = math::lookat(
      cam_trans.world_pos(),
      cam_trans.world_pos() + cam_trans.local_forward(),
      cam_trans.local_up());
  math::matrix4 proj = math::perspective(
      math::deg_to_rad(cam.fovy_degree),
      screen_w / screen_h,
      cam.z_near, cam.z_far);
  math::matrix4 vp = proj * view;

  // Bind our FBO
  fbo_.bind();
  fbo_.set_viewport(0, 0, width_, height_);

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);
  glClearColor(sky_color.x(), sky_color.y(), sky_color.z(), 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  chunk_shader_.use();
  chunk_shader_.set_mat4("uVP", vp);
  chunk_shader_.set_vec3("uSunDir", sun_dir);
  chunk_shader_.set_vec3("uSunColor", sun_color);
  chunk_shader_.set_vec3("uAmbient", ambient);
  chunk_shader_.set_vec3("uSkyColor", sky_color);
  chunk_shader_.set_vec3("uCamPos", cam_trans.world_pos());
  chunk_shader_.set_float("uFogStart", fog_start);
  chunk_shader_.set_float("uFogEnd", fog_end);

  world.for_each_chunk([&](Chunk &chunk) {
    if (chunk.index_count == 0) return;
    chunk.mesh_vao.bind();
    glDrawElements(GL_TRIANGLES, chunk.index_count, GL_UNSIGNED_INT, nullptr);
    chunk.mesh_vao.unbind();
  });

  glDisable(GL_CULL_FACE);
  fbo_.unbind();
}

} // namespace craft
