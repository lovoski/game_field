#pragma once

#include "toolkit/opengl/base.hpp"
#include "toolkit/opengl/draw.hpp"
#include "toolkit/system.hpp"

#include "toolkit/opengl/components/camera.hpp"
#include "toolkit/opengl/components/lights.hpp"
#include "toolkit/opengl/components/mesh.hpp"

namespace toolkit::opengl {

struct light_data_pacakge {
  /**
   * [0]: type of light, 0 for dir_light, 1 for point_light
   */
  int idata[4];
  math::vector4 pos;
  math::vector4 color;
  math::vector4 fdata0;
  math::vector4 fdata1;
};

class defered_forward_mixed : public isystem {
public:
  void init0(entt::registry &registry) override;
  void init1(entt::registry &registry) override;

  void resize(int width, int height);

  void preupdate(entt::registry &registry, float dt) override;
  void render(entt::registry &registry);

  /**
   * Gets called inside `render`, apply mesh translation, blendshape and
   * skinning if neccessary. Update the variable `scene_vertex_buffer` and
   * `scene_index_buffer`.
   * The bounding boxes for all meshes will also gets computed in this function.
   * Mind that the bounding boxes need to apply the transform matrices to
   * correctly represent its true position. (except from skinned mesh, skinned
   * mesh bounding boxes always represents itself.)
   */
  void update_scene_buffers(entt::registry &registry);

  void update_scene_lights(entt::registry &registry);

  texture get_target_texture() const { return color_tex; }

  void draw_gui(entt::registry &registry, entt::entity entity) override;

  void draw_menu_gui() override;

  bool should_draw_grid = true;
  int grid_spacing = 1;

  int msaa_samples = 8;

  bool should_draw_debug = true;

  bool enable_ao_pass = false;
  int ao_filter_size = 9;
  float ao_filter_sigma = 6.0f;
  float ssao_noise_scale = 64.0f, ssao_radius = 0.2f;

protected:
  framebuffer gbuffer, cbuffer, msaa_buffer;
  shader gbuffer_geometry_pass, defered_phong_pass;
  texture pos_tex, normal_tex, gbuffer_depth_tex, mask_tex;
  unsigned int msaa_color_buffer, msaa_depth_buffer;

  framebuffer ao_buffer;
  texture ao_color;

  // uniform buffer storing all active lights
  buffer light_data_buffer;

  texture color_tex;

  // scene unique buffer
  vao scene_vao;
  buffer scene_vertex_buffer, scene_index_buffer;
  int work_group_size = 256, scene_mesh_counter = 0;
  compute_shader collect_scene_vertex_buffer_program;
  compute_shader collect_scene_index_buffer_program;
  compute_shader scene_buffer_apply_blendshape_program;
  compute_shader scene_buffer_apply_mesh_skinning_program;

  buffer skeleton_matrices_buffer;

  int64_t scene_vertex_counter = 0, scene_index_counter = 0;
};
DECLARE_SYSTEM(defered_forward_mixed, should_draw_grid, grid_spacing,
               should_draw_debug)

}; // namespace toolkit::opengl
