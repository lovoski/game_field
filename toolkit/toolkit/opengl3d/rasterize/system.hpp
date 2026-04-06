#pragma once

#include "toolkit/opengl3d/base.hpp"
#include "toolkit/opengl3d/draw.hpp"
#include "toolkit/system.hpp"

#include "toolkit/opengl3d/components/camera.hpp"
#include "toolkit/opengl3d/components/mesh.hpp"
#include "toolkit/opengl3d/compute/tools.hpp"

#include "toolkit/opengl3d/effects/sky.hpp"

namespace toolkit::opengl3d {

class defered_render_system : public isystem {
public:
  void init0(entt::registry &registry) override;
  void init1(entt::registry &registry) override;

  void resize(int width, int height);

  void preupdate(entt::registry &registry);
  void render(entt::registry &registry, transform &cam_trans, camera &cam_comp);

  void update_scene_buffers(entt::registry &registry);
  void update_scene_lights(entt::registry &registry);
  void update_scene_data_structures(entt::registry &registry);

  void save_color_buffer_as_png(std::string filepath);

  texture get_target_texture() const { return color_tex; }

  void draw_menu_gui() override;

  math::vector2 get_render_size() {
    return math::vector2(canvas_width, canvas_height);
  }

  void push_custom_draw(std::function<void(void)> f) {
    custom_draw_func.push_back(std::move(f));
  }

  bool should_draw_grid = true;
  int grid_spacing = 1;

  bool should_draw_debug = true;
  bool enable_wireframe = true;
  bool show_textures_wnd = false;
  void show_textures_wnd_func();

  bool enable_ao_pass = false;
  int ao_filter_size = 9;
  float ao_filter_sigma = 6.0f;
  float ssao_noise_scale = 64.0f, ssao_radius = 0.2f;

  bool enable_sun = true;
  float sun_turbidity = 2.5f, sun_h = 0.0f, sun_v = 80.0f;
  math::vector3 sun_color = math::vector3(0.9, 0.9, 0.9);
  // direction point away from the sun
  math::vector3 sun_direction;
  preetham_sun_sky ss_model;

protected:
  unsigned int canvas_width = 1920, canvas_height = 1080;

  // background
  bool use_pure_color_bg = false;
  math::vector3 bg_color = math::vector3(1.0f, 1.0f, 1.0f);

  shader gbuffer_geometry_pass, defered_default_pass;
  // gbuffer
  framebuffer gbuffer;
  texture pos_tex, normal_tex, gbuffer_depth_tex, mask_tex, albedo_tex;
  // cbuffer
  framebuffer cbuffer;
  texture cbuffer_depth, color_tex, color_backup_tex;

  framebuffer ao_buffer;
  texture ao_color;

  float shadowmap_max_bias = 0.00006f, shadowmap_min_bias = 0.00002f;
  float light_mask_shadow_weight = 0.5f;

  bool enable_fxaa = false;
  shader fxaa_program;

  // scene shadow related
  framebuffer csm_buffer, scene_light_mask_buffer;
  texture csm_depth_atlas, scene_light_mask_tex;
  shader shadow_depth_program, csm_selection_mask_program, shadow_mask_program,
      static_mesh_light_mask_program;
  int num_cascades = 3, csm_depth_dim = 2048, pcf_kernal_size = 1;
  texture noise_tex_random;
  bool enable_csm = true, csm_debug_cascades = false;
  float csm_split_lambda = 0.93f, csm_cascades[10];
  float csm_normal_offset_scale = 1.5f, csm_bias_scale = 1.0f;
  float csm_texel_sizes[10];
  void resize_csm_buffer();
  void compute_csm_matrices(camera &cam_comp, transform &cam_trans);
  // csm cache
  buffer csm_vp_matrix_buffer;
  std::vector<math::matrix4> csm_vp_matrix;
  std::array<math::vector4, 6> csm_frustom_planes;

  // scene unique buffer
  vao scene_vao;
  buffer scene_vertex_buffer, scene_index_buffer;
  int work_group_size = 256, scene_mesh_counter = 0;
  compute_shader collect_scene_vertex_buffer_program;
  compute_shader collect_scene_index_buffer_program;
  compute_shader scene_buffer_apply_blendshape_program;
  compute_shader scene_buffer_apply_mesh_skinning_program;
  per_mesh_global_aabb per_mesh_global_aabb_program;

  buffer skeleton_matrices_buffer;

  int64_t scene_vertex_counter = 0, scene_index_counter = 0;

  std::map<entt::entity, bool> main_cam_visible_cache;

  std::vector<std::function<void(void)>> custom_draw_func;

  REFLECT_PRIVATE(defered_render_system)
};
DECLARE_SYSTEM(defered_render_system, should_draw_grid, grid_spacing,
               should_draw_debug, enable_ao_pass, ao_filter_size,
               ao_filter_sigma, ssao_noise_scale, ssao_radius, enable_sun,
               sun_v, sun_h, sun_turbidity, sun_color, enable_csm,
               num_cascades, csm_depth_dim, pcf_kernal_size, csm_split_lambda,
               csm_normal_offset_scale, csm_bias_scale, shadowmap_max_bias,
               shadowmap_min_bias, enable_fxaa, canvas_width, canvas_height,
               use_pure_color_bg, bg_color, enable_wireframe)

}; // namespace toolkit::opengl3d
