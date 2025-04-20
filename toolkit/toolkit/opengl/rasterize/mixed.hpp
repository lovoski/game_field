#pragma once

#include "toolkit/opengl/base.hpp"
#include "toolkit/opengl/draw.hpp"
#include "toolkit/system.hpp"

#include "toolkit/opengl/components/camera.hpp"
#include "toolkit/opengl/components/lights.hpp"
#include "toolkit/opengl/components/mesh.hpp"

namespace toolkit::opengl {

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
   */
  void update_scene_buffers(entt::registry &registry);
  /**
   * Gets called after `update_scene_buffers`, update the bounding boxes for
   * meshes if neccessary.
   */
  void update_obj_bbox(entt::registry &registry);

  texture get_target_texture() const { return color_tex; }

  void draw_gui(entt::registry &registry, entt::entity entity) override;

  void draw_menu_gui() override;

  bool should_draw_grid = true;
  int grid_spacing = 10, grid_size = 100;

protected:
  framebuffer gbuffer, cbuffer;
  shader gbuffer_geometry_pass;
  texture pos_tex, normal_tex, depth_tex;

  buffer scene_vertex_buffer, scene_index_buffer;

  texture color_tex;
};
DECLARE_SYSTEM(defered_forward_mixed)

}; // namespace toolkit::opengl
