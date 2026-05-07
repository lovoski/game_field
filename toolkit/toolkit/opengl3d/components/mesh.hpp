#pragma once

#include "toolkit/opengl3d/assets.hpp"
#include "toolkit/opengl3d/base.hpp"
#include "toolkit/system.hpp"

namespace toolkit::opengl3d {

enum material_type {
  OPAQUE_LIT,
  OPAQUE_UNLIT,
  TRANSPARENT,
};

enum skinning_algorithm_type {
  SKINNING_LBS = 0,
  SKINNING_DUAL_QUATERNION = 1,
};

struct material_data {
  // render type to a texture, so we can use branching in a huge shader to
  // render different materials.
  material_type type = material_type::OPAQUE_LIT;
  math::vector3 albedo = opengl3d::White;

  // render wireframe to mask_tex.b
  bool wireframe = true;
  float wireframe_width = 0.1f;
  float wireframe_smooth = 1.0f;

  texture albedo_tex;
  std::filesystem::path albedo_tex_filepath = "";

  texture normal_tex;
  std::filesystem::path normal_tex_filepath = "";
};
REFLECT(material_data, albedo, wireframe, wireframe_width, wireframe_smooth,
        albedo_tex_filepath, normal_tex_filepath)

struct mesh_data : public icomponent {
  std::string mesh_name, model_name;
  std::vector<assets::mesh_vertex> vertices;
  std::vector<uint32_t> indices;
  std::vector<assets::blend_shape> blendshapes;

  vao vertex_array;
  buffer vertex_buffer, index_buffer;
  std::vector<buffer> blendshape_targets;

  bool should_render_mesh = true;
  int64_t scene_vertex_offset = 0, scene_index_offset = 0;

  // axis aligned bounding box with identity transform
  math::vector3 bb_min = math::vector3::Zero(), bb_max = math::vector3::Zero();

  // default material data
  material_data material;

  bool skinned = false;

  void draw_gui(entt::registry &registry, entt::entity entity) override;

  void draw(GLenum mode = GL_TRIANGLES);

  nlohmann::json late_serialize(entt::registry &registry,
                                entt::entity entity) override;
  void late_deserialize(entt::registry &registry, entt::entity entity,
                        nlohmann::json &data) override;

  bool force_update_flag = false;
  void update_buffers();
};
DECLARE_COMPONENT(mesh_data, data, mesh_name, model_name, should_render_mesh,
                  material)

struct skinned_mesh_bundle : public icomponent {
  framebuffer shadowmap_fb;
  texture shadowmap_depth;
  bool gl_initialized = false;
  int skinning_algorithm = SKINNING_LBS;
  math::matrix4 shadow_vp;
  std::array<math::vector4, 6> vis_planes;
  void try_setup();

  std::vector<std::pair<std::string, float>>
      blendshape_weights; // temporary variable, no need for serialization
  void draw_gui(entt::registry &registry, entt::entity entity) override;

  // actor related
  float actor_axes_length = 1.0f, actor_bone_alpha = 1.0f;
  bool actor_draw_skeleton = true, actor_draw_axes = false,
       actor_draw_spheres = true, actor_draw_names = false,
       actor_bones_on_top = true;
  math::vector3 actor_bone_color = math::vector3(0, 1, 0);
  std::vector<bool> actor_draw;
  std::vector<entt::entity> actor_entities;

  // temporary variables
  std::set<entt::entity> _actor_active_joint_entities;
  std::vector<math::vector3> _actor_joint_positions;
  std::vector<std::pair<math::vector3, math::vector3>> _actor_draw_queue,
      _actor_x_dir, _actor_y_dir, _actor_z_dir;

  math::vector3 bb_min = math::vector3::Zero(), bb_max = math::vector3::Zero();
  std::vector<entt::entity> bone_entities, mesh_entities;
};
DECLARE_COMPONENT(skinned_mesh_bundle, data, skinning_algorithm, bone_entities,
                  mesh_entities, actor_entities, actor_draw, actor_axes_length,
                  actor_bone_alpha, actor_draw_skeleton, actor_draw_axes,
                  actor_draw_spheres, actor_draw_names, actor_bones_on_top,
                  actor_bone_color)

void init_opengl_buffers(mesh_data &data);

void draw_mesh_data(mesh_data &data, GLenum mode = GL_TRIANGLES);

entt::entity create_cube(entt::registry &registry,
                         math::matrix4 t = math::matrix4::Identity());
entt::entity create_plane(entt::registry &registry,
                          math::matrix4 t = math::matrix4::Identity());
entt::entity create_sphere(entt::registry &registry,
                           math::matrix4 t = math::matrix4::Identity());
entt::entity create_cylinder(entt::registry &registry,
                             math::matrix4 t = math::matrix4::Identity());

}; // namespace toolkit::opengl3d
