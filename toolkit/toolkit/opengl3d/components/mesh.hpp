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
  math::matrix4 shadow_vp;
  std::array<math::vector4, 6> vis_planes;
  void try_setup();

  std::vector<std::pair<std::string, float>>
      blendshape_weights; // temporary variable, no need for serialization
  void draw_gui(entt::registry &registry, entt::entity entity) override;

  math::vector3 bb_min = math::vector3::Zero(), bb_max = math::vector3::Zero();
  std::vector<entt::entity> bone_entities, mesh_entities;
};
DECLARE_COMPONENT(skinned_mesh_bundle, data, bone_entities, mesh_entities)

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
