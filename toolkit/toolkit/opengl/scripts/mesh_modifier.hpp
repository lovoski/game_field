#pragma once

#include "toolkit/opengl/base.hpp"
#include "toolkit/opengl/editor.hpp"
#include "toolkit/scriptable.hpp"
#include "toolkit/system.hpp"

namespace toolkit::opengl {

class mesh_modifier : public sub_system {
public:
  void draw_gui(entt::registry &registry, entt::entity entity) override;
  void draw_to_scene(entt::registry &registry, transform &cam_trans,
                     camera &cam_comp) override;

private:
  bool _convex_created = false;
  std::vector<assets::mesh_vertex> _convex_vertices;
  std::vector<std::uint32_t> _convex_indices;
};
DECLARE_SUB_SYSTEM(mesh_modifier, utils)

}; // namespace toolkit::opengl