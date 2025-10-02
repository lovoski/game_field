#pragma once

#include "toolkit/opengl/base.hpp"
#include "toolkit/opengl/editor.hpp"
#include "toolkit/scriptable.hpp"
#include "toolkit/system.hpp"

namespace toolkit::opengl {

class mesh_modifier : public scriptable {
public:
  void draw_gui(toolkit::iapp *app) override;
  void draw_to_scene(toolkit::iapp *app) override;

private:
  bool _convex_created = false;
  std::vector<assets::mesh_vertex> _convex_vertices;
  std::vector<std::uint32_t> _convex_indices;
};
DECLARE_SCRIPT(mesh_modifier, utils)

}; // namespace toolkit::opengl