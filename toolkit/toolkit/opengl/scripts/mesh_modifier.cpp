#include "toolkit/opengl/scripts/mesh_modifier.hpp"
#include "toolkit/opengl/draw.hpp"

namespace toolkit::opengl {

void mesh_modifier::draw_gui(entt::registry &registry, entt::entity entity) {
  auto mesh_ptr = registry.try_get<opengl::mesh_data>(entity);
  if (ImGui::Button("Create Convex Hull", {-1, 30}) && (mesh_ptr != nullptr)) {
    auto [v, i] = quickhull(mesh_ptr->vertices, mesh_ptr->indices);
    _convex_vertices = std::move(v);
    _convex_indices = std::move(i);
    _convex_created = true;
  }
}

void mesh_modifier::draw_to_scene(entt::registry &registry,
                                  transform &cam_trans, camera &cam_comp) {
  auto mesh_ptr = registry.try_get<opengl::mesh_data>(entity);
  auto &mesh_trans = registry.get<transform>(entity);
  if (_convex_created && (mesh_ptr != nullptr)) {
    auto _render_vertices = _convex_vertices;
    for (int i = 0; i < _convex_vertices.size(); i++) {
      _render_vertices[i].position =
          mesh_trans.matrix() * _convex_vertices[i].position;
    }
    draw_mesh(_render_vertices, _convex_indices, cam_comp.vp);
  }
}

}; // namespace toolkit::opengl