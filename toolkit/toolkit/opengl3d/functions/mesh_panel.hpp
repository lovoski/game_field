// #pragma once

// #include "toolkit/opengl/base.hpp"
// #include "toolkit/opengl/editor.hpp"
// #include "toolkit/scriptable.hpp"
// #include "toolkit/system.hpp"

// namespace toolkit::opengl {

// class mesh_panel : public sub_system {
// public:
//   void draw_gui(entt::registry &registry, entt::entity entity) override;
//   void draw_to_scene(entt::registry &registry, transform &cam_trans,
//                      camera &cam_comp) override;

// private:
//   REFLECT_PRIVATE(mesh_panel)
//   std::vector<entt::entity> mesh_entities;
//   std::vector<std::pair<std::string, float>> blendshape_weights;  // Preserves order
// };
// DECLARE_SUB_SYSTEM(mesh_panel, utils, mesh_entities)

// }; // namespace toolkit::opengl