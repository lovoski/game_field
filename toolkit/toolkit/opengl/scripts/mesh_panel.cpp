#include "toolkit/opengl/scripts/mesh_panel.hpp"

namespace toolkit::opengl {

void mesh_panel::draw_gui(entt::registry &registry, entt::entity entity) {
  ImGui::SeparatorText("Drop here to add entity");
  entt::entity added_entity = entt::null, removed_entity = entt::null;
  if (ImGui::BeginDragDropTarget()) {
    if (const ImGuiPayload *payload = ImGui::AcceptDragDropPayload("ENTITY")) {
      added_entity = *(entt::entity *)payload->Data;
    }
    ImGui::EndDragDropTarget();
  }
  auto new_mesh_ptr = registry.try_get<mesh_data>(added_entity);
  if ((added_entity != entt::null) && (new_mesh_ptr != nullptr) &&
      (std::find(mesh_entities.begin(), mesh_entities.end(), added_entity) ==
       mesh_entities.end())) {
    mesh_entities.push_back(added_entity);
  }
  if (ImGui::TreeNode("Meshes")) {
    for (int i = 0; i < mesh_entities.size(); i++) {
      if (auto mesh_ptr = registry.try_get<mesh_data>(mesh_entities[i])) {
        auto &mesh_trans = registry.get<transform>(mesh_entities[i]);
        if (ImGui::TreeNode(str_format("entity %d (%s)",
                                       entt::to_integral(mesh_entities[i]),
                                       mesh_trans.name.c_str())
                                .c_str())) {
          mesh_ptr->draw_gui(registry, mesh_entities[i]);
          ImGui::TreePop();
        }
      }
    }
    ImGui::TreePop();
  }
  // display all blendshapes in the same pannel
  if (ImGui::TreeNode("Unified Pannel")) {
    std::vector<transform *> mesh_trans;
    std::vector<mesh_data *> mesh_datas;
    for (int i = 0; i < mesh_entities.size(); i++) {
      mesh_trans.emplace_back(registry.try_get<transform>(mesh_entities[i]));
      mesh_datas.emplace_back(registry.try_get<mesh_data>(mesh_entities[i]));
    }
    ImGui::SeparatorText("Should Render");
    for (int i = 0; i < mesh_entities.size(); i++)
      ImGui::Checkbox(mesh_trans[i]->name.c_str(),
                      &(mesh_datas[i]->should_render_mesh));

    ImGui::SeparatorText("Blend Shapes");
    // Collect all unique blendshape names from all meshes, preserving encounter
    // order
    for (int i = 0; i < mesh_entities.size(); i++) {
      if (mesh_datas[i] == nullptr)
        continue;
      for (int j = 0; j < mesh_datas[i]->blendshapes.size(); j++) {
        const auto &blend_name = mesh_datas[i]->blendshapes[j].name;
        float weight = mesh_datas[i]->blendshapes[j].weight;
        auto it = std::find_if(
            blendshape_weights.begin(), blendshape_weights.end(),
            [&blend_name](const auto &p) { return p.first == blend_name; });
        if (it == blendshape_weights.end()) {
          blendshape_weights.emplace_back(blend_name, weight);
        }
      }
    }
    for (auto &[blend_name, weight] : blendshape_weights) {
      if (ImGui::DragFloat(blend_name.c_str(), &weight, 0.001f, -10.0f, 10.0f)) {
        for (int i = 0; i < mesh_entities.size(); i++) {
          if (mesh_datas[i] == nullptr)
            continue;
          for (int j = 0; j < mesh_datas[i]->blendshapes.size(); j++) {
            if (mesh_datas[i]->blendshapes[j].name == blend_name) {
              mesh_datas[i]->blendshapes[j].weight = weight;
            }
          }
        }
      }
      weight = weight; // Update the stored weight
    }

    ImGui::TreePop();
  }
}

void mesh_panel::draw_to_scene(entt::registry &registry, transform &cam_trans,
                               camera &cam_comp) {}

}; // namespace toolkit::opengl