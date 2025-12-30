#include "toolkit/opengl3d/components/actor.hpp"
#include "toolkit/opengl3d/components/mesh.hpp"
#include "toolkit/opengl3d/gui.hpp"

namespace toolkit::opengl3d {

void skinned_mesh_bundle::try_setup() {
  if (!gl_initialized) {
    shadowmap_fb.create();
    shadowmap_fb.bind();
    shadowmap_depth.create(GL_TEXTURE_2D);
    shadowmap_depth.set_data(4096, 4096, GL_DEPTH_COMPONENT24,
                             GL_DEPTH_COMPONENT, GL_FLOAT);
    shadowmap_depth.set_parameters(
        {{GL_TEXTURE_MIN_FILTER, GL_LINEAR},
         {GL_TEXTURE_MAG_FILTER, GL_LINEAR},
         {GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE},
         {GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL},
         {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
         {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});
    shadowmap_fb.attach_depth_buffer(shadowmap_depth);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    if (!shadowmap_fb.check_status())
      std::cout << "skinned mesh bundle shadow buffer not complete!"
                << std::endl;
    shadowmap_fb.unbind();
    gl_initialized = true;
  }
}

void skinned_mesh_bundle::draw_gui(entt::registry &registry,
                                   entt::entity entity) {
  if (ImGui::TreeNode(
          str_format("Actor (count: %d)", actor_entities.size()).c_str())) {
    ImGui::Checkbox("Draw Skeleton", &actor_draw_skeleton);
    ImGui::Checkbox("Draw Names", &actor_draw_names);
    ImGui::Checkbox("Draw Joint Spheres", &actor_draw_spheres);
    ImGui::Checkbox("Skeleton On Top", &actor_bones_on_top);
    ImGui::Checkbox("Draw Joint Axes", &actor_draw_axes);
    ImGui::DragFloat("Axes Size", &actor_axes_length, 0.005f, 0.0f, 1.0f);
    ImGui::DragFloat("Alpha Blending", &actor_bone_alpha, 0.005f, 0.0f, 1.0f);
    color_edit_3("Bone Color", actor_bone_color);
    for (int i = 0; i < actor_entities.size(); i++) {
      auto actor_entity = actor_entities[i];
      if (ImGui::TreeNode(
              str_format(
                  "Actor Entity %d: (%s)", i,
                  (actor_entity == entt::null)
                      ? "null"
                      : registry.get<transform>(actor_entity).name.c_str())
                  .c_str())) {
        bool cur_actor_draw = actor_draw[i];
        if (ImGui::Checkbox("Draw Actor", &cur_actor_draw))
          actor_draw[i] = cur_actor_draw;
        registry.get<actor>(actor_entity).draw_gui(registry, actor_entity);
        ImGui::TreePop();
      }
      ImGui::Separator();
    }
    ImGui::TreePop();
  }

  if (ImGui::TreeNode(
          str_format("Bone Nodes (count: %d)", bone_entities.size()).c_str())) {
    for (int i = 0; i < bone_entities.size(); i++) {
      auto bone_entity = bone_entities[i];
      ImGui::TextColored(
          (bone_entity == entt::null) ? ImVec4{1, 0, 0, 1} : ImVec4{0, 1, 0, 1},
          "(%d) %s", i,
          (bone_entity == entt::null)
              ? "null"
              : registry.get<transform>(bone_entity).name.c_str());
    }
    ImGui::TreePop();
  }

  if (ImGui::TreeNode("All Meshes")) {
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
      if (ImGui::DragFloat(blend_name.c_str(), &weight, 0.001f, -10.0f,
                           10.0f)) {
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

}; // namespace toolkit::opengl3d