#include "toolkit/anim/anim_system.hpp"
#include "toolkit/anim/components/actor.hpp"
#include "toolkit/anim/scripts/motion_matching.hpp"
#include "toolkit/anim/scripts/traj_tracking.hpp"
#include "toolkit/anim/scripts/motion_player.hpp"
#include "toolkit/anim/scripts/tps_cam_controller.hpp"
#include <spdlog/spdlog.h>

namespace toolkit::anim {

void anim_system::draw_gui(entt::registry &registry, entt::entity entity) {
  if (auto ptr = registry.try_get<actor>(entity)) {
    if (ImGui::CollapsingHeader("Actor")) {
      if (ImGui::TreeNode("Skeleton Hierarchy")) {
        draw_skeleton_gui(registry, entity);
        ImGui::TreePop();
      }
      if (ImGui::Button("Export Current Pose", {-1, 30})) {
        auto bvh_str = make_current_pose_bvh(
            registry, registry.get<transform>(ptr->ordered_entities[0]));
        std::string save_filepath;
        if (save_file_dialog(str_format("Save .bvh pose"), {"*.bvh"}, "*.bvh",
                             save_filepath)) {
          std::ofstream output(save_filepath);
          if (output.is_open())
            output << bvh_str;
          output.close();
        }
      }
    }
  }
}

void draw_skeleton_gui(entt::registry &registry, entt::entity entity) {
  auto &actor_comp = registry.get<actor>(entity);
  // if (ImGui::Button("Export Active Skeleton", {-1, 30})) {
  //   std::string filepath;
  //   if (save_file_dialog("Save active joints as proxy skeleton", {"*.bvh"},
  //                        "*.bvh", filepath))
  //     export_proxy_skeleton(registry, actor_comp, filepath);
  // }
  int num_active_joints = 0, njoints = actor_comp.ordered_entities.size();
  auto [parent, children, roots] =
      estimate_actor_bone_hierarchy(registry, actor_comp);
  for (int i = 0; i < njoints; ++i)
    num_active_joints += actor_comp.joint_active[i] ? 1 : 0;
  ImGui::MenuItem(("Num Joints: " + std::to_string(num_active_joints)).c_str(),
                  nullptr, nullptr, false);

  ImGui::BeginChild("skeletonhierarchy", {-1, -1});
  for (int i = 0; i < njoints; ++i) {
    int depth = 1, cur = i;
    while (parent[cur] != -1) {
      cur = parent[cur];
      depth++;
    }
    std::string depthHeader = "";
    for (int j = 0; j < depth; ++j)
      depthHeader.push_back('-');
    depthHeader.push_back(':');
    bool current_joint_status = actor_comp.joint_active[i];
    if (ImGui::Checkbox(("##" + std::to_string(i)).c_str(),
                        &current_joint_status)) {
      if (!current_joint_status) {
        // disable all children at the disable of parent
        std::queue<int> q;
        q.push(i);
        while (!q.empty()) {
          auto tmpCur = q.front();
          actor_comp.joint_active[tmpCur] = false;
          q.pop();
          for (auto c : children[tmpCur])
            q.push(c);
        }
      } else
        actor_comp.joint_active[i] = true;
    }
    ImGui::SameLine();
    ImGui::Text(
        "%s %s", depthHeader.c_str(),
        registry.get<transform>(actor_comp.ordered_entities[i]).name.c_str());
  }
  ImGui::EndChild();
}

}; // namespace toolkit::anim