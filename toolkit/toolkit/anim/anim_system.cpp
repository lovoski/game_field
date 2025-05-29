#include "toolkit/anim/anim_system.hpp"
#include "toolkit/anim/components/actor.hpp"
#include <spdlog/spdlog.h>

namespace toolkit::anim {

void anim_system::draw_gui(entt::registry &registry, entt::entity entity) {
  if (auto ptr = registry.try_get<actor>(entity)) {
    if (ImGui::CollapsingHeader("Actor")) {
      if (ImGui::TreeNode("Skeleton Hierarchy")) {
        draw_skeleton_gui(registry, entity);
        ImGui::TreePop();
      }
    }
  }
}

void export_proxy_skeleton(entt::registry &registry, actor &actor_comp,
                           std::string filepath) {
  //   int activeJointNum = 0;
  //   assets::skeleton proxySkeleton;
  //   std::vector<int> activeJointInd;
  //   std::vector<math::quat> activeJointGlobalRot;
  //   std::vector<math::vector3> activeJointGlobalPos;
  //   std::vector<entt::entity> proxySkeletonEntityMap;
  //   std::map<std::string, int> proxySkeletonNameToInd;
  //   // reset character's pose at rest pose
  //   apply_pose(registry, actor_comp, actor_comp.skel.get_rest_pose(),
  //              actor_comp.skel);

  //   registry.ctx().get<iapp
  //   *>()->get_sys<transform_system>()->update_transform(
  //       registry);
  //   for (int i = 0; i < actor_comp.joint_active.size(); i++) {
  //     if (actor_comp.joint_active[i]) {
  //       activeJointInd.push_back(i);
  //       activeJointNum++;
  //       auto &joint_trans =
  //           registry.get<transform>(actor_comp.ordered_entities[i]);
  //       proxySkeleton.joint_names.push_back(joint_trans.name);
  //       activeJointGlobalPos.push_back(joint_trans.position());
  //       activeJointGlobalRot.push_back(joint_trans.rotation());
  //     }
  //   }
  //   proxySkeleton.as_empty(activeJointNum);
  //   proxySkeletonEntityMap.resize(activeJointNum, entt::null);
  //   for (int i = 0; i < activeJointInd.size(); i++) {
  //     // rebuild proxy joint parent child relation
  //     auto oldCurrentJointInd = activeJointInd[i];
  //     proxySkeletonNameToInd[proxySkeleton.joint_names[i]] = i;
  //     proxySkeletonEntityMap[i] =
  //     actor_comp.ordered_entities[oldCurrentJointInd]; for (int j = i - 1; j
  //     >= 0; j--) {
  //       // check if activeJointInd[j] could be a parent of activeJointInd[i]
  //       auto potentialParentJointInd = activeJointInd[j];
  //       auto cur = actor_comp.skel.joint_parent[oldCurrentJointInd];
  //       bool isParent = false;
  //       while (cur != -1) {
  //         if (cur == potentialParentJointInd) {
  //           isParent = true;
  //           break;
  //         }
  //         cur = actor_comp.skel.joint_parent[cur];
  //       }
  //       if (isParent) {
  //         proxySkeleton.joint_parent[i] = j;
  //         proxySkeleton.joint_children[j].push_back(i);
  //         break;
  //       }
  //     }
  //   }
  //   // convert global position to relative ones
  //   for (int i = 0; i < activeJointNum; i++) {
  //     int parentInd = proxySkeleton.joint_parent[i];
  //     if (parentInd != -1) {
  //       proxySkeleton.joint_offset[i] =
  //           activeJointGlobalPos[i] - activeJointGlobalPos[parentInd];
  //     } else
  //       proxySkeleton.joint_offset[i] = activeJointGlobalPos[i];
  //   }

  //   proxySkeleton.export_as_bvh(filepath);
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

void apply_pose(entt::registry &registry, actor &actor_comp,
                assets::pose pose_data, assets::skeleton &pose_skel) {
  int pose_joint_num = pose_skel.get_num_joints();
  auto root = actor_comp.name_to_entity.find(pose_skel.joint_names[0]);
  if (root == actor_comp.name_to_entity.end()) {
    spdlog::error("root joint {0} not found in skeleton, can't apply motion",
                  pose_skel.joint_names[0]);
    return;
  }
  int missing_joints_num = 0;
  std::vector<std::string> missing_joint_names;
  // apply root translation
  registry.get<transform>(root->second)
      .set_local_position(pose_data.root_local_pos);
  // apply joint rotations for joints defined in the pose
  for (int pose_joint_ind = 0; pose_joint_ind < pose_joint_num;
       ++pose_joint_ind) {
    auto boneName = pose_skel.joint_names[pose_joint_ind];
    auto joint_entity = actor_comp.name_to_entity.find(boneName);
    if (joint_entity == actor_comp.name_to_entity.end()) {
      missing_joints_num++;
      missing_joint_names.push_back(boneName);
    } else {
      registry.get<transform>(joint_entity->second)
          .set_local_rotation(pose_data.joint_local_rot[pose_joint_ind]);
    }
  }
  // output the missing joints
  if (missing_joints_num > 0) {
    std::string concat_name_str = "";
    for (auto &name : missing_joint_names)
      concat_name_str = concat_name_str + ", " + name;
    spdlog::warn("{0} joints missing from entity skeleton, names: {1}",
                 missing_joints_num, concat_name_str);
  }
}

// assets::skeleton active_joint_as_proxy_skeleton(
//     entt::registry &registry, actor &actor_comp,
//     std::vector<entt::entity> &ordered_entities,
//     std::map<std::string, entt::entity> &name_to_entity) {

//   return proxySkeleton;
// }

}; // namespace toolkit::anim