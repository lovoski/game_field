#include "toolkit/anim/components/actor.hpp"
#include "toolkit/anim/scripts/vis.hpp"

namespace toolkit::anim {

entt::entity
instantiate_skeleton_data(entt::registry &registry, assets::skeleton &skel,
                          std::vector<entt::entity> &ordered_entities,
                          std::map<std::string, entt::entity> &name_to_entity) {
  name_to_entity.clear();
  ordered_entities.clear();
  // create entities
  int njoints = skel.joint_names.size();
  ordered_entities.resize(njoints, entt::null);
  std::vector<transform *> joint_trans(njoints, nullptr);
  for (int i = 0; i < njoints; i++) {
    auto ent = registry.create();
    auto &trans = registry.emplace<transform>(ent);
    joint_trans[i] = registry.try_get<transform>(ent);
    trans.name = skel.joint_names[i];
    trans.set_local_pos(skel.joint_offset[i]);
    trans.set_local_rot(skel.joint_rotation[i]);
    trans.set_local_scale(skel.joint_scale[i]);
    name_to_entity[trans.name] = ent;
    ordered_entities[i] = ent;
  }
  // build up parent child relations
  for (int i = 0; i < njoints; i++) {
    if (i != 0) {
      joint_trans[i]->m_parent =
          name_to_entity[skel.joint_names[skel.joint_parent[i]]];
    }
    for (auto cid : skel.joint_children[i])
      joint_trans[i]->m_children.push_back(
          name_to_entity[skel.joint_names[cid]]);
  }
  // returns root joint entity
  return ordered_entities[0];
}

void create_actor_with_skeleton(entt::registry &registry,
                                entt::entity container,
                                assets::skeleton &skel) {
  auto &actor_comp = registry.emplace<actor>(container);
  auto &actor_trans = registry.get<transform>(container);
  auto skel_root = instantiate_skeleton_data(
      registry, skel, actor_comp.ordered_entities, actor_comp.name_to_entity);
  actor_comp.joint_active.resize(actor_comp.ordered_entities.size(), true);
  actor_trans.add_child(skel_root);
}

entt::entity create_bvh_actor(entt::registry &registry, std::string filepath) {
  auto container = registry.create();
  auto &container_trans = registry.emplace<transform>(container);
  auto &vis_script = registry.emplace<vis_skeleton>(container);
  container_trans.name = std::filesystem::path(filepath).filename().string();
  assets::bvh_motion motion_data;
  motion_data.load(filepath);
  create_actor_with_skeleton(registry, container, motion_data.skel);
  return container;
}

std::tuple<std::vector<int>, std::vector<std::vector<int>>, std::vector<int>>
estimate_actor_bone_hierarchy(entt::registry &registry, actor &actor_comp,
                              bool active_only) {
  int njoints = actor_comp.ordered_entities.size();
  std::vector<int> roots;
  std::vector<int> parent(njoints, -1);
  std::vector<std::vector<int>> children(njoints);
  std::map<entt::entity, int> bone_entities;
  for (int i = 0; i < actor_comp.ordered_entities.size(); i++)
    bone_entities.insert(std::make_pair(actor_comp.ordered_entities[i], i));

  auto find_closest_bone_parent = [&](entt::entity e) {
    entt::entity c = registry.get<transform>(e).m_parent;
    while (c != entt::null) {
      if (bone_entities.count(c) > 0) {
        if (!active_only || actor_comp.joint_active[bone_entities[c]])
          return bone_entities[c];
      }
      c = registry.get<transform>(c).m_parent;
    }
    return -1;
  };

  // traverse the transform hierarchy, collect parent child relation if valid
  std::stack<std::pair<entt::entity, entt::entity>> q;
  q.push(std::make_pair(actor_comp.ordered_entities[0], entt::null));
  while (!q.empty()) {
    auto current = q.top();
    q.pop();

    int closest_parent_id = find_closest_bone_parent(current.first);
    if (closest_parent_id == -1)
      roots.push_back(bone_entities[current.first]);

    if (bone_entities.count(current.first) > 0) {
      int current_id = bone_entities[current.first];
      int parent_id =
          current.second == entt::null ? -1 : bone_entities[current.second];
      if (!active_only || actor_comp.joint_active[current_id]) {
        parent[current_id] = parent_id;
        if (parent_id != -1)
          children[parent_id].push_back(current_id);
      }
    }

    for (auto c : registry.get<transform>(current.first).m_children) {
      int closest_parent_bone = find_closest_bone_parent(c);
      q.push(std::make_pair(
          c, closest_parent_bone == -1
                 ? entt::null
                 : actor_comp.ordered_entities[closest_parent_bone]));
    }
  }

  return {parent, children, roots};
}

std::string padding_tabs(int n) {
  std::string tabs = "";
  for (int i = 0; i < n; i++)
    tabs += "\t";
  return tabs;
}

std::vector<proxy_skeleton> estimate_proxy_skeleton(entt::registry &registry,
                                                    actor &actor_comp) {
  auto [parents, children, roots] =
      estimate_actor_bone_hierarchy(registry, actor_comp, true);
  std::vector<proxy_skeleton> results;
  for (auto root : roots) {
    proxy_skeleton skel;
    skel.children.resize(parents.size(), std::vector<int>());
    std::stack<int> s, p;
    s.push(root);
    p.push(-1);
    while (!s.empty()) {
      int cur_idx = s.top();
      int parent_idx = p.top();
      s.pop();
      p.pop();

      skel.ordered_entities.push_back(actor_comp.ordered_entities[cur_idx]);
      skel.parents.push_back(parent_idx);
      if (parent_idx != -1)
        skel.children[parent_idx].push_back(skel.ordered_entities.size() - 1);

      for (auto c : children[cur_idx]) {
        s.push(c);
        p.push(skel.ordered_entities.size() - 1);
      }
    }
    results.emplace_back(skel);
  }
  return results;
}

std::string make_current_pose_bvh(entt::registry &registry,
                                  proxy_skeleton &skel) {
  std::string result = "";
  result += proxy_hierarchy_as_bvh_skel(registry, skel);
  result +=
      str_format("\nMOTION\nFrames: %d\nFrame Time: %6f\n", 1, 1.0f / 30.0f);
  result += proxy_hierarchy_as_bvh_frame(registry, skel);
  return result;
}
std::string get_joint_skel_str(entt::registry &registry, proxy_skeleton &skel,
                               int joint_idx, int level) {
  auto &bone_trans = registry.get<transform>(skel.ordered_entities[joint_idx]);
  auto &parent_trans =
      registry.get<transform>(skel.ordered_entities[skel.parents[joint_idx]]);
  auto local_mat = parent_trans.matrix().inverse() * bone_trans.matrix();
  math::vector3 local_pos, local_scale;
  math::quat local_rot;
  math::decompose_transform(local_mat, local_pos, local_rot, local_scale);
  math::vector3 offset = local_pos;
  std::string joint_name = toolkit::replace(bone_trans.name, " ", "_");
  std::string result =
      padding_tabs(level) + "JOINT " + joint_name + "\n" + padding_tabs(level) +
      "{\n" + padding_tabs(level) + "\tOFFSET\t" +
      str_format("%.6f\t%.6f\t%.6f", offset.x(), offset.y(), offset.z()) +
      "\n" + padding_tabs(level) +
      "\tCHANNELS 6 Xposition Yposition Zposition Zrotation Yrotation "
      "Xrotation\n";
  if (skel.children[joint_idx].size() > 0) {
    for (auto child_idx : skel.children[joint_idx])
      result += get_joint_skel_str(registry, skel, child_idx, level + 1);
  } else {
    result += padding_tabs(level + 1) + "End Site\n" + padding_tabs(level + 1) +
              "{\n" + padding_tabs(level + 1) + "\tOFFSET\t0\t0\t0\n" +
              padding_tabs(level + 1) + "}\n";
  }
  result += padding_tabs(level) + "}\n";
  return result;
}
std::string proxy_hierarchy_as_bvh_skel(entt::registry &registry,
                                        proxy_skeleton &skel) {
  std::string result = "";
  auto &root = registry.get<transform>(skel.ordered_entities[0]);
  auto root_rot = root.world_rot();
  root.set_world_rot(math::quat::Identity());
  root.force_update_hierarchy();
  std::string joint_name = toolkit::replace(root.name, " ", "_");

  result = result + "HIERARCHY\nROOT " + joint_name +
           "\n{\n\tOFFSET\t0.00\t0.00\t0.00\n\tCHANNELS 6 Xposition "
           "Yposition Zposition Zrotation Yrotation Xrotation\n";

  for (auto child_idx : skel.children[0])
    result += get_joint_skel_str(registry, skel, child_idx, 1);
  result += "}\n";

  root.set_world_rot(root_rot);
  root.force_update_hierarchy();

  return result;
}
std::string get_joint_6dof_str(entt::registry &registry, proxy_skeleton &skel,
                               int joint_idx) {
  auto &bone_trans = registry.get<transform>(skel.ordered_entities[joint_idx]);
  auto &parent_trans =
      registry.get<transform>(skel.ordered_entities[skel.parents[joint_idx]]);
  auto local_mat = parent_trans.matrix().inverse() * bone_trans.matrix();
  math::vector3 local_pos, local_scale;
  math::quat local_rot;
  math::decompose_transform(local_mat, local_pos, local_rot, local_scale);
  auto angles = math::rad_to_deg(math::quat_to_euler(local_rot));
  math::vector3 offset = local_pos;
  std::string result_template = "%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t";
  std::string result =
      str_format(result_template.c_str(), offset.x(), offset.y(), offset.z(),
                 angles.z(), angles.y(), angles.x());
  for (auto child_idx : skel.children[joint_idx])
    result += get_joint_6dof_str(registry, skel, child_idx);
  return result;
}
std::string proxy_hierarchy_as_bvh_frame(entt::registry &registry,
                                         proxy_skeleton &skel) {
  std::string result = "";
  auto &root = registry.get<transform>(skel.ordered_entities[0]);
  auto root_angles = math::rad_to_deg(math::quat_to_euler(root.world_rot()));
  result +=
      str_format("%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t", root.world_pos().x(),
                 root.world_pos().y(), root.world_pos().z(), root_angles.z(),
                 root_angles.y(), root_angles.x());
  for (auto child_idx : skel.children[0])
    result += get_joint_6dof_str(registry, skel, child_idx);
  result += "\n";
  return result;
}

}; // namespace toolkit::anim