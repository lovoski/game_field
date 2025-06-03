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
  assets::motion motion_data;
  motion_data.load_from_bvh(filepath);
  create_actor_with_skeleton(registry, container, motion_data.skeleton);
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

}; // namespace toolkit::anim