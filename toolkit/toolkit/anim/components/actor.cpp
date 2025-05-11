#include "toolkit/anim/components/actor.hpp"
#include "toolkit/anim/scripts/player.hpp"
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
    trans.set_local_position(skel.joint_offset[i]);
    trans.set_local_rotation(skel.joint_rotation[i]);
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

entt::entity create_bvh_actor(entt::registry &registry, std::string filepath) {
  auto container = registry.create();
  auto &container_trans = registry.emplace<transform>(container);
  auto &container_actor = registry.emplace<actor>(container);
  auto &vis_script = registry.emplace<vis_skeleton>(container);
  container_trans.name = std::filesystem::path(filepath).filename().string();
  assets::motion motion_data;
  motion_data.load_from_bvh(filepath);
  auto skeleton_root = instantiate_skeleton_data(
      registry, motion_data.skeleton, container_actor.ordered_entities,
      container_actor.name_to_entity);
  container_actor.joint_active.resize(container_actor.ordered_entities.size(),
                                      true);
  container_actor.skel = motion_data.skeleton;
  container_trans.add_children(skeleton_root);
  return container;
}

entt::entity create_fbx_actor(entt::registry &registry, std::string filepath) {
  return entt::null;
}

}; // namespace toolkit::anim