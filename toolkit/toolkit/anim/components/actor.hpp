#pragma once

#include "toolkit/loaders/motion.hpp"
#include "toolkit/system.hpp"
#include "toolkit/transform.hpp"

namespace toolkit::anim {

struct actor : public icomponent {
  std::vector<bool> joint_active;
  std::vector<entt::entity> ordered_entities;
  std::map<std::string, entt::entity> name_to_entity;
};
DECLARE_COMPONENT(actor, animation, joint_active, ordered_entities,
                  name_to_entity)

struct bone_node : public icomponent {
  std::string name;
  math::matrix4 offset_matrix;
};
DECLARE_COMPONENT(bone_node, data, name, offset_matrix)

void create_actor_with_skeleton(entt::registry &registry,
                                entt::entity container, assets::skeleton &skel);

/**
 * We assume at default state, there's only one root node in the provided actor
 * component, by default, we assume it to be actor_comp.ordered_entities[0].
 *
 * The return value is alway parent/children/roots array stored as plain data
 * with length actor_comp.ordered_entities.size().
 *
 * When active_only set to true, only the bones marked as active will have valid
 * parent/children results, inactive joints would have parent set to -1 and
 * empty children.
 */
std::tuple<std::vector<int>, std::vector<std::vector<int>>, std::vector<int>>
estimate_actor_bone_hierarchy(entt::registry &registry, actor &actor_comp,
                              bool active_only = false);

entt::entity
instantiate_skeleton_data(entt::registry &registry, assets::skeleton &skel,
                          std::vector<entt::entity> &ordered_entities,
                          std::map<std::string, entt::entity> &name_to_entity);

entt::entity create_bvh_actor(entt::registry &registry, std::string filepath);

}; // namespace toolkit::anim