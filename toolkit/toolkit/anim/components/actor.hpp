#pragma once

#include "toolkit/loaders/motion.hpp"
#include "toolkit/system.hpp"
#include "toolkit/transform.hpp"

namespace toolkit::anim {

struct actor : public icomponent {
  assets::skeleton skel;
  std::vector<bool> joint_active;
  std::vector<entt::entity> ordered_entities;
  std::map<std::string, entt::entity> name_to_entity;
};
DECLARE_COMPONENT(actor, animation, skel, joint_active, ordered_entities,
                  name_to_entity)

void create_actor_with_skeleton(entt::registry &registry,
                                entt::entity container, assets::skeleton &skel);

entt::entity
instantiate_skeleton_data(entt::registry &registry, assets::skeleton &skel,
                          std::vector<entt::entity> &ordered_entities,
                          std::map<std::string, entt::entity> &name_to_entity);

entt::entity create_bvh_actor(entt::registry &registry, std::string filepath);

}; // namespace toolkit::anim