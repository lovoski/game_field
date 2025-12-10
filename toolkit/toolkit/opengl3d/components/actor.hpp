#pragma once

#include "toolkit/loaders/bvh.hpp"
#include "toolkit/loaders/motion.hpp"
#include "toolkit/system.hpp"
#include "toolkit/transform.hpp"

namespace toolkit::opengl3d {

struct actor : public icomponent {
  std::vector<bool> joint_active;
  std::vector<entt::entity> ordered_entities;
  std::map<std::string, entt::entity> name_to_entity;

  void draw_gui(entt::registry &registry, entt::entity entity) override;
};
DECLARE_COMPONENT(actor, animation, joint_active, ordered_entities,
                  name_to_entity)

struct bone_node : public icomponent {
  std::string name;
  math::matrix4 offset_matrix;
};
DECLARE_COMPONENT(bone_node, data, name, offset_matrix)

/**
 * The parent-child relation in a proxy skeleton is not neccessarily the
 * parent-child relation in the transform hierarchy, local transforms should be
 * computed from global transforms.
 */
struct proxy_skeleton {
  std::vector<entt::entity> ordered_entities;

  std::vector<int> parents;
  std::vector<std::vector<int>> children;
};

void create_actor_with_skeleton(entt::registry &registry,
                                entt::entity container, assets::skeleton &skel);

std::vector<proxy_skeleton> estimate_proxy_skeleton(entt::registry &registry,
                                                    actor &actor_comp);
std::string make_current_pose_bvh(entt::registry &registry,
                                  proxy_skeleton &skel);
std::string proxy_hierarchy_as_bvh_skel(entt::registry &registry,
                                        proxy_skeleton &skel);
std::string proxy_hierarchy_as_bvh_frame(entt::registry &registry,
                                         proxy_skeleton &skel);

void collect_skeleton_draw_queue(
    entt::registry &registry, actor &actor_comp,
    std::vector<std::pair<math::vector3, math::vector3>> &draw_queue);

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

void create_bvh_actor(entt::registry &registry, assets::bvh_data &motion,
                      entt::entity container);

std::tuple<math::vector3, math::vector3>
spring_damper_position(math::vector3 x0, math::vector3 v0, math::vector3 xt,
                       math::vector3 vt, float dt, float halflife);
std::tuple<math::quat, math::vector3>
spring_damper_rotation(math::quat q0, math::vector3 av0, math::quat qt,
                       math::vector3 avt, float dt, float halflife);

void inertialize_transition_position(std::vector<math::vector3> &off_pos,
                                     std::vector<math::vector3> &off_vel,
                                     std::vector<math::vector3> src_pos,
                                     std::vector<math::vector3> src_vel,
                                     std::vector<math::vector3> target_pos,
                                     std::vector<math::vector3> target_vel);

void inertialize_transition_rotation(std::vector<math::quat> &off_rot,
                                     std::vector<math::vector3> &off_ang,
                                     std::vector<math::quat> src_rot,
                                     std::vector<math::vector3> src_ang,
                                     std::vector<math::quat> target_rot,
                                     std::vector<math::vector3> target_ang);

std::tuple<math::vector3, math::vector3, math::vector3, math::vector3>
inertialize_update_position(math::vector3 off_pos, math::vector3 off_vel,
                            math::vector3 in_pos, math::vector3 in_vel,
                            float halflife, float dt);

std::tuple<math::quat, math::vector3, math::quat, math::vector3>
inertialize_update_rotation(math::quat off_rot, math::vector3 off_ang,
                            math::quat in_rot, math::vector3 in_ang,
                            float halflife, float dt);

}; // namespace toolkit::opengl3d