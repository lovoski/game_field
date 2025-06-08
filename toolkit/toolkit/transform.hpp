#pragma once

#include "entt/entity/registry.hpp"
#include <algorithm>
#include <queue>
#include <stack>
#include <toolkit/math.hpp>
#include <toolkit/utils.hpp>
#include <vector>

#include <toolkit/reflect.hpp>
#include <toolkit/system.hpp>

namespace toolkit {

void on_transform_created(entt::registry &registry, entt::entity entity);
void on_transform_destroyed(entt::registry &registry, entt::entity entity);

void destroy_hierarchy(entt::registry &registry, entt::entity root);

class transform : public icomponent {
public:
  friend class transform_system;
  entt::registry *registry = nullptr;
  entt::entity m_parent{entt::null}, self{entt::null};
  std::vector<entt::entity> m_children;
  std::string name = "";
  bool dirty = true;

  math::vector3 position() const { return m_pos; }
  math::vector3 scale() const { return m_scale; }
  math::quat rotation() const { return m_rot; }
  math::vector3 local_position() const { return m_local_pos; }
  math::vector3 local_scale() const { return m_local_scale; }
  math::quat local_rotation() const { return m_local_rot; }
  math::vector3 local_euler_degrees() const { return m_local_euler; }
  math::vector3 local_up() const { return m_local_up; }
  math::vector3 local_forward() const { return m_local_forward; }
  math::vector3 local_left() const { return m_local_left; }

  transform *parent() const;
  std::vector<transform *> children() const;

  void set_world_pos(math::vector3 p);
  void set_world_scale(math::vector3 s);
  void set_world_rot(math::quat q);
  void set_local_pos(math::vector3 p);
  void set_local_scale(math::vector3 s);
  void set_local_rot(math::quat q);
  void set_local_euler_degrees(math::vector3 a);
  void set_world_transform(math::matrix4 t);

  void reset();
  math::matrix4 matrix() const { return m_matrix; }
  math::matrix4 parent_matrix() const;
  math::matrix4 update_matrix();

  void add_child(entt::entity child, bool keep_transform = true);
  void set_parent(entt::entity parent, bool keep_transform = true);
  void clear_relations();
  bool remove_parent();

  void update_local_axes();

  const math::vector3 world_to_local(math::vector3 world);
  const math::vector3 local_to_world(math::vector3 local);

  void parent_local_axes(math::vector3 &p_local_forward,
                         math::vector3 &p_local_left,
                         math::vector3 &p_local_up);

  const math::vector3 parent_position() const;
  const math::quat parent_rotation() const;
  const math::vector3 parent_scale() const;

  void force_update_hierarchy();

private:
  // ---------- cache data for faster reference ----------
  math::vector3 m_pos = math::vector3::Zero();
  math::vector3 m_scale = math::vector3::Ones();
  math::quat m_rot = math::quat::Identity();
  // local euler angle in degrees
  math::vector3 m_local_euler = math::vector3::Zero();
  math::vector3 m_local_up = math::world_up,
                m_local_forward = math::world_forward,
                m_local_left = math::world_left;
  math::matrix4 m_matrix = math::matrix4::Identity();

  // ---------- actual data for hierarchy update ----------
  math::vector3 m_local_pos = math::vector3::Zero();
  math::vector3 m_local_scale = math::vector3::Ones();
  math::quat m_local_rot = math::quat::Identity();

  REFLECT_PRIVATE(transform)
};
DECLARE_COMPONENT(transform, basic, m_local_pos, m_local_scale, m_local_rot,
                  m_local_euler, name, m_parent, m_children)

class transform_system : public isystem {
public:
  void init0(entt::registry &registry) override {
    registry.on_construct<transform>().connect<&on_transform_created>();
    registry.on_destroy<transform>().connect<&on_transform_destroyed>();
  }
  void init1(entt::registry &registry) override {}

  void draw_gui(entt::registry &registry, entt::entity entity) override;

  void update_transform(entt::registry &registry);

  std::set<entt::entity> root_entities;

private:
  std::vector<std::pair<bool, entt::entity>> entity_refresh_queue;
};
DECLARE_SYSTEM(transform_system)

}; // namespace toolkit