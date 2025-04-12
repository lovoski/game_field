#pragma once

#include <algorithm>
#include <entt.hpp>
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

class transform {
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

  void set_global_position(math::vector3 p);
  void set_global_scale(math::vector3 s);
  void set_global_rotation(math::quat q);
  void set_local_position(math::vector3 p);
  void set_local_scale(math::vector3 s);
  void set_local_rotation(math::quat q);
  void set_local_euler_degrees(math::vector3 a);
  void set_transform_matrix(math::matrix4 t);

  void reset();
  math::matrix4 matrix() const { return m_matrix; }
  math::matrix4 update_matrix();

  void add_children(entt::entity child);
  void clear_relations();
  bool remove_parent();

  void update_local_axes();

  const math::vector3 world_to_local(math::vector3 world);
  const math::vector3 local_to_world(math::vector3 local);

  void get_parent_local_axes(math::vector3 &pLocalForward,
                             math::vector3 &pLocalLeft,
                             math::vector3 &pLocalUp);

  const math::vector3 parent_position() const;
  const math::quat parent_rotation() const;
  const math::vector3 parent_scale() const;

private:
  math::vector3 m_pos = math::vector3::Zero(),
                m_local_pos = math::vector3::Zero();
  math::vector3 m_scale = math::vector3::Ones(),
                m_local_scale = math::vector3::Ones();
  math::quat m_rot = math::quat::Identity(),
             m_local_rot = math::quat::Identity();
  // local euler angle in degrees
  math::vector3 m_local_euler = math::vector3::Zero();

  math::matrix3 _M, _M_p;

  math::vector3 m_local_up = math::world_up,
                m_local_forward = math::world_forward,
                m_local_left = math::world_left;

  math::matrix4 m_matrix = math::matrix4::Identity();

  DECLARE_COMPONENT(transform, basic, m_pos, m_local_pos, m_scale,
                         m_local_scale, m_rot, m_local_up, m_local_forward,
                         m_local_left, m_local_rot, m_local_euler, m_matrix,
                         name, dirty, m_parent, m_children)
};

class transform_system : public isystem {
public:
  void init0(entt::registry &registry) override {
    registry.on_construct<transform>().connect<&on_transform_created>();
    registry.on_destroy<transform>().connect<&on_transform_destroyed>();
  }
  void init1(entt::registry &registry) override {}

  void draw_gui(entt::registry &registry, entt::entity entity) override;

  void update_transform(entt::registry &registry);

  std::vector<entt::entity> root_entities;

private:
  std::vector<std::pair<bool, entt::entity>> entity_refresh_queue;

  DECLARE_SYSTEM(transform_system)
};

}; // namespace toolkit