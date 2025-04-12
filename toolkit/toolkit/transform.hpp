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
  math::vector3 local_euler_angles() const { return m_local_euler; }
  math::vector3 local_up() const { return m_local_up; }
  math::vector3 local_forward() const { return m_local_forward; }
  math::vector3 local_left() const { return m_local_left; }

  void set_global_position(math::vector3 p) {
    m_pos = p;
    m_local_pos = world_to_local(p);
    dirty = true;
  }
  void set_global_scale(math::vector3 s) {
    m_local_scale = s.array() / parent_scale().array();
    m_scale = s;
    dirty = true;
  }
  void set_global_rotation(math::quat q) {
    m_rot = q;
    math::quat pRot = parent_rotation();
    m_local_rot = pRot.inverse() * m_rot;
    m_local_euler = math::rad_to_deg(math::quat_to_euler(m_local_rot));
    update_local_axes();
    dirty = true;
  }
  void set_local_position(math::vector3 p) {
    m_local_pos = p;
    dirty = true;
  }
  void set_local_scale(math::vector3 s) {
    m_local_scale = s;
    dirty = true;
  }
  void set_local_rotation(math::quat q) {
    m_local_rot = q;
    m_local_euler = math::rad_to_deg(math::quat_to_euler(m_local_rot));
    dirty = true;
  }
  void set_local_euler_angles(math::vector3 a) {
    m_local_rot = math::euler_to_quat(math::deg_to_rad(a));
    m_local_euler = a;
    dirty = true;
  }

  void reset();
  math::matrix4 matrix() const { return m_matrix; }
  math::matrix4 update_matrix();

  void add_children(entt::entity child) {
    if (child == entt::null) {
      std::cout << "Can't add nullptr as a child entity." << std::endl;
      return;
    }
    auto &cTrans = registry->get<transform>(child);
    if (m_parent != entt::null) {
      auto current = m_parent;
      while (current != entt::null) {
        if (current == child) {
          std::cout << "Can't add ancestor as a child entity." << std::endl;
          return;
        }
        current = registry->get<transform>(current).m_parent;
      }
    }
    if (cTrans.m_parent != entt::null) {
      auto &cpTrans = registry->get<transform>(cTrans.m_parent);
      // remove the entity off its parent
      auto it = std::find(cpTrans.m_children.begin(), cpTrans.m_children.end(),
                          child);
      if (it != cpTrans.m_children.end())
        cpTrans.m_children.erase(it);
    }
    cTrans.m_parent = self;
    m_children.push_back(child);
    cTrans.set_global_position(cTrans.m_pos);
    cTrans.set_global_rotation(cTrans.m_rot);
    cTrans.set_global_scale(cTrans.m_scale);
  }

  void clear_relations() {
    if (m_parent != entt::null) {
      auto &pTrans = registry->get<transform>(m_parent);
      auto it =
          std::find(pTrans.m_children.begin(), pTrans.m_children.end(), self);
      if (it != pTrans.m_children.end())
        pTrans.m_children.erase(it);
      m_parent = entt::null;
    }
    for (auto c : m_children)
      registry->get<transform>(c).m_parent = entt::null;
    m_children.clear();
  }

  bool remove_parent() {
    if (m_parent != entt::null) {
      auto &pTrans = registry->get<transform>(m_parent);
      auto it =
          std::find(pTrans.m_children.begin(), pTrans.m_children.end(), self);
      if (it != pTrans.m_children.end())
        pTrans.m_children.erase(it);
      m_parent = entt::null;
      return true;
    } else
      return false;
  }

  void update_local_axes() {
    math::quat q = parent_rotation() * m_local_rot;
    m_local_forward = q * math::world_forward;
    m_local_left = q * math::world_left;
    m_local_up = q * math::world_up;
  }

  const math::vector3 world_to_local(math::vector3 world) {
    math::vector3 pLocalForward, pLocalLeft, pLocalUp;
    get_parent_local_axes(pLocalForward, pLocalLeft, pLocalUp);
    _M_p << pLocalLeft, pLocalUp, pLocalForward;
    _M << math::world_left, math::world_up, math::world_forward;
    return _M_p.inverse() * _M * (world - parent_position());
  }

  const math::vector3 local_to_world(math::vector3 local) {
    math::vector3 pLocalForward, pLocalLeft, pLocalUp;
    get_parent_local_axes(pLocalForward, pLocalLeft, pLocalUp);
    _M_p << pLocalLeft, pLocalUp, pLocalForward;
    _M << math::world_left, math::world_up, math::world_forward;
    return (_M.inverse() * _M_p * local) + parent_position();
  }

  void get_parent_local_axes(math::vector3 &pLocalForward,
                             math::vector3 &pLocalLeft,
                             math::vector3 &pLocalUp) {
    if (m_parent == entt::null) {
      pLocalForward = math::world_forward;
      pLocalLeft = math::world_left;
      pLocalUp = math::world_up;
    } else {
      auto &pTrans = registry->get<transform>(m_parent);
      // parent->update_local_axes();
      pLocalForward = pTrans.m_local_forward;
      pLocalLeft = pTrans.m_local_left;
      pLocalUp = pTrans.m_local_up;
    }
  }

  const math::vector3 parent_position() const {
    if (m_parent != entt::null)
      return registry->get<transform>(m_parent).m_pos;
    else
      return math::vector3::Zero();
  }

  const math::quat parent_rotation() const {
    if (m_parent != entt::null)
      return registry->get<transform>(m_parent).m_rot;
    else
      return math::quat::Identity();
  }

  const math::vector3 parent_scale() const {
    if (m_parent != entt::null)
      return registry->get<transform>(m_parent).m_scale;
    else
      return math::vector3::Ones();
  }

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