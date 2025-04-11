#include "ToolKit/Transform.hpp"

namespace toolkit {

void on_transform_created(entt::registry &registry, entt::entity entity) {
  auto &tf = registry.get<transform>(entity);
  tf.registry = &registry;
  tf.self = entity;
}

void on_transform_destroyed(entt::registry &registry, entt::entity entity) {
  if (auto parent = registry.get<transform>(entity).m_parent;
      parent != entt::null) {
    auto &parentTf = registry.get<transform>(parent);
    auto it =
        std::remove(parentTf.m_children.begin(), parentTf.m_children.end(), entity);
    parentTf.m_children.erase(it, parentTf.m_children.end());
  }
}

void transform::reset() {
  m_pos << 0.0, 0.0, 0.0;
  m_local_pos << 0.0, 0.0, 0.0;
  m_scale << 1.0, 1.0, 1.0;
  m_local_scale << 1.0, 1.0, 1.0;
  m_rot = math::quat::Identity();
  m_local_rot = math::quat::Identity();
  m_local_euler << 0.0, 0.0, 0.0;
  m_matrix = math::matrix4::Identity();
  m_local_up << math::world_up;
  m_local_left << math::world_left;
  m_local_forward << math::world_forward;
  dirty = true;

  name = "";

  m_parent = entt::null;
  m_children.clear();
}

math::matrix4 transform::update_matrix() {
  Eigen::Transform<float, 3, 2> transform =
      Eigen::Transform<float, 3, 2>::Identity();
  transform.translate(m_pos).rotate(m_rot).scale(m_scale);
  m_matrix = transform.matrix();
  return m_matrix;
}

}; // namespace toolkit