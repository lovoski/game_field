#include "toolkit/transform.hpp"

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
    auto it = std::remove(parentTf.m_children.begin(),
                          parentTf.m_children.end(), entity);
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

void transform_system::draw_gui(entt::registry &registry, entt::entity entity) {
  if (auto trans = registry.try_get<transform>(entity)) {
    if (ImGui::CollapsingHeader("Transform")) {
      auto position = trans->position();
      auto localEuler = trans->local_euler_angles();
      auto scale = trans->local_scale();
      if (ImGui::DragFloat3("Positions", position.data(), 0.01f))
        trans->set_global_position(position);
      if (ImGui::DragFloat3("Rotations", localEuler.data(), 0.01f))
        trans->set_local_euler_angles(localEuler);
      if (ImGui::DragFloat3("   Scales", scale.data(), 0.01f))
        trans->set_global_scale(scale);
    }
  }
}

void transform_system::update_transform(entt::registry &registry) {
  root_entities.clear();
  // find the root entities
  auto view = registry.view<transform>();
  int qFront = 0, qBack = 0, size = view.size() + 1;
  entity_refresh_queue.resize(size);
  view.each([&](entt::entity ent, transform &trans) {
    if (trans.m_parent == entt::null) {
      root_entities.push_back(ent);
      entity_refresh_queue[qBack++] = std::make_pair(trans.dirty, ent);
    }
  });
  // traverse the hierarchy, update transform if dirty
  while (qFront != qBack) {
    auto [dirty, ent] = entity_refresh_queue[qFront];
    qFront = (qFront + 1) % size;
    auto &trans = registry.get<transform>(ent);
    for (auto child : trans.m_children) {
      entity_refresh_queue[qBack] =
          std::make_pair(dirty || registry.get<transform>(child).dirty, child);
      qBack = (qBack + 1) % size;
    }
    // update global positions with local positions
    if (dirty) {
      trans.m_pos = trans.local_to_world(trans.m_local_pos);
      // the global rotation is constructed from m_local_rot
      // all dirty children will get this updated parent orientation
      trans.m_rot = trans.parent_rotation() * trans.m_local_rot;
      trans.m_scale =
          trans.parent_scale().array() * trans.m_local_scale.array();
      trans.update_matrix();
      trans.update_local_axes();
      trans.dirty = false;
    }
  }
}

}; // namespace toolkit