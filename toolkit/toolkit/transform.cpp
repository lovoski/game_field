#include "toolkit/transform.hpp"
#include <spdlog/spdlog.h>

namespace toolkit {

void on_transform_created(entt::registry &registry, entt::entity entity) {
  auto &tf = registry.get<transform>(entity);
  tf.registry = &registry;
  tf.self = entity;
}

void on_transform_destroyed(entt::registry &registry, entt::entity entity) {
  auto parent = registry.get<transform>(entity).m_parent;
  if (parent != entt::null && registry.valid(parent)) {
    auto &parentTf = registry.get<transform>(parent);
    auto it = std::remove(parentTf.m_children.begin(),
                          parentTf.m_children.end(), entity);
    parentTf.m_children.erase(it, parentTf.m_children.end());
  }
}

void destroy_hierarchy(entt::registry &registry, entt::entity root) {
  std::vector<entt::entity> hierarchy;
  std::function<void(entt::entity)> collect_hierarchy_entities =
      [&](entt::entity e) {
        auto &trans = registry.get<transform>(e);
        hierarchy.push_back(e);
        for (auto c : trans.m_children)
          collect_hierarchy_entities(c);
      };
  collect_hierarchy_entities(root);
  for (int i = hierarchy.size() - 1; i >= 0; i--)
    registry.destroy(hierarchy[i]);
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

transform *transform::parent() const {
  if (m_parent == entt::null)
    return nullptr;
  return &(registry->get<transform>(m_parent));
}
std::vector<transform *> transform::children() const {
  std::vector<transform *> ret;
  for (auto c : m_children)
    ret.push_back(&(registry->get<transform>(c)));
  return ret;
}

math::matrix4 transform::update_matrix() {
  Eigen::Transform<float, 3, 2> transform =
      Eigen::Transform<float, 3, 2>::Identity();
  transform.translate(m_pos).rotate(m_rot).scale(m_scale);
  m_matrix = transform.matrix();
  return m_matrix;
}

void transform::set_transform_matrix(math::matrix4 t) {}

void transform::set_global_position(math::vector3 p) {
  m_pos = p;
  m_local_pos = world_to_parent_local(p);
  dirty = true;
}

void transform::set_global_scale(math::vector3 s) {
  m_local_scale = s.array() / parent_scale().array();
  m_scale = s;
  dirty = true;
}

void transform::set_global_rotation(math::quat q) {
  m_rot = q;
  math::quat pRot = parent_rotation();
  m_local_rot = pRot.inverse() * m_rot;
  m_local_euler = math::rad_to_deg(math::quat_to_euler(m_local_rot));
  update_local_axes();
  dirty = true;
}

void transform::set_local_position(math::vector3 p) {
  m_local_pos = p;
  dirty = true;
}

void transform::set_local_scale(math::vector3 s) {
  m_local_scale = s;
  dirty = true;
}

void transform::set_local_rotation(math::quat q) {
  m_local_rot = q;
  m_local_euler = math::rad_to_deg(math::quat_to_euler(m_local_rot));
  dirty = true;
}

void transform::set_local_euler_degrees(math::vector3 a) {
  m_local_rot = math::euler_to_quat(math::deg_to_rad(a));
  m_local_euler = a;
  dirty = true;
}

void transform::add_child(entt::entity child) {
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
    auto it =
        std::find(cpTrans.m_children.begin(), cpTrans.m_children.end(), child);
    if (it != cpTrans.m_children.end())
      cpTrans.m_children.erase(it);
  }
  cTrans.m_parent = self;
  m_children.push_back(child);
  cTrans.set_global_position(cTrans.m_pos);
  cTrans.set_global_rotation(cTrans.m_rot);
  cTrans.set_global_scale(cTrans.m_scale);
}

void transform::clear_relations() {
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

bool transform::remove_parent() {
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

void transform::update_local_axes() {
  math::quat q = parent_rotation() * m_local_rot;
  m_local_forward = q * math::world_forward;
  m_local_left = q * math::world_left;
  m_local_up = q * math::world_up;
}

const math::vector3 transform::world_to_local(math::vector3 world) {
  _M << m_local_left, m_local_up, m_local_forward;
  return _M.inverse() * (world - m_pos);
}
const math::vector3 transform::local_to_world(math::vector3 local) {
  _M << m_local_left, m_local_up, m_local_forward;
  return (_M * local) + m_pos;
}
const math::vector3 transform::world_to_parent_local(math::vector3 world) {
  math::vector3 pLocalForward, pLocalLeft, pLocalUp;
  get_parent_local_axes(pLocalForward, pLocalLeft, pLocalUp);
  _M_p << pLocalLeft, pLocalUp, pLocalForward;
  return _M_p.inverse() * (world - parent_position());
}
const math::vector3 transform::parent_local_to_world(math::vector3 local) {
  math::vector3 pLocalForward, pLocalLeft, pLocalUp;
  get_parent_local_axes(pLocalForward, pLocalLeft, pLocalUp);
  _M_p << pLocalLeft, pLocalUp, pLocalForward;
  return (_M_p * local) + parent_position();
}

void transform::get_parent_local_axes(math::vector3 &pLocalForward,
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

const math::vector3 transform::parent_position() const {
  if (m_parent != entt::null)
    return registry->get<transform>(m_parent).m_pos;
  else
    return math::vector3::Zero();
}

const math::quat transform::parent_rotation() const {
  if (m_parent != entt::null)
    return registry->get<transform>(m_parent).m_rot;
  else
    return math::quat::Identity();
}

const math::vector3 transform::parent_scale() const {
  if (m_parent != entt::null)
    return registry->get<transform>(m_parent).m_scale;
  else
    return math::vector3::Ones();
}

void transform::force_update_all() {
  if (auto ptr = registry->ctx().get<iapp *>()->get_sys<transform_system>()) {
    ptr->update_transform(*registry);
  } else {
    spdlog::error("Failed to find transform system.");
  }
}

void transform_system::draw_gui(entt::registry &registry, entt::entity entity) {
  if (auto trans = registry.try_get<transform>(entity)) {
    if (ImGui::CollapsingHeader("Transform")) {
      auto position = trans->position();
      auto localEuler = trans->local_euler_degrees();
      auto scale = trans->local_scale();
      if (ImGui::DragFloat3("Positions", position.data(), 0.01f))
        trans->set_global_position(position);
      if (ImGui::DragFloat3("Rotations", localEuler.data(), 0.01f))
        trans->set_local_euler_degrees(localEuler);
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
      root_entities.insert(ent);
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
      trans.m_pos = trans.parent_local_to_world(trans.m_local_pos);
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