#include "toolkit/system.hpp"
#include "toolkit/transform.hpp"

namespace toolkit {

nlohmann::json iapp::serialize() {
  nlohmann::json reg, sys, all;
  registry.view<entt::entity>().each([&](entt::entity entity) {
    nlohmann::json j;
    for (auto &p0 : __comp_serializer_callbacks__) {
      auto name = p0.first;
      auto &serialize = p0.second.first;
      serialize(registry, entity, j);
    }
    reg[std::to_string(entt::to_integral(entity))] = j;
  });
  for (auto &p0 : __sys_serializer_callbacks__) {
    auto name = p0.first;
    auto &serialize = p0.second.first;
    serialize(this, sys);
  }
  all["registry"] = reg;
  all["systems"] = sys;
  late_serialize(all);
  return all;
}

void iapp::deserialize(nlohmann::json &j) {
  registry.clear();
  systems.clear();
  auto reg = j["registry"];
  auto sys = j["systems"];
  // system data initialization, init0 gets called inside deserialize function
  for (auto &p0 : __sys_serializer_callbacks__) {
    auto name = p0.first;
    auto &deserialize = p0.second.second;
    deserialize(this, sys);
  }

  // create all entities
  for (auto &[k, v] : reg.items()) {
    std::uint32_t id = static_cast<std::uint32_t>(std::stoul(k));
    entt::entity _entity = entt::entity{id};
    auto entity = registry.create(_entity);
  }
  // initialize all components
  registry.view<entt::entity>().each([&](entt::entity entity) {
    auto key = std::to_string(entt::to_integral(entity));
    for (auto &p0 : __comp_serializer_callbacks__) {
      auto name = p0.first;
      auto &deserialize = p0.second.second;
      deserialize(registry, entity, reg[key]);
    }
  });

  // component init1
  registry.view<entt::entity>().each([&](entt::entity entity) {
    for (auto &p : __comp_init1_funcs__) {
      auto name = p.first;
      auto &init1 = p.second;
      init1(registry, entity);
    }
  });

  // system init1
  for (auto &ptr : systems)
    ptr->init1(registry);
  // application post-deserialization
  late_deserialize(j);
}

nlohmann::json iapp::make_prefab(entt::entity root) {
  nlohmann::json data;
  std::vector<entt::entity> hierarchy_entities;
  std::queue<entt::entity> q;
  q.push(root);
  while (!q.empty()) {
    auto ent = q.front();
    hierarchy_entities.push_back(ent);
    q.pop();
    auto &trans = registry.get<transform>(ent);
    for (auto c : trans.m_children)
      q.push(c);
  }

  data["entities"] = hierarchy_entities;
  for (auto ent : hierarchy_entities) {
    nlohmann::json j;
    for (auto &p0 : __comp_serializer_callbacks__) {
      auto name = p0.first;
      auto &serialize = p0.second.first;
      serialize(registry, ent, j);
    }
    data["data"][std::to_string(entt::to_integral(ent))] = j;
  }

  return data;
}

void iapp::load_prefab(nlohmann::json &j) {
  __entity_mapping__.clear();
  std::vector<entt::entity> old_entities =
      j["entities"].get<std::vector<entt::entity>>();
  for (auto old_ent : old_entities) {
    auto new_ent = registry.create();
    __entity_mapping__[old_ent] = new_ent;
  }
  for (auto [k, v] : j["data"].items()) {
    entt::entity old_ent =
        entt::entity{static_cast<std::uint32_t>(std::stoul(k))};
    entt::entity new_ent = __entity_mapping__[old_ent];
    for (auto &p0 : __comp_serializer_callbacks__) {
      auto name = p0.first;
      auto deserialize = p0.second.second;
      deserialize(registry, new_ent, j["data"][k]);
    }
  }
  // init1 for all components
  for (auto &p : __entity_mapping__) {
    auto new_ent = p.second;
    for (auto &fp : __comp_init1_funcs__)
      fp.second(registry, new_ent);
  }
}

void iapp::update(float dt) {
  for (auto sys : systems)
    if (sys->active)
      sys->preupdate(registry, dt);
  for (auto sys : systems)
    if (sys->active)
      sys->update(registry, dt);
  for (auto sys : systems)
    if (sys->active)
      sys->lateupdate(registry, dt);
}

}; // namespace toolkit