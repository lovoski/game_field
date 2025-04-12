#include "toolkit/system.hpp"

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

  // system init1
  for (auto &ptr : systems)
    ptr->init1(registry);
  // application post-deserialization
  late_deserialize(j);
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