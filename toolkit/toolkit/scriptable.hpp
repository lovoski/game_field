#pragma once

#include "toolkit/system.hpp"

namespace toolkit {

class __script_base__ {
public:
  __script_base__() { __registered_scripts__.insert(this); }

  virtual void draw_to_scene(iapp *app) {}
  virtual void draw_gui(iapp *app) {}

  virtual void preupdate(iapp *app, float dt) {}
  virtual void update(iapp *app, float dt) {}
  virtual void lateupdate(iapp *app, float dt) {}

  virtual nlohmann::json serialize() const { return nlohmann::json(); }
  virtual void deserialize(const nlohmann::json &j) {}

  virtual bool belong_to_entity(entt::registry &registry, entt::entity entity) {
    return false;
  }

  virtual std::string get_name() { return ""; }

  bool enabled = true;

  static inline std::set<__script_base__ *> __registered_scripts__;
};

template <typename derived> class scriptable : public __script_base__ {
public:
  virtual bool belong_to_entity(entt::registry &registry, entt::entity entity) {
    return registry.try_get<std::decay_t<derived>>(entity) != nullptr;
  }
  virtual std::string get_name() {
    return typeid(std::decay_t<derived>).name();
  }
};

class script_system : public isystem {
public:
  void init0(entt::registry &registry) override {
    __script_base__::__registered_scripts__.clear();
  }

  void draw_gui(entt::registry &registry, entt::entity entity) override {
    auto ptr = registry.ctx().get<iapp *>();
    for (auto script : __script_base__::__registered_scripts__) {
      if (script->enabled && script->belong_to_entity(registry, entity)) {
        if (ImGui::CollapsingHeader(script->get_name().c_str())) {
          ImGui::Checkbox("Active", &script->enabled);
          script->draw_gui(ptr);
        }
      }
    }
  }
  void draw_to_scene(iapp *app) {
    for (auto script : __script_base__::__registered_scripts__)
      if (script->enabled)
        script->draw_to_scene(app);
  }

  void preupdate(iapp *app, float dt) {
    for (auto script : __script_base__::__registered_scripts__)
      if (script->enabled)
        script->preupdate(app, dt);
  }
  void update(iapp *app, float dt) {
    for (auto script : __script_base__::__registered_scripts__)
      if (script->enabled)
        script->update(app, dt);
  }
  void lateupdate(iapp *app, float dt) {
    for (auto script : __script_base__::__registered_scripts__)
      if (script->enabled)
        script->lateupdate(app, dt);
  }

  DECLARE_SYSTEM(script_system)
};

#define DECLARE_SCRIPT(class_name, category, ...)                              \
  DECLARE_COMPONENT(class_name, category, __VA_ARGS__)                         \
private:                                                                       \
  static inline bool _register_script_##class_name = []() { return true; }();

}; // namespace toolkit