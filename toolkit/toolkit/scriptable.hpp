#pragma once

#include "toolkit/system.hpp"

namespace toolkit {

class scriptable {
public:
  scriptable() { __registered_scripts__.insert(this); }

  virtual void start() {}
  virtual void destroy() {}

  virtual void init1() {}

  virtual void draw_to_scene(iapp *app) {}
  virtual void draw_gui(iapp *app) {}

  virtual void preupdate(iapp *app, float dt) {}
  virtual void update(iapp *app, float dt) {}
  virtual void lateupdate(iapp *app, float dt) {}

  virtual std::string get_name() { return typeid(*this).name(); }

  bool enabled = true;

  entt::registry *registry = nullptr;
  entt::entity entity = entt::null;

  static inline std::set<scriptable *> __registered_scripts__;
};

class script_system : public isystem {
public:
  static inline std::vector<std::function<void(entt::registry &)>>
      __construct_destroy_registry__;

  void init0(entt::registry &registry) override {
    scriptable::__registered_scripts__.clear();
    for (auto &f : __construct_destroy_registry__)
      f(registry);
  }

  void init1(entt::registry &registry) override {
    for (auto script : scriptable::__registered_scripts__)
      script->init1();
  }

  void draw_gui(entt::registry &registry, entt::entity entity) override {
    auto ptr = registry.ctx().get<iapp *>();
    for (auto script : scriptable::__registered_scripts__) {
      if (script->entity == entity) {
        if (ImGui::CollapsingHeader(script->get_name().c_str())) {
          ImGui::Checkbox("Active", &script->enabled);
          ImGui::Separator();
          if (!script->enabled)
            ImGui::BeginDisabled();
          script->draw_gui(ptr);
          if (!script->enabled)
            ImGui::EndDisabled();
        }
      }
    }
  }
  void draw_to_scene(iapp *app) {
    for (auto script : scriptable::__registered_scripts__)
      if (script->enabled)
        script->draw_to_scene(app);
  }

  void preupdate(iapp *app, float dt) {
    for (auto script : scriptable::__registered_scripts__)
      if (script->enabled)
        script->preupdate(app, dt);
  }
  void update(iapp *app, float dt) {
    for (auto script : scriptable::__registered_scripts__)
      if (script->enabled)
        script->update(app, dt);
  }
  void lateupdate(iapp *app, float dt) {
    for (auto script : scriptable::__registered_scripts__)
      if (script->enabled)
        script->lateupdate(app, dt);
  }
};
DECLARE_SYSTEM(script_system)

#define DECLARE_SCRIPT(class_name, category, ...)                              \
  DECLARE_COMPONENT(class_name, category, __VA_ARGS__)                         \
  inline void __on_construct_##class_name(entt::registry &registry,            \
                                          entt::entity entity) {               \
    auto &script = registry.get<class_name>(entity);                           \
    script.registry = &registry;                                               \
    script.entity = entity;                                                    \
    script.start();                                                            \
  }                                                                            \
  inline void __on_destroy_##class_name(entt::registry &registry,              \
                                        entt::entity entity) {                 \
    auto &script = registry.get<class_name>(entity);                           \
    scriptable::__registered_scripts__.erase(&script);                         \
    script.destroy();                                                          \
  }                                                                            \
  struct __register_construct_destroy_##class_name {                           \
    __register_construct_destroy_##class_name() {                              \
      toolkit::script_system::__construct_destroy_registry__.push_back(        \
          [](entt::registry &registry) {                                       \
            registry.on_construct<class_name>()                                \
                .connect<&__on_construct_##class_name>();                      \
            registry.on_destroy<class_name>()                                  \
                .connect<&__on_destroy_##class_name>();                        \
          });                                                                  \
    }                                                                          \
  };                                                                           \
  static __register_construct_destroy_##class_name                             \
      __register_construct_destroy_instance_##class_name =                     \
          __register_construct_destroy_##class_name();

}; // namespace toolkit