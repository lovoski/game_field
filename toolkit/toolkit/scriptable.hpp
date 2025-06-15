#pragma once

#include "toolkit/system.hpp"

namespace toolkit {

/**
 * This should be the base class of all custom scripts.
 *
 * The construction and deconstruction of class scriptable derived won't do
 * initialization and cleanup properly, please do initialization inside `start`
 * and cleanup inside `destroy`.
 *
 * `start` function would gets executed at the start of the next main loop when
 * one script is created and added to a valid entity. So deserializing scripts
 * from scenes and prefabs would still gets correct `start` functions since the
 * member variables are already properly deserialized when the next main loop
 * starts.
 *
 * To write scripts for opengl::editor, you can access the editor's member
 * variable by `dynamic_cast<opengl::editor*>(app)`, some global variables like
 * dimension of the window, the scene, input signals, active camera etc. are
 * stored in `opengl::g_instance`.
 */
class scriptable : public icomponent {
public:
  scriptable() {}

  virtual void start() {}
  virtual void destroy() {}

  virtual void draw_to_scene(iapp *app) {}

  virtual void preupdate(iapp *app, float dt) {}
  virtual void update(iapp *app, float dt) {}
  virtual void lateupdate(iapp *app, float dt) {}

  bool enabled = true;

  entt::registry *registry = nullptr;
  entt::entity entity = entt::null;
};

/**
 * !!!REMINDER!!!
 *
 * The preupdate, update and lateupdate in `script_system` are not the same ones
 * as the templates in `isystem` since we need to handle properties related to
 * the application, don't forget the call these functions manually in the custom
 * main loop.
 */
class script_system : public isystem {
public:
  static inline std::map<
      std::string,
      std::function<void(entt::registry &,
                         std::function<void(entt::entity, scriptable *)>)>>
      script_views;
  static inline std::map<std::string, std::function<void(entt::registry &)>>
      __construct_destroy_registry__;

  std::vector<scriptable *> scripts_wait_to_start;

  void init0(entt::registry &registry) override {
    scripts_wait_to_start.clear();
    for (auto &f : __construct_destroy_registry__)
      f.second(registry);
  }

  void init1(entt::registry &registry) override {}

  void draw_gui(entt::registry &registry, entt::entity entity) override {
    auto ptr = registry.ctx().get<iapp *>();
    for (auto &sv : script_views) {
      sv.second(registry, [&](entt::entity it_entity, scriptable *script) {
        if (it_entity == entity) {
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
      });
    }
  }
  void draw_to_scene(iapp *app) {
    for (auto &sv : script_views) {
      sv.second(app->registry, [&](entt::entity it_entity, scriptable *script) {
        if (script->enabled)
          script->draw_to_scene(app);
      });
    }
  }

  void preupdate(iapp *app, float dt) {
    if (scripts_wait_to_start.size() > 0) {
      for (auto script : scripts_wait_to_start)
        script->start();
      scripts_wait_to_start.clear();
    }
    for (auto &sv : script_views) {
      sv.second(app->registry, [&](entt::entity it_entity, scriptable *script) {
        if (script->enabled)
          script->preupdate(app, dt);
      });
    }
  }
  void update(iapp *app, float dt) {
    for (auto &sv : script_views) {
      sv.second(app->registry, [&](entt::entity it_entity, scriptable *script) {
        if (script->enabled)
          script->update(app, dt);
      });
    }
  }
  void lateupdate(iapp *app, float dt) {
    for (auto &sv : script_views) {
      sv.second(app->registry, [&](entt::entity it_entity, scriptable *script) {
        if (script->enabled)
          script->lateupdate(app, dt);
      });
    }
  }
};
DECLARE_SYSTEM(script_system)

#define DECLARE_SCRIPT(class_name, category, ...)                              \
  DECLARE_COMPONENT(class_name, category, enabled __VA_OPT__(, ) __VA_ARGS__)  \
  inline void __on_construct_##class_name(entt::registry &registry,            \
                                          entt::entity entity) {               \
    auto &script = registry.get<class_name>(entity);                           \
    script.registry = &registry;                                               \
    script.entity = entity;                                                    \
    registry.ctx()                                                             \
        .get<toolkit::iapp *>()                                                \
        ->get_sys<toolkit::script_system>()                                    \
        ->scripts_wait_to_start.push_back(&script);                            \
  }                                                                            \
  inline void __on_destroy_##class_name(entt::registry &registry,              \
                                        entt::entity entity) {                 \
    auto &script = registry.get<class_name>(entity);                           \
    script.destroy();                                                          \
  }                                                                            \
  struct __register_##class_name {                                             \
    __register_##class_name() {                                                \
      toolkit::script_system::__construct_destroy_registry__.insert(           \
          std::make_pair(#class_name, [](entt::registry &registry) {           \
            registry.on_construct<class_name>()                                \
                .connect<&__on_construct_##class_name>();                      \
            registry.on_destroy<class_name>()                                  \
                .connect<&__on_destroy_##class_name>();                        \
          }));                                                                 \
      toolkit::script_system::script_views.insert(std::make_pair(              \
          #class_name,                                                         \
          [](entt::registry &registry,                                         \
             std::function<void(entt::entity, toolkit::scriptable *)>          \
                 &&func) {                                                     \
            registry.view<entt::entity, class_name>().each(                    \
                [&](entt::entity it_entity, class_name &script) {              \
                  func(it_entity, &script);                                    \
                });                                                            \
          }));                                                                 \
    }                                                                          \
  };                                                                           \
  static __register_##class_name __register_instance_##class_name =            \
      __register_##class_name();

}; // namespace toolkit