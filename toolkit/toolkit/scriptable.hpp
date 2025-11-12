#pragma once

#include "toolkit/system.hpp"

namespace toolkit {

struct camera;
class transform;

/**
 * This should be the base class of all custom sub_system.
 *
 * The construction and deconstruction of class sub_system derived won't do
 * initialization and cleanup properly, please do initialization inside `start`
 * and cleanup inside `destroy`.
 *
 * `start` function would gets executed at the start of the next main loop when
 * one script is created and added to a valid entity. So deserializing scripts
 * from scenes and bundles would still gets correct `start` functions since the
 * member variables are already properly deserialized when the next main loop
 * starts.
 */
class sub_system : public icomponent {
public:
  sub_system() {}

  virtual void start() {}
  virtual void destroy() {}

  virtual void draw_to_scene(entt::registry &registry, transform &cam_trans,
                             camera &cam_comp) {}

  virtual void preupdate(entt::registry &registry, float dt) {}
  virtual void update(entt::registry &registry, float dt) {}
  virtual void lateupdate(entt::registry &registry, float dt) {}

  virtual void fixedupdate(entt::registry &registry, float dt) {}

  void __fixedupdate_caller__(entt::registry &registry, float dt) {
    double residual = __cur_time - __cur_exec_fixed * fixed_interval;
    while (residual > fixed_interval) {
      residual -= fixed_interval;
      fixedupdate(registry, static_cast<float>(fixed_interval));
      __cur_exec_fixed += 1;
    }
    __cur_time += dt;
  }

  bool enabled = true;

  entt::registry *registry_ptr = nullptr;
  entt::entity entity = entt::null;

protected:
  std::int64_t __cur_exec_fixed = 0;
  double __cur_time = 0.0f, fixed_interval = 1.0f / 60.0f;
};

class sub_system_handler : public isystem {
public:
  static inline std::map<
      std::string,
      std::function<void(entt::registry &,
                         std::function<void(entt::entity, sub_system *)>)>>
      sub_system_views;
  static inline std::map<std::string, std::function<void(entt::registry &)>>
      __construct_destroy_registry__;

  std::vector<sub_system *> sub_system_wait_to_start;

  void init0(entt::registry &registry) override {
    sub_system_wait_to_start.clear();
    for (auto &f : __construct_destroy_registry__)
      f.second(registry);
  }

  void init1(entt::registry &registry) override {}

  void draw_gui(entt::registry &registry, entt::entity entity) override {
    auto ptr = registry.ctx().get<iapp *>();
    for (auto &sv : sub_system_views) {
      sv.second(registry, [&](entt::entity it_entity, sub_system *ss) {
        if (it_entity == entity) {
          if (ImGui::CollapsingHeader(ss->get_name().c_str())) {
            ImGui::Checkbox("Active", &ss->enabled);
            ImGui::Separator();
            if (!ss->enabled)
              ImGui::BeginDisabled();
            ss->draw_gui(registry, entity);
            if (!ss->enabled)
              ImGui::EndDisabled();
          }
        }
      });
    }
  }
  void draw_to_scene(entt::registry &registry, transform &cam_trans,
                     camera &cam_comp) {
    for (auto &sv : sub_system_views) {
      sv.second(registry, [&](entt::entity it_entity, sub_system *ss) {
        if (ss->enabled)
          ss->draw_to_scene(registry, cam_trans, cam_comp);
      });
    }
  }

  void preupdate(entt::registry &registry, float dt) override {
    if (sub_system_wait_to_start.size() > 0) {
      for (auto ss : sub_system_wait_to_start)
        ss->start();
      sub_system_wait_to_start.clear();
    }
    for (auto &sv : sub_system_views) {
      sv.second(registry, [&](entt::entity it_entity, sub_system *ss) {
        if (ss->enabled)
          ss->preupdate(registry, dt);
      });
    }
  }
  void update(entt::registry &registry, float dt) override {
    for (auto &sv : sub_system_views) {
      sv.second(registry, [&](entt::entity it_entity, sub_system *ss) {
        if (ss->enabled) {
          ss->update(registry, dt);
          ss->__fixedupdate_caller__(registry, dt);
        }
      });
    }
  }
  void lateupdate(entt::registry &registry, float dt) override {
    for (auto &sv : sub_system_views) {
      sv.second(registry, [&](entt::entity it_entity, sub_system *ss) {
        if (ss->enabled)
          ss->lateupdate(registry, dt);
      });
    }
  }
};
DECLARE_SYSTEM(sub_system_handler)

#define DECLARE_SUB_SYSTEM(class_name, category, ...)                          \
  DECLARE_COMPONENT(class_name, category, enabled __VA_OPT__(, ) __VA_ARGS__)  \
  inline void __on_construct_##class_name(entt::registry &registry,            \
                                          entt::entity entity) {               \
    auto &ss = registry.get<class_name>(entity);                               \
    ss.entity = entity;                                                        \
    ss.registry_ptr = &registry;                                               \
    registry.ctx()                                                             \
        .get<toolkit::iapp *>()                                                \
        ->get_sys<toolkit::sub_system_handler>()                               \
        ->sub_system_wait_to_start.push_back(&ss);                             \
  }                                                                            \
  inline void __on_destroy_##class_name(entt::registry &registry,              \
                                        entt::entity entity) {                 \
    auto &ss = registry.get<class_name>(entity);                               \
    ss.destroy();                                                              \
  }                                                                            \
  struct __register_##class_name {                                             \
    __register_##class_name() {                                                \
      toolkit::sub_system_handler::__construct_destroy_registry__.insert(      \
          std::make_pair(#class_name, [](entt::registry &registry) {           \
            registry.on_construct<class_name>()                                \
                .connect<&__on_construct_##class_name>();                      \
            registry.on_destroy<class_name>()                                  \
                .connect<&__on_destroy_##class_name>();                        \
          }));                                                                 \
      toolkit::sub_system_handler::sub_system_views.insert(std::make_pair(     \
          #class_name,                                                         \
          [](entt::registry &registry,                                         \
             std::function<void(entt::entity, toolkit::sub_system *)>          \
                 &&func) {                                                     \
            registry.view<entt::entity, class_name>().each(                    \
                [&](entt::entity it_entity, class_name &ss) {                  \
                  func(it_entity, &ss);                                        \
                });                                                            \
          }));                                                                 \
    }                                                                          \
  };                                                                           \
  static __register_##class_name __register_instance_##class_name =            \
      __register_##class_name();

}; // namespace toolkit