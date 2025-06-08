#pragma once

#include "entt/entity/registry.hpp"
#include "toolkit/reflect.hpp"
#include "toolkit/utils.hpp"
#include <imgui.h>
#include <iostream>
#include <json.hpp>

namespace toolkit {

class isystem {
public:
  isystem() {}
  ~isystem() {}

  /**
   * The function gets executed during add_sys and deserialize of iapp, no
   * entities or components are created at the deserialization stage when this
   * function gets called, you can set up some sink functions inside
   * entt::registry here.
   */
  virtual void init0(entt::registry &registry) {}
  /**
   * The function gets executed only during deserialize of iapp, at this time
   * all entities and components are deserilaized, so you can check the status
   * of deserialization, and perform some post deserialization here.
   */
  virtual void init1(entt::registry &registry) {}

  virtual void preupdate(entt::registry &registry, float dt) {}
  virtual void update(entt::registry &registry, float dt) {}
  virtual void lateupdate(entt::registry &registry, float dt) {}

  virtual void draw_menu_gui() {}
  virtual void draw_gui(entt::registry &registry, entt::entity entity) {}
  virtual std::string get_name() { return typeid(*this).name(); }

  bool active = true;
};

class iapp {
public:
  iapp() { registry.ctx().emplace<iapp *>(this); }
  ~iapp() {}

  template <typename SystemType> SystemType *get_sys() {
    static_assert(std::is_base_of_v<isystem, SystemType>,
                  "SystemType should derive from isystem");
    for (auto &sys : systems)
      if (dynamic_cast<SystemType *>(sys.get()) != nullptr)
        return static_cast<SystemType *>(sys.get());
    printf("Required system doesn't exists in app, returns nullptr instead.\n");
    return nullptr;
  }
  template <typename SystemType> SystemType *add_sys() {
    static_assert(std::is_base_of_v<isystem, SystemType>,
                  "SystemType should derive from isystem");
    for (auto &ptr : systems) {
      if (typeid(*(ptr.get())) == typeid(SystemType)) {
        printf("System exists in app, don't create new system "
               "instance.\n");
        return static_cast<SystemType *>(ptr.get());
      }
    }
    systems.emplace_back(std::make_shared<SystemType>());
    systems[systems.size() - 1]->init0(registry);
    return static_cast<SystemType *>(systems[systems.size() - 1].get());
  }

  template <typename T> void view_as(std::function<void(T *)> &&f) {
    if (auto ptr = dynamic_cast<T *>(this))
      f(ptr);
  }

  void update(float dt);

  virtual nlohmann::json serialize();
  virtual void late_serialize(nlohmann::json &j) {}
  virtual void deserialize(nlohmann::json &j);
  virtual void late_deserialize(nlohmann::json &j) {}

  nlohmann::json make_prefab(entt::entity root);
  void load_prefab(nlohmann::json &j);

  entt::registry registry;

  static inline std::map<
      std::string, std::pair<std::function<void(entt::registry &, entt::entity,
                                                nlohmann::json &)>,
                             std::function<void(entt::registry &, entt::entity,
                                                nlohmann::json &)>>>
      __comp_serializer_callbacks__;
  static inline std::map<std::string,
                         std::function<void(entt::registry &, entt::entity)>>
      __comp_init1_funcs__;
  static inline std::map<
      std::string, std::pair<std::function<void(iapp *app, nlohmann::json &)>,
                             std::function<void(iapp *app, nlohmann::json &)>>>
      __sys_serializer_callbacks__;

  static inline std::map<
      std::string, std::map<std::string, std::function<void(entt::registry &,
                                                            entt::entity)>>>
      __add_comp_map__;

  static inline std::map<entt::entity, entt::entity> __entity_mapping__;

  static inline std::vector<
      std::function<void(iapp *, entt::registry &, entt::entity)>>
      __try_draw_gui_funcs__;

protected:
  std::vector<std::shared_ptr<isystem>> systems;
};

class icomponent {
public:
  virtual void init1() {}
  virtual void draw_gui(iapp *app) {}
  virtual std::string get_name() { return typeid(*this).name(); }
};

#define DECLARE_COMPONENT(class_name, category, ...)                           \
  REFLECT(class_name, __VA_ARGS__)                                             \
  inline void __comp_serializer__##class_name(                                 \
      entt::registry &registry, entt::entity entity, nlohmann::json &j) {      \
    if (auto *ptr = registry.try_get<class_name>(entity)) {                    \
      j[#class_name] = *ptr;                                                   \
    }                                                                          \
  }                                                                            \
  inline void __comp_deserializer__##class_name(                               \
      entt::registry &registry, entt::entity entity, nlohmann::json &j) {      \
    if (j.contains(#class_name)) {                                             \
      auto &comp = registry.emplace<class_name>(entity);                       \
      from_json(j[#class_name], comp);                                         \
    }                                                                          \
  }                                                                            \
  inline void __add_comp_##class_name(entt::registry &registry,                \
                                      entt::entity entity) {                   \
    if (auto ptr = registry.try_get<class_name>(entity))                       \
      return;                                                                  \
    registry.emplace<class_name>(entity);                                      \
  }                                                                            \
  inline void __try_draw_gui_##class_name(                                     \
      toolkit::iapp *app, entt::registry &registry, entt::entity entity) {     \
    if (auto ptr = registry.try_get<class_name>(entity)) {                     \
      if (ImGui::CollapsingHeader(ptr->get_name().c_str()))                    \
        ptr->draw_gui(app);                                                    \
    }                                                                          \
  }                                                                            \
  inline void __comp_init1_##class_name(entt::registry &registry,              \
                                        entt::entity entity) {                 \
    if (auto ptr = registry.try_get<class_name>(entity)) {                     \
      ptr->init1();                                                            \
    }                                                                          \
  }                                                                            \
  struct __register_funcs_##class_name {                                       \
    __register_funcs_##class_name() {                                          \
      toolkit::iapp::__comp_serializer_callbacks__.insert(std::make_pair(      \
          #class_name, std::make_pair(__comp_serializer__##class_name,         \
                                      __comp_deserializer__##class_name)));    \
      if (toolkit::iapp::__add_comp_map__.find(std::string(#category)) ==      \
          toolkit::iapp::__add_comp_map__.end()) {                             \
        toolkit::iapp::__add_comp_map__[std::string(#category)] =              \
            std::map<std::string,                                              \
                     std::function<void(entt::registry &, entt::entity)>>();   \
      }                                                                        \
      toolkit::iapp::__add_comp_map__[std::string(#category)].insert(          \
          std::make_pair(std::string(#class_name), __add_comp_##class_name));  \
      toolkit::iapp::__comp_init1_funcs__.insert(                              \
          std::make_pair(#class_name, __comp_init1_##class_name));             \
      toolkit::iapp::__try_draw_gui_funcs__.push_back(                         \
          __try_draw_gui_##class_name);                                        \
    }                                                                          \
  };                                                                           \
  static __register_funcs_##class_name                                         \
      __register_funcs_instance_##class_name =                                 \
          __register_funcs_##class_name();

#define DECLARE_SYSTEM(class_name, ...)                                        \
  REFLECT(class_name, __VA_ARGS__)                                             \
  inline void __sys_serializer__##class_name(iapp *app, nlohmann::json &j) {   \
    if (auto *ptr = app->get_sys<class_name>()) {                              \
      j[#class_name] = *ptr;                                                   \
    }                                                                          \
  }                                                                            \
  inline void __sys_deserializer__##class_name(iapp *app, nlohmann::json &j) { \
    if (j.contains(#class_name)) {                                             \
      auto sys = app->add_sys<class_name>();                                   \
      from_json(j[#class_name], *sys);                                         \
    }                                                                          \
  }                                                                            \
  struct __register_serializer_##class_name {                                  \
    __register_serializer_##class_name() {                                     \
      toolkit::iapp::__sys_serializer_callbacks__.insert(std::make_pair(       \
          #class_name, std::make_pair(__sys_serializer__##class_name,          \
                                      __sys_deserializer__##class_name)));     \
    }                                                                          \
  };                                                                           \
  static __register_serializer_##class_name                                    \
      __register_serializer_instance_##class_name =                            \
          __register_serializer_##class_name();

}; // namespace toolkit

namespace nlohmann {
template <> struct adl_serializer<entt::entity> {
  static void to_json(json &j, const entt::entity &e) {
    j = static_cast<uint32_t>(entt::to_integral(e));
  }

  static void from_json(const json &j, entt::entity &e) {
    uint32_t raw_id = j.get<uint32_t>();
    entt::entity original = entt::entity{raw_id};

    e = original;
    if (toolkit::iapp::__entity_mapping__.find(original) !=
        toolkit::iapp::__entity_mapping__.end()) {
      e = toolkit::iapp::__entity_mapping__[original];
    }
  }
};
} // namespace nlohmann