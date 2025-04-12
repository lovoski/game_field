#pragma once

#include "toolkit/reflect.hpp"
#include "toolkit/utils.hpp"
#include <entt.hpp>
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
  iapp() {}
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

  void update(float dt);

  virtual nlohmann::json serialize();
  virtual void late_serialize(nlohmann::json &j) {}
  virtual void deserialize(nlohmann::json &j);
  virtual void late_deserialize(nlohmann::json &j) {}

  entt::registry registry;

  static inline std::map<
      std::string, std::pair<std::function<void(entt::registry &, entt::entity,
                                                nlohmann::json &)>,
                             std::function<void(entt::registry &, entt::entity,
                                                nlohmann::json &)>>>
      __comp_serializer_callbacks__;
  static inline std::map<
      std::string, std::pair<std::function<void(iapp *app, nlohmann::json &)>,
                             std::function<void(iapp *app, nlohmann::json &)>>>
      __sys_serializer_callbacks__;

  static inline std::map<
      std::string,
      std::vector<std::pair<
          std::string, std::function<void(entt::registry &, entt::entity)>>>>
      __add_comp_map__;

protected:
  std::vector<std::shared_ptr<isystem>> systems;
};

#define DECLARE_COMPONENT(class_name, category, ...)                           \
  REFLECT(class_name, __VA_ARGS__)                                             \
public:                                                                        \
  static void __comp_serializer__##class_name(                                 \
      entt::registry &registry, entt::entity entity, nlohmann::json &j) {      \
    if (auto *ptr = registry.try_get<class_name>(entity)) {                    \
      j[get_reflect_name()] = ptr->serialize();                                \
    }                                                                          \
  }                                                                            \
  static void __comp_deserializer__##class_name(                               \
      entt::registry &registry, entt::entity entity, nlohmann::json &j) {      \
    if (j.contains(get_reflect_name())) {                                      \
      auto &comp = registry.emplace<class_name>(entity);                       \
      comp.deserialize(j[get_reflect_name()]);                                 \
    }                                                                          \
  }                                                                            \
  static void __add_comp_##class_name(entt::registry &registry,                \
                                      entt::entity entity) {                   \
    if (auto ptr = registry.try_get<class_name>(entity))                       \
      return;                                                                  \
    registry.emplace<class_name>(entity);                                      \
  }                                                                            \
                                                                               \
private:                                                                       \
  static inline bool _register_##class_name = []() {                           \
    toolkit::iapp::__comp_serializer_callbacks__.insert(                       \
        std::make_pair(get_reflect_name(),                                     \
                       std::make_pair(__comp_serializer__##class_name,         \
                                      __comp_deserializer__##class_name)));    \
    if (toolkit::iapp::__add_comp_map__.find(#category) ==                     \
        toolkit::iapp::__add_comp_map__.end()) {                               \
      toolkit::iapp::__add_comp_map__[#category] = std::vector<                \
          std::pair<std::string,                                               \
                    std::function<void(entt::registry &, entt::entity)>>>();   \
    }                                                                          \
    toolkit::iapp::__add_comp_map__[#category].push_back(                      \
        std::make_pair(get_reflect_name(), __add_comp_##class_name));          \
    return true;                                                               \
  }();

#define DECLARE_SYSTEM(class_name, ...)                                        \
  REFLECT(class_name, __VA_ARGS__)                                             \
public:                                                                        \
  static void __sys_serializer__##class_name(iapp *app, nlohmann::json &j) {   \
    if (auto *ptr = app->get_sys<class_name>()) {                              \
      j[get_reflect_name()] = ptr->serialize();                                \
    }                                                                          \
  }                                                                            \
  static void __sys_deserializer__##class_name(iapp *app, nlohmann::json &j) { \
    if (j.contains(get_reflect_name())) {                                      \
      auto sys = app->add_sys<class_name>();                                   \
      sys->deserialize(j[get_reflect_name()]);                                 \
    }                                                                          \
  }                                                                            \
                                                                               \
private:                                                                       \
  static inline bool _register_serializer_##class_name = []() {                \
    toolkit::iapp::__sys_serializer_callbacks__.insert(                        \
        std::make_pair(get_reflect_name(),                                     \
                       std::make_pair(__sys_serializer__##class_name,          \
                                      __sys_deserializer__##class_name)));     \
    return true;                                                               \
  }();

}; // namespace toolkit