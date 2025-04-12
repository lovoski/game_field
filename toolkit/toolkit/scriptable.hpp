#pragma once

#include "toolkit/system.hpp"

namespace toolkit {

class scriptable {
public:
  scriptable() { scriptable::scripts.insert(this); }

  virtual void draw_to_scene(iapp *app) {}
  virtual void draw_gui(iapp *app) {}

  virtual void preupdate(iapp *app, float dt) {}
  virtual void update(iapp *app, float dt) {}
  virtual void lateupdate(iapp *app, float dt) {}

  virtual nlohmann::json serialize() const { return nlohmann::json(); }
  virtual void deserialize(const nlohmann::json &j) {}

  bool enabled = true;

  static inline std::set<scriptable *> scripts;
};

class script_system : public isystem {
public:
  void init0(entt::registry &registry) override { scriptable::scripts.clear(); }

  void preupdate(iapp *app, float dt) {
    for (auto script : scriptable::scripts)
      if (script->enabled)
        script->preupdate(app, dt);
  }
  void update(iapp *app, float dt) {
    for (auto script : scriptable::scripts)
      if (script->enabled)
        script->update(app, dt);
  }
  void lateupdate(iapp *app, float dt) {
    for (auto script : scriptable::scripts)
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