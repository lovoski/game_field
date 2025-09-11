#pragma once

#include "toolkit/anim/components/actor.hpp"
#include "toolkit/opengl/base.hpp"
#include "toolkit/opengl/draw.hpp"
#include "toolkit/scriptable.hpp"
#include <cnpy.h>

namespace toolkit::anim {

class bvh_motion_player : public scriptable {
public:
  void start() override {}
  void destroy() override {}

  void lateupdate(iapp *app, float dt) override {
    auto actor_comp = registry->try_get<anim::actor>(entity);
    if (actor_comp != nullptr && motion_loaded) {
      for (int i = 0; i < motion.skeleton.get_num_joints(); i++) {
        auto bvh_joint_name = motion.skeleton.joint_names[i];
        if (actor_comp->name_to_entity.find(bvh_joint_name) !=
            actor_comp->name_to_entity.end()) {
          auto &joint_trans = registry->get<transform>(
              actor_comp->name_to_entity[bvh_joint_name]);
        }
      }
    }
  }

  void draw_gui(iapp *app) override {
    ImGui::Text("BVH Motion Path: %s", filepath.c_str());
    if (ImGui::Button("Import Motion", {-1, 30})) {
      if (open_file_dialog("Slect Motion File", {"*.bvh"}, "*.bvh", filepath)) {
        motion.load(filepath);
        motion_loaded = true;
      }
    }
  }

private:
  std::string filepath = "";
  bool motion_loaded = false;
  assets::bvh_motion motion;
};
DECLARE_SCRIPT(bvh_motion_player, animation)

}; // namespace toolkit::anim
