#pragma once

#include "toolkit/anim/components/actor.hpp"
#include "toolkit/opengl/editor.hpp"
#include "toolkit/scriptable.hpp"
#include "toolkit/system.hpp"

class mixamo_manipulate : public toolkit::scriptable {
public:
  void start() override {
    if (auto actor_comp = registry->try_get<toolkit::anim::actor>(entity)) {
      entt::entity target = entt::null, pole = entt::null;
      for (auto &p : actor_comp->name_to_entity) {
        if (toolkit::has_substr(p.first, "LeftFoot") && target == entt::null) {
          target = p.second;
        }
        if (toolkit::has_substr(p.first, "LeftLeg") && pole == entt::null) {
          pole = p.second;
        }
      }
      if (left_foot_target == entt::null) {
        left_foot_target = registry->create();
        auto &lft_trans =
            registry->emplace<toolkit::transform>(left_foot_target);
        lft_trans.name = "left foot ik target";
        lft_trans.set_parent(entity);
        if (target != entt::null)
          lft_trans.set_world_pos(
              registry->get<toolkit::transform>(target).position());
      }
      if (left_foot_pole == entt::null) {
        left_foot_pole = registry->create();
        auto &lfp_trans = registry->emplace<toolkit::transform>(left_foot_pole);
        lfp_trans.name = "left foot ik pole";
        lfp_trans.set_parent(entity);
        if (pole != entt::null)
          lfp_trans.set_world_pos(
              registry->get<toolkit::transform>(pole).position());
      }
    }
  }

  void draw_gui(toolkit::iapp *app) override {}
  void draw_to_scene(toolkit::iapp *app) override {
    toolkit::opengl::script_draw_to_scene_proxy(
        app, [&](toolkit::opengl::editor *editor, toolkit::transform &trans,
                 toolkit::opengl::camera &cam_comp) {
          if (auto actor_comp =
                  registry->try_get<toolkit::anim::actor>(entity)) {
          }
        });
  }

  entt::entity left_foot_target = entt::null, left_foot_pole = entt::null;
};
DECLARE_SCRIPT(mixamo_manipulate, animation, left_foot_target, left_foot_pole)
