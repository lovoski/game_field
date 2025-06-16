#pragma once

#include "toolkit/anim/components/actor.hpp"
#include "toolkit/opengl/editor.hpp"
#include "toolkit/scriptable.hpp"
#include "toolkit/system.hpp"

class mixamo_manipulate : public toolkit::scriptable {
public:
  void start() override {
    if (auto actor_comp = registry->try_get<toolkit::anim::actor>(entity)) {
      if (target == entt::null || pole == entt::null || root == entt::null) {
        for (auto &p : actor_comp->name_to_entity) {
          if (toolkit::has_substr(p.first, "LeftFoot") &&
              target == entt::null) {
            target = p.second;
          }
          if (toolkit::has_substr(p.first, "LeftLeg") && pole == entt::null) {
            pole = p.second;
          }
          if (toolkit::has_substr(p.first, "LeftUpLeg"))
            root = p.second;
        }
      }
      if (left_foot_target == entt::null) {
        left_foot_target = registry->create();
        auto &lft_trans =
            registry->emplace<toolkit::transform>(left_foot_target);
        lft_trans.name = "left foot ik target";
        lft_trans.set_parent(entity);
        if (target != entt::null) {
          lft_trans.set_world_pos(
              registry->get<toolkit::transform>(target).position());
          lft_trans.set_world_rot(
              registry->get<toolkit::transform>(target).rotation());
        }
      }
      if (left_foot_pole == entt::null) {
        left_foot_pole = registry->create();
        auto &lfp_trans = registry->emplace<toolkit::transform>(left_foot_pole);
        lfp_trans.name = "left foot ik pole";
        lfp_trans.set_parent(entity);
        if (pole != entt::null) {
          auto p0 = registry->get<toolkit::transform>(root).position();
          auto p1 = registry->get<toolkit::transform>(pole).position();
          auto p2 = registry->get<toolkit::transform>(target).position();
          toolkit::math::vector3 h02 =
              ((p1 - p0) -
               (p1 - p0).dot((p2 - p0).normalized()) * (p2 - p0).normalized())
                  .normalized();
          lfp_trans.set_world_pos(0.2 * h02 + p1);
        }
      }
    }
  }

  void draw_gui(toolkit::iapp *app) override {}

  toolkit::math::vector3 tp0, tp1, tp2;
  std::vector<toolkit::math::vector3> vis_pos;
  void draw_to_scene(toolkit::iapp *app) override {
    toolkit::opengl::script_draw_to_scene_proxy(
        app, [&](toolkit::opengl::editor *editor, toolkit::transform &trans,
                 toolkit::opengl::camera &cam_comp) {
          if (auto actor_comp =
                  registry->try_get<toolkit::anim::actor>(entity)) {
            vis_pos.clear();
            if (left_foot_target != entt::null) {
              vis_pos.push_back(
                  registry->get<toolkit::transform>(left_foot_target)
                      .position());
            }
            if (left_foot_pole != entt::null) {
              vis_pos.push_back(
                  registry->get<toolkit::transform>(left_foot_pole).position());
            }
            vis_pos.push_back(tp0);
            vis_pos.push_back(tp1);
            vis_pos.push_back(tp2);
            toolkit::opengl::draw_spheres(vis_pos, cam_comp.vp, 0.06f,
                                          toolkit::opengl::Purple);
          }
        });
  }

  void lateupdate(toolkit::iapp *app, float dt) override {
    if (auto actor_comp = registry->try_get<toolkit::anim::actor>(entity)) {
      auto &t0 = registry->get<toolkit::transform>(root);
      auto &t1 = registry->get<toolkit::transform>(pole);
      auto &t2 = registry->get<toolkit::transform>(target);
      auto &pole = registry->get<toolkit::transform>(left_foot_pole);
      auto &target = registry->get<toolkit::transform>(left_foot_target);

      float l01 = (t1.position() - t0.position()).norm();
      float l12 = (t2.position() - t1.position()).norm();
      float dist = (target.position() - t0.position()).norm();

      if (l01 + l12 <= dist) {
        tp0 = t0.position();
        tp1 = l01 * (target.position() - t0.position()).normalized() + tp0;
        tp2 = (l01 + l12) * (target.position() - t0.position()).normalized() +
              tp0;
      } else {
        auto n = (pole.position() - t0.position())
                     .cross(target.position() - t0.position())
                     .normalized();
        auto h = n.cross((t0.position() - target.position()).normalized())
                     .normalized();
        float cos_theta =
            std::clamp((l01 * l01 + dist * dist - l12 * l12) / (2 * l01 * dist),
                       -1.0f, 1.0f);
        float sin_theta = std::sqrt(1 - cos_theta * cos_theta);
        tp0 = t0.position();
        tp2 = target.position();
        tp1 = tp0 + sin_theta * l01 * h +
              cos_theta * l01 * ((tp2 - tp0).normalized());
      }

      auto dq0 =
          toolkit::math::from_to_rot(t1.position() - t0.position(), tp1 - tp0);
      auto rp2 = dq0 * (t2.position() - t0.position()) + t0.position();
      auto dq1 = toolkit::math::from_to_rot(rp2 - tp1, tp2 - tp1);
      t0.set_world_rot(dq0 * t0.rotation());
      t1.set_world_rot(dq1 * t1.parent_rotation() * t1.local_rotation());
      t2.set_world_rot(target.rotation());
    }
  }

  entt::entity target = entt::null, pole = entt::null, root = entt::null;
  entt::entity left_foot_target = entt::null, left_foot_pole = entt::null;
};
DECLARE_SCRIPT(mixamo_manipulate, animation, left_foot_target, left_foot_pole,
               target, pole, root)
