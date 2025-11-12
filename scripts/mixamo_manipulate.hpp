#pragma once

#include "toolkit/anim/components/actor.hpp"
#include "toolkit/opengl/editor.hpp"
#include "toolkit/scriptable.hpp"
#include "toolkit/system.hpp"

class mixamo_manipulate : public toolkit::sub_system {
public:
  void start() override {
    if (auto actor_comp = registry_ptr->try_get<toolkit::anim::actor>(entity)) {
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
        left_foot_target = registry_ptr->create();
        auto &lft_trans =
            registry_ptr->emplace<toolkit::transform>(left_foot_target);
        lft_trans.name = "left foot ik target";
        lft_trans.set_parent(entity);
        if (target != entt::null) {
          lft_trans.set_world_pos(
              registry_ptr->get<toolkit::transform>(target).world_pos());
          lft_trans.set_world_rot(
              registry_ptr->get<toolkit::transform>(target).world_rot());
        }
      }
      if (left_foot_pole == entt::null) {
        left_foot_pole = registry_ptr->create();
        auto &lfp_trans =
            registry_ptr->emplace<toolkit::transform>(left_foot_pole);
        lfp_trans.name = "left foot ik pole";
        lfp_trans.set_parent(entity);
        if (pole != entt::null) {
          auto p0 = registry_ptr->get<toolkit::transform>(root).world_pos();
          auto p1 = registry_ptr->get<toolkit::transform>(pole).world_pos();
          auto p2 = registry_ptr->get<toolkit::transform>(target).world_pos();
          toolkit::math::vector3 h02 =
              ((p1 - p0) -
               (p1 - p0).dot((p2 - p0).normalized()) * (p2 - p0).normalized())
                  .normalized();
          lfp_trans.set_world_pos(0.2 * h02 + p1);
        }
      }
    }
  }

  void draw_gui(entt::registry &registry, entt::entity entity) override {}

  toolkit::math::vector3 tp0, tp1, tp2;
  std::vector<toolkit::math::vector3> vis_pos;
  void draw_to_scene(entt::registry &registry, toolkit::transform &cam_trans,
                     toolkit::camera &cam_comp) override {
    if (auto actor_comp = registry.try_get<toolkit::anim::actor>(entity)) {
      vis_pos.clear();
      if (left_foot_target != entt::null) {
        vis_pos.push_back(
            registry.get<toolkit::transform>(left_foot_target).world_pos());
      }
      if (left_foot_pole != entt::null) {
        vis_pos.push_back(
            registry.get<toolkit::transform>(left_foot_pole).world_pos());
      }
      vis_pos.push_back(tp0);
      vis_pos.push_back(tp1);
      vis_pos.push_back(tp2);
      toolkit::opengl::draw_spheres(vis_pos, cam_comp.vp, 0.06f,
                                    toolkit::opengl::Purple);
    }
  }

  void lateupdate(entt::registry &registry, float dt) override {
    if (auto actor_comp = registry.try_get<toolkit::anim::actor>(entity)) {
      auto &t0 = registry.get<toolkit::transform>(root);
      auto &t1 = registry.get<toolkit::transform>(pole);
      auto &t2 = registry.get<toolkit::transform>(target);
      auto &pole = registry.get<toolkit::transform>(left_foot_pole);
      auto &target = registry.get<toolkit::transform>(left_foot_target);

      float l01 = (t1.world_pos() - t0.world_pos()).norm();
      float l12 = (t2.world_pos() - t1.world_pos()).norm();
      float dist = (target.world_pos() - t0.world_pos()).norm();

      if (l01 + l12 <= dist) {
        tp0 = t0.world_pos();
        tp1 = l01 * (target.world_pos() - t0.world_pos()).normalized() + tp0;
        tp2 = (l01 + l12) * (target.world_pos() - t0.world_pos()).normalized() +
              tp0;
      } else {
        auto n = (pole.world_pos() - t0.world_pos())
                     .cross(target.world_pos() - t0.world_pos())
                     .normalized();
        auto h = n.cross((t0.world_pos() - target.world_pos()).normalized())
                     .normalized();
        float cos_theta =
            std::clamp((l01 * l01 + dist * dist - l12 * l12) / (2 * l01 * dist),
                       -1.0f, 1.0f);
        float sin_theta = std::sqrt(1 - cos_theta * cos_theta);
        tp0 = t0.world_pos();
        tp2 = target.world_pos();
        tp1 = tp0 + sin_theta * l01 * h +
              cos_theta * l01 * ((tp2 - tp0).normalized());
      }

      auto dq0 = toolkit::math::from_to_rot(t1.world_pos() - t0.world_pos(),
                                            tp1 - tp0);
      auto rp2 = dq0 * (t2.world_pos() - t0.world_pos()) + t0.world_pos();
      auto dq1 = toolkit::math::from_to_rot(rp2 - tp1, tp2 - tp1);
      t0.set_world_rot(dq0 * t0.world_rot());
      t1.set_world_rot(dq1 * t1.parent_rotation() * t1.local_rot());
      t2.set_world_rot(target.world_rot());
    }
  }

  entt::entity target = entt::null, pole = entt::null, root = entt::null;
  entt::entity left_foot_target = entt::null, left_foot_pole = entt::null;
};
DECLARE_SUB_SYSTEM(mixamo_manipulate, animation, left_foot_target,
                   left_foot_pole, target, pole, root)
