#include "toolkit/anim/scripts/vis.hpp"
#include "toolkit/anim/components/actor.hpp"
#include "toolkit/opengl/draw.hpp"
#include "toolkit/opengl/editor.hpp"
#include "toolkit/opengl/gui/utils.hpp"

namespace toolkit::anim {

void vis_skeleton::draw_to_scene(iapp *app) {
  opengl::script_draw_to_scene_proxy(app, [&](opengl::editor *eptr,
                                              transform &cam_trans,
                                              opengl::camera &cam_comp) {
    if (auto actor_ptr = eptr->registry.try_get<actor>(entity)) {
      collect_skeleton_draw_queue(eptr->registry, *actor_ptr, draw_queue,
                                  active_joint_entities);
      opengl::draw_bones(draw_queue, opengl::g_instance.get_scene_size(),
                         cam_comp.vp, bone_color);
      // get the average length of bone
      float avg_bone_length = 0.0f;
      for (int i = 0; i < draw_queue.size(); i++)
        avg_bone_length += (draw_queue[i].first - draw_queue[i].second).norm();
      avg_bone_length /= draw_queue.size();

      if (draw_axes) {
        for (auto joint_entity : active_joint_entities) {
          auto &joint_trans = eptr->registry.get<transform>(joint_entity);
          opengl::draw_arrow(
              joint_trans.position(),
              joint_trans.position() +
                  axes_length * avg_bone_length * joint_trans.local_left(),
              cam_comp.vp, opengl::Red, 0.1f * axes_length * avg_bone_length);
          opengl::draw_arrow(
              joint_trans.position(),
              joint_trans.position() +
                  axes_length * avg_bone_length * joint_trans.local_up(),
              cam_comp.vp, opengl::Green, 0.1f * axes_length * avg_bone_length);
          opengl::draw_arrow(
              joint_trans.position(),
              joint_trans.position() +
                  axes_length * avg_bone_length * joint_trans.local_forward(),
              cam_comp.vp, opengl::Blue, 0.1f * axes_length * avg_bone_length);
        }
      }
      for (auto joint_entity : active_joint_entities) {
        auto &joint_trans = eptr->registry.get<transform>(joint_entity);
        opengl::draw_wire_sphere(joint_trans.position(), cam_comp.vp,
                                 0.08f * avg_bone_length, bone_color);
      }
    }
  });
}

void vis_skeleton::draw_gui(iapp *app) {
  ImGui::Checkbox("Draw Axes", &draw_axes);
  ImGui::DragFloat("Axes Size", &axes_length, 0.05f, 0.0f, 1.0f);
  gui::color_edit_3("Bone Color", bone_color);
}

void collect_skeleton_draw_queue(
    entt::registry &registry, actor &actor_comp,
    std::vector<std::pair<math::vector3, math::vector3>> &draw_queue,
    std::set<entt::entity> &active_joint_entities) {
  draw_queue.clear();
  active_joint_entities.clear();
  // only collect joints defined in the actor
  int njoints = actor_comp.skel.get_num_joints();
  std::vector<int> start_points;
  std::set<std::pair<int, int>> tmp_queue;
  // collect end effectors as start points
  for (int i = 0; i < njoints; ++i) {
    if (actor_comp.skel.joint_children[i].size() == 0)
      start_points.push_back(i);
  }
  // traverse from start points to root joint
  for (auto start_point_ind : start_points) {
    int current = start_point_ind, parent;
    bool currentActive = actor_comp.joint_active[current], parentActive = false;
    entt::entity currentEntity = actor_comp.ordered_entities[current],
                 parentEntity = entt::null;
    int tobe_matched = -1;
    while (actor_comp.skel.joint_parent[current] != -1) {
      parent = actor_comp.skel.joint_parent[current];
      parentEntity = actor_comp.ordered_entities[parent];
      parentActive = actor_comp.joint_active[parent];
      if (currentActive && parentActive) {
        tmp_queue.insert(std::make_pair(parent, current));
        active_joint_entities.insert(actor_comp.ordered_entities[current]);
        active_joint_entities.insert(actor_comp.ordered_entities[parent]);
      } else if (currentActive && !parentActive) {
        tobe_matched = current;
      } else if (!currentActive && parentActive && tobe_matched != -1) {
        tmp_queue.insert(std::make_pair(parent, tobe_matched));
        active_joint_entities.insert(actor_comp.ordered_entities[tobe_matched]);
        active_joint_entities.insert(actor_comp.ordered_entities[parent]);
      }
      current = parent;
      currentActive = parentActive;
      currentEntity = parentEntity;
    }
  }
  for (auto &entry : tmp_queue)
    draw_queue.push_back(std::make_pair(
        registry.get<transform>(actor_comp.ordered_entities[entry.first])
            .position(),
        registry.get<transform>(actor_comp.ordered_entities[entry.second])
            .position()));
}

}; // namespace toolkit::anim