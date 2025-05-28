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
      spdlog::info("Entity {0} draw to scene, active entitiies {1}", entt::to_integral(entity), (uint64_t)&active_joint_entities);
      collect_skeleton_draw_queue(*actor_ptr);
      opengl::draw_bones(draw_queue, cam_comp.vp, bone_color);
      // get the average length of bone
      float avg_bone_length = 0.0f;
      for (int i = 0; i < draw_queue.size(); i++)
        avg_bone_length += (draw_queue[i].first - draw_queue[i].second).norm();
      avg_bone_length /= draw_queue.size();

      if (draw_axes) {
        x_dir.clear();
        y_dir.clear();
        z_dir.clear();
        x_dir.reserve(active_joint_entities.size());
        y_dir.reserve(active_joint_entities.size());
        z_dir.reserve(active_joint_entities.size());
        for (auto joint_entity : active_joint_entities) {
          auto &joint_trans = eptr->registry.get<transform>(joint_entity);
          x_dir.emplace_back(std::make_pair(joint_trans.position(),
                                            joint_trans.position() +
                                                axes_length * avg_bone_length *
                                                    joint_trans.local_left()));
          y_dir.emplace_back(std::make_pair(joint_trans.position(),
                                            joint_trans.position() +
                                                axes_length * avg_bone_length *
                                                    joint_trans.local_up()));
          z_dir.emplace_back(std::make_pair(
              joint_trans.position(),
              joint_trans.position() +
                  axes_length * avg_bone_length * joint_trans.local_forward()));
        }
        opengl::draw_arrows(x_dir, cam_comp.vp, opengl::Red,
                            0.1f * axes_length * avg_bone_length);
        opengl::draw_arrows(y_dir, cam_comp.vp, opengl::Green,
                            0.1f * axes_length * avg_bone_length);
        opengl::draw_arrows(z_dir, cam_comp.vp, opengl::Blue,
                            0.1f * axes_length * avg_bone_length);
      }
      if (draw_spheres) {
        joint_positions.clear();
        joint_positions.reserve(active_joint_entities.size());
        for (auto joint_entity : active_joint_entities) {
          joint_positions.push_back(
              eptr->registry.get<transform>(joint_entity).position());
        }
        opengl::draw_wire_spheres(joint_positions, cam_comp.vp,
                                  0.08f * avg_bone_length, bone_color);
      }
    }
  });
}

void vis_skeleton::draw_gui(iapp *app) {
  ImGui::Checkbox("Draw Axes", &draw_axes);
  ImGui::DragFloat("Axes Size", &axes_length, 0.05f, 0.0f, 1.0f);
  ImGui::Checkbox("Draw Spheres", &draw_spheres);
  gui::color_edit_3("Bone Color", bone_color);
}

void vis_skeleton::collect_skeleton_draw_queue(actor &actor_comp) {
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
        registry->get<transform>(actor_comp.ordered_entities[entry.first])
            .position(),
        registry->get<transform>(actor_comp.ordered_entities[entry.second])
            .position()));
}

}; // namespace toolkit::anim