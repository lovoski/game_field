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
  for (int i = 0; i < actor_comp.joint_active.size(); i++)
    if (actor_comp.joint_active[i])
      active_joint_entities.insert(actor_comp.ordered_entities[i]);
  auto [parent, children, roots] =
      estimate_actor_bone_hierarchy(*registry, actor_comp, true);
  for (auto root : roots) {
    std::queue<int> q;
    q.push(root);
    while (!q.empty()) {
      auto current = q.front();
      auto &current_trans =
          registry->get<transform>(actor_comp.ordered_entities[current]);
      q.pop();
      for (auto c : children[current]) {
        auto &child_trans =
            registry->get<transform>(actor_comp.ordered_entities[c]);
        draw_queue.emplace_back(
            std::make_pair(current_trans.position(), child_trans.position()));
        q.push(c);
      }
    }
  }
}

}; // namespace toolkit::anim