#include "camdmpp.hpp"
#include "toolkit/opengl3d/components/actor.hpp"

namespace toolkit::opengl3d {

void camdmpp::handle_engine_gui() {
  ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(450, 500), ImGuiCond_Always);
  ImGui::SetNextWindowBgAlpha(0.7f);
  ImGui::Begin("Settings", nullptr,
               ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);

  ImGui::SeparatorText("Inertia Blending");
  ImGui::Checkbox("Camera As Facing Direction", &camera_as_facing_direction);
  ImGui::DragFloat("Rotation HL", &rot_halflife, 0.001f, 0.0f, 1.0f);
  ImGui::DragFloat("Velocity HL", &vel_halflife, 0.001f, 0.0f, 1.0f);
  ImGui::DragFloat("Walk Velocity", &sim_move_speed_walk, 0.01f, 0.0f, 10.0f);
  ImGui::DragFloat("Run Velocity", &sim_move_speed_run, 0.01f, 0.0f, 10.0f);

  // ImGui::SeparatorText("Commands");
  // ImGui::TextColored({0, 1, 0, 1}, "LCTRL+LMB");
  // ImGui::SameLine();
  // ImGui::Text(": Rotate View");
  // ImGui::TextColored({0, 1, 0, 1}, "J");
  // ImGui::SameLine();
  // ImGui::Text(": Previous Style");
  // ImGui::TextColored({0, 1, 0, 1}, "K");
  // ImGui::SameLine();
  // ImGui::Text(": Next Style");

  // ImGui::SeparatorText("Draws");
  // ImGui::DragFloat("Camera Offset", &cam_distance, 0.01f, 0.0f, 10.0f);
  // ImGui::Checkbox("Draw Trajectory", &debug_draw_trajectory);
  // if (ImGui::Checkbox("Draw Ground", &draw_ground_mesh)) {
  //   registry.get<mesh_data>(ground_entity).should_render_mesh =
  //       draw_ground_mesh;
  // }

  ImGui::SeparatorText("Inertia Blending");
  ImGui::Checkbox("Enable", &enable_inertia_blending);
  ImGui::DragFloat("Half Life", &inertia_halflife, 0.0001f, 0.0f, 1.0f);
  ImGui::InputInt("Blend Window", &inertia_blend_wnd);

  ImGui::SeparatorText("Statistics");
  // int cur_style_idx = static_cast<int>(model.style_idx_data[0]);
  // ImGui::Text("Current Style: %s (%d)",
  //             model.style_names[cur_style_idx].c_str(), cur_style_idx);
  ImGui::Text("Buffer In Use: %s", use_front_buffer ? "Front" : "Back");
  ImGui::Text("Inference Time: %.3f ms", display_inference_time);
  ImGui::Text("Applied Frames: %d", applied_frames);

  ImGui::SeparatorText("Post Processing");
  ImGui::Checkbox("Enable Foot Locking", &enable_foot_locking);
  ImGui::Checkbox("Motion Terrain Adjustment", &enable_motion_terrain_adjustment);
  ImGui::Text("IK Value Right: %.3f", ik_value_right);
  ImGui::Text("IK Value Left:  %.3f", ik_value_left);

  ImGui::End();
}

void camdmpp::debug_draw() {
  auto &cam_comp = registry.get<camera>(active_camera);
  auto &cam_trans = registry.get<transform>(active_camera);
  auto &player_trans = registry.get<transform>(player_entity);
  auto &bundle_data = registry.get<skinned_mesh_bundle>(player_entity);
  auto &player_actor = registry.get<actor>(bundle_data.actor_entities[0]);
  auto &root_trans = registry.get<transform>(player_actor.ordered_entities[0]);
  math::vector3 char_pos = root_trans.world_pos();
  math::vector3 proj_char_facing =
      root_trans.world_rot() * math::vector3(0, 0, 1);
  proj_char_facing.y() = 0.0f;
  proj_char_facing.normalize();
  math::quat proj_char_rot =
      math::from_to_rot(math::vector3(0, 0, 1), proj_char_facing);

  std::vector<math::vector3> traj_points_pos;
  std::vector<std::pair<math::vector3, math::vector3>> traj_points_dir;
  for (int i = 0; i < model.future_points; i++) {
    if (i % 5 == 0) {
      math::vector3 _terrain_left = math::world_up.cross(_traj_world_dir[i]);
      if (_terrain_left.squaredNorm() > 1e-6f)
        _terrain_left.normalize();
      else
        _terrain_left = math::vector3(1, 0, 0);
      math::vector2 terrain_left(_terrain_left.x(), _terrain_left.z());

      math::vector2 _xz =
          math::vector2(_traj_world_pos[i].x(), _traj_world_pos[i].z());
      for (int j = 0; j < model.lateral_offsets_m.size(); j++) {
        const math::vector2 offset_xz =
            terrain_left * model.lateral_offsets_m[j];
        const math::vector2 sample_xz = _xz + offset_xz;
        traj_points_pos.push_back(math::vector3(
            sample_xz.x(), _traj_world_height[i][j], sample_xz.y()));
      }

      draw_arrow(math::vector3(_xz.x(), sample_terrain_height(_xz, 0.0f), _xz.y()),
                 math::vector3(_xz.x(), sample_terrain_height(_xz, 0.0f), _xz.y()) + 0.2f * _traj_world_dir[i],
                 cam_comp.vp, Green, 0.01f, 1.0f, false);
      // traj_points_dir.push_back(
      //     std::make_pair(_traj_world_pos[i] + math::vector3(0.0f, 0.01f,
      //     0.0f),
      //                    _traj_world_pos[i] + 0.5f * _traj_world_dir[i] +
      //                        math::vector3(0.0f, 0.01f, 0.0f)));
    }
  }
  draw_spheres(traj_points_pos, cam_comp.vp, 0.01f, Green, false, 1.0f, false);
  // draw_arrows(traj_points_dir, cam_comp.vp, Green, 0.1f, 1.0f, true);
  // draw_wire_sphere(char_root_world_pos, cam_comp.vp, 0.03f, Purple, 10, 1.0f,
  //                  false);

  math::vector2 proj_char_pos_xz = math::vector2(char_pos.x(), char_pos.z());
  draw_sphere(math::vector3(proj_char_pos_xz.x(),
                            sample_terrain_height(proj_char_pos_xz, 0.0f),
                            proj_char_pos_xz.y()),
              cam_comp.vp, 0.03f, Purple, false, 1.0f, true);

  // ik
  for (int i = 0; i < player_actor.ordered_entities.size(); i++) {
    auto &joint_trans =
        registry.get<transform>(player_actor.ordered_entities[i]);
    if (joint_trans.name == "RightToeBase") {
      draw_sphere(joint_trans.world_pos(), cam_comp.vp, 0.04f,
                  Purple * ik_value_right + White * (1.0f - ik_value_right), false,
                  1.0f, false);
    } else if (joint_trans.name == "LeftToeBase") {
      draw_sphere(joint_trans.world_pos(), cam_comp.vp, 0.04f,
                  Purple * ik_value_left + White * (1.0f - ik_value_left), false,
                  1.0f, false);
    } else {
      continue;
    }
  }
}

}; // namespace toolkit::opengl3d