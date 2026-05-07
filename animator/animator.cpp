#include "animator.hpp"

namespace toolkit::opengl3d {

void animator::handle_custom_initialization() {
  if (std::filesystem::exists("animator/setup.scene") &&
      std::filesystem::exists("animator/tpose.bvh")) {
    // load setup scene
    std::ifstream scene_input("animator/setup.scene");
    if (scene_input.is_open()) {
      auto data = nlohmann::json::parse(
          std::string((std::istreambuf_iterator<char>(scene_input)),
                      std::istreambuf_iterator<char>()));
      deserialize(data);
    } else {
      std::cout << "Error loading scene (animator/setup.scene)" << std::endl;
      quit_app_running();
      return;
    }
    if (named_entities.find("player") == named_entities.end()) {
      quit_app_running();
      std::cout
          << "Error loading scene (animator/setup.scene)\n"
             "the setup scene named_entities don't contain field \"player\""
          << std::endl;
      return;
    }
    auto data_tpose = assets::load_bvh("animator/tpose.bvh");
    set_game_mode(true, hide_mouse);
    player_entity = named_entities["player"];
    default_render_sys->resize(wnd_width, wnd_height);
    transform_hierarchy_sys->update_transform(registry);

    motion_data = assets::load_bvh("animator/motion.bvh");

    auto &bundle_data = registry.get<skinned_mesh_bundle>(player_entity);
    auto &player_trans = registry.get<transform>(player_entity);
    auto &player_root_trans =
        registry.get<transform>(bundle_data.actor_entities[0]);
    auto &player_actor = registry.get<actor>(bundle_data.actor_entities[0]);
    player_trans.force_update_hierarchy();
    math::vector3 initial_forward =
        player_root_trans.world_rot() * math::vector3(0, 0, 1);
    initial_forward.y() = 0.0f;
    if (initial_forward.squaredNorm() > 1e-6f) {
      initial_forward.normalize();
    }
    // find mapping between network's prediction joint to actor's joint
    for (int i = 0; i < data_tpose.names.size(); i++) {
      if (player_actor.name_to_entity.find(data_tpose.names[i]) !=
          player_actor.name_to_entity.end()) {
        auto joint_entity = player_actor.name_to_entity[data_tpose.names[i]];
        for (int j = 0; j < player_actor.ordered_entities.size(); j++) {
          if (player_actor.ordered_entities[j] == joint_entity) {
            char_data_to_actor[i] = j;
          }
        }
      }
    }
    // estimate repair rotation given tposes
    char_repair_c.resize(player_actor.ordered_entities.size(),
                         math::quat::Identity());
    auto [_parents, _children, _roots] =
        estimate_actor_bone_hierarchy(registry, player_actor);
    char_joint_parents = _parents;
    std::vector<math::quat> tpose_ori(data_tpose.names.size(),
                                      math::quat::Identity());
    for (int i = 0; i < data_tpose.names.size(); i++) {
      if (i != 0)
        tpose_ori[i] =
            tpose_ori[data_tpose.parents[i]] * data_tpose.local_rot[0][i];
      else
        tpose_ori[i] = data_tpose.local_rot[0][i];
    }
    for (auto [i, j] : char_data_to_actor) {
      auto joint_name = data_tpose.names[i];
      for (int k = 0; k < data_tpose.names.size(); k++) {
        if (data_tpose.names[k] == joint_name) {
          auto &joint_trans =
              registry.get<transform>(player_actor.ordered_entities[j]);
          char_repair_c[j] = tpose_ori[k].inverse() * joint_trans.world_rot();
          break;
        }
      }
    }
  } else {
    std::cout << "Assets incomplete, can't start demo." << std::endl;
    quit_app_running();
  }
}

void animator::handle_game_logic_tick(float dt) {
  // most of the logic are executed with fixed interval (60fps)
  {
    motion_time += dt * motion_time_scale;
    if (motion_time > motion_data.frametime * motion_data.local_rot.size()) {
      motion_time = 0.0f;
    }
    int frame0 = std::clamp((int)(motion_time / motion_data.frametime), 0,
                            (int)motion_data.local_rot.size() - 1);
    int frame1 =
        std::clamp(frame0 + 1, 0, (int)motion_data.local_rot.size() - 1);
    float alpha =
        (motion_time - frame0 * motion_data.frametime) / motion_data.frametime;
    auto &player_actor = registry.get<actor>(
        registry.get<skinned_mesh_bundle>(player_entity).actor_entities[0]);

    std::vector<math::quat> world_rot(motion_data.local_rot[0].size(),
                                      math::quat::Identity());
    for (int i = 0; i < motion_data.parents.size(); i++) {
      if (motion_data.parents[i] != -1) {
        world_rot[i] = world_rot[motion_data.parents[i]] *
                       motion_data.local_rot[frame0][i].slerp(
                           alpha, motion_data.local_rot[frame1][i]);
      } else {
        world_rot[i] = motion_data.local_rot[frame0][i].slerp(
            alpha, motion_data.local_rot[frame1][i]);
      }
    }
    int da_entry_idx = 0;
    for (auto [di, ai] : char_data_to_actor) {
      auto &joint_trans =
          registry.get<transform>(player_actor.ordered_entities[ai]);
      if (ai == 0) {
        // root joint
        joint_trans.set_world_pos(motion_data.local_pos[frame0][di] *
                                      (1 - alpha) +
                                  motion_data.local_pos[frame1][di] * alpha);
        joint_trans.set_world_rot(
            motion_data.local_rot[frame0][di].slerp(
                alpha, motion_data.local_rot[frame1][di]) *
            char_repair_c[ai]);
      } else {
        joint_trans.set_world_rot(world_rot[di] * char_repair_c[ai]);
      }
      da_entry_idx++;
    }
  }

  if (is_key_triggered(SDLK_ESCAPE)) {
    hide_mouse = !hide_mouse;
    set_game_mode(true, hide_mouse);
  }

  update_camera(dt);
  default_render_sys->push_custom_draw([this]() { debug_draw(); });
}

void animator::update_camera(float dt) {
  auto delta = get_mouse_screen_delta();
  if (!hide_mouse)
    delta = math::vector2::Zero();
  camera_horizontal_angle -= mouse_sensitivity * delta.x();
  camera_vertical_angle += mouse_sensitivity * delta.y();
  camera_vertical_angle =
      std::clamp(camera_vertical_angle, min_vertical_angle, max_vertical_angle);

  auto &player_trans = registry.get<transform>(player_entity);
  auto &bundle_data = registry.get<skinned_mesh_bundle>(player_entity);
  auto &player_actor = registry.get<actor>(bundle_data.actor_entities[0]);
  auto &root_trans = registry.get<transform>(player_actor.ordered_entities[0]);
  auto &cam_trans = registry.get<transform>(active_camera);
  math::vector3 cam_offset =
      math::angle_axis(math::deg_to_rad(camera_horizontal_angle),
                       math::world_up) *
      math::angle_axis(math::deg_to_rad(-camera_vertical_angle),
                       math::world_right) *
      math::vector3(0.0f, 0.0f, camera_distance);
  math::vector3 cam_pos = root_trans.world_pos() +
                          math::vector3(0.0f, camera_height, 0.0f) + cam_offset;
  // math::vector3 cam_pos = math::vector3(0.0f, camera_height, 0.0f) + cam_offset;
  math::matrix3 cam_rot_mat = math::matrix3::Identity();
  math::vector3 _z = cam_offset.normalized();
  math::vector3 _x = math::world_up.cross(_z).normalized();
  math::vector3 _y = _z.cross(_x).normalized();
  cam_rot_mat.col(0) = _x;
  cam_rot_mat.col(1) = _y;
  cam_rot_mat.col(2) = _z;
  math::quat cam_rot = math::quat(cam_rot_mat);
  cam_trans.set_world_pos(cam_pos);
  cam_trans.set_world_rot(cam_rot);
}

}; // namespace toolkit::opengl3d