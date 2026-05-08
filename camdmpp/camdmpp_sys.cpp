#include "camdmpp.hpp"

namespace toolkit::opengl3d {

void camdmpp::handle_custom_initialization() {
  if (std::filesystem::exists("camdmpp/model.onnx") &&
      std::filesystem::exists("camdmpp/model.json") &&
      std::filesystem::exists("camdmpp/setup.scene") &&
      std::filesystem::exists("camdmpp/tpose.bvh")) {
    // load setup scene
    std::ifstream scene_input("camdmpp/setup.scene");
    if (scene_input.is_open()) {
      auto data = nlohmann::json::parse(
          std::string((std::istreambuf_iterator<char>(scene_input)),
                      std::istreambuf_iterator<char>()));
      deserialize(data);
    } else {
      std::cout << "Error loading scene (camdmpp/setup.scene)" << std::endl;
      quit_app_running();
      return;
    }
    if (named_entities.find("player") == named_entities.end()) {
      quit_app_running();
      std::cout
          << "Error loading scene (camdmpp/setup.scene)\n"
             "the setup scene named_entities don't contain field \"player\""
          << std::endl;
      return;
    }
    set_game_mode(true, hide_mouse);
    player_entity = named_entities["player"];
    ground_entity = named_entities["ground"];
    left_thigh_entity = named_entities["left_thigh"];
    right_thigh_entity = named_entities["right_thigh"];
    default_render_sys->resize(wnd_width, wnd_height);
    transform_hierarchy_sys->update_transform(registry);
    build_terrain_sampler();

    // setup diffusion model
    model.setup("camdmpp/model.onnx", "camdmpp/model.json");
    i_past_motion.resize(model.pose_token_dim * model.past_points, 0.0f);
    i_traj.resize(model.traj_shape[1] * model.traj_shape[2], 0.0f);
    _traj_world_pos.resize(model.traj_shape[1], math::vector3::Zero());
    _traj_world_vel.resize(model.traj_shape[1], math::vector3::Zero());
    _traj_world_dir.resize(model.traj_shape[1], math::world_forward);
    _traj_world_height.resize(model.traj_shape[1]);
    for (int i = 0; i < model.traj_shape[1]; i++) {
      _traj_world_height[i].resize(model.lateral_offsets_m.size(), 0.0f);
    }

    auto &bundle_data = registry.get<skinned_mesh_bundle>(player_entity);
    auto &player_trans = registry.get<transform>(player_entity);
    auto &player_root_trans =
        registry.get<transform>(bundle_data.actor_entities[0]);
    auto &player_actor = registry.get<actor>(bundle_data.actor_entities[0]);
    player_trans.force_update_hierarchy();

    // // compute initial forward with hips and thigh position
    // auto &left_thigh_trans =
    //     registry.get<transform>(player_actor.name_to_entity["LeftUpLeg"]);
    // auto &right_thigh_trans =
    //     registry.get<transform>(player_actor.name_to_entity["RightUpLeg"]);
    // math::vector3 initial_forward = math::vector3(0.0f, 1.0f, 0.0f)
    //                                     .cross((right_thigh_trans.world_pos()
    //                                     -
    //                                             left_thigh_trans.world_pos()));
    // initial_forward.y() = 0.0f;
    // if (initial_forward.squaredNorm() > 1e-6f) {
    //   initial_forward.normalize();
    // }

    // find mapping between network's prediction joint to actor's joint
    for (int i = 0; i < model.joint_names.size(); i++) {
      if (player_actor.name_to_entity.find(model.joint_names[i]) !=
          player_actor.name_to_entity.end()) {
        auto joint_entity = player_actor.name_to_entity[model.joint_names[i]];
        for (int j = 0; j < player_actor.ordered_entities.size(); j++) {
          if (player_actor.ordered_entities[j] == joint_entity) {
            char_data_to_actor[i] = j;
          }
        }
      }
    }

    // estimate repair rotation given tposes
    auto data_tpose = assets::load_bvh("camdmpp/tpose.bvh");
    network_root_rot = data_tpose.local_rot[0][0];
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
      auto joint_name = model.joint_names[i];
      for (int k = 0; k < data_tpose.names.size(); k++) {
        if (data_tpose.names[k] == joint_name) {
          auto &joint_trans =
              registry.get<transform>(player_actor.ordered_entities[j]);
          char_repair_c[j] = tpose_ori[k].inverse() * joint_trans.world_rot();
          break;
        }
      }
    }
    // create caches for pose application and network input
    int matched_joint_count = char_data_to_actor.size();
    for (int i = 0; i < cache_size; i++) {
      root_rel_pos_cache[i] = math::vector3::Zero();
      root_rel_rot_cache[i] = math::quat::Identity();
      root_height_cache[i] = 0;
      joint_rotation_cache[i].resize(matched_joint_count,
                                     math::quat::Identity());
      int da_entry_idx = 0;
      for (auto [di, ai] : char_data_to_actor) {
        if (ai != 0) {
          joint_rotation_cache[i][da_entry_idx] =
              char_repair_c[char_joint_parents[ai]].inverse() *
              char_repair_c[ai];
        } else {
          joint_rotation_cache[i][da_entry_idx] = char_repair_c[ai];
        }
        da_entry_idx++;
      }
    }
  } else {
    std::cout << "Assets incomplete, can't start demo." << std::endl;
    quit_app_running();
  }
}

void camdmpp::handle_game_logic_tick(float dt) {
  // printf("frame delta time=%.3f\n", dt);
  model.process_completions();

  // most of the logic are executed with fixed interval (60fps)
  double residual = __cur_time - __cur_exec_fixed * fixed_interval;
  while (residual > fixed_interval) {
    fixed_interval_logic();
    residual -= fixed_interval;
    __cur_exec_fixed += 1;
  }
  __cur_time += dt;

  if (is_key_triggered(SDLK_ESCAPE)) {
    hide_mouse = !hide_mouse;
    set_game_mode(true, hide_mouse);
  }
  if (is_key_triggered(SDLK_f)) {
    camera_as_facing_direction = !camera_as_facing_direction;
  }

  update_camera(dt);
  update_character_states(dt);
  default_render_sys->push_custom_draw([this]() { debug_draw(); });
}

void camdmpp::update_character_states(float dt) {
  if (is_key_pressed(SDLK_LSHIFT))
    char_running = true;
  else
    char_running = false;

  if (is_key_pressed(SDLK_LCTRL)) {
    char_crouching = true;
    char_running = false;
  } else {
    char_crouching = false;
  }
}

void camdmpp::update_camera(float dt) {
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