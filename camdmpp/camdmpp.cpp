#include "camdmpp.hpp"

#include "toolkit/opengl3d/components/actor.hpp"

namespace toolkit::opengl3d {

math::vector3 shortest_arc_rot_vec(math::quat from, math::quat to) {
  from.normalize();
  to.normalize();
  if (from.dot(to) < 0.0f)
    to.coeffs() *= -1.0f;
  return math::quat_to_rot_vec(from * to.inverse());
}

float wrap_degrees(float degrees) {
  while (degrees > 180.0f)
    degrees -= 360.0f;
  while (degrees < -180.0f)
    degrees += 360.0f;
  return degrees;
}

math::vector2 apply_radial_deadzone(math::vector2 stick, float deadzone) {
  float mag = stick.norm();
  if (mag <= deadzone)
    return math::vector2::Zero();
  float scaled_mag = std::clamp((mag - deadzone) / (1.0f - deadzone), 0.0f,
                                1.0f);
  return stick / mag * scaled_mag;
}

void camdmpp::handle_custom_initialization() {
  if (std::filesystem::exists("camdmpp/model.onnx") &&
      std::filesystem::exists("camdmpp/model.json") &&
      std::filesystem::exists("camdmpp/setup.scene") &&
      std::filesystem::exists("camdmpp/tpose.bvh") &&
      (get_game_controllers().size() > 0)) {
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
    set_game_mode(true, false);
    player_entity = named_entities["player"];
    ground_entity = named_entities["ground"];
    default_render_sys->resize(wnd_width, wnd_height);
    transform_hierarchy_sys->update_transform(registry);
    build_terrain_sampler();
    camera_follow_vel = math::vector3::Zero();
    camera_follow_ang = math::vector3::Zero();

    // setup diffusion model
    model.setup("camdmpp/model.onnx", "camdmpp/model.json");
    i_past_motion.resize(model.pose_token_dim * model.past_points, 0.0f);
    i_traj.resize(model.traj_shape[1] * model.traj_shape[2], 0.0f);
    // i_traj_pos.resize(10);
    // i_traj_facing.resize(10);
    // i_style_idx.resize(1);

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
      cam_angle_horizontal = wrap_degrees(
          math::rad_to_deg(std::atan2(-initial_forward.x(),
                                      -initial_forward.z())));
    }
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
        if (ai != 0)
          joint_rotation_cache[i][da_entry_idx] =
              char_repair_c[char_joint_parents[ai]].inverse() *
              char_repair_c[ai];
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

  // update application state based on user input
  if (is_key_triggered(SDLK_ESCAPE))
    quit_app_running();
  // if (is_key_triggered(SDLK_j))
  //   i_style_idx[0] = (i_style_idx[0] + 99) % 100;
  // if (is_key_triggered(SDLK_k))
  //   i_style_idx[0] = (i_style_idx[0] + 1) % 100;

  // update the camera movement every logic tick after character update
  {
    auto &player_trans = registry.get<transform>(
        registry.get<skinned_mesh_bundle>(player_entity).actor_entities[0]);
    math::vector2 look_stick = math::vector2::Zero();
    auto controllers = get_game_controllers();
    if (!controllers.empty()) {
      auto [_left_stick_raw, right_stick_raw, _left_trigger, _right_trigger] =
          get_game_controller_analog_inputs(controllers[0]);
      look_stick = apply_radial_deadzone(right_stick_raw, camera_turn_deadzone);
    }

    if (is_mouse_button_pressed(SDL_BUTTON_LEFT) &&
        is_key_pressed(SDLK_LCTRL)) {
      cam_angle_horizontal += dt * cam_move_speed * mouse_screen_delta.x();
      cam_angle_vertical += dt * cam_move_speed * mouse_screen_delta.y();
    }
    cam_angle_horizontal += dt * cam_stick_speed * look_stick.x();
    cam_angle_vertical += dt * cam_stick_speed * look_stick.y();
    cam_angle_horizontal = wrap_degrees(cam_angle_horizontal);
    cam_angle_vertical = std::clamp(cam_angle_vertical, 5.0f, 45.0f);

    const float yaw = math::deg_to_rad(cam_angle_horizontal);
    const float pitch = math::deg_to_rad(cam_angle_vertical);
    const float cos_pitch = std::cos(pitch);
    math::vector3 cam_z(cos_pitch * std::sin(yaw), std::sin(pitch),
                        cos_pitch * std::cos(yaw));
    cam_z.normalize();

    math::vector3 focus_pos =
        player_trans.world_pos() + math::vector3(0.0f, cam_focus_height, 0.0f);
    math::vector3 move_dir = desired_vel;
    move_dir.y() = 0.0f;
    if (move_dir.squaredNorm() > 1e-6f)
      focus_pos += cam_look_ahead * move_dir.normalized();

    math::vector3 cam_y(0.0, 1.0, 0.0);
    math::vector3 cam_x = (cam_y.cross(cam_z)).normalized();
    cam_y = (cam_z.cross(cam_x)).normalized();
    math::matrix3 cam_rot = math::matrix3::Identity();
    cam_rot << cam_x, cam_y, cam_z;
    auto &cam_trans = registry.get<transform>(active_camera);
    const math::quat target_rot = math::quat(cam_rot).normalized();
    const math::vector3 target_pos = focus_pos + cam_z * cam_distance;

    auto [new_pos, new_vel] = spring_damper_position(
        cam_trans.world_pos(), camera_follow_vel, target_pos,
        math::vector3::Zero(), dt, camera_follow_halflife);
    camera_follow_vel = new_vel;
    cam_trans.set_world_pos(new_pos);

    auto [new_rot, new_ang] = spring_damper_rotation(
        cam_trans.world_rot(), camera_follow_ang, target_rot,
        math::vector3::Zero(), dt, camera_follow_halflife);
    camera_follow_ang = new_ang;
    cam_trans.set_world_rot(new_rot);
  }

  default_render_sys->push_custom_draw([this]() { debug_draw(); });
}

void camdmpp::fixed_interval_logic() {
  // make new predictions to the trajectory based on user input
  predict_trajectory();

  // apply pose to the character, fill in caches for network input
  apply_pose_and_refill();

  // submit a new prediction when counter reaches a threashold
  predict_new_tokens();
}

void camdmpp::predict_trajectory() {
  // TODO: the trajectory should be adapted
  math::vector2 move_stick = math::vector2::Zero();
  auto controllers = get_game_controllers();
  if (!controllers.empty()) {
    auto [left_stick_raw, _right_stick_raw, _left_trigger, _right_trigger] =
        get_game_controller_analog_inputs(controllers[0]);
    move_stick = apply_radial_deadzone(left_stick_raw, move_stick_deadzone);
  }

  const float yaw = math::deg_to_rad(cam_angle_horizontal);
  const math::vector3 cam_forward(-std::sin(yaw), 0.0f, -std::cos(yaw));
  const math::vector3 cam_right(std::cos(yaw), 0.0f, -std::sin(yaw));
  const math::vector3 move_input =
      cam_right * move_stick.x() - cam_forward * move_stick.y();

  math::quat desired_rot;
  desired_vel = velocity_scale * move_input;
  if (move_input.squaredNorm() > 1e-6f)
    desired_dir = move_input.normalized();
  desired_rot = math::from_to_rot(math::vector3(0, 0, 1), desired_dir);
  for (int i = 0; i < 5; i++) {
    auto [vel, acc] = spring_damper_position(
        char_root_world_vel, char_root_world_acc, desired_vel,
        math::vector3::Zero(), traj_sample_time * (i + 1), vel_halflife);
    auto [rot, ang] = spring_damper_rotation(
        proj_char_root_world_rot, char_root_world_ang, desired_rot,
        math::vector3::Zero(), traj_sample_time * (i + 1), rot_halflife);
    _traj_world_vel[i] = vel;
    if (i == 0) {
      _traj_world_pos[i] =
          (char_root_world_vel + vel) * 0.5f * traj_sample_time +
          char_root_world_pos;
    } else {
      _traj_world_pos[i] = (_traj_world_vel[i - 1] + _traj_world_vel[i]) *
                               0.5f * traj_sample_time +
                           _traj_world_pos[i - 1];
    }
    _traj_world_dir[i] = (rot * math::vector3(0, 0, 1));
    _traj_world_dir[i].y() = 0.0f;
    if (_traj_world_dir[i].squaredNorm() > 1e-6f)
      _traj_world_dir[i].normalize();
    else
      _traj_world_dir[i] = desired_dir;
    _traj_world_pos[i].y() = 0.0f;

    // sample terrain height
    math::vector3 _terrain_left = math::world_up.cross(_traj_world_dir[i]);
    if (_terrain_left.squaredNorm() > 1e-6f)
      _terrain_left.normalize();
    else
      _terrain_left = math::vector3(1, 0, 0);
    math::vector2 terrain_left(_terrain_left.x(), _terrain_left.z());

    const math::vector2 center_xz(_traj_world_pos[i].x(),
                                  _traj_world_pos[i].z());
    const math::vector2 left_xz =
        center_xz + terrain_probe_half_width * terrain_left;
    const math::vector2 right_xz =
        center_xz - terrain_probe_half_width * terrain_left;
    const float center_height = sample_terrain_height(center_xz, 0.0f);
    const float left_height = sample_terrain_height(left_xz, center_height);
    const float right_height = sample_terrain_height(right_xz, center_height);

    _traj_world_pos[i].y() = center_height;
    _traj_world_height[i] =
        math::vector3(center_height, left_height, right_height);
  }
}

void camdmpp::apply_pose_and_refill() {
  auto &player_trans = registry.get<transform>(player_entity);
  auto &bundle_data = registry.get<skinned_mesh_bundle>(player_entity);
  auto &player_actor = registry.get<actor>(bundle_data.actor_entities[0]);

  printf("applied_frames=%d\n", applied_frames);

  // find matching joint names and apply transform
  int da_entry_idx = 0;
  for (auto [di, ai] : char_data_to_actor) {
    auto &joint_trans =
        registry.get<transform>(player_actor.ordered_entities[ai]);
    if (ai == 0) {
      // root joint
      if (root_rel_pos_cache[applied_frames].norm() > 1e-5f) {
        char_root_world_pos =
            char_root_world_pos +
            proj_char_root_world_rot * root_rel_pos_cache[applied_frames];
      }
      char_root_world_pos.y() =
          root_height_cache[applied_frames] +
          sample_terrain_height(math::vector2(joint_trans.world_pos().x(),
                                              joint_trans.world_pos().z()),
                                0.0f);
      proj_char_root_world_rot =
          root_rel_rot_cache[applied_frames] * proj_char_root_world_rot;
      joint_trans.set_world_pos(char_root_world_pos);
      joint_trans.set_world_rot(
          proj_char_root_world_rot *
          joint_rotation_cache[applied_frames][da_entry_idx]);
    } else {
      joint_trans.set_local_rot(
          joint_rotation_cache[applied_frames][da_entry_idx]);
    }
    da_entry_idx++;
  }
  player_trans.force_update_hierarchy();

  // update network input cache
  // input trajectory
  for (int i = 0; i < 5; i++) {
    auto _traj_pos = proj_char_root_world_rot.inverse() *
                     (_traj_world_pos[i] - char_root_world_pos);
    auto _traj_facing = proj_char_root_world_rot.inverse() * _traj_world_dir[i];
    auto _traj_height = _traj_world_height[i];
    i_traj[7 * i + 0] = _traj_pos.x();
    i_traj[7 * i + 1] = _traj_pos.z();
    // TODO: normalize the height
    i_traj[7 * i + 2] = _traj_height[0] - _traj_world_height[0].x(); // center
    i_traj[7 * i + 3] = _traj_height[1] - _traj_world_height[0].y(); // left
    i_traj[7 * i + 4] = _traj_height[2] - _traj_world_height[0].z(); // right
    i_traj[7 * i + 5] = _traj_facing.x();
    i_traj[7 * i + 6] = _traj_facing.z();
  }

  // increase the apply frame counter
  applied_frames++;
  // check for buffer swap
  if (applied_frames ==
      (switch_prediction_interval + (use_front_buffer ? 0 : cache_size / 2))) {
    applied_frames = use_front_buffer ? cache_size / 2 : 0;
    use_front_buffer = !use_front_buffer;
  }
}

void camdmpp::predict_new_tokens() {
  // only start new prediction when condition met
  if (applied_frames ==
      (submit_prediction_interval + (use_front_buffer ? 0 : cache_size / 2))) {
    // fill in model input data
    model.past_motion_data = i_past_motion;
    model.traj_data = i_traj;
    printf("Dispatch inference when applied_frames=%d\n", applied_frames);
    model.submit_inference([this](std::vector<float> model_output,
                                  float inference_time) {
      printf("Inference finished when applied_frames=%d, update pose cache\n",
             applied_frames);
      display_inference_time = inference_time;
      // keep using current active buffer, swap when one runs out
      int buffer_start_idx = (use_front_buffer ? (cache_size / 2) : 0);

      // reset the past motion input
      for (int pf = 0; pf < model.past_points; pf++) {
        for (int ti = 0; ti < model.pose_token_dim; ti++) {
          i_past_motion[ti * model.past_points + pf] =
              model_output[ti * model.future_points + pf +
                           switch_prediction_interval - model.past_points];
        }
      }

      // convert predicted pose tokens into cache format
      // (1, pose_token_dim, future_points)
      int rot_channel_size = model.joint_num * 6;
      std::vector<math::quat> pred_world_rot(model.joint_num,
                                             math::quat::Identity());
      std::array<float, 6> tmp_data6d;
      inertia_lambda = std::log(2.0f) / (inertia_halflife * std::log(2.71828f));
      for (int f = 0; f < model.future_points; f++) {
        // fill in joint local rotations
        for (int jid = 0; jid < model.joint_num; jid++) {
          for (int k = 0; k < 6; k++) {
            tmp_data6d[k] =
                model_output[(6 * jid + k) * model.future_points + f] *
                    model.data_std[6 * jid + k] +
                model.data_mean[6 * jid + k];
          }
          auto &&joint_local_rot = repr6d_to_quat(tmp_data6d);
          if (jid == 0)
            pred_world_rot[jid] = joint_local_rot;
          else
            pred_world_rot[jid] =
                pred_world_rot[model.joint_parents[jid]] * joint_local_rot;
        }
        int da_entry_idx = 0;
        for (auto [di, ai] : char_data_to_actor) {
          if (di == 0) {
            joint_rotation_cache[buffer_start_idx + f][da_entry_idx] =
                pred_world_rot[di];
          } else {
            joint_rotation_cache[buffer_start_idx + f][da_entry_idx] =
                (pred_world_rot[model.joint_parents[di]] *
                 char_repair_c[char_joint_parents[ai]])
                    .inverse() *
                (pred_world_rot[di] * char_repair_c[ai]);
          }
          da_entry_idx++;
        }

        // fill in relative root position
        for (int k = 0; k < 3; k++) {
          root_rel_pos_cache[buffer_start_idx + f](k) =
              model_output[(rot_channel_size + k) * model.future_points + f] *
                  model.data_std[rot_channel_size + k] +
              model.data_mean[rot_channel_size + k];
        }
        // fill in relative root rotation
        for (int k = 3; k < 9; k++) {
          tmp_data6d[k - 3] =
              model_output[(rot_channel_size + k) * model.future_points + f] *
                  model.data_std[rot_channel_size + k] +
              model.data_mean[rot_channel_size + k];
        }
        root_rel_rot_cache[buffer_start_idx + f] = repr6d_to_quat(tmp_data6d);
        // fill in root height (relative offset from character root to terrain)
        root_height_cache[buffer_start_idx + f] =
            model_output[(rot_channel_size + 9) * model.future_points + f] *
                model.data_std[rot_channel_size + 9] +
            model.data_mean[rot_channel_size + 9];
      }

      // inertial blending over the predicted motion
      if (enable_inertia_blending) {
        int ib_from_idx = (use_front_buffer ? switch_prediction_interval - 1
                                            : switch_prediction_interval - 1 +
                                                  cache_size / 2);
        int ib_to_idx = (use_front_buffer ? cache_size / 2 : 0);
        std::vector<math::vector3> ib_off_rot(char_data_to_actor.size(),
                                              math::vector3::Zero()),
            ib_off_ang(char_data_to_actor.size(), math::vector3::Zero());
        for (int jid = 0; jid < char_data_to_actor.size(); jid++) {
          ib_off_rot[jid] =
              shortest_arc_rot_vec(joint_rotation_cache[ib_from_idx][jid],
                                   joint_rotation_cache[ib_to_idx][jid]);
          math::vector3 from_ang =
              shortest_arc_rot_vec(joint_rotation_cache[ib_from_idx][jid],
                                   joint_rotation_cache[ib_from_idx - 1][jid]) /
              fixed_interval;
          math::vector3 to_ang =
              shortest_arc_rot_vec(joint_rotation_cache[ib_to_idx + 1][jid],
                                   joint_rotation_cache[ib_to_idx][jid]) /
              fixed_interval;
          ib_off_ang[jid] = from_ang - to_ang;
        }
        for (int bf = 0; bf < inertia_blend_wnd; bf++) {
          float bf_dt = fixed_interval * (bf + 1);
          for (int jid = 0; jid < char_data_to_actor.size(); jid++) {
            auto no_blend =
                joint_rotation_cache[ib_to_idx + bf][jid].normalized();
            auto r = ib_off_rot[jid];
            auto av = ib_off_ang[jid];
            r = (r + (av + inertia_lambda * r) * bf_dt) *
                std::exp(-inertia_lambda * bf_dt);
            joint_rotation_cache[ib_to_idx + bf][jid] =
                (math::rot_vec_to_quat(r) * no_blend).normalized();
          }
        }
      }
    });
  }
}

}; // namespace toolkit::opengl3d