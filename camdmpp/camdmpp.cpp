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

math::vector2 apply_radial_deadzone(math::vector2 stick, float deadzone) {
  float mag = stick.norm();
  if (mag <= deadzone)
    return math::vector2::Zero();
  float scaled_mag =
      std::clamp((mag - deadzone) / (1.0f - deadzone), 0.0f, 1.0f);
  return stick / mag * scaled_mag;
}

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
    math::vector3 initial_forward =
        player_root_trans.world_rot() * math::vector3(0, 0, 1);
    initial_forward.y() = 0.0f;
    if (initial_forward.squaredNorm() > 1e-6f) {
      initial_forward.normalize();
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

void camdmpp::fixed_interval_logic() {
  // apply pose to the character, fill in caches for network input
  apply_pose_and_refill();

  // make new predictions to the trajectory based on user input
  predict_trajectory();

  // submit a new prediction when counter reaches a threashold
  predict_new_tokens();
}

void camdmpp::predict_trajectory() {
  auto &player_trans = registry.get<transform>(player_entity);
  auto &bundle_data = registry.get<skinned_mesh_bundle>(player_entity);
  auto &player_actor = registry.get<actor>(bundle_data.actor_entities[0]);
  auto &root_trans = registry.get<transform>(player_actor.ordered_entities[0]);
  auto &camera_trans = registry.get<transform>(active_camera);
  camera_forward = -math::vector3(camera_trans.local_forward().x(), 0.0f,
                                  camera_trans.local_forward().z())
                        .normalized();
  math::vector3 root_forward = root_trans.world_rot() * math::vector3(0, 0, 1);
  root_forward.y() = 0.0f;
  root_forward.normalize();

  math::quat camera_forward_rot =
      math::from_to_rot(math::vector3(0.0f, 0.0f, -1.0f), camera_forward);
  move_input = math::vector3::Zero();
  if (is_key_pressed(SDLK_w))
    move_input += math::vector3(0.0f, 0.0f, -1.0f);
  if (is_key_pressed(SDLK_s))
    move_input += math::vector3(0.0f, 0.0f, 1.0f);
  if (is_key_pressed(SDLK_a))
    move_input += math::vector3(-1.0f, 0.0f, 0.0f);
  if (is_key_pressed(SDLK_d))
    move_input += math::vector3(1.0f, 0.0f, 0.0f);
  move_input = camera_forward_rot * move_input.normalized();

  player_vel = (player_curr_pos - player_last_pos) / fixed_interval;
  player_last_pos = player_curr_pos;
  player_curr_pos = math::vector3(root_trans.world_pos().x(), 0.0f,
                                  root_trans.world_pos().z());
  math::vector3 target_vel =
      (char_running ? sim_move_speed_run : sim_move_speed_walk) * move_input;
  math::vector3 target_dir = camera_forward;
  if (!camera_as_facing_direction)
    target_dir =
        target_vel.norm() > 1e-6f ? target_vel.normalized() : root_forward;

  math::vector3 sim_vel = player_vel, player_to_sim_acc = math::vector3::Zero();
  for (int i = 0; i < model.future_points; i++) {
    math::vector3 vel_diff = target_vel - sim_vel;
    bool is_accelerating = vel_diff.dot(target_vel) > 0;
    float acc_mag = is_accelerating ? sim_acceleration : sim_deceleration;
    math::vector3 acc = vel_diff.norm() < acc_mag * fixed_interval
                            ? vel_diff / fixed_interval
                            : math::vector3(vel_diff.normalized() * acc_mag);

    if (i == 0)
      _traj_world_pos[i] =
          (2 * sim_vel + acc * fixed_interval) * 0.5f * fixed_interval +
          player_curr_pos;
    else
      _traj_world_pos[i] =
          (2 * sim_vel + acc * fixed_interval) * 0.5f * fixed_interval +
          _traj_world_pos[i - 1];
    _traj_world_dir[i] = target_dir;
    _traj_world_dir[i].y() = 0.0f;
    _traj_world_dir[i].normalize();

    sim_vel += acc * fixed_interval;

    // // track simulated body with velocity spring
    // auto [_vel, _acc] =
    //     spring_damper_position(player_vel, player_to_sim_acc, sim_vel,
    //     math::vector3::Zero(),
    //                            fixed_interval, vel_halflife);
    // if (i == 0)
    //   _traj_world_pos[i] =
    //       (player_vel + _vel) * 0.5f * fixed_interval + player_curr_pos;
    // else
    //   _traj_world_pos[i] =
    //       (player_vel + _vel) * 0.5f * fixed_interval + _traj_world_pos[i -
    //       1];
    // _traj_world_dir[i] = target_dir;
    // _traj_world_dir[i].y() = 0.0f;
    // _traj_world_dir[i].normalize();
    // player_vel = _vel;
    // player_to_sim_acc = _acc;
  }

  resample_trajectory_on_terrain(
      math::vector2(player_curr_pos.x(), player_curr_pos.z()),
      sample_terrain_height(
          math::vector2(root_trans.world_pos().x(), root_trans.world_pos().z()),
          0.0f));
}

void camdmpp::apply_pose_and_refill() {
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

  printf("applied_frames=%d\n", applied_frames);

  // find matching joint names and apply transform
  int da_entry_idx = 0;
  for (auto [di, ai] : char_data_to_actor) {
    auto &joint_trans =
        registry.get<transform>(player_actor.ordered_entities[ai]);
    if (ai == 0) {
      // root joint
      if (root_rel_pos_cache[applied_frames].norm() > 1e-5f) {
        char_pos =
            char_pos + proj_char_rot * root_rel_pos_cache[applied_frames];
      }
      char_pos.y() =
          root_height_cache[applied_frames] +
          sample_terrain_height(math::vector2(joint_trans.world_pos().x(),
                                              joint_trans.world_pos().z()),
                                0.0f);
      proj_char_rot = root_rel_rot_cache[applied_frames] * proj_char_rot;
      joint_trans.set_world_pos(char_pos);
      joint_trans.set_world_rot(
          proj_char_rot * joint_rotation_cache[applied_frames][da_entry_idx]);
    } else {
      joint_trans.set_local_rot(
          joint_rotation_cache[applied_frames][da_entry_idx]);
    }
    da_entry_idx++;
  }
  player_trans.force_update_hierarchy();

  // update network input cache
  // input trajectory
  for (int i = 0; i < model.future_points; i++) {
    auto _traj_pos = proj_char_rot.inverse() * (_traj_world_pos[i] - char_pos);
    // auto _traj_vel = proj_char_rot.inverse() * _traj_world_vel[i];
    auto _traj_facing = proj_char_rot.inverse() * _traj_world_dir[i];
    auto _traj_height = _traj_world_height[i];
    i_traj[model.traj_shape[2] * i + 0] = _traj_pos.x();
    i_traj[model.traj_shape[2] * i + 1] = _traj_pos.z();

    // i_traj[model.traj_shape[2] * i + 2] = _traj_vel.x();
    // i_traj[model.traj_shape[2] * i + 3] = _traj_vel.y();
    // i_traj[model.traj_shape[2] * i + 4] = _traj_vel.z();

    for (int j = 0; j < model.lateral_offsets_m.size(); j++) {
      if (i < model.future_points - 1)
        i_traj[model.traj_shape[2] * i + 2 + j] =
            (_traj_world_height[i + 1][j] - _traj_world_height[i][j]) /
            fixed_interval;
      else
        i_traj[model.traj_shape[2] * i + 2 + j] =
            i_traj[model.traj_shape[2] * (i - 1) + 2 + j];

      i_traj[model.traj_shape[2] * i + 5 + j] =
          _traj_height[j] - _traj_world_height[0][model.terrain_center_idx];
    }
    i_traj[model.traj_shape[2] * i + model.lateral_offsets_m.size() + 5] =
        _traj_facing.x();
    i_traj[model.traj_shape[2] * i + model.lateral_offsets_m.size() + 6] =
        _traj_facing.z();
    // gait
    i_traj[model.traj_shape[2] * i + model.lateral_offsets_m.size() + 7] =
        0.0f; // stand
    i_traj[model.traj_shape[2] * i + model.lateral_offsets_m.size() + 8] =
        char_running ? 0.2f : 0.8f; // walk
    i_traj[model.traj_shape[2] * i + model.lateral_offsets_m.size() + 9] =
        char_running ? 0.8f : 0.2f; // jog_run
    i_traj[model.traj_shape[2] * i + model.lateral_offsets_m.size() + 10] =
        char_crouching ? 1.0f : 0.0f; // crouch_crawl
    i_traj[model.traj_shape[2] * i + model.lateral_offsets_m.size() + 11] =
        0.0f; // jump
    i_traj[model.traj_shape[2] * i + model.lateral_offsets_m.size() + 12] =
        0.0f; // unknown

    // normalize trajectory input
    for (int k = 0; k < model.traj_shape[2]; k++) {
      i_traj[model.traj_shape[2] * i + k] =
          (i_traj[model.traj_shape[2] * i + k] - model.traj_mean[k]) /
          std::max(model.traj_std[k], 1e-2f);
    }
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