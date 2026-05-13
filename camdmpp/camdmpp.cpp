#include "camdmpp.hpp"

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

  math::vector3 root_forward = network_root_rot * math::vector3(0, 0, 1);
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

  player_ang =
      shortest_arc_rot_vec(player_last_rot, player_curr_rot) / fixed_interval;
  player_last_rot = player_curr_rot;
  player_curr_rot = math::from_to_rot(math::vector3(0, 0, 1), root_forward);

  math::vector3 target_vel =
      (char_running ? sim_move_speed_run : sim_move_speed_walk) * move_input;
  math::vector3 target_dir = camera_forward;
  if (!camera_as_facing_direction)
    target_dir =
        target_vel.norm() > 1e-6f ? target_vel.normalized() : root_forward;

  math::vector3 sim_vel = player_vel, player_to_sim_acc = math::vector3::Zero();

  auto q0 = math::from_to_rot(math::vector3(0, 0, 1), root_forward);
  auto qt = math::from_to_rot(math::vector3(0, 0, 1), target_dir);
  if (q0.dot(qt) < 0.0f)
    qt = toolkit::math::quat(-qt.w(), -qt.x(), -qt.y(), -qt.z());
  toolkit::math::vector3 q = toolkit::math::quat_to_rot_vec(q0 * qt.inverse());
  math::vector3 angular_velocity = player_ang;
  const float e = 2.71828f;
  float lambda = log(2) / (rot_halflife * log(e));

  for (int i = 0; i < model.future_points; i++) {
    math::vector3 vel_diff = target_vel - sim_vel;
    bool is_accelerating = vel_diff.dot(target_vel) > 0;
    float acc_mag = is_accelerating ? sim_acceleration : sim_deceleration;
    math::vector3 acc = vel_diff.norm() < acc_mag * fixed_interval
                            ? vel_diff / fixed_interval
                            : math::vector3(vel_diff.normalized() * acc_mag);

    if (i == 0) {
      _traj_world_pos[i] =
          (2 * sim_vel + acc * fixed_interval) * 0.5f * fixed_interval +
          player_curr_pos;
    } else {
      _traj_world_pos[i] =
          (2 * sim_vel + acc * fixed_interval) * 0.5f * fixed_interval +
          _traj_world_pos[i - 1];
    }

    // damp current root forward towards target dir
    auto q_prev = q;
    q = (q_prev + (angular_velocity + lambda * q_prev) * fixed_interval) *
        exp(-lambda * fixed_interval);
    angular_velocity =
        (angular_velocity + lambda * q_prev) * exp(-lambda * fixed_interval) -
        lambda * q;

    _traj_world_dir[i] =
        (math::rot_vec_to_quat(q) * qt) * math::vector3(0, 0, 1);
    _traj_world_dir[i].y() = 0.0f;
    _traj_world_dir[i].normalize();

    sim_vel += acc * fixed_interval;
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

  math::vector3 root_forward = network_root_rot * math::vector3(0, 0, 1);
  root_forward.y() = 0.0f;
  root_forward.normalize();
  math::quat network_root_y_comp =
      math::from_to_rot(math::vector3(0, 0, 1), root_forward);

  // printf("applied_frames=%d\n", applied_frames);

  // find matching joint names and apply transform
  int da_entry_idx = 0;
  for (auto [di, ai] : char_data_to_actor) {
    auto &joint_trans =
        registry.get<transform>(player_actor.ordered_entities[ai]);
    if (ai == 0) {
      // root joint
      if (root_rel_pos_cache[applied_frames].norm() > 1e-3f) {
        char_pos =
            char_pos + network_root_y_comp * root_rel_pos_cache[applied_frames];
      }
      char_pos.y() =
          root_height_cache[applied_frames] +
          sample_terrain_height(math::vector2(char_pos.x(), char_pos.z()),
                                joint_trans.world_pos().y());
      network_root_y_comp =
          root_rel_rot_cache[applied_frames] * network_root_y_comp;
      network_root_rot = network_root_y_comp *
                         joint_rotation_cache[applied_frames][da_entry_idx];
      joint_trans.set_world_pos(char_pos);
      joint_trans.set_world_rot(network_root_rot * char_repair_c[ai]);
    } else {
      joint_trans.set_local_rot(
          joint_rotation_cache[applied_frames][da_entry_idx]);
    }
    da_entry_idx++;
  }
  ik_value_right = std::clamp(ik_right_cache[applied_frames], 0.0f, 1.0f);
  ik_value_left = std::clamp(ik_left_cache[applied_frames], 0.0f, 1.0f);
  // std::cout << ik_value_right << ", " << ik_value_left << std::endl;
  player_trans.force_update_hierarchy();

  // update network input cache
  // input trajectory
  for (int i = 0; i < model.future_points; i++) {
    auto _traj_pos =
        network_root_y_comp.inverse() * (_traj_world_pos[i] - char_pos);
    auto _traj_facing = network_root_y_comp.inverse() * _traj_world_dir[i];
    auto _traj_height = _traj_world_height[i];

    i_traj[model.traj_shape[2] * i + 0] = _traj_pos.x();
    i_traj[model.traj_shape[2] * i + 1] = _traj_pos.z();

    for (int j = 0; j < model.lateral_offsets_m.size(); j++) {
      if (i < model.future_points - 1)
        i_traj[model.traj_shape[2] * i + 2 + j] =
            (_traj_world_height[i + 1][j] - _traj_world_height[i][j]) /
            fixed_interval;
      else
        i_traj[model.traj_shape[2] * i + 2 + j] =
            i_traj[model.traj_shape[2] * (i - 1) + 2 + j];
      i_traj[model.traj_shape[2] * i + 2 + model.lateral_offsets_m.size() + j] =
          _traj_height[j] - _traj_world_height[0][model.terrain_center_idx];
    }

    i_traj[model.traj_shape[2] * i + 2 * model.lateral_offsets_m.size() + 2] =
        _traj_facing.x();
    i_traj[model.traj_shape[2] * i + 2 * model.lateral_offsets_m.size() + 3] =
        _traj_facing.z();

    // gait
    if (true) {
      i_traj[model.traj_shape[2] * i + 2 * model.lateral_offsets_m.size() + 4] =
          0.0f; // stand
      i_traj[model.traj_shape[2] * i + 2 * model.lateral_offsets_m.size() + 5] =
          (char_running ? 0.0f : 1.0f) * (char_crouching ? 0.0f : 1.0f); // walk
      i_traj[model.traj_shape[2] * i + 2 * model.lateral_offsets_m.size() + 6] =
          (char_running ? 1.0f : 0.0f) *
          (char_crouching ? 0.0f : 1.0f); // jog_run
      i_traj[model.traj_shape[2] * i + 2 * model.lateral_offsets_m.size() + 7] =
          char_crouching ? 1.0f : 0.0f; // crouch_crawl
      i_traj[model.traj_shape[2] * i + 2 * model.lateral_offsets_m.size() + 8] =
          0.0f; // jump
      i_traj[model.traj_shape[2] * i + 2 * model.lateral_offsets_m.size() + 9] =
          0.0f; // unknown
    } else {
      i_traj[model.traj_shape[2] * i + 2 * model.lateral_offsets_m.size() + 4] =
          0.0f; // stand
      i_traj[model.traj_shape[2] * i + 2 * model.lateral_offsets_m.size() + 5] =
          0.0f; // sit
      i_traj[model.traj_shape[2] * i + 2 * model.lateral_offsets_m.size() + 6] =
          1.0f; // walk
      i_traj[model.traj_shape[2] * i + 2 * model.lateral_offsets_m.size() + 7] =
          0.0f; // run
    }

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
    const bool submitted_from_front_buffer = use_front_buffer;
    const int target_buffer_start_idx = use_front_buffer ? cache_size / 2 : 0;
    const int transition_from_idx = (use_front_buffer ? 0 : cache_size / 2) +
                                    switch_prediction_interval - 1;

    // fill in model input data
    model.past_motion_data = i_past_motion;
    model.traj_data = i_traj;
    // printf("Dispatch inference when applied_frames=%d\n", applied_frames);
    model.submit_inference([this, submitted_from_front_buffer,
                            target_buffer_start_idx, transition_from_idx](
                               std::vector<float> model_output,
                               float inference_time) {
      // printf("Inference finished when applied_frames=%d, update pose
      // cache\n",
      //        applied_frames);
      if (use_front_buffer != submitted_from_front_buffer) {
        std::cout << "Discard late inference result after prediction buffer "
                     "swap."
                  << std::endl;
        return;
      }
      display_inference_time = inference_time;

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
            // joint_rotation_cache[buffer_start_idx + f][da_entry_idx] =
            //     pred_world_rot[di] * char_repair_c[ai];
            joint_rotation_cache[target_buffer_start_idx + f][da_entry_idx] =
                pred_world_rot[di];
          } else {
            joint_rotation_cache[target_buffer_start_idx + f][da_entry_idx] =
                (pred_world_rot[model.joint_parents[di]] *
                 char_repair_c[char_joint_parents[ai]])
                    .inverse() *
                (pred_world_rot[di] * char_repair_c[ai]);
          }
          da_entry_idx++;
        }

        // fill in relative root position
        for (int k = 0; k < 3; k++) {
          root_rel_pos_cache[target_buffer_start_idx + f](k) =
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
        root_rel_rot_cache[target_buffer_start_idx + f] =
            repr6d_to_quat(tmp_data6d);
        // root_rel_rot_cache[target_buffer_start_idx + f] =
        // math::quat::Identity();

        // fill in root height (relative offset from character root to terrain)
        root_height_cache[target_buffer_start_idx + f] =
            model_output[(rot_channel_size + 9) * model.future_points + f] *
                model.data_std[rot_channel_size + 9] +
            model.data_mean[rot_channel_size + 9];

        // fill in ik values
        ik_right_cache[target_buffer_start_idx + f] =
            model_output[(rot_channel_size + 10) * model.future_points + f] *
                model.data_std[rot_channel_size + 10] +
            model.data_mean[rot_channel_size + 10];
        ik_left_cache[target_buffer_start_idx + f] =
            model_output[(rot_channel_size + 11) * model.future_points + f] *
                model.data_std[rot_channel_size + 11] +
            model.data_mean[rot_channel_size + 11];
      }

      // inertial blending over the predicted motion
      if (enable_inertia_blending) {
        int ib_from_idx = transition_from_idx;
        int ib_to_idx = target_buffer_start_idx;
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

      // ik foot locking blending
      if (enable_foot_locking)
        postprocessing_ik(target_buffer_start_idx, transition_from_idx,
                          model.future_points);

      // // motion terrain adjustment
      // if (enable_motion_terrain_adjustment) {}
    });
  }
}

}; // namespace toolkit::opengl3d