#include "camdmpp.hpp"

namespace toolkit::opengl3d {

namespace {

constexpr float ik_lock_on_threshold = 0.55f;
constexpr float ik_lock_off_threshold = 0.25f;
constexpr float ik_label_blend_low = 0.15f;
constexpr float ik_label_blend_high = 0.85f;
constexpr float ik_penetration_blend_depth = 0.08f;
constexpr float ik_contact_height_offset_min = -0.02f;
constexpr float ik_contact_height_offset_max = 0.08f;
constexpr float ik_toe_probe_distance = 0.18f;
constexpr int ik_lock_transition_frames = 4;
constexpr int ik_solver_iterations = 4;

struct ik_joint_pose {
  math::vector3 pos = math::vector3::Zero();
  math::quat rot = math::quat::Identity();
};

struct ik_chain_indices {
  int thigh = -1, shin = -1, foot = -1, toe = -1;

  bool valid() const {
    return thigh >= 0 && shin >= 0 && foot >= 0 && toe >= 0;
  }
};

struct ik_foot_target {
  math::vector3 target = math::vector3::Zero();
  float weight = 0.0f;
};

float saturate(float value) { return std::clamp(value, 0.0f, 1.0f); }

float smoothstep(float edge0, float edge1, float value) {
  float t = saturate((value - edge0) / std::max(edge1 - edge0, 1e-6f));
  return t * t * (3.0f - 2.0f * t);
}

math::quat normalized_shortest_slerp(math::quat from, math::quat to,
                                     float blend) {
  from.normalize();
  to.normalize();
  if (from.dot(to) < 0.0f)
    to.coeffs() *= -1.0f;
  return from.slerp(saturate(blend), to).normalized();
}

math::vector3 planar_forward_from_quat(const math::quat &rotation) {
  math::vector3 forward = rotation * math::vector3(0.0f, 0.0f, 1.0f);
  forward.y() = 0.0f;
  if (forward.squaredNorm() <= 1e-8f)
    return math::vector3(0.0f, 0.0f, 1.0f);
  return forward.normalized();
}

} // namespace

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

    // setup ik chains
    biped_chain_left = {
        named_entities["left_thigh"], named_entities["left_shin"],
        named_entities["left_foot"], named_entities["left_toe"]};
    biped_chain_right = {
        named_entities["right_thigh"], named_entities["right_shin"],
        named_entities["right_foot"], named_entities["right_toe"]};

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
    for (int i = 0; i < data_tpose.names.size(); i++) {
      for (int j = 0; j < model.joint_names.size(); j++) {
        if (model.joint_names[j] == data_tpose.names[i]) {
          tpose_to_data[i] = j;
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

void camdmpp::postprocessing_ik(int buffer_start_idx, int transition_from_idx,
                                int frame_count) {
  auto &bundle_data = registry.get<skinned_mesh_bundle>(player_entity);
  auto &player_actor = registry.get<actor>(bundle_data.actor_entities[0]);
  auto &root_trans = registry.get<transform>(player_actor.ordered_entities[0]);

  const int actor_count = static_cast<int>(player_actor.ordered_entities.size());
  if (actor_count == 0 || char_joint_parents.size() != actor_count ||
      char_repair_c.size() != actor_count || frame_count <= 0) {
    return;
  }

  std::vector<int> actor_to_cache(actor_count, -1);
  int cache_entry_idx = 0;
  for (auto [model_joint_idx, actor_idx] : char_data_to_actor) {
    if (actor_idx >= 0 && actor_idx < actor_count)
      actor_to_cache[actor_idx] = cache_entry_idx;
    cache_entry_idx++;
  }
  const int root_cache_idx = actor_to_cache[0];
  if (root_cache_idx < 0)
    return;

  auto actor_index_for_entity = [&](entt::entity entity) {
    for (int actor_idx = 0; actor_idx < actor_count; actor_idx++) {
      if (player_actor.ordered_entities[actor_idx] == entity)
        return actor_idx;
    }
    return -1;
  };

  auto resolve_chain_indices = [&](const std::array<entt::entity, 4> &chain) {
    ik_chain_indices indices;
    for (int joint_slot = 0; joint_slot < static_cast<int>(chain.size());
         joint_slot++) {
      const int actor_idx = actor_index_for_entity(chain[joint_slot]);
      if (joint_slot == 0)
        indices.thigh = actor_idx;
      else if (joint_slot == 1)
        indices.shin = actor_idx;
      else if (joint_slot == 2)
        indices.foot = actor_idx;
      else
        indices.toe = actor_idx;
    }
    return indices;
  };

  const auto left_chain_indices = resolve_chain_indices(biped_chain_left);
  const auto right_chain_indices = resolve_chain_indices(biped_chain_right);
  auto chain_is_valid = [&](const ik_chain_indices &chain_indices) {
    const std::array<int, 4> actor_indices = {
        chain_indices.thigh, chain_indices.shin, chain_indices.foot,
        chain_indices.toe};
    for (int actor_idx : actor_indices) {
      if (actor_idx < 0 || actor_idx >= actor_count ||
          actor_to_cache[actor_idx] < 0) {
        return false;
      }
    }
    return true;
  };
  if (!chain_is_valid(left_chain_indices) ||
      !chain_is_valid(right_chain_indices)) {
    return;
  }

  auto compute_leg_length = [&](const ik_chain_indices &chain_indices) {
    auto &shin_trans = registry.get<transform>(
        player_actor.ordered_entities[chain_indices.shin]);
    auto &foot_trans = registry.get<transform>(
        player_actor.ordered_entities[chain_indices.foot]);
    const float length = shin_trans.local_pos().norm() +
                         foot_trans.local_pos().norm();
    return std::max(length, 1e-3f);
  };

  const float left_leg_length = compute_leg_length(left_chain_indices);
  const float right_leg_length = compute_leg_length(right_chain_indices);

  auto cache_frame_is_valid = [&](int frame_idx) {
    return frame_idx >= 0 && frame_idx < cache_size &&
           joint_rotation_cache[frame_idx].size() > root_cache_idx;
  };

  auto cache_local_rotation = [&](int frame_idx, int actor_idx) {
    const int entry_idx = actor_to_cache[actor_idx];
    if (entry_idx >= 0 && entry_idx < joint_rotation_cache[frame_idx].size())
      return joint_rotation_cache[frame_idx][entry_idx].normalized();
    return registry.get<transform>(player_actor.ordered_entities[actor_idx])
        .local_rot()
        .normalized();
  };

  auto set_cache_local_rotation = [&](int frame_idx, int actor_idx,
                                      const math::quat &rotation) {
    const int entry_idx = actor_to_cache[actor_idx];
    if (entry_idx >= 0 && entry_idx < joint_rotation_cache[frame_idx].size())
      joint_rotation_cache[frame_idx][entry_idx] = rotation.normalized();
  };

  auto compute_fk_pose = [&](int frame_idx, const math::vector3 &root_pos,
                             const math::quat &network_root,
                             std::vector<ik_joint_pose> &pose) {
    pose.resize(actor_count);
    pose[0].pos = root_pos;
    pose[0].rot = (network_root * char_repair_c[0]).normalized();

    for (int actor_idx = 1; actor_idx < actor_count; actor_idx++) {
      const int parent_idx = char_joint_parents[actor_idx];
      auto &joint_trans = registry.get<transform>(
          player_actor.ordered_entities[actor_idx]);
      const math::quat local_rot = cache_local_rotation(frame_idx, actor_idx);
      if (parent_idx >= 0 && parent_idx < actor_count) {
        pose[actor_idx].pos =
            pose[parent_idx].pos + pose[parent_idx].rot * joint_trans.local_pos();
        pose[actor_idx].rot = (pose[parent_idx].rot * local_rot).normalized();
      } else {
        pose[actor_idx].pos = joint_trans.local_pos();
        pose[actor_idx].rot = local_rot;
      }
    }
  };

  auto advance_root_pose = [&](int frame_idx, math::vector3 &root_pos,
                               math::quat &network_root) {
    if (!cache_frame_is_valid(frame_idx))
      return;

    math::quat network_heading = math::from_to_rot(
        math::vector3(0.0f, 0.0f, 1.0f), planar_forward_from_quat(network_root));
    if (root_rel_pos_cache[frame_idx].squaredNorm() > 1e-10f)
      root_pos += network_heading * root_rel_pos_cache[frame_idx];

    root_pos.y() = root_height_cache[frame_idx] +
                   sample_terrain_height(
                       math::vector2(root_pos.x(), root_pos.z()), root_pos.y());
    network_heading = root_rel_rot_cache[frame_idx] * network_heading;
    network_root =
        (network_heading * joint_rotation_cache[frame_idx][root_cache_idx])
            .normalized();
  };

  auto build_foot_targets = [&](const std::vector<math::vector3> &foot_fk,
                                const std::array<float, cache_size> &labels,
                                int actual_frame_count) {
    std::vector<ik_foot_target> targets(actual_frame_count);
    std::vector<float> contact_weights(actual_frame_count, 0.0f);

    for (int frame_offset = 0; frame_offset < actual_frame_count;
         frame_offset++) {
      const int frame_idx = buffer_start_idx + frame_offset;
      const math::vector3 foot_pos = foot_fk[frame_offset];
      const math::vector2 foot_xz(foot_pos.x(), foot_pos.z());
      const float terrain_height = sample_terrain_height(foot_xz, foot_pos.y());
      contact_weights[frame_offset] = smoothstep(
          ik_label_blend_low, ik_label_blend_high, saturate(labels[frame_idx]));

      const float penetration_weight = smoothstep(
          0.0f, ik_penetration_blend_depth, terrain_height - foot_pos.y());
      targets[frame_offset].target =
          math::vector3(foot_pos.x(), terrain_height, foot_pos.z());
      targets[frame_offset].weight = penetration_weight;
    }

    int frame_offset = 0;
    while (frame_offset < actual_frame_count) {
      if (contact_weights[frame_offset] <= ik_lock_on_threshold) {
        frame_offset++;
        continue;
      }

      int contact_start = frame_offset;
      while (contact_start > 0 &&
             contact_weights[contact_start - 1] > ik_lock_off_threshold) {
        contact_start--;
      }

      int contact_end = frame_offset + 1;
      while (contact_end < actual_frame_count &&
             contact_weights[contact_end] > ik_lock_off_threshold) {
        contact_end++;
      }

      math::vector2 averaged_xz = math::vector2::Zero();
      float averaged_height_offset = 0.0f;
      float total_weight = 0.0f;
      for (int sample_offset = contact_start; sample_offset < contact_end;
           sample_offset++) {
        const math::vector3 foot_pos = foot_fk[sample_offset];
        const math::vector2 foot_xz(foot_pos.x(), foot_pos.z());
        const float terrain_height =
            sample_terrain_height(foot_xz, foot_pos.y());
        const float sample_weight = std::max(contact_weights[sample_offset], 1e-3f);
        averaged_xz += foot_xz * sample_weight;
        averaged_height_offset +=
            (foot_pos.y() - terrain_height) * sample_weight;
        total_weight += sample_weight;
      }

      if (total_weight > 1e-6f) {
        averaged_xz /= total_weight;
        averaged_height_offset = std::clamp(
            averaged_height_offset / total_weight, ik_contact_height_offset_min,
            ik_contact_height_offset_max);

        const float target_height = sample_terrain_height(
                                        averaged_xz, foot_fk[frame_offset].y()) +
                                    averaged_height_offset;
        const math::vector3 contact_target(averaged_xz.x(), target_height,
                                           averaged_xz.y());
        for (int apply_offset = contact_start; apply_offset < contact_end;
             apply_offset++) {
          const float blend_in = smoothstep(
              0.0f, static_cast<float>(ik_lock_transition_frames),
              static_cast<float>(apply_offset - contact_start + 1));
          const float blend_out = smoothstep(
              0.0f, static_cast<float>(ik_lock_transition_frames),
              static_cast<float>(contact_end - apply_offset));
          const float target_blend = std::min(blend_in, blend_out);
          targets[apply_offset].target =
              targets[apply_offset].target +
              (contact_target - targets[apply_offset].target) * target_blend;
          targets[apply_offset].weight =
              std::max(targets[apply_offset].weight,
                       contact_weights[apply_offset] * target_blend);
        }
      }

      frame_offset = contact_end;
    }

    return targets;
  };

  auto solve_leg_to_foot = [&](int frame_idx,
                               const ik_chain_indices &chain_indices,
                               float leg_length,
                               const math::vector3 &root_pos,
                               const math::quat &network_root,
                               const ik_foot_target &foot_target,
                               std::vector<ik_joint_pose> &pose) {
    const float blend_weight = saturate(foot_target.weight);
    if (blend_weight <= 1e-4f)
      return;

    math::vector3 reachable_target = foot_target.target;
    const math::vector3 thigh_to_target =
        foot_target.target - pose[chain_indices.thigh].pos;
    const float target_distance = thigh_to_target.norm();
    const float max_reach = leg_length * 0.985f;
    if (target_distance > max_reach && target_distance > 1e-5f) {
      reachable_target = pose[chain_indices.thigh].pos +
                         thigh_to_target / target_distance * max_reach;
    }

    std::array<math::quat, 2> original_local_rot;
    std::array<int, 2> solver_joints = {chain_indices.shin, chain_indices.thigh};
    for (int solver_slot = 0; solver_slot < static_cast<int>(solver_joints.size());
         solver_slot++) {
      original_local_rot[solver_slot] =
          cache_local_rotation(frame_idx, solver_joints[solver_slot]);
    }

    for (int iteration_idx = 0; iteration_idx < ik_solver_iterations;
         iteration_idx++) {
      for (int solver_slot = 0; solver_slot < static_cast<int>(solver_joints.size());
          solver_slot++) {
        const int joint_actor_idx = solver_joints[solver_slot];
        const int effector_actor_idx = chain_indices.foot;
        const math::vector3 joint_pos = pose[joint_actor_idx].pos;
        const math::vector3 effector_delta = pose[effector_actor_idx].pos - joint_pos;
        const math::vector3 target_delta = reachable_target - joint_pos;
        if (effector_delta.squaredNorm() <= 1e-8f ||
            target_delta.squaredNorm() <= 1e-8f) {
          continue;
        }

        const math::quat delta_rot = math::from_to_rot(effector_delta, target_delta);
        const math::quat solved_world_rot =
            (delta_rot * pose[joint_actor_idx].rot).normalized();
        const int parent_idx = char_joint_parents[joint_actor_idx];
        const math::quat parent_world_rot =
            parent_idx >= 0 ? pose[parent_idx].rot : math::quat::Identity();
        set_cache_local_rotation(
            frame_idx, joint_actor_idx,
            (parent_world_rot.inverse() * solved_world_rot).normalized());
        compute_fk_pose(frame_idx, root_pos, network_root, pose);
      }
    }

    for (int solver_slot = 0; solver_slot < static_cast<int>(solver_joints.size());
         solver_slot++) {
      const int joint_actor_idx = solver_joints[solver_slot];
      const math::quat solved_local_rot =
          cache_local_rotation(frame_idx, joint_actor_idx);
      set_cache_local_rotation(
          frame_idx, joint_actor_idx,
          normalized_shortest_slerp(original_local_rot[solver_slot],
                                    solved_local_rot, blend_weight));
    }
    compute_fk_pose(frame_idx, root_pos, network_root, pose);
  };

  auto fit_toe_to_terrain = [&](int frame_idx,
                                const ik_chain_indices &chain_indices,
                                const math::vector3 &root_pos,
                                const math::quat &network_root,
                                float blend_weight,
                                std::vector<ik_joint_pose> &pose) {
    blend_weight = saturate(blend_weight);
    if (blend_weight <= 1e-4f)
      return;

    compute_fk_pose(frame_idx, root_pos, network_root, pose);
    const math::vector3 toe_pos = pose[chain_indices.toe].pos;
    math::vector3 toe_forward = pose[chain_indices.toe].rot *
                                math::vector3(0.0f, 0.0f, 1.0f);
    toe_forward.y() = 0.0f;
    if (toe_forward.squaredNorm() <= 1e-8f)
      toe_forward = planar_forward_from_quat(pose[chain_indices.foot].rot);
    toe_forward.normalize();

    const math::vector2 toe_xz(toe_pos.x(), toe_pos.z());
    const math::vector2 probe_xz(
        toe_pos.x() + toe_forward.x() * ik_toe_probe_distance,
        toe_pos.z() + toe_forward.z() * ik_toe_probe_distance);
    const float toe_height = sample_terrain_height(toe_xz, toe_pos.y());
    const float probe_height = sample_terrain_height(probe_xz, toe_pos.y());

    math::vector3 terrain_forward(toe_forward.x(),
                                  (probe_height - toe_height) /
                                      ik_toe_probe_distance,
                                  toe_forward.z());
    if (terrain_forward.squaredNorm() <= 1e-8f)
      return;
    terrain_forward.normalize();

    const math::vector3 current_forward =
        pose[chain_indices.toe].rot * math::vector3(0.0f, 0.0f, 1.0f);
    if (current_forward.squaredNorm() <= 1e-8f)
      return;

    const math::quat desired_world_rot =
        (math::from_to_rot(current_forward, terrain_forward) *
         pose[chain_indices.toe].rot)
            .normalized();
    const math::quat desired_local_rot =
        (pose[chain_indices.foot].rot.inverse() * desired_world_rot).normalized();
    const math::quat original_local_rot =
        cache_local_rotation(frame_idx, chain_indices.toe);
    set_cache_local_rotation(
        frame_idx, chain_indices.toe,
        normalized_shortest_slerp(original_local_rot, desired_local_rot,
                                  blend_weight));
    compute_fk_pose(frame_idx, root_pos, network_root, pose);
  };

  math::vector3 root_pos = root_trans.world_pos();
  math::quat sequence_network_root = network_root_rot;
  const int next_live_frame_idx = static_cast<int>(applied_frames);
  if (next_live_frame_idx <= transition_from_idx) {
    for (int frame_idx = next_live_frame_idx; frame_idx <= transition_from_idx;
         frame_idx++) {
      advance_root_pose(frame_idx, root_pos, sequence_network_root);
    }
  }

  const int clamped_frame_count =
      std::min({frame_count, cache_size - buffer_start_idx, cache_size / 2});
  if (clamped_frame_count <= 0)
    return;

  std::vector<math::vector3> frame_root_pos(clamped_frame_count);
  std::vector<math::quat> frame_network_root(clamped_frame_count);
  std::vector<math::vector3> left_foot_fk(clamped_frame_count);
  std::vector<math::vector3> right_foot_fk(clamped_frame_count);
  std::vector<ik_joint_pose> pose;

  int actual_frame_count = 0;
  for (int frame_offset = 0; frame_offset < clamped_frame_count; frame_offset++) {
    const int frame_idx = buffer_start_idx + frame_offset;
    if (!cache_frame_is_valid(frame_idx))
      break;

    advance_root_pose(frame_idx, root_pos, sequence_network_root);
    frame_root_pos[frame_offset] = root_pos;
    frame_network_root[frame_offset] = sequence_network_root;
    compute_fk_pose(frame_idx, root_pos, sequence_network_root, pose);
    left_foot_fk[frame_offset] = pose[left_chain_indices.foot].pos;
    right_foot_fk[frame_offset] = pose[right_chain_indices.foot].pos;
    actual_frame_count++;
  }
  if (actual_frame_count <= 0)
    return;

  const auto left_targets =
      build_foot_targets(left_foot_fk, ik_left_cache, actual_frame_count);
  const auto right_targets =
      build_foot_targets(right_foot_fk, ik_right_cache, actual_frame_count);

  for (int frame_offset = 0; frame_offset < actual_frame_count; frame_offset++) {
    const int frame_idx = buffer_start_idx + frame_offset;
    compute_fk_pose(frame_idx, frame_root_pos[frame_offset],
                    frame_network_root[frame_offset], pose);
    solve_leg_to_foot(frame_idx, right_chain_indices, right_leg_length,
                      frame_root_pos[frame_offset],
                      frame_network_root[frame_offset], right_targets[frame_offset],
                      pose);
    solve_leg_to_foot(frame_idx, left_chain_indices, left_leg_length,
                      frame_root_pos[frame_offset],
                      frame_network_root[frame_offset], left_targets[frame_offset],
                      pose);
    fit_toe_to_terrain(frame_idx, right_chain_indices, frame_root_pos[frame_offset],
                       frame_network_root[frame_offset],
                       right_targets[frame_offset].weight, pose);
    fit_toe_to_terrain(frame_idx, left_chain_indices, frame_root_pos[frame_offset],
                       frame_network_root[frame_offset],
                       left_targets[frame_offset].weight, pose);
  }
}

}; // namespace toolkit::opengl3d