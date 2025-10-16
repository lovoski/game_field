#include "toolkit/anim/scripts/traj_tracking.hpp"
#include "toolkit/anim/scripts/motion_matching.hpp"
#include "toolkit/opengl/editor.hpp"

namespace toolkit::anim {

const float const_e = 2.71828f;

void traj_tracking::destroy() {}

void traj_tracking::start() {
  auto actor_comp = registry->try_get<anim::actor>(entity);
  if (actor_comp != nullptr) {
    actor_bind_rot.resize(actor_comp->ordered_entities.size(),
                          math::quat::Identity());
    actor_bind_pos.resize(actor_comp->ordered_entities.size(),
                          math::vector3::Zero());
    actor_bind_mat.resize(actor_comp->ordered_entities.size(),
                          math::matrix4::Identity());
    registry->get<transform>(actor_comp->ordered_entities[0])
        .force_update_hierarchy();
    for (int i = 0; i < actor_comp->ordered_entities.size(); i++) {
      auto &joint_trans =
          registry->get<transform>(actor_comp->ordered_entities[i]);
      actor_bind_rot[i] = joint_trans.world_rot();
      actor_bind_pos[i] = joint_trans.local_pos();
      actor_bind_mat[i] = joint_trans.matrix();
    }
  }

  // // disable default editor camera manipulation
  // static_cast<opengl::editor *>(registry->ctx().get<iapp *>())
  //     ->editor_manipulate_camera = false;
}

void traj_tracking::update(iapp *app, float dt) {
  // // update camera settings
  // auto &cam_trans =
  // registry->get<transform>(opengl::g_instance.active_camera); math::vector3
  // cam_fixed_pos =
  //     root_pos + 3 * math::world_forward + 2 * math::world_up;
  // math::vector3 cam_focus_target = root_pos + 0.5 * math::world_up;
  // cam_trans.set_world_transform(
  //     math::lookat(cam_fixed_pos, cam_focus_target,
  //     math::world_up).inverse());
}

std::tuple<float, int, int> traj_tracking::lhmm(mm_context context,
                                                int cur_frame, int cur_range,
                                                int k, int l) {
  // recursive best motion clip search
  auto cur_query = compute_runtime_feature(cur_frame, context);
  float best;
  if (best_frame < YrangeStops[cur_range] - search_time) {
    best = feature_dist(cur_query, X[cur_frame]) - current_bias;
  } else
    best = std::numeric_limits<float>::max();
  if (l == 1) {
    // find the best and return
    int best_pred_frame = cur_frame, best_pred_range = cur_range;
    for (int range_idx = 0; range_idx < YrangeStarts.size(); range_idx++) {
      // search each range for the optimal feature
      for (int feat_idx = YrangeStarts[range_idx];
           feat_idx < YrangeStops[range_idx] - 60; feat_idx++) {
        float dist = feature_dist(cur_query, X[feat_idx]);
        // find which place this dist should be
        if (dist < best) {
          best = dist;
          best_pred_frame = feat_idx;
          best_pred_range = range_idx;
        }
      }
    }
    return {best, best_pred_frame, best_pred_range};
  } else {
    // top k best candidates
    std::vector<float> k_best_dist(k, best);
    std::vector<int> k_best_idx(k, cur_frame), k_best_range(k, cur_range);
    for (int i = 1; i < k; i++) {
      k_best_dist[i] = std::numeric_limits<float>::max();
      k_best_idx[i] = -1;
      k_best_range[i] = -1;
    }
    for (int range_idx = 0; range_idx < YrangeStarts.size(); range_idx++) {
      // search each range for the optimal feature
      for (int feat_idx = YrangeStarts[range_idx];
           feat_idx < YrangeStops[range_idx] - 60; feat_idx++) {
        float dist = feature_dist(cur_query, X[feat_idx]);
        // find which place this dist should be
        for (int bi = 0; bi < k; bi++) {
          if (dist < k_best_dist[bi]) {
            if (bi != k - 1) {
              k_best_dist[bi + 1] = k_best_dist[bi];
              k_best_idx[bi + 1] = k_best_idx[bi];
              k_best_range[bi + 1] = k_best_range[bi];
            }
            k_best_dist[bi] = dist;
            k_best_idx[bi] = feat_idx;
            k_best_range[bi] = range_idx;
          }
        }
      }
    }
    // find the most promising one as return value
    float best_pred_dist = std::numeric_limits<float>::max();
    int best_pred_k = -1;
    for (int i = 0; i < k; i++) {
      // contruct the predicted context start from "context" after "search_time"
      mm_context pred_context;
      int apply_best_idx = k_best_idx[i];
      int apply_best_range = k_best_range[i];

      math::quat _db_start_rot = Yrot[apply_best_idx][0];
      math::quat _ent_start_rot = context.root_world_rot;

      int _anim_frame = apply_best_idx;
      for (float time = 0; time < search_time; time += fixed_interval) {
        pred_context.root_world_rot =
            (Yrot[_anim_frame][0] * (_db_start_rot.inverse())) * _ent_start_rot;
        pred_context.root_world_vel =
            pred_context.root_world_rot *
            (Yrot[_anim_frame][0].inverse() * Yvel[_anim_frame][0]);
        if (pred_context.root_world_vel.norm() < 0.015)
          pred_context.root_world_vel = math::vector3::Zero();
        pred_context.root_world_pos +=
            pred_context.root_world_vel * fixed_interval;
        _anim_frame++;
      }

      // use the predicted context to perform recursive search
      auto [pred_dist, pred_frame, pred_range] =
          lhmm(pred_context, k_best_idx[i], k_best_range[i], k, l - 1);
      if (pred_dist < best_pred_dist) {
        best_pred_dist = pred_dist;
        best_pred_k = i;
      }
    }
    return {best_pred_dist, k_best_idx[best_pred_k], k_best_range[best_pred_k]};
  }
}

void traj_tracking::fixedupdate(iapp *app, float dt) {
  auto actor_comp = registry->try_get<anim::actor>(entity);
  if (db_loaded && mapping_loaded && trajectory_loaded &&
      (actor_comp != nullptr)) {
    if (search_timer <= 0.0f) {
      // find the closest point on trajectory as a start point
      float min_traj_diff = std::numeric_limits<float>::max();
      int min_traj_idx = -1;
      for (int i = std::min(applied_traj_frame + 1,
                            static_cast<int>(traj_points.size() - 1));
           i < std::min(applied_traj_frame + 60,
                        static_cast<int>(traj_points.size() - 1));
           i++) {
        float traj_diff = (cur_context.root_world_pos - traj_points[i]).norm();
        if (traj_diff < min_traj_diff) {
          min_traj_diff = traj_diff;
          min_traj_idx = i;
        }
      }
      applied_traj_frame = min_traj_idx;
      // we assume the trajectory and motion share the same frequency, so the
      // query must ensure within one search_time window, the trajectory and
      // character root motion match best
      for (int i = 0; i < 3; i++) {
        int sample_idx = std::min(applied_traj_frame + 5 * (i + 1),
                                  static_cast<int>(traj_points.size() - 1));
        cur_context.traj_world_pos[i] = traj_points[sample_idx];
        cur_context.traj_world_dir[i] = traj_facing[sample_idx];
      }
      desired_rot =
          math::from_to_rot(math::world_forward, cur_context.traj_world_dir[0]);

      // perform the optimized recursive search
      auto [search_best_dist, search_best_frame, search_best_range] =
          lhmm(cur_context, anim_frame, anim_range, 3, 3);
      best_frame = search_best_frame;
      best_range = search_best_range;

      if ((best_range != anim_range) || (best_frame != anim_frame)) {
        // make transition to the new motion
        inertialize_transition_position(off_pos, off_vel, Ypos[anim_frame],
                                        Yvel[anim_frame], Ypos[best_frame],
                                        Yvel[best_frame]);
        inertialize_transition_rotation(off_rot, off_ang, Yrot[anim_frame],
                                        Yang[anim_frame], Yrot[best_frame],
                                        Yang[best_frame]);
        anim_range = best_range;
        anim_frame = best_frame;
      }
      search_timer = search_time;

      db_start_rot = Yrot[best_frame][0];
      ent_start_rot = cur_context.root_world_rot;
    }
    // increment the animate frame by one
    anim_frame = std::clamp(anim_frame + 1.0f,
                            static_cast<float>(YrangeStarts[anim_range]),
                            static_cast<float>(YrangeStops[anim_range] - 1));
    search_timer -= dt;

    if (anim_frame >= YrangeStops[anim_range] - 4)
      search_timer = 0.0f;

    // auto [rot, ang] = spring_damper_rotation(
    //     cur_context.root_world_rot, cur_context.root_world_ang, desired_rot,
    //     math::vector3::Zero(), dt, rot_halflife);

    // update cur_context given anim_frame and anim_range
    // cur_context.root_world_ang = ang;
    // cur_context.root_world_rot = rot;
    cur_context.root_world_rot =
        (Yrot[anim_frame][0] * (db_start_rot.inverse())) * ent_start_rot;
    cur_context.root_world_vel =
        cur_context.root_world_rot *
        (Yrot[anim_frame][0].inverse() * Yvel[anim_frame][0]);
    // clamp small velocity to prevent drifting
    if (cur_context.root_world_vel.norm() < 0.015)
      cur_context.root_world_vel = math::vector3::Zero();
    // spdlog::info("anim_frame={0}", anim_frame);
    cur_context.root_world_pos += cur_context.root_world_vel * dt;

    root_pos_history.push_back(cur_context.root_world_pos);

    // return;
    // update the rest of the pose
    animate_character_with_context(dt);
  }
}

/**
 * root position and rotation from "cur_context"
 * the rest joint position and rotation from animation database
 */
void traj_tracking::animate_character_with_context(float dt) {
  data_joints_world_pos.resize(parents.size());
  // Ypos -> local position
  // Yrot -> local rotation
  std::vector<math::matrix4> local_trans(parents.size()),
      global_trans(parents.size());
  std::vector<math::quat> local_rot(parents.size(), math::quat::Identity()),
      world_rot(parents.size(), math::quat::Identity());
  std::vector<math::vector3> local_pos(parents.size(), math::vector3::Zero()),
      old_local_pos(parents.size(), math::vector3::Zero());
  math::vector3 scale_value = math::vector3::Ones();
  float iner_halflife = 0.075;
  for (int i = 0; i < parents.size(); i++) {
    if (inertialize) {
      auto [op, ov, out_pos, out_vel] = inertialize_update_position(
          off_pos[i], off_vel[i], Ypos[anim_frame][i], Yvel[anim_frame][i],
          iner_halflife, dt);
      auto [orf, oa, out_rot, out_ang] = inertialize_update_rotation(
          off_rot[i], off_ang[i], Yrot[anim_frame][i], Yang[anim_frame][i],
          iner_halflife, dt);
      off_pos[i] = op;
      off_vel[i] = ov;
      off_rot[i] = orf;
      off_ang[i] = oa;
      local_rot[i] = out_rot;
      local_pos[i] = out_pos;
    } else {
      local_rot[i] = Yrot[anim_frame][i];
      local_pos[i] = Ypos[anim_frame][i];
    }
    local_trans[i] =
        math::compose_transform(local_pos[i], local_rot[i], scale_value);
  }
  old_local_pos = local_pos;
  for (int i = 0; i < parents.size(); i++) {
    if (parents[i] != -1) {
      global_trans[i] = global_trans[parents[i]] * local_trans[i];
      world_rot[i] = world_rot[parents[i]] * local_rot[i];
    } else {
      global_trans[i] = math::compose_transform(
          cur_context.root_world_pos, cur_context.root_world_rot, scale_value);
      world_rot[i] = cur_context.root_world_rot;
    }
  }

  auto &actor_comp = registry->get<actor>(entity);
  for (int i = 0; i < actor_comp.ordered_entities.size(); i++) {
    auto joint_entity = actor_comp.ordered_entities[i];
    auto &joint_trans = registry->get<transform>(joint_entity);
    if (joint_name_to_idx.find(joint_trans.name) != joint_name_to_idx.end()) {
      int joint_data_idx = joint_name_to_idx[joint_trans.name];
      if (i == 0) {
        math::vector3 world_pos = global_trans[joint_data_idx].col(3).head<3>();
        joint_trans.set_world_pos(world_pos);
        joint_trans.set_world_rot(world_rot[joint_data_idx]);
      } else {
        joint_trans.set_local_rot(local_rot[joint_data_idx]);
      }
    }
  }
}

void traj_tracking::draw_to_scene(iapp *app) {
  opengl::script_draw_to_scene_proxy(app, [&](opengl::editor *editor,
                                              transform &cam_trans,
                                              opengl::camera &cam_comp) {
    // opengl::draw_arrow(root_pos, root_pos + desired_dir, cam_comp.vp,
    //                    opengl::Purple);
    for (int i = 0; i < 3; i++) {
      opengl::draw_wire_sphere(cur_context.traj_world_pos[i], cam_comp.vp, 0.1f,
                               opengl::Green);
      opengl::draw_arrow(cur_context.traj_world_pos[i],
                         cur_context.traj_world_pos[i] +
                             cur_context.traj_world_dir[i],
                         cam_comp.vp, opengl::Green);
    }

    if (trajectory_loaded && applied_traj_frame >= 0 &&
        applied_traj_frame < traj_points.size())
      opengl::draw_wire_sphere(traj_points[applied_traj_frame], cam_comp.vp,
                               0.1f, opengl::Purple);

    if (db_loaded && data_joints_world_pos.size() > 0) {
      std::vector<std::pair<math::vector3, math::vector3>> bone_pairs;
      for (int i = 0; i < parents.size(); i++) {
        if (parents[i] == -1 || parents[i] == 0)
          continue;
        bone_pairs.emplace_back(std::make_pair(
            data_joints_world_pos[parents[i]], data_joints_world_pos[i]));
      }
      opengl::draw_bones(bone_pairs, cam_comp.vp, opengl::Purple);
    }

    if (trajectory_loaded) {
      opengl::draw_wire_spheres(traj_points, cam_comp.vp, 0.005f, opengl::Red);
      opengl::draw_linestrip(traj_points, cam_comp.vp, opengl::Red);
      opengl::draw_wire_spheres(root_pos_history, cam_comp.vp, 0.005,
                                opengl::Purple);
    }

    opengl::draw_arrow(math::vector3::Zero(),
                       cur_context.root_world_rot * math::world_forward,
                       cam_comp.vp, opengl::Blue);
    opengl::draw_arrow(math::vector3::Zero(),
                       cur_context.root_world_rot * math::world_up, cam_comp.vp,
                       opengl::Green);
    opengl::draw_arrow(math::vector3::Zero(),
                       cur_context.root_world_rot * math::world_right,
                       cam_comp.vp, opengl::Red);
  });
}

void traj_tracking::draw_gui(iapp *app) {
  ImGui::Checkbox("Inertialize", &inertialize);
  ImGui::Text(str_format("Database: %s", db_filepath.c_str()).c_str());
  ImGui::Text(str_format("Joint Map: %s", mapping_filepath.c_str()).c_str());
  ImGui::Text(str_format("Traj File: %s", traj_filepath.c_str()).c_str());
  if (ImGui::Button("Select Database", {-1, 30}))
    if (open_file_dialog("Select Database", {"*.npz"}, db_filepath)) {
      auto data = cnpy::npz_load(db_filepath);
      int num_features = data["X"].shape[0];
      auto xarr = data["X"].as_vec<float>();
      auto xoffset_arr = data["Xoffset"].as_vec<float>();
      auto xscale_arr = data["Xscale"].as_vec<float>();
      X.resize(num_features);
      for (int i = 0; i < num_features; i++)
        for (int j = 0; j < MM_FEATURE_DIM; j++)
          X[i][j] = xarr[i * MM_FEATURE_DIM + j];
      for (int i = 0; i < MM_FEATURE_DIM; i++)
        Xoffset[i] = xoffset_arr[i];
      for (int i = 0; i < MM_FEATURE_DIM; i++)
        Xscale[i] = xscale_arr[i];
      auto &ypos_data = data["Ypos"];
      int num_joints = ypos_data.shape[1];

      auto ypos_arr = data["Ypos"].as_vec<float>();
      auto yrot_arr = data["Yrot"].as_vec<float>();
      auto yvel_arr = data["Yvel"].as_vec<float>();
      auto yang_arr = data["Yang"].as_vec<float>();
      Ypos.resize(num_features);
      Yvel.resize(num_features);
      Yang.resize(num_features);
      Yrot.resize(num_features);
      for (int i = 0; i < num_features; i++) {
        Ypos[i].resize(num_joints);
        Yvel[i].resize(num_joints);
        Yang[i].resize(num_joints);
        Yrot[i].resize(num_joints);
        for (int j = 0; j < num_joints; j++) {
          Ypos[i][j] << ypos_arr[i * num_joints * 3 + j * 3 + 0],
              ypos_arr[i * num_joints * 3 + j * 3 + 1],
              ypos_arr[i * num_joints * 3 + j * 3 + 2];
          Yvel[i][j] << yvel_arr[i * num_joints * 3 + j * 3 + 0],
              yvel_arr[i * num_joints * 3 + j * 3 + 1],
              yvel_arr[i * num_joints * 3 + j * 3 + 2];
          Yang[i][j] << yang_arr[i * num_joints * 3 + j * 3 + 0],
              yang_arr[i * num_joints * 3 + j * 3 + 1],
              yang_arr[i * num_joints * 3 + j * 3 + 2];
          Yrot[i][j].w() = yrot_arr[i * num_joints * 4 + j * 4 + 0];
          Yrot[i][j].x() = yrot_arr[i * num_joints * 4 + j * 4 + 1];
          Yrot[i][j].y() = yrot_arr[i * num_joints * 4 + j * 4 + 2];
          Yrot[i][j].z() = yrot_arr[i * num_joints * 4 + j * 4 + 3];
          Yrot[i][j].normalize();
        }
      }
      YrangeStarts = data["YrangeStarts"].as_vec<int>();
      YrangeStops = data["YrangeStops"].as_vec<int>();
      parents = data["parents"].as_vec<int>();

      off_rot.resize(parents.size(), math::quat::Identity());
      off_pos.resize(parents.size(), math::vector3::Zero());
      off_vel.resize(parents.size(), math::vector3::Zero());
      off_ang.resize(parents.size(), math::vector3::Zero());

      db_loaded = true;
    }
  if (ImGui::Button("Select Joint Mapping", {-1, 30}))
    if (open_file_dialog("Select Joint Mapping", {"*.json"},
                         mapping_filepath)) {
      std::ifstream input(mapping_filepath);
      if (input.is_open()) {
        nlohmann::json data;
        input >> data;
        // simulation bone as root, indexed 0
        for (auto [k, v] : data.items())
          joint_name_to_idx[k] = v.get<int>();
        mapping_loaded = true;
      }
    }
  if (ImGui::Button("Select Trajectory", {-1, 30})) {
    if (open_file_dialog("Select Trajectory File", {"*.txt"}, traj_filepath)) {
      std::ifstream input(traj_filepath);
      traj_points.clear();
      if (input.is_open()) {
        std::string line;
        while (std::getline(input, line)) {
          if (line == "\n" || line == "\r\n")
            continue;
          auto segs = split(trim(line));
          traj_points.push_back(math::vector3(
              std::stof(segs[0]), std::stof(segs[1]), std::stof(segs[2])));
          // traj_facing.push_back(math::vector3(
          //     std::stof(segs[3]), std::stof(segs[4]), std::stof(segs[5])));
        }
      }
      traj_facing.resize(traj_points.size());
      for (int i = 1; i < traj_points.size() - 1; i++)
        traj_facing[i] =
            (0.5 * (traj_points[i + 1] - traj_points[i - 1])).normalized();
      traj_facing[0] = traj_facing[1];
      traj_facing[traj_facing.size() - 1] = traj_facing[traj_facing.size() - 2];

      // auto data = cnpy::npz_load(traj_filepath);
      // auto &root_pos = data["pos"];
      // auto &root_vel = data["vel"];
      // auto &root_facing = data["facing"];
      // int nframes = root_pos.shape[0];
      // traj.pos.resize(nframes, math::vector3::Zero());
      // traj.vel.resize(nframes, math::vector3::Zero());
      // traj.facing.resize(nframes, math::vector3::Zero());
      // auto root_pos_arr = root_pos.as_vec<float>();
      // auto root_vel_arr = root_vel.as_vec<float>();
      // auto root_facing_arr = root_facing.as_vec<float>();
      // for (int i = 0; i < nframes; i++) {
      //   traj.pos[i] << root_pos_arr[3 * i + 0], root_pos_arr[3 * i + 1],
      //       root_pos_arr[3 * i + 2];
      //   traj.vel[i] << root_vel_arr[3 * i + 0], root_vel_arr[3 * i + 1],
      //       root_vel_arr[3 * i + 2];
      //   traj.facing[i] << root_facing_arr[3 * i + 0],
      //       root_facing_arr[3 * i + 1], root_facing_arr[3 * i + 2];
      // }
      applied_traj_frame = 0;
      trajectory_loaded = true;
    }
  }
}

std::array<float, MM_FEATURE_DIM>
traj_tracking::compute_runtime_feature(int frame, const mm_context &context) {
  std::array<float, MM_FEATURE_DIM> feature;
  // Xpos, Xvel
  for (int i = 0; i < 15; i++)
    feature[i] = X[frame][i] * Xscale[i] + Xoffset[i];
  // XtrajPos, XtrajDir
  for (int i = 0; i < 3; i++) {
    auto XtrajPos = context.root_world_rot.inverse() *
                    (context.traj_world_pos[i] - context.root_world_pos);
    feature[15 + 2 * i + 0] = XtrajPos.x();
    feature[15 + 2 * i + 1] = XtrajPos.z();
    auto XtrajDir =
        context.root_world_rot.inverse() * context.traj_world_dir[i];
    feature[21 + 2 * i + 0] = XtrajDir.x();
    feature[21 + 2 * i + 1] = XtrajDir.z();
  }
  // normalize
  for (int i = 0; i < MM_FEATURE_DIM; i++)
    feature[i] = (feature[i] - Xoffset[i]) / Xscale[i];
  return feature;
}

float traj_tracking::feature_dist(std::array<float, MM_FEATURE_DIM> &feat0,
                                  std::array<float, MM_FEATURE_DIM> &feat1) {
  float dist = 0.0f;
  for (int i = 0; i < MM_FEATURE_DIM; i++) {
    float value = (feat0[i] - feat1[i]) * (feat0[i] - feat1[i]);
    // if (i < 15)
    //   dist += value;
    // else
    //   dist += 2 * value;
    dist += value;
  }
  return std::sqrt(dist);
}

}; // namespace toolkit::anim