#include "toolkit/anim/scripts/motion_matching.hpp"
#include "toolkit/opengl/editor.hpp"

namespace toolkit::anim {

void motion_matching::destroy(entt::registry &registry) {}

void motion_matching::start(entt::registry &registry) {
  // auto actor_comp = registry.try_get<anim::actor>(entity);
  // if (actor_comp != nullptr) {
  //   actor_bind_rot.resize(actor_comp->ordered_entities.size(),
  //                         math::quat::Identity());
  //   actor_bind_pos.resize(actor_comp->ordered_entities.size(),
  //                         math::vector3::Zero());
  //   actor_bind_mat.resize(actor_comp->ordered_entities.size(),
  //                         math::matrix4::Identity());
  //   registry.get<transform>(actor_comp->ordered_entities[0])
  //       .force_update_hierarchy();
  //   for (int i = 0; i < actor_comp->ordered_entities.size(); i++) {
  //     auto &joint_trans =
  //         registry.get<transform>(actor_comp->ordered_entities[i]);
  //     actor_bind_rot[i] = joint_trans.world_rot();
  //     actor_bind_pos[i] = joint_trans.local_pos();
  //     actor_bind_mat[i] = joint_trans.matrix();
  //   }
  // }
}

void motion_matching::update(entt::registry &registry, float dt) {
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

void motion_matching::fixedupdate(entt::registry &registry, float dt) {
  auto actor_comp = registry.try_get<anim::actor>(entity);
  auto controllers = opengl::sdl_context::get_instance().get_game_controllers();
  if (controllers.size() > 0 && db_loaded && mapping_loaded &&
      (actor_comp != nullptr)) {
    // input and trajectory
    auto [left_stick_raw, right_stick_raw, left_trigger, right_trigger] =
        opengl::sdl_context::get_instance().get_game_controller_analog_inputs(
            controllers[0]);
    math::vector3 left_stick(left_stick_raw.x(), 0.0f, left_stick_raw.y());
    math::vector3 right_stick(right_stick_raw.x(), 0.0f, right_stick_raw.y());
    desired_vel = 5 * left_stick;
    if (left_stick.norm() > 0.01f)
      desired_dir = left_stick.normalized();
    if (right_stick.norm() > 0.01f)
      desired_dir = right_stick.normalized();
    desired_rot = math::from_to_rot(math::vector3(0, 0, 1), desired_dir);

    // fill in the trajectory data
    std::array<math::vector3, 3> traj_world_vel;
    for (int i = 0; i < 3; i++) {
      auto [vel, acc] = spring_damper_position(
          context.root_world_vel, context.root_world_acc, desired_vel,
          math::vector3::Zero(), traj_sample_time * (i + 1), vel_halflife);
      auto [rot, ang] = spring_damper_rotation(
          context.root_world_rot, context.root_world_ang, desired_rot,
          math::vector3::Zero(), traj_sample_time * (i + 1), rot_halflife);
      traj_world_vel[i] = vel;
      if (i == 0)
        context.traj_world_pos[i] =
            (context.root_world_vel + vel) * 0.5 * traj_sample_time * (i + 1) +
            context.root_world_pos;
      else
        context.traj_world_pos[i] =
            (traj_world_vel[i - 1] + traj_world_vel[i]) * 0.5 *
                traj_sample_time +
            context.traj_world_pos[i - 1];
      context.traj_world_dir[i] = (rot * math::vector3(0, 0, 1)).normalized();
    }

    // search
    if (search_timer <= 0.0f) {
      auto Xquery = compute_runtime_feature(anim_frame, context);

      best_range = anim_range;
      best_frame = anim_frame;

      float best;
      if (best_frame < YrangeStops[best_range] - search_time) {
        best = feature_dist(Xquery, X[best_frame]) - current_bias;
      } else
        best = std::numeric_limits<float>::max();

      for (int range_idx = 0; range_idx < YrangeStarts.size(); range_idx++) {
        // search each range for the optimal feature
        for (int feat_idx = YrangeStarts[range_idx];
             feat_idx < YrangeStops[range_idx] - 60; feat_idx++) {
          float dist = feature_dist(Xquery, X[feat_idx]);
          if (dist < best) {
            best = dist;
            best_range = range_idx;
            best_frame = feat_idx;
          }
        }
      }

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

      ent_start_rot = context.root_world_rot;
      db_start_rot = Yrot[best_frame][0];

      search_timer = search_time;
    }
    anim_frame = std::clamp(anim_frame + 1.0f,
                            static_cast<float>(YrangeStarts[anim_range]),
                            static_cast<float>(YrangeStops[anim_range] - 1));
    search_timer -= dt;

    if (anim_frame >= YrangeStops[anim_range] - 4)
      search_timer = 0.0f;

    context.root_world_rot =
        (Yrot[anim_frame][0] * (db_start_rot.inverse())) * ent_start_rot;
    context.root_world_vel =
        context.root_world_rot *
        (Yrot[anim_frame][0].inverse() * Yvel[anim_frame][0]);
    if (context.root_world_vel.norm() < 0.015)
      context.root_world_vel = math::vector3::Zero();
    context.root_world_pos =
        context.root_world_pos + context.root_world_vel * dt;

    // update the rest of the pose
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
      local_trans[i] = math::compose_transform(out_pos, out_rot, scale_value);
    }
    old_local_pos = local_pos;
    for (int i = 0; i < parents.size(); i++) {
      if (parents[i] != -1) {
        global_trans[i] = global_trans[parents[i]] * local_trans[i];
        world_rot[i] = world_rot[parents[i]] * local_rot[i];
      } else {
        // replace the transform of simulation object with the user-controlled
        // variables
        global_trans[i] = math::compose_transform(
            context.root_world_pos, context.root_world_rot, scale_value);
        world_rot[i] = context.root_world_rot;
      }
    }

    for (int i = 0; i < actor_comp->ordered_entities.size(); i++) {
      auto joint_entity = actor_comp->ordered_entities[i];
      auto &joint_trans = registry.get<transform>(joint_entity);
      if (joint_name_to_idx.find(joint_trans.name) !=
      joint_name_to_idx.end()) {
        int joint_data_idx = joint_name_to_idx[joint_trans.name];
        if (i == 0) {
          math::vector3 world_pos =
              global_trans[joint_data_idx].col(3).head<3>();
          joint_trans.set_world_pos(world_pos);
          joint_trans.set_world_rot(world_rot[joint_data_idx]);
        } else {
          joint_trans.set_local_rot(local_rot[joint_data_idx]);
        }
      }
    }
  }
}

void motion_matching::draw_to_scene(entt::registry &registry,
                                    transform &cam_trans, camera &cam_comp) {
  std::vector<math::vector3> t_pos(3);
  t_pos[0] = context.traj_world_pos[0];
  t_pos[1] = context.traj_world_pos[1];
  t_pos[2] = context.traj_world_pos[2];
  opengl::draw_wire_spheres(t_pos, cam_comp.vp, 0.1f);
  opengl::draw_arrow(context.root_world_pos,
                     context.root_world_pos + desired_dir, cam_comp.vp,
                     opengl::Purple);
  for (int i = 0; i < 3; i++)
    opengl::draw_arrow(t_pos[i], t_pos[i] + context.traj_world_dir[i],
                       cam_comp.vp, opengl::Green);
  opengl::draw_wire_sphere(context.root_world_pos, cam_comp.vp, 0.1f,
                           opengl::Red);

  // opengl::draw_wire_spheres(data_joints_world_pos, cam_comp.vp, 0.1f,
  //                           opengl::Purple);
  if (db_loaded && data_joints_world_pos.size() > 0) {
    std::vector<std::pair<math::vector3, math::vector3>> bone_pairs;
    for (int i = 0; i < parents.size(); i++) {
      if (parents[i] == -1 || parents[i] == 0)
        continue;
      bone_pairs.emplace_back(std::make_pair(data_joints_world_pos[parents[i]],
                                             data_joints_world_pos[i]));
    }
    opengl::draw_bones(bone_pairs, cam_comp.vp, opengl::Purple);
  }

  opengl::draw_arrow(math::vector3::Zero(),
                     context.root_world_rot * math::world_forward, cam_comp.vp,
                     opengl::Blue);
  opengl::draw_arrow(math::vector3::Zero(),
                     context.root_world_rot * math::world_up, cam_comp.vp,
                     opengl::Green);
  opengl::draw_arrow(math::vector3::Zero(),
                     context.root_world_rot * math::world_right, cam_comp.vp,
                     opengl::Red);
}

void motion_matching::draw_gui(entt::registry &registry, entt::entity entity) {
  ImGui::Text(str_format("Database: %s", db_filepath.c_str()).c_str());
  ImGui::Text(str_format("Joint Map: %s", mapping_filepath.c_str()).c_str());
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
}

std::array<float, MM_FEATURE_DIM>
motion_matching::compute_runtime_feature(int frame, const mm_context &ctx) {
  std::array<float, MM_FEATURE_DIM> feature;
  // Xpos, Xvel
  for (int i = 0; i < 15; i++)
    feature[i] = X[frame][i] * Xscale[i] + Xoffset[i];
  // XtrajPos, XtrajDir
  for (int i = 0; i < 3; i++) {
    auto XtrajPos = ctx.root_world_rot.inverse() *
                    (ctx.traj_world_pos[i] - ctx.root_world_pos);
    feature[15 + 2 * i + 0] = XtrajPos.x();
    feature[15 + 2 * i + 1] = XtrajPos.z();
    auto XtrajDir = ctx.root_world_rot.inverse() * ctx.traj_world_dir[i];
    feature[21 + 2 * i + 0] = XtrajDir.x();
    feature[21 + 2 * i + 1] = XtrajDir.z();
  }
  // normalize
  for (int i = 0; i < MM_FEATURE_DIM; i++)
    feature[i] = (feature[i] - Xoffset[i]) / Xscale[i];
  return feature;
}

float motion_matching::feature_dist(std::array<float, MM_FEATURE_DIM> &feat0,
                                    std::array<float, MM_FEATURE_DIM> &feat1) {
  float dist = 0.0f;
  for (int i = 0; i < MM_FEATURE_DIM; i++) {
    float value = (feat0[i] - feat1[i]) * (feat0[i] - feat1[i]);
    // if (i < 15)
    //   dist += value;
    // else
    //   dist += 2*value;
    dist += value;
  }
  return std::sqrt(dist);
}

}; // namespace toolkit::anim