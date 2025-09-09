#include "toolkit/anim/scripts/motion_matching.hpp"
#include "toolkit/opengl/editor.hpp"

namespace toolkit::anim {

void motion_matching::start() {
  auto actor_comp = registry->try_get<anim::actor>(entity);
  if (actor_comp != nullptr) {
    actor_world_rot.resize(actor_comp->ordered_entities.size(),
                           math::quat::Identity());
    actor_world_pos.resize(actor_comp->ordered_entities.size(),
                           math::vector3::Zero());
    registry->get<transform>(actor_comp->ordered_entities[0])
        .force_update_hierarchy();
    for (int i = 0; i < actor_comp->ordered_entities.size(); i++) {
      auto &joint_trans =
          registry->get<transform>(actor_comp->ordered_entities[i]);
      actor_world_rot[i] = joint_trans.rotation();
      actor_world_pos[i] = joint_trans.position();
    }
  }
}
void motion_matching::destroy() {}

const float const_e = 2.71828f;

std::tuple<math::vector3, math::vector3>
spring_damper_position(math::vector3 x0, math::vector3 v0, math::vector3 xt,
                       math::vector3 vt, float dt, float halflife) {
  float lambda = log(2) / (halflife * log(const_e));
  math::vector3 x = x0 - xt, v = v0 - vt;
  auto x_prev = x;
  x = (x_prev + (v + lambda * x_prev) * dt) * std::exp(-lambda * dt);
  v = (v + lambda * x_prev) * std::exp(-lambda * dt) - lambda * x;
  return {x + xt, v + vt};
}

std::tuple<math::quat, math::vector3>
spring_damper_rotation(math::quat q0, math::vector3 av0, math::quat qt,
                       math::vector3 avt, float dt, float halflife) {
  float lambda = log(2) / (halflife * log(const_e));
  if (q0.dot(qt) < 0.0f)
    qt = math::quat(-qt.w(), -qt.x(), -qt.y(), -qt.z());
  auto q = math::quat_to_so3(q0 * qt.inverse());
  math::vector3 av = av0 - avt;
  auto q_prev = q;
  q = (q_prev + (av + lambda * q_prev) * dt) * exp(-lambda * dt);
  av = (av + lambda * q_prev) * exp(-lambda * dt) - lambda * q;
  return {toolkit::math::so3_to_quat(q) * qt, av + avt};
}

std::tuple<math::vector3, math::vector3, math::vector3, math::vector3>
inertialize_update_position(math::vector3 off_pos, math::vector3 off_vel,
                            math::vector3 in_pos, math::vector3 in_vel,
                            float halflife, float dt) {
  auto [op, ov] =
      spring_damper_position(off_pos, off_vel, math::vector3::Zero(),
                             math::vector3::Zero(), dt, halflife);
  return {op, ov, in_pos + op, in_vel + ov};
}

std::tuple<math::quat, math::vector3, math::quat, math::vector3>
inertialize_update_rotation(math::quat off_rot, math::vector3 off_ang,
                            math::quat in_rot, math::vector3 in_ang,
                            float halflife, float dt) {
  auto [ofr, oa] =
      spring_damper_rotation(off_rot, off_ang, math::quat::Identity(),
                             math::vector3::Zero(), dt, halflife);
  return {ofr, oa, ofr * in_rot, in_ang + oa};
}

template <typename T>
std::vector<T> sub_vec(std::vector<T> &v, int start, int end) {
  std::vector<T> r(end - start);
  for (int i = start; i < end; i++)
    r[i - start] = v[i];
  return r;
}

void motion_matching::update(iapp *app, float dt) {
  float residual = cur_time - cur_exec_fixed * fixed_interval;
  while (residual > fixed_interval) {
    residual -= fixed_interval;
    fixedupdate(app, fixed_interval);
    cur_exec_fixed += 1;
  }
  cur_time += dt;
}
void motion_matching::fixedupdate(iapp *app, float dt) {
  auto actor_comp = registry->try_get<anim::actor>(entity);
  if (xnpz_loaded && ynpz_loaded && mapping_loaded && (actor_comp != nullptr)) {
    // input and trajectory
    auto [left_stick, right_stick] =
        query_left_right_joystick(joystick_deadzone);
    desired_vel = 5.0f * left_stick;
    if (left_stick.norm() > 0.01f)
      desired_dir = left_stick.normalized();
    if (right_stick.norm() > 0.01f)
      desired_dir = right_stick.normalized();
    desired_rot = math::from_to_rot(math::vector3(0, 0, 1), desired_dir);

    for (int i = 0; i < 3; i++) {
      auto [vel, acc] = spring_damper_position(root_vel, root_acc, desired_vel,
                                               math::vector3::Zero(),
                                               t_times[i], vel_halflife);
      auto [rot, ang] = spring_damper_rotation(root_rot, root_ang, desired_rot,
                                               math::vector3::Zero(),
                                               t_times[i], rot_halflife);
      if (i == 0)
        t_pos[i] = (root_vel + vel) * 0.5 * t_times[i] + root_pos;
      else
        t_pos[i] =
            (t_vel[i - 1] + t_vel[i]) * 0.5 * (t_times[i] - t_times[i - 1]) +
            t_pos[i - 1];
      t_rot[i] = rot;
      t_vel[i] = vel;
      t_dir[i] = t_rot[i] * math::vector3(0, 0, 1);
    }

    // search
    if (search_timer < 0.0f) {
      auto Xquery = compute_runtime_feature(anim_frame);

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

        // for (int i = 1; i < parents.size(); i++) {
        //   auto [sdp, sdv] = spring_damper_position(
        //       Ypos[anim_frame][i], Yvel[anim_frame][i], Ypos[best_frame][i],
        //       Yvel[best_frame][i], dt, 0.2f);
        //   auto [sdr, sda] = spring_damper_rotation(
        //       Yrot[anim_frame][i], Yang[anim_frame][i], Yrot[best_frame][i],
        //       Yang[best_frame][i], dt, 0.2f);
        //   off_pos[i] = sdp;
        //   off_rot[i] = sdr;
        // }

        anim_range = best_range;
        anim_frame = best_frame;
      }
      search_timer = search_time;
    }
    anim_frame = std::clamp(anim_frame + 2.0f,
                            static_cast<float>(YrangeStarts[anim_range]),
                            static_cast<float>(YrangeStops[anim_range] - 1));
    search_timer -= dt;

    if (anim_frame >= YrangeStops[anim_range] - 4)
      search_timer = 0.0f;

    // spdlog::info("anim frame {0}", anim_frame);

    // update root
    auto [vel, acc] =
        spring_damper_position(root_vel, root_acc, desired_vel,
                               math::vector3::Zero(), dt, vel_halflife);
    auto [rot, ang] =
        spring_damper_rotation(root_rot, root_ang, desired_rot,
                               math::vector3::Zero(), dt, rot_halflife);
    root_acc = acc;
    // root_vel = vel;
    root_ang = ang;
    root_rot = rot;
    // root_ang = root_rot * (Yrot[anim_frame][0].inverse() *
    // Yang[anim_frame][0]);
    root_vel = root_rot * (Yrot[anim_frame][0].inverse() * Yvel[anim_frame][0]);
    root_pos = root_pos + root_vel * dt;
    // root_rot = math::so3_to_quat(root_ang * dt) * root_rot;
    // std::cout << root_ang.x() << "," << root_ang.y() << "," << root_ang.z()
    // << std::endl;

    // update the rest of the pose
    data_joints_world_pos.resize(parents.size());
    // Ypos -> local position
    // Yrot -> local rotation
    std::vector<math::matrix4> local_trans(parents.size()),
        global_trans(parents.size());
    math::vector3 scale_value = math::vector3::Ones();
    float iner_halflife = 0.075;
    for (int i = 0; i < parents.size(); i++) {
      if (i == 0) {
        local_trans[i] =
            math::compose_transform(root_pos, root_rot, scale_value);
      } else {
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
        local_trans[i] =
            math::compose_transform(out_pos, out_rot, scale_value);
        // local_trans[i] = math::compose_transform(
        //     Ypos[anim_frame][i], Yrot[anim_frame][i], scale_value);
      }
    }
    for (int i = 0; i < parents.size(); i++) {
      if (parents[i] != -1)
        global_trans[i] = global_trans[parents[i]] * local_trans[i];
      else
        global_trans[i] = local_trans[i];
    }
    for (int i = 0; i < parents.size(); i++) {
      data_joints_world_pos[i] = global_trans[i].col(3).head<3>();
    }
    // for (auto &p : joint_name_to_idx) {
    //   if (actor_comp->name_to_entity.find(p.first) !=
    //       actor_comp->name_to_entity.end()) {
    //     auto joint_entity = actor_comp->name_to_entity[p.first];
    //     auto joint_data_idx = p.second;
    //   } else {
    //     if (p.second != 0)
    //       spdlog::warn("Joint named {0} not found in actor comp", p.first);
    //   }
    // }
  }
}

void motion_matching::draw_to_scene(iapp *app) {
  opengl::script_draw_to_scene_proxy(app, [&](opengl::editor *editor,
                                              transform &cam_trans,
                                              opengl::camera &cam_comp) {
    opengl::draw_wire_spheres(t_pos, cam_comp.vp, 0.1f);
    for (int i = 0; i < 3; i++)
      opengl::draw_arrow(t_pos[i], t_pos[i] + t_dir[i], cam_comp.vp,
                         opengl::Green);
    opengl::draw_wire_sphere(root_pos, cam_comp.vp, 0.1f, opengl::Red);

    // opengl::draw_wire_spheres(data_joints_world_pos, cam_comp.vp, 0.1f,
    //                           opengl::Purple);
    if (ynpz_loaded && xnpz_loaded && data_joints_world_pos.size() > 0) {
      std::vector<std::pair<math::vector3, math::vector3>> bone_pairs;
      for (int i = 0; i < parents.size(); i++) {
        if (parents[i] == -1 || parents[i] == 0)
          continue;
        bone_pairs.emplace_back(std::make_pair(
            data_joints_world_pos[parents[i]], data_joints_world_pos[i]));
      }
      opengl::draw_bones(bone_pairs, cam_comp.vp, opengl::Purple);
    }

    opengl::draw_arrow(math::vector3::Zero(), root_rot * math::world_forward,
                       cam_comp.vp, opengl::Blue);
    opengl::draw_arrow(math::vector3::Zero(), root_rot * math::world_up,
                       cam_comp.vp, opengl::Green);
    opengl::draw_arrow(math::vector3::Zero(), root_rot * math::world_left,
                       cam_comp.vp, opengl::Red);
  });
}

void motion_matching::draw_gui(iapp *app) {
  ImGui::Text(str_format("Database X: %s", x_filepath.c_str()).c_str());
  ImGui::Text(str_format("Database Y: %s", y_filepath.c_str()).c_str());
  ImGui::Text(str_format("Joint Map: %s", mapping_filepath.c_str()).c_str());
  if (ImGui::Button("Select Database X", {-1, 30}))
    if (open_file_dialog("Select Database X", {"*.npz"}, "*.npz", x_filepath)) {
      auto xdata = cnpy::npz_load(x_filepath);
      int num_features = xdata["X"].shape[0];
      if (xdata["X"].word_size == 4)
        load_x_npz(xdata["X"].as_vec<float>(), num_features);
      else if (xdata["X"].word_size == 8)
        load_x_npz(xdata["X"].as_vec<double>(), num_features);
      if (xdata["Xoffset"].word_size == 4) {
        auto xoffset_data = xdata["Xoffset"].as_vec<float>();
        for (int i = 0; i < MM_FEATURE_DIM; i++)
          Xoffset[i] = xoffset_data[i];
      } else if (xdata["Xoffset"].word_size == 8) {
        auto xoffset_data = xdata["Xoffset"].as_vec<double>();
        for (int i = 0; i < MM_FEATURE_DIM; i++)
          Xoffset[i] = xoffset_data[i];
      }
      if (xdata["Xscale"].word_size == 4) {
        auto xscale_data = xdata["Xscale"].as_vec<float>();
        for (int i = 0; i < MM_FEATURE_DIM; i++)
          Xscale[i] = xscale_data[i];
      } else if (xdata["Xscale"].word_size == 8) {
        auto xscale_data = xdata["Xscale"].as_vec<double>();
        for (int i = 0; i < MM_FEATURE_DIM; i++)
          Xscale[i] = xscale_data[i];
      }
      xnpz_loaded = true;
    }
  if (ImGui::Button("Select Database Y", {-1, 30}))
    if (open_file_dialog("Select Database Y", {"*.npz"}, "*.npz", y_filepath)) {
      auto ydata = cnpy::npz_load(y_filepath);
      auto &ypos_data = ydata["Ypos"];
      int num_features = ypos_data.shape[0];
      int num_joints = ypos_data.shape[1];
      if (ypos_data.word_size == 4)
        load_y_npz<float>(ydata, num_features, num_joints);
      else if (ypos_data.word_size == 8)
        load_y_npz<double>(ydata, num_features, num_joints);
      ynpz_loaded = true;
    }
  if (ImGui::Button("Select Joint Mapping", {-1, 30}))
    if (open_file_dialog("Select Joint Mapping", {"*.json"}, "*.json",
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
motion_matching::compute_runtime_feature(int frame) {
  std::array<float, MM_FEATURE_DIM> feature;
  // Xpos, Xvel
  for (int i = 0; i < 15; i++)
    feature[i] = X[frame][i] * Xscale[i] + Xoffset[i];
  // XtrajPos, XtrajDir
  for (int i = 0; i < 3; i++) {
    auto XtrajPos = root_rot.inverse() * (t_pos[i] - root_pos);
    feature[15 + 2 * i + 0] = XtrajPos.x();
    feature[15 + 2 * i + 1] = XtrajPos.z();
    auto XtrajDir = root_rot.inverse() * t_dir[i];
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
  for (int i = 0; i < MM_FEATURE_DIM; i++)
    dist += (feat0[i] - feat1[i]) * (feat0[i] - feat1[i]);
  return std::sqrt(dist);
}

void inertialize_transition_position(std::vector<math::vector3> &off_pos,
                                     std::vector<math::vector3> &off_vel,
                                     std::vector<math::vector3> src_pos,
                                     std::vector<math::vector3> src_vel,
                                     std::vector<math::vector3> target_pos,
                                     std::vector<math::vector3> target_vel) {
  int num_joints = off_pos.size();
  for (int i = 0; i < num_joints; i++) {
    off_pos[i] = off_pos[i] + src_pos[i] - target_pos[i];
    off_vel[i] = off_vel[i] + src_vel[i] - target_vel[i];
  }
}

void inertialize_transition_rotation(std::vector<math::quat> &off_rot,
                                     std::vector<math::vector3> &off_ang,
                                     std::vector<math::quat> src_rot,
                                     std::vector<math::vector3> src_ang,
                                     std::vector<math::quat> target_rot,
                                     std::vector<math::vector3> target_ang) {
  int num_joints = off_rot.size();
  for (int i = 0; i < num_joints; i++) {
    off_rot[i] = (off_rot[i] * src_rot[i]) * (target_rot[i].inverse());
    off_ang[i] = off_ang[i] + src_ang[i] - target_ang[i];
  }
}

std::tuple<math::vector3, math::vector3>
query_left_right_joystick(float deadzone) {
  math::vector3 left_stick = math::vector3::Zero(),
                right_stick = math::vector3::Zero();
  for (int i = GLFW_JOYSTICK_1; i < GLFW_JOYSTICK_LAST; i++) {
    if (glfwJoystickPresent(i)) {
      int axes_count;
      auto axes = glfwGetJoystickAxes(i, &axes_count);
      float left_mag = std::sqrt(axes[0] * axes[0] + axes[1] * axes[1]);
      float right_mag = std::sqrt(axes[2] * axes[2] + axes[3] * axes[3]);
      if (left_mag > deadzone) {
        float clip_mag = left_mag > 1.0 ? 1.0 : left_mag * left_mag;
        left_stick.x() = axes[0] / left_mag * clip_mag;
        left_stick.z() = axes[1] / left_mag * clip_mag;
      }
      if (right_mag > deadzone) {
        float clip_mag = right_mag > 1.0 ? 1.0 : right_mag * right_mag;
        right_stick.x() = axes[0] / right_mag * clip_mag;
        right_stick.z() = axes[1] / right_mag * clip_mag;
      }
      break;
    }
  }
  return {left_stick, right_stick};
}

}; // namespace toolkit::anim