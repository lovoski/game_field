#include "camdmpp.hpp"

#include "toolkit/opengl3d/components/actor.hpp"

namespace toolkit::opengl3d {

assets::bvh_data motion;
int motion_frame = 0;
std::vector<math::quat> repair_c;
std::map<int, int> data_to_actor;

void camdmpp::handle_custom_initialization() {
  if (std::filesystem::exists("camdmpp/model.onnx") &&
      std::filesystem::exists("camdmpp/config.json") &&
      std::filesystem::exists("camdmpp/setup.scene") &&
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
    set_game_mode(true, hide_mouse);
    player_entity = named_entities["player"];
    default_render_sys->resize(wnd_width, wnd_height);

    // setup diffusion model
    model.setup("camdmpp/model.onnx", "camdmpp/config.json");
    i_past_motion.resize(model.pose_token_dim * model.past_points);
    i_traj_pos.resize(10);
    i_traj_facing.resize(10);
    i_style_idx.resize(1);
  } else {
    std::cout << "Assets incomplete, can't start demo." << std::endl;
    quit_app_running();
  }

  motion = assets::load_bvh("camdmpp/motion_0.bvh");
  auto &bundle_data = registry.get<skinned_mesh_bundle>(player_entity);
  auto &player_trans = registry.get<transform>(player_entity);
  auto &player_actor = registry.get<actor>(bundle_data.actor_entities[0]);
  player_trans.force_update_hierarchy();
  repair_c.resize(player_actor.ordered_entities.size(), math::quat::Identity());
  for (int i = 0; i < player_actor.ordered_entities.size(); i++) {
    const auto &joint_trans =
        registry.get<transform>(player_actor.ordered_entities[i]);
    repair_c[i] = joint_trans.world_rot();
  }
  for (int i = 0; i < motion.names.size(); i++) {
    if (player_actor.name_to_entity.find(motion.names[i]) !=
        player_actor.name_to_entity.end()) {
      auto joint_entity = player_actor.name_to_entity[motion.names[i]];
      for (int j = 0; j < player_actor.ordered_entities.size(); j++) {
        if (player_actor.ordered_entities[j] == joint_entity) {
          data_to_actor[i] = j;
          break;
        }
      }
    }
  }
}

void camdmpp::handle_game_logic_tick(float dt) {
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
  // if (is_key_triggered(SDLK_1)) {
  //   hide_mouse = !hide_mouse;
  //   set_game_mode(true, hide_mouse);
  // }

  // update the camera movement every logic tick after character update
  // update camera position
  {
    auto &player_trans = registry.get<transform>(
        registry.get<skinned_mesh_bundle>(player_entity).actor_entities[0]);
    // cam_angle_horizontal -= dt * cam_move_speed * mouse_screen_delta.x();
    // cam_angle_vertical += dt * cam_move_speed * mouse_screen_delta.y();
    cam_angle_vertical = std::clamp(cam_angle_vertical, -10.0f, 80.0f);
    float cos_z = cos(math::deg_to_rad(cam_angle_vertical));
    math::vector3 cam_z =
        math::vector3(cos_z * sin(math::deg_to_rad(cam_angle_horizontal)),
                      sin(math::deg_to_rad(cam_angle_vertical)),
                      cos_z * cos(math::deg_to_rad(cam_angle_horizontal)))
            .normalized();
    math::vector3 cam_y(0.0, 1.0, 0.0);
    math::vector3 cam_x = (cam_y.cross(cam_z)).normalized();
    cam_y = (cam_z.cross(cam_x)).normalized();
    math::matrix3 cam_rot = math::matrix3::Identity();
    cam_rot << cam_x, cam_y, cam_z;
    auto &cam_trans = registry.get<transform>(active_camera);
    cam_trans.set_world_rot(math::quat(cam_rot));
    cam_trans.set_world_pos(player_trans.world_pos() + cam_z * 3);
  }

  default_render_sys->push_custom_draw([this]() { debug_draw(); });
}

void camdmpp::fixed_interval_logic() {
  auto &player_trans = registry.get<transform>(player_entity);
  auto &bundle_data = registry.get<skinned_mesh_bundle>(player_entity);
  auto &player_actor = registry.get<actor>(bundle_data.actor_entities[0]);

  if (motion_frame >= motion.local_rot.size())
    motion_frame = 0;
  std::vector<math::quat> motion_world_rot(motion.names.size(),
                                           math::quat::Identity());
  for (int i = 0; i < motion.names.size(); i++) {
    if (i == 0)
      motion_world_rot[i] = motion.local_rot[motion_frame][i];
    else
      motion_world_rot[i] = motion_world_rot[motion.parents[i]] *
                            motion.local_rot[motion_frame][i];
  }
  for (int i = 0; i < motion.names.size(); i++) {
    if (player_actor.name_to_entity.find(motion.names[i]) ==
        player_actor.name_to_entity.end())
      continue;
    auto &joint_trans =
        registry.get<transform>(player_actor.name_to_entity[motion.names[i]]);
    if (i == 0) {
      joint_trans.set_world_rot(motion_world_rot[i] *
                                repair_c[data_to_actor[i]]);
      joint_trans.set_world_pos(motion.local_pos[motion_frame][0]);
    } else
      joint_trans.set_local_rot(
          (motion_world_rot[motion.parents[i]] *
           repair_c[data_to_actor[motion.parents[i]]])
              .inverse() *
          (motion_world_rot[i] * repair_c[data_to_actor[i]]));
  }
  player_trans.force_update_hierarchy();
  motion_frame++;

  return;

  // make new predictions to the trajectory based on user input
  {
    auto [left_stick_raw, right_stick_raw, left_trigger, right_trigger] =
        get_game_controller_analog_inputs(get_game_controllers()[0]);
    math::vector3 left_stick(left_stick_raw.x(), 0.0f, left_stick_raw.y());
    math::vector3 right_stick(right_stick_raw.x(), 0.0f, right_stick_raw.y());
    math::quat desired_rot;
    desired_vel = velocity_scale * left_stick;
    if (left_stick.norm() > 0.01f)
      desired_dir = left_stick.normalized();
    if (right_stick.norm() > 0.01f)
      desired_dir = right_stick.normalized();
    desired_rot = math::from_to_rot(math::vector3(0, 0, 1), desired_dir);
    for (int i = 0; i < 5; i++) {
      auto [vel, acc] = spring_damper_position(
          char_root_world_vel, char_root_world_acc, desired_vel,
          math::vector3::Zero(), traj_sample_time * (i + 1), vel_halflife);
      auto [rot, ang] = spring_damper_rotation(
          char_root_world_rot, char_root_world_ang, desired_rot,
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
      _traj_world_dir[i].normalized();
      _traj_world_pos[i].y() = 0.0f;
    }
  }

  // apply pose to the character, fill in caches for network input
  {
    // find matching joint names and apply transform
    if (joint_rotation_cache[0].size() > 0) {
      for (int jid = 0; jid < model.joint_num; jid++) {
        auto joint_name = model.joint_names[jid];
        auto &joint_trans =
            registry.get<transform>(player_actor.name_to_entity[joint_name]);
        if (jid == 0) {
          // update root transform
          char_root_world_pos =
              char_root_world_pos +
              char_root_world_rot * root_rel_pos_cache[applied_frames];
          char_root_world_rot =
              root_rel_rot_cache[applied_frames] * char_root_world_rot;
          joint_trans.set_world_pos(char_root_world_pos);
          joint_trans.set_world_rot(char_root_world_rot);
        } else
          joint_trans.set_local_rot(joint_rotation_cache[applied_frames][jid]);
      }
      registry.get<transform>(player_actor.ordered_entities[0])
          .force_update_hierarchy();
    }

    // update network input cache
    i_style_idx[0] = 0;
    for (int i = 0; i < 5; i++) {
      auto _traj_pos = char_root_world_rot.inverse() *
                       (_traj_world_pos[i] - char_root_world_pos);
      auto _traj_facing = char_root_world_rot.inverse() * _traj_world_dir[i];
      i_traj_pos[2 * i + 0] = _traj_pos.x();
      i_traj_pos[2 * i + 1] = _traj_pos.z();
      i_traj_facing[2 * i + 0] = _traj_facing.x();
      i_traj_facing[2 * i + 1] = _traj_facing.z();
    }
    for (int p = 0; p < model.pose_token_dim; p++) {
      for (int f = 0; f < model.past_points; f++) {
        i_past_motion[p * model.past_points + f] =
            (i_past_motion[p * model.past_points + f] - model.data_mean[p]) /
            (model.data_std[p] + 1e-8f);
      }
    }

    // increase the apply frame counter
    applied_frames++;
  }

  // submit a new prediction when counter reaches a threashold
  if ((applied_frames >= submit_prediction_interval) &&
      !(waiting_for_model_output.load())) {
    waiting_for_model_output.store(true);
    model.past_motion_data = i_past_motion;
    model.traj_pos_data = i_traj_pos;
    model.traj_facing_data = i_traj_facing;
    model.style_idx_data = i_style_idx;
    model.submit_inference([this](std::vector<float> model_output) {
      printf("Inference finished when applied_frames=%d, update pose cache\n",
             applied_frames);
      applied_frames = 0;
      waiting_for_model_output.store(false);

      for (int i = 0; i < model.future_points; i++) {
      }
    });
  }
}

}; // namespace toolkit::opengl3d