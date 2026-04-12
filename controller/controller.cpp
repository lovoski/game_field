#include "controller.hpp"

#include "toolkit/opengl3d/components/character_controller.hpp"
#include "toolkit/opengl3d/components/physics_world.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>

namespace toolkit::opengl3d {

void controller::handle_game_logic_tick(float dt) {
  handle_player_input(dt);

  default_render_sys->push_custom_draw([this]() { handle_scene_draw(); });
}

void controller::handle_player_input(float dt) {
  // handle mouse visibility toggle
  if (is_key_triggered(SDLK_ESCAPE)) {
    hide_mouse = !hide_mouse;
    set_game_mode(true, hide_mouse);
  }
  if (!hide_mouse)
    return;

  update_movement(dt);
  update_camera(dt);
  update_debug_caches(dt);
}

void controller::update_movement(float dt) {
  auto &player_trans = registry.get<transform>(player_entity);
  auto &camera_trans = registry.get<transform>(active_camera);
  auto &player_cc = registry.get<character_controller>(player_entity);
  camera_forward = -math::vector3(camera_trans.local_forward().x(), 0.0f,
                                  camera_trans.local_forward().z())
                        .normalized();
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

  player_velocity += gravity * dt;
  if (player_cc.grounded && player_velocity.y() < 0.0f) {
    player_velocity.y() = -1.0f;
  }

  player_velocity.x() = move_input.x() * move_speed;
  player_velocity.z() = move_input.z() * move_speed;
  physics_world_sys->cc_move(player_cc, player_velocity * dt);
}

void controller::update_camera(float dt) {
  auto delta = get_mouse_screen_delta();
  camera_horizontal_angle -= mouse_sensitivity * delta.x();
  camera_vertical_angle += mouse_sensitivity * delta.y();
  camera_vertical_angle =
      std::clamp(camera_vertical_angle, min_vertical_angle, max_vertical_angle);

  auto &player_trans = registry.get<transform>(player_entity);
  auto &cam_trans = registry.get<transform>(active_camera);
  math::vector3 cam_offset =
      math::angle_axis(math::deg_to_rad(camera_horizontal_angle),
                       math::world_up) *
      math::angle_axis(math::deg_to_rad(-camera_vertical_angle),
                       math::world_right) *
      math::vector3(0.0f, 0.0f, camera_distance);
  math::vector3 cam_pos = player_trans.world_pos() +
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

void controller::update_debug_caches(float dt) {
  static std::int64_t __cur_exec_fixed = 0;
  static double __cur_time = 0.0f, fixed_interval = 1.0f / 20.0f;

  double residual = __cur_time - __cur_exec_fixed * fixed_interval;
  while (residual > fixed_interval) {

    auto &player_trans = registry.get<transform>(player_entity);
    for (int i = 0; i < 10; i++) {
      debug_traj_points[i] = debug_traj_points[i + 1];
      debug_traj_facing[i] = debug_traj_facing[i + 1];
    }
    debug_traj_points[10] = math::vector3(player_trans.world_pos().x(), 0.0f,
                                          player_trans.world_pos().z());
    debug_traj_facing[10] = camera_forward;

    residual -= fixed_interval;
    __cur_exec_fixed += 1;
  }
  __cur_time += dt;
}

bool controller::load_setup_scene(const std::string &path) {
  if (!std::filesystem::exists(path))
    return false;

  std::ifstream scene_input(path);
  if (!scene_input.is_open())
    return false;

  try {
    auto data = nlohmann::json::parse(
        std::string((std::istreambuf_iterator<char>(scene_input)),
                    std::istreambuf_iterator<char>()));
    deserialize(data);
    return true;
  } catch (const std::exception &ex) {
    std::cout << "[controller] failed to parse scene \"" << path
              << "\": " << ex.what() << std::endl;
    return false;
  }
}

void controller::handle_custom_initialization() {
  std::string scene_path = "controller/setup.scene";
  if (!load_setup_scene(scene_path) && !load_setup_scene("setup.scene")) {
    std::cout << "[controller] scene file not found. Expected at \""
              << scene_path << "\" (or fallback \"setup.scene\")." << std::endl;
    quit_app_running();
    return;
  }
  set_game_mode(true, hide_mouse);
  player_entity = named_entities["player"];
  default_render_sys->resize(wnd_width, wnd_height);
  transform_hierarchy_sys->update_transform(registry);

  for (int i = 0; i < 21; i++) {
    debug_traj_points[i] = math::vector3::Zero();
    debug_traj_facing[i] = math::vector3(0.0f, 0.0f, -1.0f);
  }
}

}; // namespace toolkit::opengl3d
