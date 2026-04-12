#pragma once

#include "toolkit/opengl3d/engine.hpp"

namespace toolkit::opengl3d {

class controller : public engine3d {
public:
  void handle_custom_initialization() override;
  void handle_game_logic_tick(float dt) override;
  void handle_engine_gui() override;

  void handle_scene_draw();

private:
  bool hide_mouse = true;
  entt::entity player_entity = entt::null;

  // input
  float move_speed = 15.0f;
  math::vector3 player_velocity = math::vector3::Zero();
  math::vector3 move_input = math::vector3::Zero();
  math::vector3 gravity = math::vector3(0.0f, -40.0f, 0.0f);
  void update_movement(float dt);

  // camera
  float camera_horizontal_angle = 0.0f, camera_vertical_angle = 30.0f,
        camera_distance = 5.0f, camera_height = 0.0f, mouse_sensitivity = 0.2f,
        camera_follow_speed = 8.0f;
  float min_vertical_angle = -20.0f, max_vertical_angle = 60.0f;
  math::vector3 camera_forward; // read only
  void update_camera(float dt);

  // debug render
  std::array<math::vector3, 21> debug_traj_points, debug_traj_facing;
  void update_debug_caches(float dt);

  bool load_setup_scene(const std::string &path);

  void handle_player_input(float dt);
};

}; // namespace toolkit::opengl3d