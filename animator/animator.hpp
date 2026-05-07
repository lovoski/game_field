#pragma once

#include "toolkit/opengl3d/engine.hpp"
#include "toolkit/opengl3d/components/actor.hpp"

namespace toolkit::opengl3d {

class animator : public engine3d {
public:
  void handle_custom_initialization() override;
  void handle_game_logic_tick(float dt) override;
  void handle_engine_gui() override;

private:
  bool hide_mouse = true;

  float motion_time = 0.0f, motion_time_scale = 1.0f;
  assets::bvh_data motion_data;

  // camera related
  float camera_horizontal_angle = 0.0f, camera_vertical_angle = 30.0f,
        camera_distance = 2.0f, camera_height = 0.0f, mouse_sensitivity = 0.3f,
        camera_follow_speed = 8.0f;
  math::vector3 camera_forward;
  float min_vertical_angle = -20.0f, max_vertical_angle = 80.0f;
  void update_camera(float dt);

  entt::entity player_entity = entt::null;

  std::vector<math::quat> char_repair_c;
  std::vector<int> char_joint_parents;
  std::map<int, int> char_data_to_actor;

  void debug_draw();
};

}; // namespace toolkit::opengl3d