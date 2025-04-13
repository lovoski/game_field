#pragma once

#include "toolkit/opengl/base.hpp"
#include "toolkit/scriptable.hpp"

namespace toolkit::opengl {

class editor_camera : public scriptable {
public:
  bool mouse_first_move = true;
  math::vector2 mouse_last_pos;
  math::vector3 camera_pivot{0.0, 0.0, 0.0};

  // Some parameter related to camera control
  float initial_factor = 0.6f;
  float speed_pow = 1.5f;
  float max_speed = 8e2f;
  float fps_speed = 1.2;

  bool vis_pivot = false;
  float vis_pivot_size = 0.1f;

  void preupdate(iapp *app, float dt) override;
  void draw_to_scene(iapp *app) override;
  void draw_gui(iapp *app) override;
};
DECLARE_SCRIPT(editor_camera, basic, mouse_first_move, mouse_last_pos,
               camera_pivot, initial_factor, speed_pow, max_speed, fps_speed,
               vis_pivot, vis_pivot_size)

}; // namespace toolkit::opengl