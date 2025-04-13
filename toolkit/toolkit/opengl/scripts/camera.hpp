#pragma once

#include "toolkit/opengl/base.hpp"
#include "toolkit/scriptable.hpp"

namespace toolkit::opengl {

struct editor_camera : public scriptable<editor_camera> {
  bool mouseFirstMove = true;
  math::vector2 mouseLastPos;
  math::vector3 cameraPivot{0.0, 0.0, 0.0};

  // Some parameter related to camera control
  float initialFactor = 0.6f;
  float speedPow = 1.5f;
  float maxSpeed = 8e2f;
  float fpsSpeed = 1.2;

  void preupdate(iapp *app, float dt) override;
  void draw_to_scene(iapp *app) override;

  DECLARE_SCRIPT(editor_camera, basic, mouseFirstMove, mouseLastPos, cameraPivot,
                 initialFactor, speedPow, maxSpeed, fpsSpeed)
};

}; // namespace toolkit::opengl