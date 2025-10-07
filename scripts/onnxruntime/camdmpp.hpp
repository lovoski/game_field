#pragma once

#include "onnx_model_configs.h"
#include "toolkit/anim/components/actor.hpp"
#include "toolkit/opengl/editor.hpp"
#include "toolkit/scriptable.hpp"
#include "toolkit/system.hpp"

namespace toolkit {

class camdmpp : public scriptable {
public:
  void draw_gui(iapp *app) override;
  void draw_to_scene(iapp *app) override;

  void update(iapp *app, float dt) override;
  void fixedupdate(iapp *app, float dt);

private:
  int cur_exec_fixed = 0;
  float cur_time = 0.0f, fixed_interval = 1.0f / 60.0f;
};
DECLARE_SCRIPT(camdmpp, animation)

}; // namespace toolkit