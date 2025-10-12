#pragma once

#include "onnx_model_configs.h"
#include "toolkit/anim/components/actor.hpp"
#include "toolkit/opengl/editor.hpp"
#include "toolkit/scriptable.hpp"
#include "toolkit/system.hpp"

#include "scripts/onnxruntime/camdm_diffusion.hpp"

namespace toolkit {

class camdmpp : public scriptable {
public:
  void draw_gui(iapp *app) override;
  void draw_to_scene(iapp *app) override;

  void fixedupdate(iapp *app, float dt) override;
};
DECLARE_SCRIPT(camdmpp, animation)

}; // namespace toolkit