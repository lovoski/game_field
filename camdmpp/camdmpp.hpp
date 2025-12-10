#pragma once

#include "onnx_model_configs.h"
#include "toolkit/anim/components/actor.hpp"
#include "toolkit/opengl/editor.hpp"
#include "toolkit/scriptable.hpp"
#include "toolkit/system.hpp"

#include "scripts/onnxruntime/camdm_diffusion.hpp"

namespace toolkit {

class camdmpp : public sub_system {
public:
  void draw_gui(entt::registry &registry, entt::entity entity) override;
  void draw_to_scene(entt::registry &registry, transform &cam_trans,
                     camera &cam_comp) override;

  void fixedupdate(entt::registry &registry, float dt) override;
};
DECLARE_SUB_SYSTEM(camdmpp, animation)

}; // namespace toolkit