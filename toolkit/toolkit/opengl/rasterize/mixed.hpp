#pragma once

#include "toolkit/opengl/base.hpp"
#include "toolkit/system.hpp"

#include "toolkit/opengl/components/camera.hpp"
#include "toolkit/opengl/components/lights.hpp"
#include "toolkit/opengl/components/mesh.hpp"

namespace toolkit::opengl {

class defered_forward_mixed : public isystem {
public:
  void init0(entt::registry &registry) override;

  void resize(int width, int height);

  void preupdate(entt::registry &registry, float dt) override;
  void render(entt::registry &registry);

  void draw_gui(entt::registry &registry, entt::entity entity) override;

protected:
  framebuffer fb;
  texture pos_tex, normal_tex, depth_tex;

  DECLARE_SYSTEM(defered_forward_mixed)
};

}; // namespace toolkit::opengl
