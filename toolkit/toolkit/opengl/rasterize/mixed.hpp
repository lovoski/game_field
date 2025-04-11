#pragma once

#include "toolkit/opengl/base.hpp"
#include "toolkit/system.hpp"

#include "toolkit/opengl/components/camera.hpp"
#include "toolkit/opengl/components/lights.hpp"
#include "toolkit/opengl/components/mesh.hpp"

namespace toolkit::opengl {

class default_mixed : public isystem {
public:
  void init0(entt::registry &registry) override {
    auto &instance = context::get_instance();
    fb.create();
    resize(instance.scene_width, instance.scene_height);
  }

  void resize(int width, int height) {
    fb.bind();

    pos_tex.create(GL_TEXTURE_2D);
    pos_tex.set_data(width, height, GL_RGB32F, GL_RGB);
    pos_tex.set_parameters({{GL_TEXTURE_MIN_FILTER, GL_NEAREST},
                            {GL_TEXTURE_MAG_FILTER, GL_NEAREST},
                            {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
                            {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});
    normal_tex.create(GL_TEXTURE_2D);
    normal_tex.set_data(width, height, GL_RGB32F, GL_RGB);
    normal_tex.set_parameters({{GL_TEXTURE_MIN_FILTER, GL_NEAREST},
                               {GL_TEXTURE_MAG_FILTER, GL_NEAREST},
                               {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
                               {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});
    depth_tex.create(GL_TEXTURE_2D);
    depth_tex.set_data(width, height, GL_DEPTH_COMPONENT24, GL_DEPTH_COMPONENT,
                       GL_FLOAT);
    depth_tex.set_parameters({{GL_TEXTURE_MIN_FILTER, GL_NEAREST},
                              {GL_TEXTURE_MAG_FILTER, GL_NEAREST},
                              {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
                              {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});

    fb.attach_color_buffer(pos_tex, GL_COLOR_ATTACHMENT0);
    fb.attach_color_buffer(normal_tex, GL_COLOR_ATTACHMENT1);
    fb.attach_depth_buffer(depth_tex);

    fb.unbind();
  }

  void preupdate(entt::registry &registry, float dt) override {
    // update vp matrices for cameras
    registry.view<camera>().each([&](entt::entity entity, camera &camera) {});
    // perform cpu frustom culling
  }

  void render(entt::registry &registry) {
    fb.bind();

    // registry.view<MeshData, MeshProxy>().each(
    //     [&](entt::entity entity, MeshData &meshData, MeshProxy &meshProxy)
    //     {});

    fb.unbind();
  }

protected:
  framebuffer fb;
  texture pos_tex, normal_tex, depth_tex;
};

}; // namespace toolkit::opengl
