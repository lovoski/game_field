#include "toolkit/opengl/rasterize/mixed.hpp"

namespace toolkit::opengl {

void defered_forward_mixed::draw_gui(entt::registry &registry,
                                     entt::entity entity) {
  if (auto ptr = registry.try_get<camera>(entity)) {
    if (ImGui::CollapsingHeader("Camera")) {
    }
  }
  if (auto ptr = registry.try_get<dir_light>(entity)) {
    if (ImGui::CollapsingHeader("Dir Light")) {
    }
  }
  if (auto ptr = registry.try_get<point_light>(entity)) {
    if (ImGui::CollapsingHeader("Point Light")) {
    }
  }
}

void defered_forward_mixed::init0(entt::registry &registry) {
  auto &instance = context::get_instance();
  pos_tex.create(GL_TEXTURE_2D);
  normal_tex.create(GL_TEXTURE_2D);
  depth_tex.create(GL_TEXTURE_2D);

  fb.create();
  resize(instance.scene_width, instance.scene_height);
}

void defered_forward_mixed::resize(int width, int height) {
  fb.bind();

  pos_tex.set_data(width, height, GL_RGB32F, GL_RGB);
  pos_tex.set_parameters({{GL_TEXTURE_MIN_FILTER, GL_NEAREST},
                          {GL_TEXTURE_MAG_FILTER, GL_NEAREST},
                          {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
                          {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});
  normal_tex.set_data(width, height, GL_RGB32F, GL_RGB);
  normal_tex.set_parameters({{GL_TEXTURE_MIN_FILTER, GL_NEAREST},
                             {GL_TEXTURE_MAG_FILTER, GL_NEAREST},
                             {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
                             {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});
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

void defered_forward_mixed::preupdate(entt::registry &registry, float dt) {
  // update vp matrices for cameras
  registry.view<camera>().each([&](entt::entity entity, camera &camera) {});
  // perform cpu frustom culling
}

void defered_forward_mixed::render(entt::registry &registry) {
  auto &instance = context::get_instance();
  fb.bind();
  fb.set_viewport(0, 0, instance.scene_width, instance.scene_height);

  fb.unbind();
}

}; // namespace toolkit::opengl