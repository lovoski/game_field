#include "scripts/onnxruntime/camdmpp.hpp"

namespace toolkit {

void camdmpp::update(iapp *app, float dt) {
  float residual = cur_time - cur_exec_fixed * fixed_interval;
  while (residual > fixed_interval) {
    residual -= fixed_interval;
    fixedupdate(app, fixed_interval);
    cur_exec_fixed += 1;
  }
  cur_time += dt;
}

void camdmpp::draw_gui(iapp *app) {}

void camdmpp::draw_to_scene(iapp *app) {
  opengl::script_draw_to_scene_proxy(app, [&](opengl::editor *editor,
                                              transform &cam_trans,
                                              opengl::camera &cam_comp) {});
}

void camdmpp::fixedupdate(iapp *app, float dt) {
  spdlog::info("AAA");
}

}; // namespace toolkit