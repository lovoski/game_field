#include "scripts/onnxruntime/camdmpp.hpp"

namespace toolkit {

void camdmpp::draw_gui(iapp *app) {
  if (ImGui::Button("Select .onnx model", {-1, 30})) {
    // std::string filepath;
    // if (open_file_dialog("Select .onnx model", {"*.onnx"}, filepath))
    //   diffusion.setup(filepath, "");
  }
}

void camdmpp::draw_to_scene(iapp *app) {
  opengl::script_draw_to_scene_proxy(app, [&](opengl::editor *editor,
                                              transform &cam_trans,
                                              opengl::camera &cam_comp) {});
}

void camdmpp::fixedupdate(iapp *app, float dt) {
}

}; // namespace toolkit