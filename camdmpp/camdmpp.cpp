#include "scripts/onnxruntime/camdmpp.hpp"

namespace toolkit {

void camdmpp::draw_gui(entt::registry &registry, entt::entity entity) {
  if (ImGui::Button("Select .onnx model", {-1, 30})) {
    // std::string filepath;
    // if (open_file_dialog("Select .onnx model", {"*.onnx"}, filepath))
    //   diffusion.setup(filepath, "");
  }
}

void camdmpp::draw_to_scene(entt::registry &registry, transform &cam_trans, camera &cam_comp) {}

void camdmpp::fixedupdate(entt::registry &registry, float dt) {}

}; // namespace toolkit