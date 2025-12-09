#include "toolkit/sdl2d/engine.hpp"

namespace toolkit::sdl2d {

void engine2d::draw_editor_gui() {
  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("File")) {
      // ---------------------- Scene save/load menu ----------------------
      ImGui::MenuItem("Scene", nullptr, false, false);
      if (ImGui::MenuItem("Reset Scene")) {
        reset();
        add_default_objects();
        SDL_Log("Reset scene");
      }
      if (ImGui::MenuItem("Save  Scene")) {
        std::string filepath;
        if (save_file_dialog("Serialize scene file", {"*.scene"}, filepath)) {
          auto data = serialize();
          std::ofstream output(filepath);
          if (output.is_open()) {
            output << data.dump() << std::endl;
            output.close();
            SDL_Log("Save scene to %s", filepath.c_str());
          } else {
            SDL_Log("Failed to save scene to %s", filepath.c_str());
          }
        }
      }
      if (ImGui::MenuItem("Load  Scene")) {
        std::string filepath;
        if (open_file_dialog("Deserialize scene file", {"*.scene"}, filepath)) {
          std::ifstream input(filepath);
          if (input.is_open()) {
            auto data = nlohmann::json::parse(
                std::string((std::istreambuf_iterator<char>(input)),
                            std::istreambuf_iterator<char>()));
            deserialize(data);
            SDL_Log("Load scene from %s", filepath.c_str());
          } else {
            SDL_Log("Failed to load scene from %s", filepath.c_str());
          }
        }
      }
      ImGui::Separator();

      ImGui::MenuItem("Bundle", nullptr, false, false);
      if (ImGui::MenuItem("Import Bundle")) {
        std::string filepath;
        if (open_file_dialog("Import bundle to current scene", {"*.bundle"},
                             filepath)) {
          std::ifstream input(filepath);
          if (input.is_open()) {
            auto data = nlohmann::json::parse(
                std::string((std::istreambuf_iterator<char>(input)),
                            std::istreambuf_iterator<char>()));
            load_bundle(data);
            SDL_Log("Import bundle to current scene from %s", filepath.c_str());
          } else {
            SDL_Log("Failed to import bundle from %s", filepath.c_str());
          }
          input.close();
        }
      }
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Settings")) {
      // system configuration
      ImGui::MenuItem("Configure Systems", nullptr, false, false);
      for (auto &sys : systems) {
        if (ImGui::BeginMenu(sys->get_name().c_str())) {
          ImGui::Checkbox("Active", &(sys->active));
          ImGui::Separator();
          sys->draw_menu_gui();
          ImGui::EndMenu();
        }
      }

      ImGui::Separator();
      ImGui::MenuItem("Editor Settings", nullptr, false, false);
      if (ImGui::Checkbox("VSync On", &engine_vsync_on)) {
        SDL_RenderSetVSync(renderer, engine_vsync_on ? 1 : 0);
      }
      if (ImGui::Button("Reset Camera", {-1, 30})) {
        camera_zoom = 20.0f;
        camera_rotation = 0.0f;
        camera_position = math::vector2::Zero();
      }
      ImGui::EndMenu();
    }

    static stopwatch _timer;
    static int _frameCount = 0, _displayFPS = 0;
    static float _frameCountTimer = 0.0f, _displayFT = 0.0;

    auto deltaTime = _timer.elapse_s();
    _frameCount += 1;
    _frameCountTimer += deltaTime;
    if (_frameCountTimer >= 1.0f) {
      _displayFPS = _frameCount;
      _displayFT = 1000.0f / _frameCount;
      _frameCount = 0;
      _frameCountTimer = 0.0f;
    }
    _timer.reset();
    ImGui::SameLine(ImGui::GetWindowWidth() -
                    ImGui::CalcTextSize("Frame Time: 0.000 ms, FPS: 000000").x -
                    ImGui::GetStyle().ItemSpacing.x);
    ImGui::PushStyleColor(ImGuiCol_Text, {1.0, 1.0, 0.0, 1.0});
    ImGui::Text("Frame Time: %.3f ms, FPS: %d", _displayFT, _displayFPS);
    ImGui::PopStyleColor();

    ImGui::EndMainMenuBar();
  }
}

}; // namespace toolkit::sdl2d