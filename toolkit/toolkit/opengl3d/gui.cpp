#include "toolkit/opengl3d/gui.hpp"

namespace toolkit::opengl3d {

bool color_edit_3(std::string label, math::vector3 &color) {
  return ImGui::ColorEdit3(label.c_str(), color.data());
}
bool color_edit_4(std::string label, math::vector4 &color) {
  return ImGui::ColorEdit4(label.c_str(), color.data());
}

void combo_default(std::string label, int &index,
                   std::vector<std::string> names,
                   std::function<void(int)> handleCurrent,
                   std::string default_name) {
  std::vector<std::string> augNames{default_name};
  augNames.insert(augNames.end(), names.begin(), names.end());
  if (index < 0 || index >= names.size())
    index = -1;
  if (ImGui::BeginCombo(label.c_str(), augNames[index + 1].c_str())) {
    for (int comboIndex = 0; comboIndex < augNames.size(); ++comboIndex) {
      bool isSelected = (index + 1 == comboIndex);
      if (ImGui::Selectable(augNames[comboIndex].c_str(), isSelected)) {
        index = comboIndex - 1;
        handleCurrent(index);
      }
      if (isSelected)
        ImGui::SetItemDefaultFocus();
    }
    ImGui::EndCombo();
  }
}

void combo(std::string label, int &index, std::vector<std::string> names,
           std::function<void(int)> handleCurrent) {
  if (index < 0 || index >= names.size())
    index = 0;
  if (ImGui::BeginCombo(label.c_str(), names[index].c_str())) {
    for (int comboIndex = 0; comboIndex < names.size(); ++comboIndex) {
      bool isSelected = (index == comboIndex);
      if (ImGui::Selectable(names[comboIndex].c_str(), isSelected)) {
        index = comboIndex;
        handleCurrent(index);
      }
      if (isSelected)
        ImGui::SetItemDefaultFocus();
    }
    ImGui::EndCombo();
  }
}

void texture_select(texture &tex, std::filesystem::path &filepath,
                    texture &checkerboard_tex) {
  auto size = ImGui::GetContentRegionAvail();
  auto select_image = [&]() {
    std::string tmp_filepath;
    if (open_file_dialog("Select texture image",
                         {"*.png", "*.jpg", "*.jpeg", "*.bmp"}, tmp_filepath)) {
      assets::image img;
      filepath = relpath(tmp_filepath);
      img.load(filepath.string(), true);
      if (!tex.inited())
        tex.create();
      tex.set_data_from_image(img);
      tex.set_parameters({{GL_TEXTURE_MIN_FILTER, GL_NEAREST},
                          {GL_TEXTURE_MAG_FILTER, GL_NEAREST},
                          {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
                          {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});
    }
  };
  if (!tex.inited()) {
    if (ImGui::ImageButton(
            (void *)static_cast<std::uintptr_t>(checkerboard_tex.get_handle()),
            {size.x * 0.5f, size.x * 0.5f}, ImVec2(0, 1), ImVec2(1, 0)))
      select_image();
  } else {
    if (ImGui::ImageButton(
            (void *)static_cast<std::uintptr_t>(tex.get_handle()),
            {size.x * 0.5f, size.x * 0.5f}, ImVec2(0, 1), ImVec2(1, 0)))
      select_image();
  }
}

}; // namespace toolkit::opengl3d
