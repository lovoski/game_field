#include "toolkit/opengl/gui/utils.hpp"

namespace toolkit::gui {

bool color_edit_3(std::string label, math::vector3 &color) {
  return ImGui::ColorEdit3(label.c_str(), color.data());
}
bool color_edit_4(std::string label, math::vector4 &color) {
  return ImGui::ColorEdit4(label.c_str(), color.data());
}

void combo_default(std::string label, int &index,
                   std::vector<std::string> names,
                   std::function<void(int)> handleCurrent) {
  std::vector<std::string> augNames{"None:-1"};
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

}; // namespace toolkit::gui
