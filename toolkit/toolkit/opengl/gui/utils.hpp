#pragma once

#include "toolkit/opengl/base.hpp"

namespace toolkit::gui {

bool color_edit_3(std::string label, math::vector3 &color);
bool color_edit_4(std::string label, math::vector4 &color);
/**
 * `label`: name for this component
 * `index`: range [-1, names.size()-1], -1 for "none:-1"
 * `names`: names for each of the entry
 * `handleCurrent`: take `index` as its parameter, range [-1, names.size()-1]
 */
void combo_default(
    std::string label, int &index, std::vector<std::string> names,
    std::function<void(int)> handleCurrent = [](int) {},
    std::string default_name = "None:-1");
/**
 * `label`: name for this component
 * `index`: range [0, names.size()-1]
 * `names`: names for each of the entry
 * `handleCurrent`: take `index` as its parameter, range [0, names.size()-1]
 */
void combo(
    std::string label, int &index, std::vector<std::string> names,
    std::function<void(int)> handleCurrent = [](int) {});

}; // namespace toolkit::gui
