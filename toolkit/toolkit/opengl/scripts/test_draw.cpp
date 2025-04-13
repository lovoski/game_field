#include "toolkit/opengl/scripts/test_draw.hpp"

namespace toolkit::opengl {

void test_draw::draw_gui(iapp *app) {
  ImGui::SeparatorText("Text");
  ImGui::SliderFloat("Text Scale", &text_scale, 0.0f, 10.0f);
  ImGui::SliderFloat("Text Width", &text_width, 0.0f, 10.0f);
  ImGui::SliderFloat("Text Height", &text_height, 0.0f, 10.0f);
  ImGui::SliderFloat("Text Spacing", &text_spacing, 0.0f, 10.0f);
  ImGui::SliderFloat("Text Line Height", &text_line_height, 0.0f, 10.0f);
}

void test_draw::draw_to_scene(iapp *app) {
  if (auto editor_ptr = dynamic_cast<editor *>(app)) {
    auto &trans = editor_ptr->registry.get<transform>(entity);
    if (auto cam_trans =
            editor_ptr->registry.try_get<transform>(g_instance.active_camera)) {
      auto &cam_comp =
          editor_ptr->registry.get<camera>(g_instance.active_camera);
      draw_text3d("This is the test for debug text draw\nThere can be multiple lines.", trans.position(), trans.rotation(),
                  cam_comp.vp, 0.0f, text_scale, text_width, text_height,
                  text_spacing, text_line_height);
    }
  }
}

}; // namespace toolkit::opengl