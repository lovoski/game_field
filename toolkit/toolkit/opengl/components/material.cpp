#include "toolkit/opengl/components/material.hpp"
#include "toolkit/opengl/components/mesh.hpp"
#include "toolkit/opengl/gui/utils.hpp"

namespace toolkit::opengl {

bool has_any_materials(entt::registry &registry, entt::entity entity) {
  bool result = false;
  for (auto &f : material::__material_exists__)
    result = result || f(registry, entity);
  return result;
}

void material::draw_gui(iapp *app) {
  for (auto &field : material_fields) {
    if (field.type == "float") {
      float value = field.value.get<float>();
      if (ImGui::DragFloat(field.name.c_str(), &value, 0.1f, -1e5, 1e5)) {
        field.value = value;
      }
    } else if (field.type == "int") {
      int value = field.value.get<int>();
      if (ImGui::DragInt(field.name.c_str(), &value, 0.1f, -1e5, 1e5)) {
        field.value = value;
      }
    } else if (field.type == "bool") {
      bool value = field.value.get<bool>();
      if (ImGui::Checkbox(field.name.c_str(), &value)) {
        field.value = value;
      }
    } else if (field.type == "vec2") {
      math::vector2 value = field.value.get<math::vector2>();
      if (ImGui::DragFloat2(field.name.c_str(), value.data(), 0.1f, -1e5,
                            1e5)) {
        field.value = value;
      }
    } else if (field.type == "vec3") {
      math::vector3 value = field.value.get<math::vector3>();
      if (gui::color_edit_3(field.name.c_str(), value)) {
        field.value = value;
      }
    } else if (field.type == "vec4") {
      math::vector4 value = field.value.get<math::vector4>();
      if (gui::color_edit_4(field.name.c_str(), value)) {
        field.value = value;
      }
    }
  }
}
void material::bind_uniforms(shader &mat_shader) {
  for (auto &field : material_fields) {
    if (field.type == "float") {
      mat_shader.set_float(field.name, field.value.get<float>());
    } else if (field.type == "int") {
      mat_shader.set_int(field.name, field.value.get<int>());
    } else if (field.type == "bool") {
      mat_shader.set_bool(field.name, field.value.get<bool>());
    } else if (field.type == "vec2") {
      mat_shader.set_vec2(field.name, field.value.get<math::vector2>());
    } else if (field.type == "vec3") {
      mat_shader.set_vec3(field.name, field.value.get<math::vector3>());
    } else if (field.type == "vec4") {
      mat_shader.set_vec4(field.name, field.value.get<math::vector4>());
    }
  }
}

std::vector<material_field>
parse_glsl_uniforms(std::vector<std::string> sources) {
  std::vector<material_field> results;
  std::map<std::string, std::string> name_to_type;
  for (int i = 0; i < sources.size(); i++) {
    std::string source = replace(replace(sources[i], "\r", ""), ";", "");
    int previous = 0;
    for (int j = 0; j < source.size(); j++) {
      if (source[j] == '\n' && previous < source.size()) {
        std::vector<std::string> segments;
        std::string line = "";
        for (int k = previous; k < j; k++) {
          line.push_back(source[k]);
        }
        line.push_back(' ');
        int seg_start = 0;
        for (int k = 0; k < line.size(); k++) {
          if (line[k] == ' ') {
            std::string segment = "";
            for (int kk = seg_start; kk < k; kk++)
              segment.push_back(line[kk]);
            segments.push_back(segment);
            seg_start = k + 1;
          }
        }
        if (segments.size() > 0 && segments[0] == "uniform") {
          name_to_type[segments[2]] = segments[1];
        }
        previous = j + 1;
      }
    }
  }
  for (auto &p : name_to_type) {
    if (p.first[0] == 'g')
      continue;
    material_field mf;
    mf.name = p.first;
    mf.type = p.second;
    if (p.second == "vec2") {
      mf.value = (math::vector2)math::vector2::Zero();
    } else if (p.second == "vec3") {
      mf.value = (math::vector3)math::vector3::Ones();
    } else if (p.second == "vec4") {
      mf.value = (math::vector4)math::vector4::Ones();
    } else if (p.second == "mat2") {
      mf.value = (math::matrix2)math::matrix2::Identity();
    } else if (p.second == "mat3") {
      mf.value = (math::matrix3)math::matrix3::Identity();
    } else if (p.second == "mat4") {
      mf.value = (math::matrix4)math::matrix4::Identity();
    } else if (p.second == "float") {
      mf.value = 0.0f;
    } else if (p.second == "bool") {
      mf.value = false;
    } else if (p.second == "int") {
      mf.value = 0;
    }
    results.push_back(mf);
  }
  return results;
}

}; // namespace toolkit::opengl