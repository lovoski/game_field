#include "toolkit/opengl/components/material.hpp"
#include "toolkit/opengl/components/mesh.hpp"
#include "toolkit/opengl/gui/utils.hpp"

namespace toolkit::opengl {

bool has_any_materials(entt::registry &registry, entt::entity entity) {
  bool result = false;
  for (auto &f : material::__material_exists__)
    result |= f.second(registry, entity);
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

std::vector<std::string> get_segs(std::string str, char sep) {
  std::vector<std::string> segments;
  std::string current;
  for (char ch : str) {
    if (ch == sep) {
      segments.push_back(current);
      current.clear();
    } else {
      current += ch;
    }
  }
  segments.push_back(current);
  return segments;
}

std::vector<material_field>
parse_glsl_uniforms(std::vector<std::string> sources) {
  std::vector<material_field> results;
  std::map<std::string, std::string> name_to_type;
  std::map<std::string, std::string> meta_infos;
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
        if (segments.size() > 0 && segments[0] == "//@meta") {
          // meta informations
          std::string info = segments[1];
          int lb = -1, rb = -1;
          for (int i = 0; i < info.size(); i++) {
            if (info[i] == '(')
              lb = i;
            if (info[i] == ')')
              rb = i;
          }
          if (lb == -1 || rb == -1) {
            spdlog::error("Failed to parse meta info {0}", info);
          } else {
            meta_infos[info.substr(0, lb)] = info.substr(lb + 1, rb - lb - 1);
          }
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
      if (meta_infos.find(mf.name) != meta_infos.end()) {
        auto segs = get_segs(meta_infos[mf.name], ',');
        if (segs.size() < 2)
          spdlog::error("Failed to setup meta info for variable {0}", mf.name);
        else {
          mf.value = math::vector2(std::atof(segs[0].c_str()),
                                   std::atof(segs[1].c_str()));
        }
      }
    } else if (p.second == "vec3") {
      mf.value = (math::vector3)math::vector3::Ones();
      if (meta_infos.find(mf.name) != meta_infos.end()) {
        auto segs = get_segs(meta_infos[mf.name], ',');
        if (segs.size() < 3)
          spdlog::error("Failed to setup meta info for variable {0}", mf.name);
        else {
          mf.value = math::vector3(std::atof(segs[0].c_str()),
                                   std::atof(segs[1].c_str()),
                                   std::atof(segs[2].c_str()));
        }
      }
    } else if (p.second == "vec4") {
      mf.value = (math::vector4)math::vector4::Ones();
      if (meta_infos.find(mf.name) != meta_infos.end()) {
        auto segs = get_segs(meta_infos[mf.name], ',');
        if (segs.size() < 4)
          spdlog::error("Failed to setup meta info for variable {0}", mf.name);
        else {
          mf.value = math::vector4(
              std::atof(segs[0].c_str()), std::atof(segs[1].c_str()),
              std::atof(segs[2].c_str()), std::atof(segs[3].c_str()));
        }
      }
    } else if (p.second == "mat2") {
      mf.value = (math::matrix2)math::matrix2::Identity();
    } else if (p.second == "mat3") {
      mf.value = (math::matrix3)math::matrix3::Identity();
    } else if (p.second == "mat4") {
      mf.value = (math::matrix4)math::matrix4::Identity();
    } else if (p.second == "float") {
      mf.value = 0.0f;
      if (meta_infos.find(mf.name) != meta_infos.end()) {
        mf.value = std::atof(meta_infos[mf.name].c_str());
      }
    } else if (p.second == "bool") {
      mf.value = false;
      if (meta_infos.find(mf.name) != meta_infos.end()) {
        auto meta_info = lower_case(meta_infos[mf.name]);
        if (meta_info == "true")
          mf.value = true;
        else if (meta_info == "false")
          mf.value = false;
        else
          spdlog::error("Failed to recognize boolean meta info for field {0}",
                        mf.name);
      }
    } else if (p.second == "int") {
      mf.value = 0;
      if (meta_infos.find(mf.name) != meta_infos.end()) {
        mf.value = std::atoi(meta_infos[mf.name].c_str());
      }
    }
    results.push_back(mf);
  }
  return results;
}

void material::init1() {
  auto mf = parse_glsl_uniforms({get_vertex_shader_source(),
                                 get_fragment_shader_source(),
                                 get_geometry_shader_source()});
  for (auto p : material_fields) {
    for (int i = 0; i < mf.size(); i++)
      if (mf[i].name == p.name) {
        mf[i] = p;
        break;
      }
  }
  material_fields = mf;
}

}; // namespace toolkit::opengl