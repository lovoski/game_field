#pragma once

#include "toolkit/opengl/base.hpp"
#include "toolkit/reflect.hpp"
#include "toolkit/system.hpp"

namespace toolkit::opengl {

struct material_field {
  std::string name, type;
  nlohmann::json value;
};
REFLECT(material_field, name, type, value)

std::vector<material_field>
parse_glsl_uniforms(std::vector<std::string> sources);

/**
 * Each material type has a shader instance attached to it. The instance of
 * material merely holds the data needed for the shader.
 */
struct material : public icomponent {
  material() {}
  ~material() {}

  void draw_gui(iapp *app) override;
  void bind_uniforms(shader &mat_shader);

  std::string vertex_shader_source = "";
  std::string fragment_shader_source = "";
  std::string geometry_shader_source = "none";

  static inline std::map<std::string, std::function<void(entt::registry &)>>
      __material_constructors__;
  static inline std::map<std::string, bool> __shader_initialized__;
  static inline std::map<std::string, shader> __material_shaders__;
  static inline std::map<
      std::string,
      std::function<void(entt::registry &,
                         std::function<void(entt::entity, material *)>)>>
      __material_view__;
  static inline std::map<std::string,
                         std::function<void(entt::registry &, entt::entity)>>
      __material_draw_gui__;
  static inline std::vector<std::function<bool(entt::registry &, entt::entity)>>
      __material_exists__;

  std::vector<material_field> material_fields;
};

bool has_any_materials(entt::registry &registry, entt::entity entity);

#define DECLARE_MATERIAL(class_name)                                           \
  DECLARE_COMPONENT(class_name, material, material_fields)                     \
  inline void __on_construct_##class_name(entt::registry &registry,            \
                                          entt::entity entity) {               \
    auto &mat_instance = registry.get<class_name>(entity);                     \
    if (!toolkit::opengl::material::__shader_initialized__[#class_name]) {     \
      toolkit::opengl::shader material_shader;                                 \
      material_shader.compile_shader_from_source(                              \
          mat_instance.vertex_shader_source,                                   \
          mat_instance.fragment_shader_source,                                 \
          mat_instance.geometry_shader_source);                                \
      toolkit::opengl::material::__material_shaders__[#class_name] =           \
          material_shader;                                                     \
      toolkit::opengl::material::__shader_initialized__[#class_name] = true;   \
    }                                                                          \
    mat_instance.material_fields =                                             \
        parse_glsl_uniforms({mat_instance.vertex_shader_source,                \
                             mat_instance.fragment_shader_source,              \
                             mat_instance.geometry_shader_source});            \
  }                                                                            \
  inline void __material_view_for_each_##class_name(                           \
      entt::registry &registry,                                                \
      std::function<void(entt::entity, material *)> &&func) {                  \
    registry.view<entt::entity, class_name>().each(                            \
        [&](entt::entity entity, class_name &instance) {                       \
          func(entity, &instance);                                             \
        });                                                                    \
  }                                                                            \
  inline void __material_draw_gui_##class_name(entt::registry &registry,       \
                                               entt::entity entity) {          \
    if (auto mat = registry.try_get<class_name>(entity)) {                     \
      if (ImGui::TreeNode(#class_name)) {                                      \
        mat->draw_gui(nullptr);                                                \
        ImGui::TreePop();                                                      \
      }                                                                        \
    }                                                                          \
  }                                                                            \
  inline bool __material_exists_##class_name(entt::registry &registry,         \
                                             entt::entity entity) {            \
    return registry.try_get<class_name>(entity) != nullptr;                    \
  }                                                                            \
  struct __register_material_##class_name {                                    \
    __register_material_##class_name() {                                       \
      toolkit::opengl::material::__shader_initialized__[#class_name] = false;  \
      toolkit::opengl::material::__material_constructors__[#class_name] =      \
          [](entt::registry &registry) {                                       \
            registry.on_construct<class_name>()                                \
                .connect<&__on_construct_##class_name>();                      \
          };                                                                   \
      toolkit::opengl::material::__material_view__[#class_name] =              \
          __material_view_for_each_##class_name;                               \
      toolkit::opengl::material::__material_draw_gui__[#class_name] =          \
          __material_draw_gui_##class_name;                                    \
      toolkit::opengl::material::__material_exists__.push_back(                \
          __material_exists_##class_name);                                     \
    }                                                                          \
  };                                                                           \
  static __register_material_##class_name                                      \
      __register_material_instance_##class_name =                              \
          __register_material_##class_name();

}; // namespace toolkit::opengl
