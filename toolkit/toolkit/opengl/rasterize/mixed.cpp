#include "toolkit/opengl/rasterize/mixed.hpp"
#include "toolkit/scriptable.hpp"

namespace toolkit::opengl {

const std::string gbuffer_geometry_pass_vs = R"(
#version 430 core
layout (location = 0) in vec4 aPos;
layout (location = 1) in vec4 aNormal;

// struct _packed_vertex {
//   vec4 position;
//   vec4 normal;
// };
// layout(std430, binding = 0) buffer WorldSpaceVertexBuffer {
//   _packed_vertex gWorldVertex[];
// };
// uniform int gVertexOffset;

uniform mat4 gModel;
uniform mat4 gVP;

out vec3 worldPos;
out vec3 worldNormal;

void main() {
  // _packed_vertex worldVertex = gWorldVertex[gl_VertexID + gVertexOffset];

  // worldNormal = normalize(worldVertex.normal.xyz);
  // worldPos = worldVertex.position.xyz;;

  worldNormal = normalize(mat3(gModel)*aNormal.xyz);
  worldPos = (gModel * aPos).xyz;

  gl_Position = gVP * vec4(worldPos, 1.0);
}
)";
const std::string gbuffer_geometry_pass_fs = R"(
#version 430 core

in vec3 worldPos;
in vec3 worldNormal;

layout (location = 0) out vec4 gPosition; // G-buffer position output
layout (location = 1) out vec4 gNormal;   // G-buffer normal output

void main() {
  gPosition = vec4(worldPos, 1.0);
  // map normal to range [0, 1]
  gNormal = vec4(normalize(worldNormal) * 0.5 + 0.5, 1.0);
}
)";

void defered_forward_mixed::draw_menu_gui() {
  ImGui::Checkbox("Show Grid", &should_draw_grid);
  ImGui::InputInt("Grid Size", &grid_size);
  ImGui::InputInt("Grid Spacing", &grid_spacing);
}

void defered_forward_mixed::draw_gui(entt::registry &registry,
                                     entt::entity entity) {
  if (auto ptr = registry.try_get<camera>(entity)) {
    if (ImGui::CollapsingHeader("Camera")) {
    }
  }
  if (auto ptr = registry.try_get<dir_light>(entity)) {
    if (ImGui::CollapsingHeader("Dir Light")) {
    }
  }
  if (auto ptr = registry.try_get<point_light>(entity)) {
    if (ImGui::CollapsingHeader("Point Light")) {
    }
  }
}

void defered_forward_mixed::init0(entt::registry &registry) {
  auto &instance = context::get_instance();
  pos_tex.create(GL_TEXTURE_2D);
  normal_tex.create(GL_TEXTURE_2D);
  depth_tex.create(GL_TEXTURE_2D);
  color_tex.create(GL_TEXTURE_2D);

  gbuffer_geometry_pass.compile_shader_from_source(gbuffer_geometry_pass_vs,
                                                   gbuffer_geometry_pass_fs);

  gbuffer.create();
  cbuffer.create();
  resize(instance.scene_width, instance.scene_height);
}

void defered_forward_mixed::resize(int width, int height) {
  gbuffer.bind();
  gbuffer.begin_draw_buffers();
  pos_tex.set_data(width, height, GL_RGBA32F, GL_RGBA, GL_FLOAT);
  pos_tex.set_parameters({{GL_TEXTURE_MIN_FILTER, GL_NEAREST},
                          {GL_TEXTURE_MAG_FILTER, GL_NEAREST},
                          {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
                          {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});
  normal_tex.set_data(width, height, GL_RGBA32F, GL_RGBA, GL_FLOAT);
  normal_tex.set_parameters({{GL_TEXTURE_MIN_FILTER, GL_NEAREST},
                             {GL_TEXTURE_MAG_FILTER, GL_NEAREST},
                             {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
                             {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});
  depth_tex.set_data(width, height, GL_DEPTH_COMPONENT24, GL_DEPTH_COMPONENT,
                     GL_FLOAT);
  depth_tex.set_parameters({{GL_TEXTURE_MIN_FILTER, GL_NEAREST},
                            {GL_TEXTURE_MAG_FILTER, GL_NEAREST},
                            {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
                            {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});
  gbuffer.attach_color_buffer(pos_tex, GL_COLOR_ATTACHMENT0);
  gbuffer.attach_color_buffer(normal_tex, GL_COLOR_ATTACHMENT1);
  gbuffer.end_draw_buffers();
  gbuffer.attach_depth_buffer(depth_tex);
  if (!gbuffer.check_status())
    spdlog::error("gbuffer not complete!");
  gbuffer.unbind();

  cbuffer.bind();
  cbuffer.begin_draw_buffers();
  color_tex.set_data(width, height, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE);
  color_tex.set_parameters({{GL_TEXTURE_MIN_FILTER, GL_LINEAR},
                            {GL_TEXTURE_MAG_FILTER, GL_LINEAR},
                            {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
                            {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});
  cbuffer.attach_color_buffer(color_tex, GL_COLOR_ATTACHMENT0);
  cbuffer.end_draw_buffers();
  if (!cbuffer.check_status())
    spdlog::error("cbuffer not complete!");
  cbuffer.unbind();
}

void defered_forward_mixed::preupdate(entt::registry &registry, float dt) {
  auto &instance = context::get_instance();
  // update vp matrices for cameras
  registry.view<camera>().each([&](entt::entity entity, camera &camera) {
    compute_vp_matrix(registry, entity, instance.scene_width,
                      instance.scene_height);
  });
  // perform cpu frustom culling
}

void defered_forward_mixed::render(entt::registry &registry) {
  if (auto cam_ptr = registry.try_get<camera>(g_instance.active_camera)) {
    auto &cam_trans = registry.get<transform>(g_instance.active_camera);
    auto &cam_comp = *cam_ptr;

    gbuffer.bind();
    gbuffer.set_viewport(0, 0, g_instance.scene_width, g_instance.scene_height);

    glEnable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0, 0, 0, 1);

    // render gbuffer
    gbuffer_geometry_pass.use();
    registry.view<mesh_data, transform>().each(
        [&](entt::entity entity, mesh_data &mesh, transform &trans) {
          gbuffer_geometry_pass.set_mat4("gModel", trans.matrix());
          gbuffer_geometry_pass.set_mat4("gVP", cam_comp.vp);
          draw_mesh_data(mesh);
        });

    gbuffer.unbind();

    // render color buffer
    cbuffer.bind();
    cbuffer.set_viewport(0, 0, g_instance.scene_width, g_instance.scene_height);

    glClear(GL_COLOR_BUFFER_BIT);
    glClearColor(0, 0, 0, 1);

    if (should_draw_grid)
      draw_grid(grid_size, grid_spacing, cam_comp.vp);

    // debug rendering
    if (auto app_ptr = registry.ctx().get<iapp*>()) {
      auto script_sys = app_ptr->get_sys<script_system>();
      script_sys->draw_to_scene(app_ptr);
    }

    cbuffer.unbind();
  }
}

}; // namespace toolkit::opengl