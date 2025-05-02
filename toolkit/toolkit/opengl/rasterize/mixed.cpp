#include "toolkit/opengl/rasterize/mixed.hpp"
#include "toolkit/scriptable.hpp"

namespace toolkit::opengl {

const std::string gbuffer_geometry_pass_vs = R"(
#version 430 core
layout (location = 0) in vec4 aPos;
layout (location = 1) in vec4 aNormal;

uniform mat4 gModel;
uniform mat4 gVP;

out vec3 worldPos;
out vec3 worldNormal;

void main() {
  worldNormal = normalize(mat3(gModel)*aNormal.xyz);
  worldPos = (gModel * aPos).xyz;

  gl_Position = gVP * vec4(worldPos, 1.0);
}
)";
const std::string gbuffer_geometry_pass_fs = R"(
#version 430 core

layout (location = 0) out vec4 gPosition; // G-buffer position output
layout (location = 1) out vec4 gNormal;   // G-buffer normal output
layout (location = 2) out vec4 gMask;

in vec3 worldPos;
in vec3 worldNormal;

void main() {
  gPosition = vec4(worldPos, 1.0);
  // map normal to range [0, 1]
  gNormal = vec4(normalize(worldNormal) * 0.5 + 0.5, 1.0);
  gMask = vec4(1,0,0,0);
}
)";

const std::string defered_phong_pass_vs = R"(
#version 430 core

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec2 aTexCoord;

out vec2 texCoord;

void main() {
  texCoord = aTexCoord;
  gl_Position = vec4(aPos, 1.0);
}
)";

const std::string defered_phong_pass_fs = R"(
#version 430 core

uniform sampler2D gPosTex;
uniform sampler2D gNormalTex;
uniform sampler2D gGbufferDepthTex;
uniform sampler2D gCbufferDepthTex;
uniform sampler2D gMaskTex;

uniform vec3 gViewDir;

in vec2 texCoord;

out vec4 FragColor;

struct light_data_pacakge {
  ivec4 idata;
  vec4 pos;
  vec4 color;
  vec4 fdata0;
  vec4 fdata1;
};
layout(std430, binding = 0) buffer SceneLights {
  light_data_pacakge gLights[];
};

void main() {
  vec4 mask = texture(gMaskTex, texCoord);
  // only perform color pass rendering when there's actual fragment.
  if (mask.x == 0) {
    discard;
  }

  float g_depth = texture(gGbufferDepthTex, texCoord).r;
  float c_depth = texture(gCbufferDepthTex, texCoord).r;
  // manual depth test
  if (g_depth > c_depth) {
    discard;
  }
  vec3 result = vec3(0.0);
  vec3 worldPos = texture(gPosTex, texCoord).xyz;
  vec3 worldNormal = normalize(texture(gNormalTex, texCoord).xyz);
  for (int i = 0; i < gLights.length(); i++) {
    light_data_pacakge lightData = gLights[i];
    vec3 lightColor = lightData.color.xyz;
    vec3 lightPos = lightData.pos.xyz;
    vec3 lightDir;
    if (lightData.idata[0] == 0) {
      lightDir = -normalize(lightData.fdata0.xyz);
    } else if (lightData.idata[0] == 1) {
      lightDir = normalize(lightPos-worldPos);
    }
    float diff = 0.5 * (dot(worldNormal, lightDir) + 1.0);
    vec3 diffuse = diff * lightColor;
    // vec3 halfwayDir = normalize(lightDir + normalize(gViewDir));
    // float spec = pow(max(dot(fragWorldNormal, halfwayDir), 0.0), 32.0); // Shininess factor
    // vec3 specular = spec * lightColor;
    result += diffuse;
  }

  FragColor = vec4(result, 1.0);
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
  pos_tex.create(GL_TEXTURE_2D);
  normal_tex.create(GL_TEXTURE_2D);
  gbuffer_depth_tex.create(GL_TEXTURE_2D);
  color_tex.create(GL_TEXTURE_2D);
  mask_tex.create(GL_TEXTURE_2D);
  color_buffer_depth_tex.create(GL_TEXTURE_2D);

  gbuffer_geometry_pass.compile_shader_from_source(gbuffer_geometry_pass_vs,
                                                   gbuffer_geometry_pass_fs);
  defered_phong_pass.compile_shader_from_source(defered_phong_pass_vs,
                                                defered_phong_pass_fs);

  gbuffer.create();
  cbuffer.create();
  resize(g_instance.scene_width, g_instance.scene_height);

  scene_vertex_buffer.create();
  scene_index_buffer.create();
  light_data_buffer.create();
}

void defered_forward_mixed::init1(entt::registry &registry) {}

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
  mask_tex.set_data(width, height, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE);
  mask_tex.set_parameters({{GL_TEXTURE_MIN_FILTER, GL_NEAREST},
                           {GL_TEXTURE_MAG_FILTER, GL_NEAREST},
                           {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
                           {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});
  gbuffer_depth_tex.set_data(width, height, GL_DEPTH_COMPONENT24,
                             GL_DEPTH_COMPONENT, GL_FLOAT);
  gbuffer_depth_tex.set_parameters({{GL_TEXTURE_MIN_FILTER, GL_NEAREST},
                                    {GL_TEXTURE_MAG_FILTER, GL_NEAREST},
                                    {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
                                    {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});
  gbuffer.attach_color_buffer(pos_tex, GL_COLOR_ATTACHMENT0);
  gbuffer.attach_color_buffer(normal_tex, GL_COLOR_ATTACHMENT1);
  gbuffer.attach_color_buffer(mask_tex, GL_COLOR_ATTACHMENT2);
  gbuffer.end_draw_buffers();
  gbuffer.attach_depth_buffer(gbuffer_depth_tex);
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
  color_buffer_depth_tex.set_data(width, height, GL_DEPTH_COMPONENT24,
                                  GL_DEPTH_COMPONENT, GL_FLOAT);
  color_buffer_depth_tex.set_parameters(
      {{GL_TEXTURE_MIN_FILTER, GL_NEAREST},
       {GL_TEXTURE_MAG_FILTER, GL_NEAREST},
       {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
       {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});
  cbuffer.attach_color_buffer(color_tex, GL_COLOR_ATTACHMENT0);
  cbuffer.end_draw_buffers();
  cbuffer.attach_depth_buffer(color_buffer_depth_tex);
  if (!cbuffer.check_status())
    spdlog::error("cbuffer not complete!");
  cbuffer.unbind();
}

void defered_forward_mixed::preupdate(entt::registry &registry, float dt) {
  // update vp matrices for cameras
  registry.view<camera>().each([&](entt::entity entity, camera &camera) {
    compute_vp_matrix(registry, entity, g_instance.scene_width,
                      g_instance.scene_height);
  });
}

void defered_forward_mixed::update_scene_buffers(entt::registry &registry) {}

void defered_forward_mixed::update_obj_bbox(entt::registry &registry) {}

void defered_forward_mixed::update_scene_lights(entt::registry &registry) {
  std::vector<light_data_pacakge> lights;
  registry.view<dir_light, transform>().each(
      [&](entt::entity entity, dir_light &light, transform &trans) {
        light_data_pacakge package;
        package.idata[0] = 0;
        package.pos << trans.position(), 1.0f;
        package.color << light.color, 1.0f;
        package.fdata0 << trans.local_forward(), 0.0f;
        lights.emplace_back(package);
      });
  registry.view<point_light, transform>().each(
      [&](entt::entity entity, point_light &light, transform &trans) {
        light_data_pacakge package;
        package.idata[0] = 1;
        package.pos << trans.position(), 1.0f;
        package.color << light.color, 1.0f;
        lights.emplace_back(package);
      });
  light_data_buffer.set_data_ssbo(lights);
}

void defered_forward_mixed::render(entt::registry &registry) {
  if (auto cam_ptr = registry.try_get<camera>(g_instance.active_camera)) {
    auto &cam_trans = registry.get<transform>(g_instance.active_camera);
    auto &cam_comp = *cam_ptr;

    update_scene_lights(registry);

    gbuffer.bind();
    gbuffer.set_viewport(0, 0, g_instance.scene_width, g_instance.scene_height);

    glEnable(GL_DEPTH_TEST);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0, 0, 0, 1);

    // render gbuffer
    gbuffer_geometry_pass.use();
    gbuffer_geometry_pass.set_mat4("gVP", cam_comp.vp);
    registry.view<mesh_data, transform>().each(
        [&](entt::entity entity, mesh_data &mesh, transform &trans) {
          gbuffer_geometry_pass.set_mat4("gModel", trans.matrix());
          draw_mesh_data(mesh);
        });

    gbuffer.unbind();

    // render color buffer
    cbuffer.bind();
    cbuffer.set_viewport(0, 0, g_instance.scene_width, g_instance.scene_height);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0, 0, 0, 1);

    if (should_draw_grid)
      draw_grid(grid_size, grid_spacing, cam_comp.vp);
    // debug rendering
    if (auto app_ptr = registry.ctx().get<iapp *>()) {
      auto script_sys = app_ptr->get_sys<script_system>();
      script_sys->draw_to_scene(app_ptr);
    }

    defered_phong_pass.use();
    defered_phong_pass.set_vec3("gViewDir", -cam_trans.local_forward());
    defered_phong_pass.set_texture2d("gPosTex", pos_tex.get_handle(), 0);
    defered_phong_pass.set_texture2d("gNormalTex", normal_tex.get_handle(), 1);
    defered_phong_pass.set_texture2d("gGbufferDepthTex",
                                     gbuffer_depth_tex.get_handle(), 2);
    defered_phong_pass.set_texture2d("gCbufferDepthTex",
                                     color_buffer_depth_tex.get_handle(), 3);
    defered_phong_pass.set_texture2d("gMaskTex", mask_tex.get_handle(), 4);
    defered_phong_pass.set_buffer_ssbo(light_data_buffer, 0);
    quad_draw_call();

    cbuffer.unbind();
  }
}

}; // namespace toolkit::opengl