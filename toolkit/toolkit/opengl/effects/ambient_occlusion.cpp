#include "toolkit/opengl/effects/ambient_occlusion.hpp"
#include "toolkit/opengl/draw.hpp"
#include <random>

namespace toolkit::opengl {

void create_hemisphere_kernal(int kernal_size,
                              std::vector<math::vector4> &kernal) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis01(0.0, 1.0);
  std::uniform_real_distribution<> dis11(-1.0f, 1.0);
  kernal.resize(kernal_size);
  for (int i = 0; i < kernal_size; i++) {
    kernal[i].x() = dis11(gen);
    kernal[i].y() = dis11(gen);
    kernal[i].z() = dis01(gen);
    kernal[i].w() = 0.0f;
    kernal[i].normalize();
    float scale = i / (float)kernal_size;
    kernal[i] *= (0.1f + 0.9f * scale * scale);
  }
}
assets::image create_noise_image(int dim) {
  assets::image img;
  img.resize(dim, dim, 3);
  for (int i = 0; i < dim; i++) {
    for (int j = 0; j < dim; j++) {
      img.pixel(i, j, 0) = (unsigned char)(255 * math::rand(0, 1));
      img.pixel(i, j, 1) = (unsigned char)(255 * math::rand(0, 1));
      img.pixel(i, j, 2) = 0;
    }
  }
  return img;
}
std::string ao_pass_vs = R"(
#version 430 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec2 aTexCoord;
out vec2 texcoord;
void main() {
  texcoord = aTexCoord;
  gl_Position = vec4(aPos, 1.0);
}
)";
std::string ssao_pass_fs = R"(
#version 430 core
in vec2 texcoord;
out vec4 fragcolor;
uniform sampler2D gbuffer_pos;
uniform sampler2D gbuffer_normal;
uniform sampler2D gbuffer_mask;
uniform sampler2D noise_tex;
uniform mat4 gview;
uniform mat4 gproj;
uniform int kernal_size;
uniform float radius;
uniform float noise_scale;
layout(std430, binding = 0) buffer kernal_buffer {
  vec4 gkernals[];
};
void main() {
  vec3 world_pos = texture(gbuffer_pos, texcoord).xyz;
  vec3 world_normal = normalize(texture(gbuffer_normal, texcoord).xyz * 2 - 1);
  vec3 view_pos = (gview * vec4(world_pos, 1)).xyz;
  vec4 mask_value = texture(gbuffer_mask, texcoord);
  // vec4 clip_pos = (gproj*vec4(view_pos, 1.0));
  // vec3 ndc_pos = clip_pos.xyz/clip_pos.w;
  // float frag_depth = ndc_pos.z*0.5+0.5;

  vec3 view_normal = normalize(mat3(gview)*world_normal);
  vec3 normal = view_normal;
  vec3 rot_vec = normalize(texture(noise_tex, texcoord*noise_scale).xyz * 2 - 1);
  vec3 tangent = normalize(rot_vec - normal*dot(rot_vec,normal));
  vec3 bitangent = cross(normal, tangent);
  mat3 tbn = mat3(tangent,bitangent,normal);
  float occlusion = 0.0;
  for (int i = 0; i < kernal_size; i++) {
    vec3 kernal_sample = tbn*(gkernals[i].xyz);
    kernal_sample = kernal_sample*radius+view_pos;
    vec4 offset = gproj*vec4(kernal_sample,1.0);
    if (offset.w <= 0.0) continue;
    offset.xy /= offset.w;
    offset.xy = offset.xy*0.5+0.5;

    // depth of target fragment in view space
    // the linearized depth has been stored in a texture during 
    // gbuffer rendering, no additional actions needed here.
    float linear_depth = texture(gbuffer_mask,offset.xy).g;
    float range_check = abs(view_pos.z - linear_depth) < radius ? 1.0 : 0.0;
    occlusion += (linear_depth > kernal_sample.z ? 1.0 : 0.0) * range_check;
  }
  occlusion = 1.0 - occlusion / kernal_size * mask_value.r;
  fragcolor = vec4(occlusion, occlusion, occlusion, 1.0);
}
)";
void ssao(texture &gbuffer_pos, texture &gbuffer_normal, texture &gbuffer_mask,
          math::matrix4 &view, math::matrix4 &proj, float noise_scale,
          float radius) {
  const int noise_tex_dim = 4;
  const int kernal_size = 64;
  static shader program;
  static texture noise_tex;
  static buffer kernal_buffer;
  static bool initialized = false;
  if (!initialized) {
    program.compile_shader_from_source(ao_pass_vs, ssao_pass_fs);
    auto img = create_noise_image(16);
    noise_tex.create_from_image(img);
    noise_tex.set_parameters({{GL_TEXTURE_MIN_FILTER, GL_NEAREST},
                              {GL_TEXTURE_MAG_FILTER, GL_NEAREST},
                              {GL_TEXTURE_WRAP_S, GL_REPEAT},
                              {GL_TEXTURE_WRAP_T, GL_REPEAT}});
    std::vector<math::vector4> hemisphere_kernal;
    create_hemisphere_kernal(kernal_size, hemisphere_kernal);
    kernal_buffer.create();
    kernal_buffer.set_data_ssbo(hemisphere_kernal);
    initialized = true;
  }
  program.use();
  program.set_mat4("gview", view);
  program.set_mat4("gproj", proj);
  program.set_float("radius", radius);
  program.set_int("kernal_size", kernal_size);
  program.set_float("noise_scale", noise_scale);
  program.set_texture2d("gbuffer_pos", gbuffer_pos.get_handle(), 0);
  program.set_texture2d("gbuffer_normal", gbuffer_normal.get_handle(), 1);
  program.set_texture2d("gbuffer_mask", gbuffer_mask.get_handle(), 2);
  program.set_texture2d("noise_tex", noise_tex.get_handle(), 3);
  program.set_buffer_ssbo(kernal_buffer, 0);
  quad_draw_call();
}

std::string box_filter_fs = R"(
#version 430 core
in vec2 texcoord;
out vec4 fragcolor;
uniform int kernal_size;
uniform sampler2D input_tex;
void main() {
  vec2 texel_size = 1.0/vec2(textureSize(input_tex, 0));
  float result = 0.0;
  vec2 hlim = vec2(float(-kernal_size) * 0.5 + 0.5);
  for (int i = 0; i < kernal_size; ++i) {
     for (int j = 0; j < kernal_size; ++j) {
        vec2 offset = (hlim + vec2(i, j)) * texel_size;
        result += texture(input_tex, texcoord + offset).r;
     }
  }
  result = result/(kernal_size*kernal_size);
  fragcolor = vec4(vec3(0.0),1.0-result);
}
)";
void ao_box_filter(texture &input_tex, int kernal_size) {
  static shader program;
  static bool initialized = false;
  if (!initialized) {
    program.compile_shader_from_source(ao_pass_vs, box_filter_fs);
    initialized = true;
  }
  program.use();
  program.set_int("kernal_size", kernal_size);
  program.set_texture2d("input_tex", input_tex.get_handle(), 0);
  quad_draw_call();
}

std::string gaussian_filter_fs = R"(
#version 430 core
in vec2 texcoord;
out vec4 fragcolor;
uniform int kernal_size;
uniform float sigma;
uniform sampler2D input_tex;
void main() {
  vec2 texel_size = 1.0 / vec2(textureSize(input_tex, 0));
  float result = 0.0;
  float weight_sum = 0.0;
  int half_k = kernal_size / 2;
  for (int i = -half_k; i <= half_k; ++i) {
    for (int j = -half_k; j <= half_k; ++j) {
      vec2 offset = vec2(i, j) * texel_size;
      float weight = exp(-(i*i + j*j) / (2.0 * sigma * sigma));
      weight_sum += weight;
      result += texture(input_tex, texcoord + offset).r * weight;
    }
  }
  result = clamp(result / weight_sum,0.0,1.0);
  fragcolor = vec4(vec3(0.0), 1.0-result);
}
)";
void ao_gaussian_filter(texture &input_tex, int kernal_size, float sigma) {
  static shader program;
  static bool initialized = false;
  if (!initialized) {
    program.compile_shader_from_source(ao_pass_vs, gaussian_filter_fs);
    initialized = true;
  }
  program.use();
  program.set_float("sigma", sigma);
  program.set_int("kernal_size", kernal_size);
  program.set_texture2d("input_tex", input_tex.get_handle(), 0);
  quad_draw_call();
}

}; // namespace toolkit::opengl