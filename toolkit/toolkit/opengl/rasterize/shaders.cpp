#include "toolkit/opengl/rasterize/shaders.hpp"

std::string gbuffer_geometry_pass_vs = R"(
#version 460 core
layout (location = 0) in vec4 aPos;
layout (location = 1) in vec4 aNormal;

layout(std430, binding = 0) buffer ModelMatrices {
  mat4 gModels[];
};

uniform mat4 gVP;
uniform mat4 gModel;

out vec3 worldPos;
out vec3 worldNormal;

void main() {
  worldNormal = normalize(mat3(gModel)*aNormal.xyz);
  worldPos = (gModel * aPos).xyz;

  gl_Position = gVP * vec4(worldPos, 1.0);
}
)";
std::string gbuffer_geometry_pass_fs = R"(
#version 460 core

layout (location = 0) out vec4 gPosition; // G-buffer position output
layout (location = 1) out vec4 gNormal;   // G-buffer normal output
layout (location = 2) out vec4 gMask;

in vec3 worldPos;
in vec3 worldNormal;

uniform mat4 gproj;

float linearize_depth(float depth) {
  vec4 ndc = vec4(gl_FragCoord.x*2-1,gl_FragCoord.y*2-1, depth * 2.0 - 1.0, 1.0);
  vec4 view = inverse(gproj) * ndc;
  return view.z / view.w;
}

void main() {
  gPosition = vec4(worldPos, 1.0);
  // map normal to range [0, 1]
  gNormal = vec4(normalize(worldNormal) * 0.5 + 0.5, 1.0);
  gMask = vec4(1,linearize_depth(gl_FragCoord.z),0,0);
}
)";

std::string defered_phong_pass_vs = R"(
#version 430 core

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec2 aTexCoord;

out vec2 texCoord;

void main() {
  texCoord = aTexCoord;
  gl_Position = vec4(aPos, 1.0);
}
)";

std::string defered_phong_pass_fs = R"(
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
  gl_FragDepth = g_depth;
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

std::string csm_vs = R"(
#version 460 core
layout (location = 0) in vec4 aPos;

uniform mat4 gVP;
uniform mat4 gModel;
void main() {
  gl_Position = gVP * gModel * aPos;
}
)";
std::string csm_fs = R"(
#version 460 core
void main() {}
)";

std::string quad_vs = R"(
#version 460 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec2 aTexCoord;
out vec2 texcoord;
void main() {
  texcoord = aTexCoord;
  gl_Position = vec4(aPos, 1.0);
}
)";
std::string csm_selection_mask_fs = R"(
#version 430 core
layout(std430, binding = 0) buffer CascadeVPMatrices {
  mat4 csm_vp_mat[];
};
uniform int num_cascades;
uniform int csm_depth_dim;
uniform float bias_scale;
uniform float max_bias;

uniform vec2 viewport_size;

uniform int pcf_kernal_size;

uniform vec3 light_dir;

uniform float csm_cascades[10];

uniform sampler2D scene_pos;
uniform sampler2D scene_normal;
uniform sampler2D scene_mask;

uniform sampler2D cascade_depth;
uniform sampler2D cascade_pos;

in vec2 texcoord;
out vec4 frag_color;

void main() {
  vec3 L = -normalize(light_dir);
  vec2 viewport_texel = vec2(1.0)/viewport_size;

  vec4 mask_value = texture(scene_mask, texcoord);
  float scene_depth = mask_value.r == 1.0 ? -mask_value.g : -1.0;
  int match_cascade = -1;
  for (int i = 0; i < num_cascades; i++) {
    if (scene_depth > csm_cascades[i] && scene_depth <= csm_cascades[i+1]) {
      match_cascade = i;
      break;
    }
  }

  if (match_cascade < 0)
    return;

  float shadow = 0.0, cos_alpha, bias, sampled_csm_depth, scene_light_depth, shadow0 = 0.0;
  vec4 point;
  vec3 N=normalize(texture(scene_normal, texcoord).xyz * 2 - 1), world_pos;
  vec2 csm_texcoord;
  cos_alpha = dot(N, L);
  if (cos_alpha < 0.05) {
    shadow = 1.0;
  } else {
    for (int i = -pcf_kernal_size; i <= pcf_kernal_size; i++) {
      for (int j = -pcf_kernal_size; j <= pcf_kernal_size; j++) {
        vec2 pcf_texcoord = texcoord + vec2(i*viewport_texel.x,j*viewport_texel.y);
        if (texture(scene_mask, pcf_texcoord).r != 1.0)
          continue;
        N = normalize(texture(scene_normal, pcf_texcoord).xyz * 2 - 1);
        cos_alpha = dot(N, L);
        if (cos_alpha >= 0.05) {
          world_pos = texture(scene_pos, pcf_texcoord).xyz;
          cos_alpha = max(0.01, cos_alpha);
          point = vec4(world_pos, 1.0);
          point = csm_vp_mat[match_cascade]*point;
          point.xyz = point.xyz/point.w;
          vec2 csm_texcoord = vec2(match_cascade/float(num_cascades)+0.5*(point.x+1.0)/num_cascades,0.5*(point.y+1.0));
          sampled_csm_depth = texture(cascade_depth, csm_texcoord).r;
          scene_light_depth = 0.5*(point.z+1.0);
          bias = min(bias_scale*0.5/csm_depth_dim*sqrt(1.0-cos_alpha*cos_alpha)/cos_alpha, max_bias);
  
          if (sampled_csm_depth + bias < scene_light_depth) {
            shadow += 1.0;
          }
        } else {
          shadow += 1.0;
        }
      }
    }
    shadow /= (2*pcf_kernal_size+1)*(2*pcf_kernal_size+1);
  }

  frag_color = vec4(vec3(mix(0.5, 1.0, clamp(1.0-shadow, 0.0, 1.0))),1.0);
  // if (match_cascade == 0)
  //   frag_color = vec4(1.0,0.0,0.0,1.0);
  // else if (match_cascade == 1)
  //   frag_color = vec4(0.0,1.0,0.0,1.0);
  // else if (match_cascade == 2)
  //   frag_color = vec4(0.0,0.0,1.0,1.0);
  // else if (match_cascade == 3)
  //   frag_color = vec4(1.0,0.0,1.0,1.0);
  // else if (match_cascade == 4)
  //   frag_color = vec4(0.0,1.0,1.0,1.0);
  // else if (match_cascade == 5)
  //   frag_color = vec4(1.0,1.0,0.0,1.0);
  // else
  //   frag_color = vec4(1.0,1.0,1.0,1.0);
}
)";