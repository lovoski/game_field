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

std::string shadow_vs = R"(
#version 460 core
layout (location = 0) in vec4 aPos;

uniform mat4 gVP;
uniform mat4 gModel;
void main() {
  gl_Position = gVP * gModel * aPos;
}
)";
std::string shadow_fs = R"(
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

uniform float max_bias;
uniform float min_bias;

uniform vec2 viewport_size;

uniform int pcf_kernal_size;

uniform vec3 light_dir;

uniform float csm_cascades[10];

uniform sampler2D scene_pos;
uniform sampler2D scene_normal;
uniform sampler2D scene_mask;
uniform mat4 cam_view;

uniform sampler2DShadow cascade_depth;

in vec2 texcoord;
out vec4 frag_color;

const vec2 POISSON_DISK[16] = vec2[](
  vec2( -0.94201624, -0.39906216 ), vec2( 0.94558609, -0.76890725 ),
  vec2( -0.094184101, -0.92938870 ), vec2( 0.34495938, 0.29387760 ),
  vec2( -0.91588581, 0.45771432 ), vec2( -0.81544232, -0.87912464 ),
  vec2( -0.38277543, 0.27676845 ), vec2( 0.97484398, 0.75648379 ),
  vec2( 0.44323325, -0.97511554 ), vec2( 0.53742981, -0.47373420 ),
  vec2( -0.26496911, -0.41893023 ), vec2( 0.79197514, 0.19090188 ),
  vec2( -0.24188840, 0.99706507 ), vec2( -0.81409955, 0.91437590 ),
  vec2( 0.19984126, 0.78641367 ), vec2( 0.14383161, -0.14100790 )
);

float random(vec2 st) {
  return fract(sin(dot(st.xy, vec2(12.9898,78.233))) * 43758.5453123);
}

void main() {
  frag_color = vec4(1.0);
  return;

  // vec3 l_dir = -normalize(light_dir);
  // vec2 viewport_texel = vec2(1.0)/viewport_size;

  // vec4 mask_value = texture(scene_mask, texcoord);
  // if (mask_value.r == 0) {
  //   // force white background
  //   frag_color = vec4(1.0);
  //   return;
  // }

  // vec3 frag_world_pos = texture(scene_pos, texcoord).xyz;
  // vec3 frag_world_normal = normalize(2*texture(scene_normal, texcoord).xyz-vec3(1.0));
  // vec4 cam_space_pos = cam_view * vec4(frag_world_pos, 1.0);
  // cam_space_pos.xyz /= cam_space_pos.w;
  // float linear_depth = -cam_space_pos.z;
  // int match_cascade = -1;
  // for (int i = 0; i < num_cascades; i++) {
  //   if (linear_depth > csm_cascades[i] && linear_depth <= csm_cascades[i+1]) {
  //     match_cascade = i;
  //     break;
  //   }
  // }
  // if (match_cascade < 0)
  //   discard;

  // mat4 shadow_vp = csm_vp_mat[match_cascade];
  // vec4 lp_frag_world_pos = shadow_vp * vec4(frag_world_pos, 1.0);
  // lp_frag_world_pos.xyz /= lp_frag_world_pos.w;

  // float cos_alpha = max(0.05, dot(frag_world_normal, l_dir));
  // float bias = mix(max_bias, min_bias, cos_alpha);
  // float frag_depth_value = (lp_frag_world_pos.z + 1.0) * 0.5;
  // float repaired_depth = frag_depth_value - bias;

  // vec2 shadow_texcoord = 0.5 * (lp_frag_world_pos.xy + vec2(1.0));
  // shadow_texcoord.x = shadow_texcoord.x/float(num_cascades)+float(match_cascade)/float(num_cascades);
  // // float shadowmap_value = texture(cascade_depth, shadow_texcoord).r;
  // // float shadow = repaired_depth > shadowmap_value ? 0.0 : 1.0;

  // float shadow = 0.0;
  // float rand_angle = random(texcoord) * 2.0 * 3.14159265;
  // float search_radius = 2.5 / float(csm_depth_dim);
  // mat2 rotation_matrix = mat2(cos(rand_angle), sin(rand_angle), -sin(rand_angle), cos(rand_angle));
  // for (int i = 0; i < POISSON_DISK.length(); i++) {
  //   vec2 offset = rotation_matrix * POISSON_DISK[i];
  //   offset.x /= float(num_cascades);
  //   vec2 tmp_shadow_texcoord = shadow_texcoord + offset * search_radius;
  //   shadow += texture(cascade_depth, vec3(tmp_shadow_texcoord, repaired_depth));
  // }
  // shadow /= POISSON_DISK.length();
  // frag_color = vec4(vec3(shadow), 1.0);

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

std::string shadow_mask_fs = R"(
#version 430 core

uniform mat4 shadow_vp;
uniform int shadowmap_dim;

uniform float max_bias;
uniform float min_bias;

uniform vec3 light_dir;

// Deferred rendering G-Buffer samplers
uniform sampler2D scene_pos;
uniform sampler2D scene_normal;
uniform sampler2D scene_mask;

uniform sampler2DShadow shadowmap;

in vec2 texcoord;
out vec4 frag_color;

const vec2 POISSON_DISK[16] = vec2[](
  vec2( -0.94201624, -0.39906216 ), vec2( 0.94558609, -0.76890725 ),
  vec2( -0.094184101, -0.92938870 ), vec2( 0.34495938, 0.29387760 ),
  vec2( -0.91588581, 0.45771432 ), vec2( -0.81544232, -0.87912464 ),
  vec2( -0.38277543, 0.27676845 ), vec2( 0.97484398, 0.75648379 ),
  vec2( 0.44323325, -0.97511554 ), vec2( 0.53742981, -0.47373420 ),
  vec2( -0.26496911, -0.41893023 ), vec2( 0.79197514, 0.19090188 ),
  vec2( -0.24188840, 0.99706507 ), vec2( -0.81409955, 0.91437590 ),
  vec2( 0.19984126, 0.78641367 ), vec2( 0.14383161, -0.14100790 )
);

float random(vec2 st) {
  return fract(sin(dot(st.xy, vec2(12.9898,78.233))) * 43758.5453123);
}

void main() {
    if (texture(scene_mask, texcoord).r != 1.0) {
        discard;
    }
    vec3 frag_world_pos = texture(scene_pos, texcoord).xyz;
    vec3 frag_normal = normalize(2.0 * texture(scene_normal, texcoord).xyz - 1.0);
    vec3 l_dir = -normalize(light_dir);

    vec4 lp_frag_world_pos = shadow_vp * vec4(frag_world_pos, 1.0);
    lp_frag_world_pos.xyz /= lp_frag_world_pos.w;

    if (abs(lp_frag_world_pos.x) > 1.0 || abs(lp_frag_world_pos.y) > 1.0) {
        discard;
    }

    float bias = max(min_bias, max_bias * (1.0 - abs(dot(frag_normal, l_dir))));

    float frag_depth_value = (lp_frag_world_pos.z + 1.0) * 0.5;
    float repaired_depth = frag_depth_value - bias;

    vec2 shadow_texcoord = 0.5 * (lp_frag_world_pos.xy + vec2(1.0));
    float shadow = 0.0;
    float search_radius = 2.5 / float(shadowmap_dim); // How far to spread the samples
    float rand_angle = random(texcoord) * 2.0 * 3.14159265;
    mat2 rotation_matrix = mat2(cos(rand_angle), sin(rand_angle), -sin(rand_angle), cos(rand_angle));

    for (int i = 0; i < 16; i++) {
        vec2 offset = rotation_matrix * POISSON_DISK[i];
        vec2 tmp_shadow_texcoord = shadow_texcoord + offset * search_radius;
        shadow += texture(shadowmap, vec3(tmp_shadow_texcoord, repaired_depth));
    }
    shadow /= 16.0;

    float diffuse = max(0.0, dot(frag_normal, l_dir));
    // float diffuse = 0.5*(dot(frag_normal, l_dir) + 1.0);
    shadow = min(diffuse, shadow);
    shadow = 0.5 + 0.5*shadow;

    frag_color = vec4(vec3(shadow), 1.0);
}
)";
