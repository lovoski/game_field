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