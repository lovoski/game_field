#pragma once

#include "toolkit/opengl/components/material.hpp"

namespace toolkit::opengl {

class basic_material : public material {
public:
  void prepare0(entt::registry &registry) override {}
  void prepare1(entt::registry &registry) override {}

  std::string vertex_shader_source = R"(
#version 430 core
layout (location = 0) in vec4 aPos;
layout (location = 1) in vec4 aNormal;
layout (location = 2) in vec4 aTexCoord;

out vec3 vworldPos;
out vec3 vworldNormal;
out vec2 vtexCoord;

uniform mat4 gModelToWorldPoint;
uniform mat3 gModelToWorldDir;
uniform mat4 gViewMat;
uniform mat4 gProjMat;
void main() {
  vtexCoord = aTexCoord.xy;
  vworldNormal = normalize(gModelToWorldDir * aNormal.xyz);
  vworldPos = (gModelToWorldPoint * aPos).xyz;

  gl_Position = gProjMat * gViewMat * vec4(vworldPos, 1.0);
}
)";
  std::string fragment_shader_source = R"(
#version 430 core
uniform vec3 Albedo;
uniform vec2 gViewport;

uniform bool Wireframe;
uniform float WireframeWidth;
uniform float WireframeSmooth;
uniform vec3 WireframeColor;

uniform vec3 gViewDir;

in vec2 texCoord;
in vec3 worldPos;
in vec3 worldNormal;
in vec3 edgeDistance;

uniform sampler2D gLightMask;
uniform vec3 gSunDir;

out vec4 FragColor;

void main() {
  vec2 screenTexCoord = gl_FragCoord.xy / gViewport;
  float light_mask_value = clamp(texture(gLightMask, screenTexCoord).r, 0.0, 1.0);
  vec3 result = light_mask_value * Albedo;
  // wireframe related
  if (Wireframe) {
    float d = min(edgeDistance.x, min(edgeDistance.y, edgeDistance.z));
    float alpha = 0.0;
    if (d < WireframeWidth - WireframeSmooth) {
      alpha = 1.0;
    } else if (d > WireframeWidth + WireframeSmooth) {
      alpha = 0.0;
    } else {
      float x = d - (WireframeWidth - WireframeSmooth);
      alpha = exp2(-2.0 * x * x);
    }
    result = mix(result, WireframeColor, clamp(alpha, 0.0, 1.0));
  }
  FragColor = vec4(result, 1.0);
}
)";
  std::string geometry_shader_source = R"(
#version 430 core
layout (triangles) in;
layout (triangle_strip) out;
layout (max_vertices = 3) out;

uniform vec2 gViewport;

in vec2 vtexCoord[];
in vec3 vworldPos[];
in vec3 vworldNormal[];

out vec3 worldPos;
out vec3 worldNormal;
out vec2 texCoord;
// use linear interpolation instead of perspective interpolation
noperspective out vec3 edgeDistance;

void main() {
  vec3 ndc0 = gl_in[0].gl_Position.xyz / gl_in[0].gl_Position.w;
  vec3 ndc1 = gl_in[1].gl_Position.xyz / gl_in[1].gl_Position.w;
  vec3 ndc2 = gl_in[2].gl_Position.xyz / gl_in[2].gl_Position.w;

  vec2 p0 = (ndc0.xy + 1.0) * 0.5 * gViewport;
  vec2 p1 = (ndc1.xy + 1.0) * 0.5 * gViewport;
  vec2 p2 = (ndc2.xy + 1.0) * 0.5 * gViewport;

  // Compute edge lengths (avoid zero)
  float e01 = max(length(p0 - p1), 1e-6);
  float e12 = max(length(p1 - p2), 1e-6);
  float e20 = max(length(p2 - p0), 1e-6);

  // Compute edge heights
  float a1 = acos(clamp((e01 * e01 + e12 * e12 - e20 * e20) / (2.0 * e01 * e12), -1.0, 1.0));
  float a2 = acos(clamp((e12 * e12 + e20 * e20 - e01 * e01) / (2.0 * e12 * e20), -1.0, 1.0));

  float h20 = e12 * sin(a2);
  float h12 = e01 * sin(a1);
  float h01 = e12 * sin(a1);

  // Emit vertices with edge distance
  gl_Position = gl_in[0].gl_Position;
  texCoord = vtexCoord[0];
  worldPos = vworldPos[0];
  worldNormal = vworldNormal[0];
  edgeDistance = vec3(h12, 0.0, 0.0);
  EmitVertex();

  gl_Position = gl_in[1].gl_Position;
  texCoord = vtexCoord[1];
  worldPos = vworldPos[1];
  worldNormal = vworldNormal[1];
  edgeDistance = vec3(0.0, h20, 0.0);
  EmitVertex();

  gl_Position = gl_in[2].gl_Position;
  texCoord = vtexCoord[2];
  worldPos = vworldPos[2];
  worldNormal = vworldNormal[2];
  edgeDistance = vec3(0.0, 0.0, h01);
  EmitVertex();

  EndPrimitive();
}
)";
};
DECLARE_MATERIAL(basic_material)

}; // namespace toolkit::opengl