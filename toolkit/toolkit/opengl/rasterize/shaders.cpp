#include "toolkit/opengl/rasterize/shaders.hpp"

std::string gbuffer_geometry_pass_vs = R"(
#version 450 core
layout (location = 0) in vec4 aPos;
layout (location = 1) in vec4 aNormal;
layout (location = 2) in vec4 aTexCoord;

layout(std430, binding = 0) readonly buffer ModelMatrices {
  mat4 gModels[]; // kept for compatibility if you want indexed models
};

uniform mat4 gVP;    // projection * view
uniform mat4 gModel; // model (per-mesh)

out vec3 vworldPos;
out vec3 vworldNormal;

vec3 safe_normalize(vec3 v) {
  float len = length(v);
  if (len > 1e-8) return v / len;
  return vec3(0.0, 0.0, 1.0); // fallback normal
}

void main() {
  vec3 worldPos = (gModel * aPos).xyz;
  vworldPos = worldPos;

  // transform normal properly (use inverse-transpose for non-uniform scale)
  mat3 normalMat = mat3(transpose(inverse(gModel)));
  vworldNormal = safe_normalize(normalMat * aNormal.xyz);

  gl_Position = gVP * vec4(worldPos, 1.0);
}
)";
std::string gbuffer_geometry_pass_gs = R"(
#version 450 core
layout (triangles) in;
layout (triangle_strip) out;
layout (max_vertices = 3) out;

uniform vec2 gViewport; // viewport size in pixels

in vec3 vworldPos[];
in vec3 vworldNormal[];

out vec3 worldPos;
out vec3 worldNormal;
noperspective out vec3 edgeDistance; // per-vertex distance to opposite edge (in pixels)

const float EPS = 1e-6;

void main() {
  // Recover clip-space NDC coords (x,y,z) for screen-space projection
  vec3 ndc0 = gl_in[0].gl_Position.xyz / gl_in[0].gl_Position.w;
  vec3 ndc1 = gl_in[1].gl_Position.xyz / gl_in[1].gl_Position.w;
  vec3 ndc2 = gl_in[2].gl_Position.xyz / gl_in[2].gl_Position.w;

  // convert to pixel-space positions
  vec2 p0 = (ndc0.xy * 0.5 + 0.5) * gViewport;
  vec2 p1 = (ndc1.xy * 0.5 + 0.5) * gViewport;
  vec2 p2 = (ndc2.xy * 0.5 + 0.5) * gViewport;

  // compute edge vectors and lengths (in pixels)
  vec2 e01 = p1 - p0;
  vec2 e12 = p2 - p1;
  vec2 e20 = p0 - p2;

  float l01 = max(length(e01), EPS);
  float l12 = max(length(e12), EPS);
  float l20 = max(length(e20), EPS);

  // point-to-line distance (per-vertex distance to opposite edge)
  // for vertex 0 -> distance to edge p1-p2
  // formula: abs(cross(edge, point - edgeStart)) / length(edge)
  float d0 = abs(e12.x * (p0.y - p1.y) - e12.y * (p0.x - p1.x)) / l12;
  float d1 = abs(e20.x * (p1.y - p2.y) - e20.y * (p1.x - p2.x)) / l20;
  float d2 = abs(e01.x * (p2.y - p0.y) - e01.y * (p2.x - p0.x)) / l01;

  // Emit vertices with distances packed per-vertex
  worldPos = vworldPos[0];
  worldNormal = vworldNormal[0];
  edgeDistance = vec3(d0, 0.0, 0.0);
  gl_Position = gl_in[0].gl_Position;
  EmitVertex();

  worldPos = vworldPos[1];
  worldNormal = vworldNormal[1];
  edgeDistance = vec3(0.0, d1, 0.0);
  gl_Position = gl_in[1].gl_Position;
  EmitVertex();

  worldPos = vworldPos[2];
  worldNormal = vworldNormal[2];
  edgeDistance = vec3(0.0, 0.0, d2);
  gl_Position = gl_in[2].gl_Position;
  EmitVertex();

  EndPrimitive();
}
)";
std::string gbuffer_geometry_pass_fs = R"(
#version 450 core

layout (location = 0) out vec4 gPosition; // world-space position
layout (location = 1) out vec4 gNormal;   // world-space normal (packed to 0..1)
layout (location = 2) out vec4 gMask;     // x: reserved, y: linear depth [0..1], z: wireframe mask, w: reserved
layout (location = 3) out vec4 gAlbedo;

in vec3 worldPos;
in vec3 worldNormal;
noperspective in vec3 edgeDistance;

uniform bool wireframe;
uniform mat4 gproj;      // projection matrix (for depth recon)
uniform vec3 albedo;

uniform float wireframe_width;  // in pixels
uniform float wireframe_smooth; // smoothing size in pixels

uniform float zNear;
uniform float zFar;

float linearize_depth_from_ndc(float depth) {
  // depth is the window-space depth (0..1). Convert to normalized device z (-1..1)
  float ndc_z = depth * 2.0 - 1.0;
  // reconstruct view-space z (negative in RH convention with typical projection)
  // viewZ = (2 * n * f) / (f + n - ndc_z * (f - n))
  float num = 2.0 * zNear * zFar;
  float denom = zFar + zNear - ndc_z * (zFar - zNear);
  float viewZ = num / max(denom, 1e-6);
  // convert to positive linear depth in [0..1]
  float linear = (-viewZ - zNear) / max((zFar - zNear), 1e-6);
  return clamp(linear, 0.0, 1.0);
}

float compute_wire_mask() {
  // smallest distance to any edge (pixel units)
  float d = min(edgeDistance.x, min(edgeDistance.y, edgeDistance.z));

  // If smoothing is zero, use hard threshold
  if (wireframe_smooth <= 0.0) {
    float inside = (d >= wireframe_width) ? 0.0 : 1.0;
    return wireframe ? inside : 1.0;
  }

  // construct smooth transition: 0 at line center, 1 in interior
  float outer = wireframe_width + wireframe_smooth;
  float inner = max(wireframe_width - wireframe_smooth, 0.0);
  // t = 0 at outer, 1 at inner; then interior = 1, near edge -> 0
  float t = smoothstep(outer, inner, d);
  // if wireframe is enabled, return interior fraction; otherwise fully interior
  return wireframe ? 1.0 - t : 1.0;
}

void main() {
  // outputs
  gPosition = vec4(worldPos, 1.0);
  gNormal = vec4(normalize(worldNormal) * 0.5 + 0.5, 1.0);
  float linDepth = linearize_depth_from_ndc(gl_FragCoord.z);
  float wf = compute_wire_mask(); // 1.0 = filled interior, 0.0 = line center
  gMask = vec4(1.0, linDepth, wf, 0.0);
  gAlbedo = vec4(albedo, 1.0);
}
)";
std::string defered_default_pass_fs = R"(
#version 430 core

uniform sampler2D pos_tex;
uniform sampler2D normal_tex;
uniform sampler2D mask_tex;
uniform sampler2D albedo_tex;
uniform sampler2D light_mask;

uniform sampler2D gbuffer_depth;
uniform sampler2D cbuffer_depth;

in vec2 texcoord;

out vec4 frag_color;

void main() {
  vec4 mask_value = texture(mask_tex, texcoord);
  if (mask_value.r != 1.0)
    discard;

  float gdepth = texture(gbuffer_depth, texcoord).r;
  float cdepth = texture(cbuffer_depth, texcoord).r;
  if (gdepth > cdepth)
    discard;
  gl_FragDepth = gdepth;

  // float ldepth = mask_value.g;
  float wireframe = mask_value.b;
  float light_value = texture(light_mask, texcoord).r;
  vec3 albedo = texture(albedo_tex, texcoord).xyz;

  // vec3 frag_world_pos = texture(pos_tex, texcoord);
  // vec3 frag_world_normal = texture(normal_tex, texcoord).xyz * 2.0 - vec3(1.0);

  frag_color = vec4(clamp(albedo * light_value * wireframe, 0.0, 1.0), 1.0);
}
)";

std::string shadow_vs = R"(
#version 450 core
layout (location = 0) in vec4 aPos;

uniform mat4 gVP;
uniform mat4 gModel;
void main() {
  gl_Position = gVP * gModel * aPos;
}
)";
std::string shadow_fs = R"(
#version 450 core
void main() {}
)";

std::string quad_vs = R"(
#version 450 core
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

std::string static_mesh_light_mask_fs = R"(
#version 430 core

uniform vec3 light_dir;

uniform sampler2D scene_pos;
uniform sampler2D scene_normal;
uniform sampler2D scene_mask;

uniform float shadow_weight;

in vec2 texcoord;
out vec4 frag_color;

void main() {
  if (texture(scene_mask, texcoord).r != 1.0) {
    discard;
  }
  vec3 frag_world_pos = texture(scene_pos, texcoord).xyz;
  vec3 frag_normal = normalize(2.0 * texture(scene_normal, texcoord).xyz - 1.0);
  vec3 l_dir = -normalize(light_dir);

  float diffuse = max(0.0, dot(frag_normal, l_dir));
  float shadow = diffuse;
  shadow = 1.0+shadow_weight*(shadow-1);

  frag_color = vec4(vec3(shadow), 1.0);
}
)";

std::string shadow_mask_fs = R"(
#version 430 core

uniform mat4 shadow_vp;
uniform int shadowmap_dim;

uniform float max_bias;
uniform float min_bias;

uniform float shadow_weight;

uniform vec3 light_dir;

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
    shadow = 1.0+shadow_weight*(shadow-1);

    frag_color = vec4(vec3(shadow), 1.0);
}
)";

std::string fxaa_fs = R"(
#version 430 core

in vec2 texcoord;
out vec4 frag_color;

uniform vec2 viewport_size;
uniform sampler2D color_tex;

void main() {
    vec2 invTextureResolution = vec2(1.0)/viewport_size;
    const float spanMax = 4.0;
    const float reduceAmount = 1.0 / 4.0;
    const float reduceMin = (1.0 / 64.0);

    vec3 luma = vec3(0.299, 0.587, 0.114);
    float lumaNW = dot(texture(color_tex, texcoord + (vec2(-1.0, -1.0) * invTextureResolution)).rgb, luma);
    float lumaNE = dot(texture(color_tex, texcoord + (vec2( 1.0, -1.0) * invTextureResolution)).rgb, luma);
    float lumaSW = dot(texture(color_tex, texcoord + (vec2(-1.0,  1.0) * invTextureResolution)).rgb, luma);
    float lumaSE = dot(texture(color_tex, texcoord + (vec2( 1.0,  1.0) * invTextureResolution)).rgb, luma);
    float lumaMI = dot(texture(color_tex, texcoord).rgb, luma);

    float lumaMin = min(lumaMI, min(min(lumaNW, lumaNE), min(lumaSW, lumaSE)));
    float lumaMax = max(lumaMI, max(max(lumaNW, lumaNE), max(lumaSW, lumaSE)));

    vec2 dir = vec2(
        -((lumaNW + lumaNE) - (lumaSW + lumaSE)),
        +((lumaNW + lumaSW) - (lumaNE + lumaSE)));

    float dirReduce = max((lumaNW + lumaNE + lumaSW + lumaSE) * (0.25 * reduceAmount), reduceMin);
    float dirRcpMin = 1.0/(min(abs(dir.x), abs(dir.y)) + dirReduce);

    dir = min(vec2(spanMax,  spanMax), max(vec2(-spanMax, -spanMax), dir * dirRcpMin)) * invTextureResolution;

    vec3 rgba0 = texture(color_tex, texcoord + dir * (1.0 / 3.0 - 0.5)).rgb;
    vec3 rgba1 = texture(color_tex, texcoord + dir * (2.0 / 3.0 - 0.5)).rgb;
    vec3 rgba2 = texture(color_tex, texcoord + dir * (0.0 / 3.0 - 0.5)).rgb;
    vec3 rgba3 = texture(color_tex, texcoord + dir * (3.0 / 3.0 - 0.5)).rgb;

    vec3 rgbA = (1.0/ 2.0) * (rgba0 + rgba1);
    vec3 rgbB = rgbA * (1.0/ 2.0) + (1.0/ 4.0) * (rgba2 + rgba3);

    float lumaB = dot(rgbB, luma);
    
    frag_color = vec4((lumaB < lumaMin) || (lumaB > lumaMax) ? rgbA : rgbB, 1.0);
}
)";
