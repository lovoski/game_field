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

float rgb2luma(vec3 rgb){
    return sqrt(dot(rgb, vec3(0.299, 0.587, 0.114)));
}

const float EDGE_THRESHOLD_MIN = 0.0312;
const float EDGE_THRESHOLD_MAX = 0.125;
const float SUBPIXEL_QUALITY = 0.75;
#define ITERATIONS 12

void main() {
vec2 inverseScreenSize = vec2(1.0)/viewport_size;
vec3 colorCenter = texture(color_tex,texcoord).rgb;

// Luma at the current fragment
float lumaCenter = rgb2luma(colorCenter);

// Luma at the four direct neighbours of the current fragment.
float lumaDown = rgb2luma(textureOffset(color_tex,texcoord,ivec2(0,-1)).rgb);
float lumaUp = rgb2luma(textureOffset(color_tex,texcoord,ivec2(0,1)).rgb);
float lumaLeft = rgb2luma(textureOffset(color_tex,texcoord,ivec2(-1,0)).rgb);
float lumaRight = rgb2luma(textureOffset(color_tex,texcoord,ivec2(1,0)).rgb);

// Find the maximum and minimum luma around the current fragment.
float lumaMin = min(lumaCenter,min(min(lumaDown,lumaUp),min(lumaLeft,lumaRight)));
float lumaMax = max(lumaCenter,max(max(lumaDown,lumaUp),max(lumaLeft,lumaRight)));

// Compute the delta.
float lumaRange = lumaMax - lumaMin;

// If the luma variation is lower that a threshold (or if we are in a really dark area), we are not on an edge, don't perform any AA.
if(lumaRange < max(EDGE_THRESHOLD_MIN,lumaMax*EDGE_THRESHOLD_MAX)){
    frag_color = vec4(colorCenter, 1.0);
    return;
}
// Query the 4 remaining corners lumas.
float lumaDownLeft = rgb2luma(textureOffset(color_tex,texcoord,ivec2(-1,-1)).rgb);
float lumaUpRight = rgb2luma(textureOffset(color_tex,texcoord,ivec2(1,1)).rgb);
float lumaUpLeft = rgb2luma(textureOffset(color_tex,texcoord,ivec2(-1,1)).rgb);
float lumaDownRight = rgb2luma(textureOffset(color_tex,texcoord,ivec2(1,-1)).rgb);

// Combine the four edges lumas (using intermediary variables for future computations with the same values).
float lumaDownUp = lumaDown + lumaUp;
float lumaLeftRight = lumaLeft + lumaRight;

// Same for corners
float lumaLeftCorners = lumaDownLeft + lumaUpLeft;
float lumaDownCorners = lumaDownLeft + lumaDownRight;
float lumaRightCorners = lumaDownRight + lumaUpRight;
float lumaUpCorners = lumaUpRight + lumaUpLeft;

// Compute an estimation of the gradient along the horizontal and vertical axis.
float edgeHorizontal =  abs(-2.0 * lumaLeft + lumaLeftCorners)  + abs(-2.0 * lumaCenter + lumaDownUp ) * 2.0    + abs(-2.0 * lumaRight + lumaRightCorners);
float edgeVertical =    abs(-2.0 * lumaUp + lumaUpCorners)      + abs(-2.0 * lumaCenter + lumaLeftRight) * 2.0  + abs(-2.0 * lumaDown + lumaDownCorners);

// Is the local edge horizontal or vertical ?
bool isHorizontal = (edgeHorizontal >= edgeVertical);
// Select the two neighboring texels lumas in the opposite direction to the local edge.
float luma1 = isHorizontal ? lumaDown : lumaLeft;
float luma2 = isHorizontal ? lumaUp : lumaRight;
// Compute gradients in this direction.
float gradient1 = luma1 - lumaCenter;
float gradient2 = luma2 - lumaCenter;

// Which direction is the steepest ?
bool is1Steepest = abs(gradient1) >= abs(gradient2);

// Gradient in the corresponding direction, normalized.
float gradientScaled = 0.25*max(abs(gradient1),abs(gradient2));
// Choose the step size (one pixel) according to the edge direction.
float stepLength = isHorizontal ? inverseScreenSize.y : inverseScreenSize.x;

// Average luma in the correct direction.
float lumaLocalAverage = 0.0;

if(is1Steepest){
    // Switch the direction
    stepLength = - stepLength;
    lumaLocalAverage = 0.5*(luma1 + lumaCenter);
} else {
    lumaLocalAverage = 0.5*(luma2 + lumaCenter);
}

// Shift UV in the correct direction by half a pixel.
vec2 currentUv = texcoord;
if(isHorizontal){
    currentUv.y += stepLength * 0.5;
} else {
    currentUv.x += stepLength * 0.5;
}
// Compute offset (for each iteration step) in the right direction.
vec2 offset = isHorizontal ? vec2(inverseScreenSize.x,0.0) : vec2(0.0,inverseScreenSize.y);
// Compute UVs to explore on each side of the edge, orthogonally. The QUALITY allows us to step faster.
vec2 uv1 = currentUv - offset;
vec2 uv2 = currentUv + offset;

// Read the lumas at both current extremities of the exploration segment, and compute the delta wrt to the local average luma.
float lumaEnd1 = rgb2luma(texture(color_tex,uv1).rgb);
float lumaEnd2 = rgb2luma(texture(color_tex,uv2).rgb);
lumaEnd1 -= lumaLocalAverage;
lumaEnd2 -= lumaLocalAverage;

// If the luma deltas at the current extremities are larger than the local gradient, we have reached the side of the edge.
bool reached1 = abs(lumaEnd1) >= gradientScaled;
bool reached2 = abs(lumaEnd2) >= gradientScaled;
bool reachedBoth = reached1 && reached2;

// If the side is not reached, we continue to explore in this direction.
if(!reached1){
    uv1 -= offset;
}
if(!reached2){
    uv2 += offset;
}
// If both sides have not been reached, continue to explore.
if(!reachedBoth){

    for(int i = 2; i < ITERATIONS; i++){
        // If needed, read luma in 1st direction, compute delta.
        if(!reached1){
            lumaEnd1 = rgb2luma(texture(color_tex, uv1).rgb);
            lumaEnd1 = lumaEnd1 - lumaLocalAverage;
        }
        // If needed, read luma in opposite direction, compute delta.
        if(!reached2){
            lumaEnd2 = rgb2luma(texture(color_tex, uv2).rgb);
            lumaEnd2 = lumaEnd2 - lumaLocalAverage;
        }
        // If the luma deltas at the current extremities is larger than the local gradient, we have reached the side of the edge.
        reached1 = abs(lumaEnd1) >= gradientScaled;
        reached2 = abs(lumaEnd2) >= gradientScaled;
        reachedBoth = reached1 && reached2;

        // If the side is not reached, we continue to explore in this direction, with a variable quality.
        float quality = i <= 5 ? 1.0 : 1.0+0.5*(i-5);
        if(!reached1){
            uv1 -= offset * quality;
        }
        if(!reached2){
            uv2 += offset * quality;
        }

        // If both sides have been reached, stop the exploration.
        if(reachedBoth){ break;}
    }
}
// Compute the distances to each extremity of the edge.
float distance1 = isHorizontal ? (texcoord.x - uv1.x) : (texcoord.y - uv1.y);
float distance2 = isHorizontal ? (uv2.x - texcoord.x) : (uv2.y - texcoord.y);

// In which direction is the extremity of the edge closer ?
bool isDirection1 = distance1 < distance2;
float distanceFinal = min(distance1, distance2);

// Length of the edge.
float edgeThickness = (distance1 + distance2);

// UV offset: read in the direction of the closest side of the edge.
float pixelOffset = - distanceFinal / edgeThickness + 0.5;
// Is the luma at center smaller than the local average ?
bool isLumaCenterSmaller = lumaCenter < lumaLocalAverage;

// If the luma at center is smaller than at its neighbour, the delta luma at each end should be positive (same variation).
// (in the direction of the closer side of the edge.)
bool correctVariation = ((isDirection1 ? lumaEnd1 : lumaEnd2) < 0.0) != isLumaCenterSmaller;

// If the luma variation is incorrect, do not offset.
float finalOffset = correctVariation ? pixelOffset : 0.0;
// Sub-pixel shifting
// Full weighted average of the luma over the 3x3 neighborhood.
float lumaAverage = (1.0/12.0) * (2.0 * (lumaDownUp + lumaLeftRight) + lumaLeftCorners + lumaRightCorners);
// Ratio of the delta between the global average and the center luma, over the luma range in the 3x3 neighborhood.
float subPixelOffset1 = clamp(abs(lumaAverage - lumaCenter)/lumaRange,0.0,1.0);
float subPixelOffset2 = (-2.0 * subPixelOffset1 + 3.0) * subPixelOffset1 * subPixelOffset1;
// Compute a sub-pixel offset based on this delta.
float subPixelOffsetFinal = subPixelOffset2 * subPixelOffset2 * SUBPIXEL_QUALITY;

// Pick the biggest of the two offsets.
finalOffset = max(finalOffset,subPixelOffsetFinal);
// Compute the final UV coordinates.
vec2 finalUv = texcoord;
if(isHorizontal){
    finalUv.y += finalOffset * stepLength;
} else {
    finalUv.x += finalOffset * stepLength;
}

// Read the color at the new UV coordinates, and use it.
vec3 finalColor = texture(color_tex,finalUv).rgb;
frag_color = vec4(finalColor, 1.0);
}
)";
