#include "toolkit/opengl/effects/sky.hpp"
#include "toolkit/opengl/draw.hpp"

namespace toolkit::opengl {

float ZenithLuminance(float thetaS, float T) {
  float chi = (4.0f / 9.0f - T / 120.0f) * (3.1415927f - 2.0f * thetaS);
  float Lz = (4.0453f * T - 4.9710f) * tanf(chi) - 0.2155f * T + 2.4192f;
  Lz *= 1000.0; // conversion from kcd/m^2 to cd/m^2
  return Lz;
}
inline float PerezUpper(const float *lambdas, float cosTheta, float gamma,
                        float cosGamma) {
  return (1.0f + lambdas[0] * expf(lambdas[1] / (cosTheta + 1e-6f))) *
         (1.0f + lambdas[2] * expf(lambdas[3] * gamma) +
          lambdas[4] * cosGamma * cosGamma);
}

inline float PerezLower(const float *lambdas, float cosThetaS, float thetaS) {
  return (1.0f + lambdas[0] * expf(lambdas[1])) *
         (1.0f + lambdas[2] * expf(lambdas[3] * thetaS) +
          lambdas[4] * cosThetaS * cosThetaS);
}
inline float lerp(float a, float b, float s) { return (1.0f - s) * a + s * b; }

// XYZ/RGB for sRGB primaries
static math::vector3 kXYZToR(3.2404542f, -1.5371385f, -0.4985314f);
static math::vector3 kXYZToG(-0.9692660f, 1.8760108f, 0.0415560f);
static math::vector3 kXYZToB(0.0556434f, -0.2040259f, 1.0572252f);

static math::vector3 kRGBToX(0.4124564f, 0.3575761f, 0.1804375f);
static math::vector3 kRGBToY(0.2126729f, 0.7151522f, 0.0721750f);
static math::vector3 kRGBToZ(0.0193339f, 0.1191920f, 0.9503041f);

static math::vector2 kClearChroma(2.0f / 3.0f, 1.0f / 3.0f);
static math::vector2 kOvercastChroma(1.0f / 3.0f, 1.0f / 3.0f);
static math::vector2 kPartlyCloudyChroma(1.0f / 3.0f, 1.0f / 3.0f);

inline math::vector3 xyYToXYZ(const math::vector3 &c) {
  return math::vector3(c.x(), c.y(), 1.0f - c.x() - c.y()) * c.z() / c.y();
}
inline math::vector3 xyYToRGB(const math::vector3 &xyY) {
  math::vector3 XYZ = xyYToXYZ(xyY);
  return math::vector3(kXYZToR.dot(XYZ), kXYZToG.dot(XYZ), kXYZToB.dot(XYZ));
}
inline math::vector3 XYZToRGB(const math::vector3 &XYZ) {
  return math::vector3(kXYZToR.dot(XYZ), kXYZToG.dot(XYZ), kXYZToB.dot(XYZ));
}
inline math::vector3 RGBToXYZ(const math::vector3 &rgb) {
  return math::vector3(kRGBToX.dot(rgb), kRGBToY.dot(rgb), kRGBToZ.dot(rgb));
}
inline float ClampUnit(float s) {
  if (s <= 0.0f)
    return 0.0f;
  if (s >= 1.0f)
    return 1.0f;
  return s;
}

void preetham_sun_sky::update(const math::vector3 &sun, float turbidity,
                              float overcast, float horizCrush) {
  mToSun = sun.normalized();
  float T = turbidity;

  mPerez_Y[0] = 0.17872f * T - 1.46303f;
  mPerez_Y[1] = -0.35540f * T + 0.42749f;
  mPerez_Y[2] = -0.02266f * T + 5.32505f;
  mPerez_Y[3] = 0.12064f * T - 2.57705f;
  mPerez_Y[4] = -0.06696f * T + 0.37027f;

  mPerez_x[0] = -0.01925f * T - 0.25922f;
  mPerez_x[1] = -0.06651f * T + 0.00081f;
  mPerez_x[2] = -0.00041f * T + 0.21247f;
  mPerez_x[3] = -0.06409f * T - 0.89887f;
  mPerez_x[4] = -0.00325f * T + 0.04517f;

  mPerez_y[0] = -0.01669f * T - 0.26078f;
  mPerez_y[1] = -0.09495f * T + 0.00921f;
  mPerez_y[2] = -0.00792f * T + 0.21023f;
  mPerez_y[3] = -0.04405f * T - 1.65369f;
  mPerez_y[4] = -0.01092f * T + 0.05291f;

  float cosTheta = mToSun.z();
  float theta = acosf(cosTheta); // angle from zenith rather than horizon
  float theta2 = theta * theta;
  float theta3 = theta2 * theta;
  float T2 = T * T;

  // mZenith stored as xyY
  mZenith.z() = ZenithLuminance(theta, T);

  mZenith.x() =
      (0.00165f * theta3 - 0.00374f * theta2 + 0.00208f * theta + 0.00000f) *
          T2 +
      (-0.02902f * theta3 + 0.06377f * theta2 - 0.03202f * theta + 0.00394f) *
          T +
      (0.11693f * theta3 - 0.21196f * theta2 + 0.06052f * theta + 0.25885f);

  mZenith.y() =
      (0.00275f * theta3 - 0.00610f * theta2 + 0.00316f * theta + 0.00000f) *
          T2 +
      (-0.04214f * theta3 + 0.08970f * theta2 - 0.04153f * theta + 0.00515f) *
          T +
      (0.15346f * theta3 - 0.26756f * theta2 + 0.06669f * theta + 0.26688f);

  // Adjustments (extensions)

  if (cosTheta < 0.0f) // Handle sun going below the horizon
  {
    float s =
        ClampUnit(1.0f + cosTheta * 50.0f); // goes from 1 to 0 as the sun sets

    // Take C/E which control sun term to zero
    mPerez_x[2] *= s;
    mPerez_y[2] *= s;
    mPerez_Y[2] *= s;
    mPerez_x[4] *= s;
    mPerez_y[4] *= s;
    mPerez_Y[4] *= s;
  }

  if (overcast != 0.0f) // Handle overcast term
  {
    float invOvercast = 1.0f - overcast;

    // lerp back towards unity
    mPerez_x[0] *= invOvercast; // main sky chroma -> base
    mPerez_y[0] *= invOvercast;

    // sun flare -> 0 strength/base chroma
    mPerez_x[2] *= invOvercast;
    mPerez_y[2] *= invOvercast;
    mPerez_Y[2] *= invOvercast;
    mPerez_x[4] *= invOvercast;
    mPerez_y[4] *= invOvercast;
    mPerez_Y[4] *= invOvercast;

    // lerp towards a fit of the CIE cloudy sky model: 4, -0.7
    mPerez_Y[0] = lerp(mPerez_Y[0], 4.0f, overcast);
    mPerez_Y[1] = lerp(mPerez_Y[1], -0.7f, overcast);

    // lerp base colour towards white point
    mZenith.x() = mZenith.x() * invOvercast + 0.333f * overcast;
    mZenith.y() = mZenith.y() * invOvercast + 0.333f * overcast;
  }

  if (horizCrush != 0.0f) {
    // The Preetham sky model has a "muddy" horizon, which can be objectionable
    // in typical game views. We allow artistic control over it.
    mPerez_Y[1] *= horizCrush;
    mPerez_x[1] *= horizCrush;
    mPerez_y[1] *= horizCrush;
  }

  // initialize sun-constant parts of the Perez functions

  math::vector3 perezLower // denominator terms are dependent on sun angle only
      (PerezLower(mPerez_x, cosTheta, theta),
       PerezLower(mPerez_y, cosTheta, theta),
       PerezLower(mPerez_Y, cosTheta, theta));

  mPerezInvDen = mZenith.array() / perezLower.array();
}

math::vector3 preetham_sun_sky::sky_rgb(const math::vector3 &v) const {
  math::vector3 rgb = xyYToRGB(sky_xyY(v));
  // linear rgb to non-linear rgb (srgb)
  rgb = 5e-5f * rgb;
  // gamma correction
  rgb.x() = powf(rgb.x(), 1.0f / 2.2f);
  rgb.y() = powf(rgb.y(), 1.0f / 2.2f);
  rgb.z() = powf(rgb.z(), 1.0f / 2.2f);
  return rgb;
}

math::vector3 preetham_sun_sky::sky_xyY(const math::vector3 &v) const {
  float cosTheta = v.z();
  float cosGamma = mToSun.dot(v);
  float gamma = acosf(cosGamma);

  if (cosTheta < 0.0f)
    cosTheta = 0.0f;

  math::vector3 xyY(PerezUpper(mPerez_x, cosTheta, gamma, cosGamma),
                    PerezUpper(mPerez_y, cosTheta, gamma, cosGamma),
                    PerezUpper(mPerez_Y, cosTheta, gamma, cosGamma));
  xyY = xyY.array() * mPerezInvDen.array();

  return xyY;
}

std::string bg_vs = R"(
#version 430 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec2 aTexCoord;
out vec2 texcoord;
void main() {
  texcoord = aTexCoord;
  gl_Position = vec4(aPos, 1.0);
}
)";
std::string bg_fs = R"(
#version 430 core
in vec2 texcoord;
out vec4 fragcolor;

const vec3 kXYZToR = vec3(3.2404542, -1.5371385, -0.4985314);
const vec3 kXYZToG = vec3(-0.9692660, 1.8760108, 0.0415560);
const vec3 kXYZToB = vec3(0.0556434, -0.2040259, 1.0572252);

const vec3 kRGBToX = vec3(0.4124564, 0.3575761, 0.1804375);
const vec3 kRGBToY = vec3(0.2126729, 0.7151522, 0.0721750);
const vec3 kRGBToZ = vec3(0.0193339, 0.1191920, 0.9503041);

vec3 xyYToXYZ(vec3 c) {
  return vec3(c.x, c.y, 1.0 - c.x - c.y) * c.z / c.y;
}
vec3 xyYToRGB(vec3 xyY) {
  vec3 XYZ = xyYToXYZ(xyY);
  return vec3(dot(kXYZToR,XYZ), dot(kXYZToG,XYZ), dot(kXYZToB,XYZ));
}

float PerezUpper(const float lambdas[5], float cosTheta, float gamma,
                        float cosGamma) {
  return (1.0 + lambdas[0] * exp(lambdas[1] / (cosTheta + 1e-6))) *
         (1.0 + lambdas[2] * exp(lambdas[3] * gamma) +
          lambdas[4] * cosGamma * cosGamma);
}

uniform vec3 view_pos;
uniform vec3 mToSun;
uniform vec3 mPerezInvDen;
uniform mat4 vp;

uniform float mPerez_x[5];
uniform float mPerez_y[5];
uniform float mPerez_Y[5];

void main() {
  vec4 ndc_pos = vec4(texcoord.x, 1.0-texcoord.y, 0.0, 1.0);
  vec4 p0 = inverse(vp) * ndc_pos;
  vec3 p1 = vec3(p0.x/p0.w,p0.y/p0.w,p0.z/p0.w);
  vec3 v = normalize(p1-view_pos);
  // axes conversion
  v = vec3(v.z,v.x,v.y);

  float cosTheta = v.z;
  float cosGamma = dot(mToSun,v);
  float gamma = acos(cosGamma);

  if (cosTheta < 0.0)
    cosTheta = 0.0;

  vec3 xyY = vec3(PerezUpper(mPerez_x, cosTheta, gamma, cosGamma),
                  PerezUpper(mPerez_y, cosTheta, gamma, cosGamma),
                  PerezUpper(mPerez_Y, cosTheta, gamma, cosGamma));
  xyY = xyY * mPerezInvDen;

  vec3 rgb = xyYToRGB(xyY);
  // linear rgb to non-linear rgb (srgb)
  rgb = 5e-5 * rgb;
  // gamma correction
  rgb.x = clamp(pow(rgb.x, 1.0 / 2.2),0.0,1.0);
  rgb.y = clamp(pow(rgb.y, 1.0 / 2.2),0.0,1.0);
  rgb.z = clamp(pow(rgb.z, 1.0 / 2.2),0.0,1.0);

  // fragcolor = vec4(texcoord.x, texcoord.y, 0.0, 1.0);
  fragcolor = vec4(rgb,1.0);
}
)";
void preetham_sun_sky::render(math::matrix4 &vp, math::vector3 view_pos) {
  static shader program;
  static bool initialized = false;
  if (!initialized) {
    initialized = true;
    program.compile_shader_from_source(bg_vs, bg_fs);
  }
  program.use();
  program.set_mat4("vp", vp);
  program.set_vec3("view_pos", view_pos);
  program.set_vec3("mToSun", mToSun);
  program.set_vec3("mPerezInvDen", mPerezInvDen);
  glUniform1fv(glGetUniformLocation(program.gl_handle, "mPerez_x"), 5,
               mPerez_x);
  glUniform1fv(glGetUniformLocation(program.gl_handle, "mPerez_y"), 5,
               mPerez_y);
  glUniform1fv(glGetUniformLocation(program.gl_handle, "mPerez_Y"), 5,
               mPerez_Y);
  quad_draw_call();
}

}; // namespace toolkit::opengl