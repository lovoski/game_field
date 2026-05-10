#include "toolkit/opengl3d/effects/sky.hpp"
#include "toolkit/opengl3d/draw.hpp"

namespace toolkit::opengl3d {

namespace {

constexpr float kPi = 3.1415926535f;

math::vector3 base_rayleigh_beta() {
  return math::vector3(5.802e-3f, 13.558e-3f, 33.100e-3f);
}

math::vector3 base_mie_beta() {
  return math::vector3(3.996e-3f, 3.996e-3f, 3.996e-3f);
}

math::vector3 base_ozone_beta() {
  return math::vector3(0.650e-3f, 1.881e-3f, 0.085e-3f);
}

float safe_pow(float value, float power) {
  return std::pow(std::max(value, 0.0f), power);
}

} // namespace

const std::string &physical_atmosphere_glsl() {
  static const std::string source = R"(
const float ATMOSPHERE_PI = 3.14159265358979323846;
const int ATMOSPHERE_MAX_VIEW_STEPS = 64;
const int ATMOSPHERE_MAX_LIGHT_STEPS = 16;

uniform bool uAtmosphereEnabled;
uniform bool uAtmosphereSunDisk;
uniform bool uAtmosphereAerialPerspective;
uniform bool uVolumetricFogEnabled;

uniform int uAtmosphereViewSamples;
uniform int uAtmosphereLightSamples;

uniform vec3 uAtmosphereSunDir;
uniform vec3 uAtmosphereRayleighBeta;
uniform vec3 uAtmosphereMieBeta;
uniform vec3 uAtmosphereOzoneBeta;
uniform vec3 uVolumetricFogTint;

uniform float uAtmospherePlanetRadius;
uniform float uAtmosphereRadius;
uniform float uAtmosphereRayleighHeight;
uniform float uAtmosphereMieHeight;
uniform float uAtmosphereOzoneCenter;
uniform float uAtmosphereOzoneWidth;
uniform float uAtmosphereMieG;
uniform float uAtmosphereSunIntensity;
uniform float uAtmosphereSunAngularRadius;
uniform float uAtmosphereExposure;
uniform float uAtmosphereGamma;
uniform float uAtmosphereObserverHeight;
uniform float uAtmosphereWorldToKm;
uniform float uAtmosphereGroundLevel;
uniform float uAtmosphereGroundAlbedo;
uniform float uAtmosphereAerialMaxDistance;
uniform float uVolumetricFogDensity;
uniform float uVolumetricFogHeightFalloff;
uniform float uVolumetricFogAnisotropy;
uniform float uVolumetricFogSunStrength;
uniform float uVolumetricFogAmbientStrength;

struct AtmosphereResult {
  vec3 radiance;
  vec3 transmittance;
};

vec2 atmosphere_ray_sphere(vec3 ro, vec3 rd, float radius) {
  float b = dot(ro, rd);
  float c = dot(ro, ro) - radius * radius;
  float h = b * b - c;
  if (h < 0.0) {
    return vec2(1e20, -1e20);
  }
  h = sqrt(h);
  return vec2(-b - h, -b + h);
}

vec3 atmosphere_camera_origin(vec3 world_pos) {
  float height = max(world_pos.y - uAtmosphereGroundLevel, 0.0) *
                 uAtmosphereWorldToKm + uAtmosphereObserverHeight;
  return vec3(0.0, uAtmospherePlanetRadius + height, 0.0);
}

float atmosphere_rayleigh_phase(float mu) {
  return 3.0 * (1.0 + mu * mu) / (16.0 * ATMOSPHERE_PI);
}

float atmosphere_mie_phase(float mu) {
  float g = clamp(uAtmosphereMieG, -0.95, 0.95);
  float g2 = g * g;
  float denom = pow(max(1.0 + g2 - 2.0 * g * mu, 1e-3), 1.5);
  return 3.0 * (1.0 - g2) * (1.0 + mu * mu) /
         (8.0 * ATMOSPHERE_PI * (2.0 + g2) * denom);
}

float atmosphere_ozone_density(float height) {
  float width = max(uAtmosphereOzoneWidth, 1e-3);
  return max(1.0 - abs(height - uAtmosphereOzoneCenter) / width, 0.0);
}

vec3 atmosphere_extinction(vec3 optical_rayleigh, vec3 optical_mie,
                           vec3 optical_ozone) {
  vec3 tau = optical_rayleigh * uAtmosphereRayleighBeta +
             optical_mie * uAtmosphereMieBeta +
             optical_ozone * uAtmosphereOzoneBeta;
  return exp(-tau);
}

AtmosphereResult atmosphere_integrate(vec3 ro, vec3 rd, float max_dist_km,
                                      bool include_sun_disk) {
  AtmosphereResult result;
  result.radiance = vec3(0.0);
  result.transmittance = vec3(1.0);

  if (!uAtmosphereEnabled) {
    return result;
  }

  vec2 atmosphere_hit = atmosphere_ray_sphere(ro, rd, uAtmosphereRadius);
  if (atmosphere_hit.y <= 0.0) {
    return result;
  }

  float t0 = max(atmosphere_hit.x, 0.0);
  float t1 = atmosphere_hit.y;
  if (max_dist_km > 0.0) {
    t1 = min(t1, max_dist_km);
  }

  vec2 ground_hit = atmosphere_ray_sphere(ro, rd, uAtmospherePlanetRadius);
  bool hit_ground = ground_hit.x > 0.0;
  if (hit_ground) {
    t1 = min(t1, ground_hit.x);
  }

  if (t1 <= t0) {
    return result;
  }

  int view_steps = clamp(uAtmosphereViewSamples, 2, ATMOSPHERE_MAX_VIEW_STEPS);
  int light_steps = clamp(uAtmosphereLightSamples, 2, ATMOSPHERE_MAX_LIGHT_STEPS);
  float segment_length = (t1 - t0) / float(view_steps);
  vec3 sun_dir = normalize(uAtmosphereSunDir);
  float mu = dot(rd, sun_dir);
  float rayleigh_phase = atmosphere_rayleigh_phase(mu);
  float mie_phase = atmosphere_mie_phase(mu);

  vec3 optical_rayleigh = vec3(0.0);
  vec3 optical_mie = vec3(0.0);
  vec3 optical_ozone = vec3(0.0);

  for (int i = 0; i < ATMOSPHERE_MAX_VIEW_STEPS; ++i) {
    if (i >= view_steps) {
      break;
    }

    float t = t0 + (float(i) + 0.5) * segment_length;
    vec3 sample_pos = ro + rd * t;
    float height = max(length(sample_pos) - uAtmospherePlanetRadius, 0.0);

    float rayleigh_density = exp(-height / max(uAtmosphereRayleighHeight, 1e-3));
    float mie_density = exp(-height / max(uAtmosphereMieHeight, 1e-3));
    float ozone_density = atmosphere_ozone_density(height);

    vec2 sun_hit = atmosphere_ray_sphere(sample_pos, sun_dir, uAtmosphereRadius);
    float sun_segment_length = max(sun_hit.y, 0.0) / float(light_steps);
    vec3 sun_optical_rayleigh = vec3(0.0);
    vec3 sun_optical_mie = vec3(0.0);
    vec3 sun_optical_ozone = vec3(0.0);

    for (int j = 0; j < ATMOSPHERE_MAX_LIGHT_STEPS; ++j) {
      if (j >= light_steps) {
        break;
      }

      float sun_t = (float(j) + 0.5) * sun_segment_length;
      vec3 sun_sample_pos = sample_pos + sun_dir * sun_t;
      float sun_height = max(length(sun_sample_pos) - uAtmospherePlanetRadius, 0.0);

      sun_optical_rayleigh += vec3(exp(-sun_height / max(uAtmosphereRayleighHeight, 1e-3)) * sun_segment_length);
      sun_optical_mie += vec3(exp(-sun_height / max(uAtmosphereMieHeight, 1e-3)) * sun_segment_length);
      sun_optical_ozone += vec3(atmosphere_ozone_density(sun_height) * sun_segment_length);
    }

    vec3 optical_to_sample_rayleigh = optical_rayleigh + vec3(rayleigh_density * segment_length * 0.5);
    vec3 optical_to_sample_mie = optical_mie + vec3(mie_density * segment_length * 0.5);
    vec3 optical_to_sample_ozone = optical_ozone + vec3(ozone_density * segment_length * 0.5);

    vec3 transmittance = atmosphere_extinction(
        optical_to_sample_rayleigh + sun_optical_rayleigh,
        optical_to_sample_mie + sun_optical_mie,
        optical_to_sample_ozone + sun_optical_ozone);

    vec3 scattering = rayleigh_density * uAtmosphereRayleighBeta * rayleigh_phase +
                      mie_density * uAtmosphereMieBeta * mie_phase;
    result.radiance += transmittance * scattering *
                       (uAtmosphereSunIntensity * segment_length);

    optical_rayleigh += vec3(rayleigh_density * segment_length);
    optical_mie += vec3(mie_density * segment_length);
    optical_ozone += vec3(ozone_density * segment_length);
  }

  result.transmittance = atmosphere_extinction(optical_rayleigh, optical_mie,
                                               optical_ozone);

  if (hit_ground && max_dist_km <= 0.0) {
    vec3 ground_pos = ro + rd * t1;
    vec3 ground_normal = normalize(ground_pos);
    float ndotl = max(dot(ground_normal, sun_dir), 0.0);
    result.radiance += result.transmittance *
                       vec3(uAtmosphereGroundAlbedo * ndotl *
                            uAtmosphereSunIntensity / ATMOSPHERE_PI);
  }

  if (include_sun_disk && uAtmosphereSunDisk && !hit_ground) {
    float cos_radius = cos(uAtmosphereSunAngularRadius);
    float disk = smoothstep(cos_radius, min(cos_radius + 0.00003, 1.0), mu);
    result.radiance += result.transmittance * vec3(uAtmosphereSunIntensity) * disk;
  }

  return result;
}

vec3 atmosphere_tonemap(vec3 radiance) {
  vec3 mapped = vec3(1.0) - exp(-max(radiance, vec3(0.0)) * uAtmosphereExposure);
  return pow(clamp(mapped, vec3(0.0), vec3(1.0)),
             vec3(1.0 / max(uAtmosphereGamma, 1e-3)));
}

vec3 atmosphere_render_sky(vec3 world_view_pos, vec3 ray_dir) {
  AtmosphereResult atmosphere = atmosphere_integrate(
      atmosphere_camera_origin(world_view_pos), normalize(ray_dir), -1.0,
      true);
  return atmosphere_tonemap(atmosphere.radiance);
}

vec3 atmosphere_render_sky_hdr(vec3 world_view_pos, vec3 ray_dir) {
  AtmosphereResult atmosphere = atmosphere_integrate(
      atmosphere_camera_origin(world_view_pos), normalize(ray_dir), -1.0,
      true);
  return max(atmosphere.radiance, vec3(0.0));
}

vec3 atmosphere_apply_to_scene(vec3 scene_color, vec3 world_view_pos,
                               vec3 world_pos) {
  vec3 ray = world_pos - world_view_pos;
  float world_distance = length(ray);
  if (world_distance <= 1e-4) {
    return scene_color;
  }

  vec3 ray_dir = ray / world_distance;
  vec3 result = scene_color;

  if (uAtmosphereEnabled && uAtmosphereAerialPerspective) {
    float finite_distance = min(world_distance, uAtmosphereAerialMaxDistance) *
                            uAtmosphereWorldToKm;
    AtmosphereResult aerial = atmosphere_integrate(
        atmosphere_camera_origin(world_view_pos), ray_dir, finite_distance,
        false);
    result = result * aerial.transmittance + atmosphere_tonemap(aerial.radiance);
  }

  if (uVolumetricFogEnabled && uVolumetricFogDensity > 0.0) {
    float mid_height = max((world_view_pos.y + world_pos.y) * 0.5 -
                           uAtmosphereGroundLevel, 0.0);
    float height_density = exp(-mid_height * max(uVolumetricFogHeightFalloff, 0.0));
    float fog_amount = 1.0 - exp(-uVolumetricFogDensity * world_distance *
                                 height_density);
    fog_amount = clamp(fog_amount, 0.0, 1.0);

    float mu = dot(ray_dir, normalize(uAtmosphereSunDir));
    float fog_phase = mix(atmosphere_rayleigh_phase(mu),
                          atmosphere_mie_phase(mu),
                          clamp(uVolumetricFogAnisotropy, 0.0, 1.0));
    float horizon_weight = clamp(1.0 - abs(ray_dir.y), 0.0, 1.0);
    vec3 horizon_sky = atmosphere_tonemap(
      (uAtmosphereRayleighBeta * (0.35 + 0.65 * max(ray_dir.y, 0.0)) +
       uAtmosphereMieBeta * (0.8 + 2.5 * pow(max(mu, 0.0), 8.0))) *
      uAtmosphereSunIntensity * (1.0 + 1.5 * horizon_weight));
    vec3 sun_scatter = atmosphere_tonemap(
        vec3(uAtmosphereSunIntensity * fog_phase * uVolumetricFogSunStrength));
    vec3 fog_color = uVolumetricFogTint *
                     (horizon_sky * uVolumetricFogAmbientStrength + sun_scatter);
    result = mix(result, fog_color, fog_amount);
  }

  return result;
}
  )";
  return source;
}

std::string atmosphere_bg_vs = R"(
#version 430 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec2 aTexCoord;
out vec2 texcoord;
void main() {
  texcoord = aTexCoord;
  gl_Position = vec4(aPos, 1.0);
}
)";

std::string atmosphere_bg_fs = std::string(R"(
#version 430 core
in vec2 texcoord;
out vec4 fragcolor;

uniform vec3 view_pos;
uniform mat4 vp;

)") + physical_atmosphere_glsl() + R"(

void main() {
  vec4 ndc_pos = vec4(2.0 * texcoord.x - 1.0, 2.0 * texcoord.y - 1.0, 0.0, 1.0);
  vec4 p0 = inverse(vp) * ndc_pos;
  vec3 p1 = p0.xyz / p0.w;
  vec3 ray_dir = normalize(p1 - view_pos);
  fragcolor = vec4(atmosphere_render_sky(view_pos, ray_dir), 1.0);
}
)";

std::string atmosphere_env_fs = std::string(R"(
#version 430 core
in vec2 texcoord;
out vec4 fragcolor;

uniform vec3 view_pos;
uniform int cube_face;

)") + physical_atmosphere_glsl() + R"(

vec3 cube_face_direction(int face, vec2 uv) {
  vec2 p = uv * 2.0 - 1.0;
  if (face == 0) return normalize(vec3( 1.0, -p.y, -p.x));
  if (face == 1) return normalize(vec3(-1.0, -p.y,  p.x));
  if (face == 2) return normalize(vec3( p.x,  1.0,  p.y));
  if (face == 3) return normalize(vec3( p.x, -1.0, -p.y));
  if (face == 4) return normalize(vec3( p.x, -p.y,  1.0));
  return normalize(vec3(-p.x, -p.y, -1.0));
}

void main() {
  vec3 ray_dir = cube_face_direction(cube_face, texcoord);
  fragcolor = vec4(atmosphere_render_sky_hdr(view_pos, ray_dir), 1.0);
}
)";

physical_atmosphere_sky::physical_atmosphere_sky() {}

void physical_atmosphere_sky::set_settings(
    const atmosphere_settings &new_settings) {
  settings = new_settings;
  mark_environment_dirty();
}

void physical_atmosphere_sky::mark_environment_dirty() {
  environment_dirty = true;
}

void physical_atmosphere_sky::update(const math::vector3 &sun, float turbidity,
                                     float, float) {
  math::vector3 normalized_sun = sun.normalized();
  float new_turbidity_scale = std::max(turbidity, 0.05f) / 2.5f;
  if ((normalized_sun - mToSun).norm() > 1e-4f ||
      std::abs(new_turbidity_scale - turbidity_scale) > 1e-4f) {
    mToSun = normalized_sun;
    turbidity_scale = new_turbidity_scale;
    mark_environment_dirty();
  }
}

void physical_atmosphere_sky::ensure_background_program() {
  if (background_program.gl_handle == 0) {
    background_program.compile_shader_from_source(atmosphere_bg_vs,
                                                  atmosphere_bg_fs);
  }
}

void physical_atmosphere_sky::ensure_environment_resources() {
  if (!environment_cubemap.inited()) {
    environment_cubemap.create(GL_TEXTURE_CUBE_MAP);
  }
  if (environment_fbo == 0) {
    glGenFramebuffers(1, &environment_fbo);
  }
  if (environment_program.gl_handle == 0) {
    environment_program.compile_shader_from_source(atmosphere_bg_vs,
                                                   atmosphere_env_fs);
  }

  int resolution = std::clamp(settings.environment_resolution, 16, 1024);
  if (current_environment_resolution == resolution) {
    return;
  }

  current_environment_resolution = resolution;
  environment_cubemap.bind();
  for (int face = 0; face < 6; ++face) {
    glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + face, 0, GL_RGBA16F,
                 resolution, resolution, 0, GL_RGBA, GL_FLOAT, nullptr);
  }
  glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER,
                  GL_LINEAR_MIPMAP_LINEAR);
  glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
  glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
  mark_environment_dirty();
}

void physical_atmosphere_sky::render(const math::matrix4 &vp,
                                     const math::vector3 &view_pos) {
  ensure_background_program();
  background_program.use();
  background_program.set_mat4("vp", vp);
  background_program.set_vec3("view_pos", view_pos);
  setup_uniforms(background_program);
  quad_draw_call();
}

void physical_atmosphere_sky::update_environment_map(
    const math::vector3 &view_pos, bool force_update) {
  if (!settings.enable_environment_map) {
    return;
  }

  ensure_environment_resources();
  if (!force_update && !environment_dirty) {
    return;
  }

  GLint previous_fbo = 0;
  GLint previous_viewport[4] = {0, 0, 0, 0};
  GLboolean previous_depth_test = glIsEnabled(GL_DEPTH_TEST);
  glGetIntegerv(GL_FRAMEBUFFER_BINDING, &previous_fbo);
  glGetIntegerv(GL_VIEWPORT, previous_viewport);

  glBindFramebuffer(GL_FRAMEBUFFER, environment_fbo);
  glViewport(0, 0, current_environment_resolution,
             current_environment_resolution);
  glDisable(GL_DEPTH_TEST);

  environment_program.use();
  environment_program.set_vec3("view_pos", view_pos);
  setup_uniforms(environment_program);

  for (int face = 0; face < 6; ++face) {
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                           GL_TEXTURE_CUBE_MAP_POSITIVE_X + face,
                           environment_cubemap.get_handle(), 0);
    glDrawBuffer(GL_COLOR_ATTACHMENT0);
    glClear(GL_COLOR_BUFFER_BIT);
    environment_program.set_int("cube_face", face);
    quad_draw_call();
  }

  environment_cubemap.bind();
  glGenerateMipmap(GL_TEXTURE_CUBE_MAP);
  glBindTexture(GL_TEXTURE_CUBE_MAP, 0);

  glBindFramebuffer(GL_FRAMEBUFFER, previous_fbo);
  glViewport(previous_viewport[0], previous_viewport[1], previous_viewport[2],
             previous_viewport[3]);
  if (previous_depth_test) {
    glEnable(GL_DEPTH_TEST);
  }
  environment_dirty = false;
}

void physical_atmosphere_sky::setup_uniforms(shader &program) const {
  atmosphere_settings clamped = settings;
  clamped.view_samples = std::clamp(clamped.view_samples, 2, 64);
  clamped.light_samples = std::clamp(clamped.light_samples, 2, 16);
  clamped.planet_radius_km = std::max(clamped.planet_radius_km, 1.0f);
  clamped.atmosphere_height_km = std::max(clamped.atmosphere_height_km, 1.0f);
  clamped.world_to_km = std::max(clamped.world_to_km, 1e-6f);

  math::vector3 rayleigh_beta = base_rayleigh_beta() * clamped.rayleigh_strength;
  math::vector3 mie_beta = base_mie_beta() * clamped.mie_strength * turbidity_scale;
  math::vector3 ozone_beta = base_ozone_beta() * clamped.ozone_strength;

  program.set_bool("uAtmosphereEnabled", clamped.enabled)
      .set_bool("uAtmosphereSunDisk", clamped.enable_sun_disk)
      .set_bool("uAtmosphereAerialPerspective",
                clamped.enable_aerial_perspective)
      .set_bool("uVolumetricFogEnabled", clamped.enable_volumetric_fog)
      .set_int("uAtmosphereViewSamples", clamped.view_samples)
      .set_int("uAtmosphereLightSamples", clamped.light_samples)
      .set_vec3("uAtmosphereSunDir", mToSun)
      .set_vec3("uAtmosphereRayleighBeta", rayleigh_beta)
      .set_vec3("uAtmosphereMieBeta", mie_beta)
      .set_vec3("uAtmosphereOzoneBeta", ozone_beta)
      .set_vec3("uVolumetricFogTint", clamped.fog_tint)
      .set_float("uAtmospherePlanetRadius", clamped.planet_radius_km)
      .set_float("uAtmosphereRadius",
                 clamped.planet_radius_km + clamped.atmosphere_height_km)
      .set_float("uAtmosphereRayleighHeight",
                 clamped.rayleigh_scale_height_km)
      .set_float("uAtmosphereMieHeight", clamped.mie_scale_height_km)
      .set_float("uAtmosphereOzoneCenter", clamped.ozone_center_height_km)
      .set_float("uAtmosphereOzoneWidth", clamped.ozone_width_km)
      .set_float("uAtmosphereMieG", clamped.mie_anisotropy)
      .set_float("uAtmosphereSunIntensity", clamped.sun_intensity)
      .set_float("uAtmosphereSunAngularRadius", clamped.sun_angular_radius)
      .set_float("uAtmosphereExposure", clamped.exposure)
      .set_float("uAtmosphereGamma", clamped.gamma)
      .set_float("uAtmosphereObserverHeight", clamped.observer_height_km)
      .set_float("uAtmosphereWorldToKm", clamped.world_to_km)
      .set_float("uAtmosphereGroundLevel", clamped.ground_level)
      .set_float("uAtmosphereGroundAlbedo", clamped.ground_albedo)
      .set_float("uAtmosphereAerialMaxDistance",
                 clamped.aerial_perspective_max_distance)
      .set_float("uVolumetricFogDensity", clamped.fog_density)
      .set_float("uVolumetricFogHeightFalloff", clamped.fog_height_falloff)
      .set_float("uVolumetricFogAnisotropy", clamped.fog_anisotropy)
      .set_float("uVolumetricFogSunStrength", clamped.fog_sun_strength)
      .set_float("uVolumetricFogAmbientStrength",
                 clamped.fog_ambient_strength);
}

math::vector3 physical_atmosphere_sky::sky_rgb(const math::vector3 &v) const {
  float mu = std::clamp(v.normalized().dot(mToSun), -1.0f, 1.0f);
  float rayleigh_phase = 3.0f * (1.0f + mu * mu) / (16.0f * kPi);
  float mie_g = std::clamp(settings.mie_anisotropy, -0.95f, 0.95f);
  float mie_g2 = mie_g * mie_g;
  float mie_phase = 3.0f * (1.0f - mie_g2) * (1.0f + mu * mu) /
                    (8.0f * kPi * (2.0f + mie_g2) *
                     safe_pow(1.0f + mie_g2 - 2.0f * mie_g * mu, 1.5f));
  float horizon = std::clamp(0.5f + 0.5f * v.normalized().y(), 0.0f, 1.0f);
  math::vector3 radiance = base_rayleigh_beta() * settings.rayleigh_strength *
                               rayleigh_phase * (0.2f + 0.8f * horizon) +
                           base_mie_beta() * settings.mie_strength *
                               turbidity_scale * mie_phase;
  radiance *= settings.sun_intensity;
  return math::vector3(1.0f, 1.0f, 1.0f) -
         (-radiance * settings.exposure).array().exp().matrix();
}

math::vector3 physical_atmosphere_sky::sky_xyY(const math::vector3 &v) const {
  math::vector3 rgb = sky_rgb(v).cwiseMax(0.0f);
  float xyz_x = 0.4124564f * rgb.x() + 0.3575761f * rgb.y() +
                0.1804375f * rgb.z();
  float xyz_y = 0.2126729f * rgb.x() + 0.7151522f * rgb.y() +
                0.0721750f * rgb.z();
  float xyz_z = 0.0193339f * rgb.x() + 0.1191920f * rgb.y() +
                0.9503041f * rgb.z();
  float sum = std::max(xyz_x + xyz_y + xyz_z, 1e-6f);
  return math::vector3(xyz_x / sum, xyz_y / sum, xyz_y);
}

}; // namespace toolkit::opengl3d