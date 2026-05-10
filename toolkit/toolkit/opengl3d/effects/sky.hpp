#pragma once

#include "toolkit/opengl3d/base.hpp"

namespace toolkit::opengl3d {

struct atmosphere_settings {
  bool enabled = true;
  bool enable_sun_disk = true;
  bool enable_environment_map = true;
  bool auto_update_environment_map = true;
  bool enable_aerial_perspective = true;
  bool enable_volumetric_fog = true;

  int view_samples = 12;
  int light_samples = 6;
  int environment_resolution = 128;

  float planet_radius_km = 6360.0f;
  float atmosphere_height_km = 100.0f;
  float observer_height_km = 0.02f;
  float world_to_km = 0.001f;
  float ground_level = 0.0f;

  float rayleigh_strength = 1.0f;
  float mie_strength = 1.0f;
  float ozone_strength = 1.0f;
  float rayleigh_scale_height_km = 8.0f;
  float mie_scale_height_km = 1.2f;
  float ozone_center_height_km = 25.0f;
  float ozone_width_km = 15.0f;
  float mie_anisotropy = 0.76f;

  float sun_intensity = 22.0f;
  float sun_angular_radius = 0.004675f;
  float exposure = 1.0f;
  float gamma = 2.2f;
  float ground_albedo = 0.05f;

  float aerial_perspective_max_distance = 200000.0f;

  float fog_density = 0.0015f;
  float fog_height_falloff = 0.02f;
  float fog_anisotropy = 0.35f;
  float fog_sun_strength = 1.0f;
  float fog_ambient_strength = 0.65f;
  math::vector3 fog_tint = math::vector3(1.0f, 1.0f, 1.0f);
};
REFLECT(atmosphere_settings, enabled, enable_sun_disk,
        enable_environment_map, auto_update_environment_map,
        enable_aerial_perspective, enable_volumetric_fog, view_samples,
        light_samples, environment_resolution, planet_radius_km,
        atmosphere_height_km, observer_height_km, world_to_km, ground_level,
        rayleigh_strength, mie_strength, ozone_strength,
        rayleigh_scale_height_km, mie_scale_height_km,
        ozone_center_height_km, ozone_width_km, mie_anisotropy, sun_intensity,
        sun_angular_radius, exposure, gamma, ground_albedo,
        aerial_perspective_max_distance, fog_density, fog_height_falloff,
        fog_anisotropy, fog_sun_strength, fog_ambient_strength, fog_tint)

const std::string &physical_atmosphere_glsl();

/**
 * Single-scattering physical atmosphere renderer with an HDR cubemap output.
 */
class physical_atmosphere_sky {
public:
  physical_atmosphere_sky();

  void update(const math::vector3 &sun, float turbidity, float overcast = 0.0f,
              float hCrush = 0.0f);
  void set_settings(const atmosphere_settings &new_settings);
  void mark_environment_dirty();

  // render sun sky as background on a screen size quad
  void render(const math::matrix4 &vp, const math::vector3 &view_pos);
  void update_environment_map(const math::vector3 &view_pos,
                              bool force_update = false);

  void setup_uniforms(shader &program) const;
  const texture &environment_map() const { return environment_cubemap; }

  // rgb color ranges [0.0,1.0+]
  math::vector3 sky_rgb(const math::vector3 &v) const;
  math::vector3 sky_xyY(const math::vector3 &v) const;

  atmosphere_settings settings;

private:
  void ensure_background_program();
  void ensure_environment_resources();

  math::vector3 mToSun = math::vector3(0.0f, 1.0f, 0.0f);
  float turbidity_scale = 1.0f;
  bool environment_dirty = true;

  shader background_program;
  shader environment_program;
  texture environment_cubemap;
  GLuint environment_fbo = 0;
  int current_environment_resolution = 0;
};

using preetham_sun_sky = physical_atmosphere_sky;

}; // namespace toolkit::opengl