#pragma once

#include "toolkit/opengl/base.hpp"

namespace toolkit::opengl {

/**
 * A Practical Analytic Model for Daylight
 */
class preetham_sun_sky {
public:
  void update(const math::vector3 &sun, float turbidity, float overcast = 0.0f,
              float hCrush = 0.0f);

  // render sun sky as background on a screen size quad
  void render(math::matrix4 &vp, math::vector3 view_pos);

  // rgb color ranges [0.0,1.0+]
  math::vector3 sky_rgb(const math::vector3 &v) const;
  math::vector3 sky_xyY(const math::vector3 &v) const;

  float mPerez_x[5];
  float mPerez_y[5];
  float mPerez_Y[5];

  math::vector3 mToSun = math::vector3(0, 1, 0);
  math::vector3 mZenith = math::vector3::Zero();
  math::vector3 mPerezInvDen = math::vector3::Ones();
};

}; // namespace toolkit::opengl