#pragma once

#include "toolkit/opengl/base.hpp"

namespace toolkit::opengl {

void create_hemisphere_kernal(int kernal_size,
                              std::vector<math::vector4> &kernal);

void ao_box_filter(texture &input_tex, int kernal_size);
void ao_gaussian_filter(texture &input_tex, int kernal_size, float sigma = 2.0f);

/**
 * Screen space ambient occlusion.
 *
 * Reference:
 * https://john-chapman-graphics.blogspot.com/2013/01/ssao-tutorial.html
 */
void ssao(texture &gbuffer_pos, texture &gbuffer_normal, texture &gbuffer_mask,
          math::matrix4 &view, math::matrix4 &proj, float noise_scale,
          float radius);

}; // namespace toolkit::opengl