#pragma once

#include "toolkit/system.hpp"
#include "toolkit/utils.hpp"
#include "toolkit/math.hpp"

#include <SDL_image.h>
#include <SDL_mixer.h>
#include <SDL_ttf.h>

#include <imgui.h>
#include <backends/imgui_impl_sdl2.h>
#include <backends/imgui_impl_sdlrenderer2.h>

namespace toolkit::sdl2d {

math::matrix2 from_angle(float angle);

struct transform2d : public icomponent {
  transform2d() { reset(); }
  math::vector2 position, scale;
  float rotation;
  void reset();
};
DECLARE_COMPONENT(transform2d, basic, position, rotation, scale)

}; // namespace toolkit::sdl2d
