#pragma once

#include "toolkit/math.hpp"
#include "toolkit/system.hpp"
#include "toolkit/transform.hpp"
#include "toolkit/utils.hpp"

#include <SDL_image.h>
#include <SDL_mixer.h>
#include <SDL_ttf.h>

#include <imgui.h>
#include <backends/imgui_impl_sdl2.h>
#include <backends/imgui_impl_sdlrenderer2.h>

namespace toolkit::math {

math::matrix2 from_angle(float angle);

}; // namespace toolkit::math
