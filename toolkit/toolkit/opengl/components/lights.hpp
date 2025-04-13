#pragma once

#include "toolkit/opengl/base.hpp"
#include "toolkit/system.hpp"

namespace toolkit::opengl {

struct dir_light {
  math::vector3 dir = math::world_forward;
  math::vector3 color = White;
};
DECLARE_COMPONENT(dir_light, graphics, dir, color)

struct point_light {
  math::vector3 color = White;
};
DECLARE_COMPONENT(point_light, graphics, color)

}; // namespace toolkit::opengl