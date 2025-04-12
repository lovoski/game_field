#pragma once

#include "toolkit/opengl/base.hpp"

namespace toolkit::opengl {

void draw_lines(std::vector<std::pair<math::vector3, math::vector3>> &lines,
                math::matrix4 vp, math::vector3 color = White);

void draw_linestrip(std::vector<math::vector3> &lineStrip, math::matrix4 vp,
                    math::vector3 color = White);

void draw_grid(unsigned int gridSize, unsigned int gridSpacing,
               math::matrix4 mvp = math::matrix4::Identity(),
               math::vector3 color = White);

void draw_wire_sphere(math::vector3 position, math::matrix4 vp,
                      float radius = 1.0f, math::vector3 color = Green,
                      unsigned int segs = 10);

void draw_arrow(math::vector3 start, math::vector3 end, math::matrix4 vp,
                math::vector3 color = White, float size = 0.2f);

void draw_cube(math::vector3 position, math::vector3 forward,
               math::vector3 left, math::vector3 up, math::matrix4 vp,
               math::vector3 size, math::vector3 color = White);

// Visualize a list of bones, with <start, end> pair.
void draw_bones(std::vector<std::pair<math::vector3, math::vector3>> &bones,
                math::vector2 viewport, math::matrix4 vp,
                math::vector3 color = Green);

void quad_draw_call();

}; // namespace toolkit::opengl