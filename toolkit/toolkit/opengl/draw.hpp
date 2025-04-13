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

/**
 * Visualize a list of bones, with <start, end> pair.
 */
void draw_bones(std::vector<std::pair<math::vector3, math::vector3>> &bones,
                math::vector2 viewport, math::matrix4 vp,
                math::vector3 color = Green);

void quad_draw_call();

void draw_quads(std::vector<math::vector3> positions, math::vector3 right,
                math::vector3 up, math::matrix4 vp, float size = 0.2f,
                math::vector3 color = White);

/**
 * Taken from: https://theorangeduck.com/page/debug-draw-text-lines
 *
 * The parameter `location` is the position for the left most corner, by
 * default, texts are plot on the xy plane, facing +z axis.
 */
void draw_text3d(std::string text, math::vector3 location, math::quat rotation,
                 math::matrix4 vp, float thick = 0.0f, float scale = 1.0f,
                 float width = 1.0f, float height = 1.0f, float spacing = 0.0f,
                 float lineheight = 1.0f, math::vector3 color = White);

}; // namespace toolkit::opengl