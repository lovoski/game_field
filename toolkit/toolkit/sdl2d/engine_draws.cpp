#include "toolkit/sdl2d/engine.hpp"

namespace toolkit::sdl2d {

// --- 2D drawing helper implementations ---
inline Uint8 to_u8(float v) {
  if (v <= 0.0f)
    return 0;
  if (v >= 1.0f)
    return 255;
  return static_cast<Uint8>(v * 255.0f);
}

void engine2d::ss_draw_point(int x, int y, const math::vector4 &color) {
  Uint8 r = to_u8(color.x());
  Uint8 g = to_u8(color.y());
  Uint8 b = to_u8(color.z());
  Uint8 a = to_u8(color.w());
  Uint8 old_r, old_g, old_b, old_a;
  SDL_GetRenderDrawColor(renderer, &old_r, &old_g, &old_b, &old_a);
  SDL_SetRenderDrawColor(renderer, r, g, b, a);
  SDL_RenderDrawPoint(renderer, x, y);
  SDL_SetRenderDrawColor(renderer, old_r, old_g, old_b, old_a);
}

void engine2d::ss_draw_points(const std::vector<math::vector2> &points,
                              const math::vector4 &color) {
  if (points.empty())
    return;
  Uint8 r = to_u8(color.x());
  Uint8 g = to_u8(color.y());
  Uint8 b = to_u8(color.z());
  Uint8 a = to_u8(color.w());
  Uint8 old_r, old_g, old_b, old_a;
  SDL_GetRenderDrawColor(renderer, &old_r, &old_g, &old_b, &old_a);
  SDL_SetRenderDrawColor(renderer, r, g, b, a);
  std::vector<SDL_Point> pts;
  pts.reserve(points.size());
  for (auto &p : points)
    pts.push_back(SDL_Point{static_cast<int>(p.x()), static_cast<int>(p.y())});
  SDL_RenderDrawPoints(renderer, pts.data(), static_cast<int>(pts.size()));
  SDL_SetRenderDrawColor(renderer, old_r, old_g, old_b, old_a);
}

void engine2d::ss_draw_line(int x1, int y1, int x2, int y2,
                            const math::vector4 &color) {
  Uint8 r = to_u8(color.x());
  Uint8 g = to_u8(color.y());
  Uint8 b = to_u8(color.z());
  Uint8 a = to_u8(color.w());
  Uint8 old_r, old_g, old_b, old_a;
  SDL_GetRenderDrawColor(renderer, &old_r, &old_g, &old_b, &old_a);
  SDL_SetRenderDrawColor(renderer, r, g, b, a);
  SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
  SDL_SetRenderDrawColor(renderer, old_r, old_g, old_b, old_a);
}

void engine2d::ss_draw_lines(const std::vector<math::vector2> &points,
                             bool closed, const math::vector4 &color) {
  if (points.size() < 2)
    return;
  Uint8 r = to_u8(color.x());
  Uint8 g = to_u8(color.y());
  Uint8 b = to_u8(color.z());
  Uint8 a = to_u8(color.w());
  Uint8 old_r, old_g, old_b, old_a;
  SDL_GetRenderDrawColor(renderer, &old_r, &old_g, &old_b, &old_a);
  SDL_SetRenderDrawColor(renderer, r, g, b, a);

  for (size_t i = 0; i + 1 < points.size(); ++i) {
    const auto &p0 = points[i];
    const auto &p1 = points[i + 1];
    SDL_RenderDrawLine(renderer, static_cast<int>(p0.x()),
                       static_cast<int>(p0.y()), static_cast<int>(p1.x()),
                       static_cast<int>(p1.y()));
  }
  if (closed) {
    const auto &p0 = points.back();
    const auto &p1 = points.front();
    SDL_RenderDrawLine(renderer, static_cast<int>(p0.x()),
                       static_cast<int>(p0.y()), static_cast<int>(p1.x()),
                       static_cast<int>(p1.y()));
  }

  SDL_SetRenderDrawColor(renderer, old_r, old_g, old_b, old_a);
}

void engine2d::ss_draw_rectangle(int x, int y, int w, int h, bool filled,
                                 const math::vector4 &color) {
  Uint8 r = to_u8(color.x());
  Uint8 g = to_u8(color.y());
  Uint8 b = to_u8(color.z());
  Uint8 a = to_u8(color.w());
  Uint8 old_r, old_g, old_b, old_a;
  SDL_GetRenderDrawColor(renderer, &old_r, &old_g, &old_b, &old_a);
  SDL_SetRenderDrawColor(renderer, r, g, b, a);
  SDL_Rect rct{x, y, w, h};
  if (filled)
    SDL_RenderFillRect(renderer, &rct);
  else
    SDL_RenderDrawRect(renderer, &rct);
  SDL_SetRenderDrawColor(renderer, old_r, old_g, old_b, old_a);
}

void engine2d::ss_draw_circle(int x, int y, int radius, bool filled,
                              const math::vector4 &color) {
  Uint8 rr = to_u8(color.x());
  Uint8 gg = to_u8(color.y());
  Uint8 bb = to_u8(color.z());
  Uint8 aa = to_u8(color.w());
  Uint8 old_r, old_g, old_b, old_a;
  SDL_GetRenderDrawColor(renderer, &old_r, &old_g, &old_b, &old_a);
  SDL_SetRenderDrawColor(renderer, rr, gg, bb, aa);

  // Sanity checks
  if (radius <= 0) {
    // nothing to draw
    SDL_SetRenderDrawColor(renderer, old_r, old_g, old_b, old_a);
    return;
  }

  if (filled) {
    // Allocate vertex array
    const int segments = 48;
    std::vector<SDL_Vertex> verts;
    verts.reserve(segments + 2);
    // Center vertex (index 0)
    SDL_Vertex center;
    center.position = {static_cast<float>(x), static_cast<float>(y)};
    center.color = {rr, gg, bb, aa};
    center.tex_coord = {0, 0};
    verts.push_back(center);
    // Perimeter vertices (indices 1 to segments+1)
    const float step = 2.0f * M_PI / segments;
    for (int i = 0; i <= segments; i++) {
      float angle = i * step;
      float px = x + radius * cosf(angle);
      float py = y + radius * sinf(angle);
      SDL_Vertex v;
      v.position = {px, py};
      v.color = {rr, gg, bb, aa};
      v.tex_coord = {0, 0};
      verts.push_back(v);
    }
    // Build triangle fan indices: for each segment, emit triangle (0, i+1, i+2)
    std::vector<int> indices;
    indices.reserve(segments * 3);
    for (int i = 0; i < segments; i++) {
      indices.push_back(0);           // center
      indices.push_back(i + 1);       // perimeter vertex i
      indices.push_back(i + 2);       // perimeter vertex i+1
    }
    // Render the triangle fan
    SDL_RenderGeometry(renderer, nullptr, verts.data(), verts.size(),
                       indices.data(), indices.size());
  } else {
    // Outline: use Bresenham / midpoint circle algorithm to draw a 1-pixel
    // wide circumference. This avoids trig and is fast and consistent.
    int cx = x;
    int cy = y;
    int r = radius;
    int dx = 0;
    int dy = r;
    int d = 1 - r; // decision parameter
    while (dx <= dy) {
      // plot the 8 symmetric points
      SDL_RenderDrawPoint(renderer, cx + dx, cy + dy);
      SDL_RenderDrawPoint(renderer, cx - dx, cy + dy);
      SDL_RenderDrawPoint(renderer, cx + dx, cy - dy);
      SDL_RenderDrawPoint(renderer, cx - dx, cy - dy);
      SDL_RenderDrawPoint(renderer, cx + dy, cy + dx);
      SDL_RenderDrawPoint(renderer, cx - dy, cy + dx);
      SDL_RenderDrawPoint(renderer, cx + dy, cy - dx);
      SDL_RenderDrawPoint(renderer, cx - dy, cy - dx);

      ++dx;
      if (d < 0) {
        d += 2 * dx + 1;
      } else {
        --dy;
        d += 2 * (dx - dy) + 1;
      }
    }
  }

  SDL_SetRenderDrawColor(renderer, old_r, old_g, old_b, old_a);
}

}; // namespace toolkit::sdl2d