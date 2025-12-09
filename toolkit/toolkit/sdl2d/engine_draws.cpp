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

void engine2d::ss_draw_point(int x, int y, const math::vector3 &color) {
  Uint8 r = to_u8(color.x());
  Uint8 g = to_u8(color.y());
  Uint8 b = to_u8(color.z());
  Uint8 a = 255;
  Uint8 old_r, old_g, old_b, old_a;
  SDL_GetRenderDrawColor(renderer, &old_r, &old_g, &old_b, &old_a);
  SDL_SetRenderDrawColor(renderer, r, g, b, a);
  SDL_RenderDrawPoint(renderer, x, y);
  SDL_SetRenderDrawColor(renderer, old_r, old_g, old_b, old_a);
}

void engine2d::ss_draw_points(const std::vector<math::vector2> &points,
                           const math::vector3 &color) {
  if (points.empty())
    return;
  Uint8 r = to_u8(color.x());
  Uint8 g = to_u8(color.y());
  Uint8 b = to_u8(color.z());
  Uint8 a = 255;
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
                         const math::vector3 &color) {
  Uint8 r = to_u8(color.x());
  Uint8 g = to_u8(color.y());
  Uint8 b = to_u8(color.z());
  Uint8 a = 255;
  Uint8 old_r, old_g, old_b, old_a;
  SDL_GetRenderDrawColor(renderer, &old_r, &old_g, &old_b, &old_a);
  SDL_SetRenderDrawColor(renderer, r, g, b, a);
  SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
  SDL_SetRenderDrawColor(renderer, old_r, old_g, old_b, old_a);
}

void engine2d::ss_draw_lines(const std::vector<math::vector2> &points, bool closed,
                          const math::vector3 &color) {
  if (points.size() < 2)
    return;
  Uint8 r = to_u8(color.x());
  Uint8 g = to_u8(color.y());
  Uint8 b = to_u8(color.z());
  Uint8 a = 255;
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
                              const math::vector3 &color) {
  Uint8 r = to_u8(color.x());
  Uint8 g = to_u8(color.y());
  Uint8 b = to_u8(color.z());
  Uint8 a = 255;
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
                           const math::vector3 &color) {
  Uint8 rr = to_u8(color.x());
  Uint8 gg = to_u8(color.y());
  Uint8 bb = to_u8(color.z());
  Uint8 aa = 255;
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
    // Filled circle using the midpoint algorithm to produce the same
    // symmetric extents as the outline Bresenham algorithm. For each
    // computed (dx,dy) we draw horizontal spans for the corresponding
    // symmetric scanlines. This avoids per-row sqrt and guarantees
    // consistency between filled and unfilled versions (no stray pixels).
    int cx = x;
    int cy = y;
    int dx = 0;
    int dy = radius;
    int d = 1 - radius;
    while (dx <= dy) {
      // draw horizontal spans for the 8 symmetric octants
      // span at y + dy: from cx - dx .. cx + dx
      SDL_RenderDrawLine(renderer, cx - dx, cy + dy, cx + dx, cy + dy);
      // span at y - dy
      SDL_RenderDrawLine(renderer, cx - dx, cy - dy, cx + dx, cy - dy);
      // span at y + dx (use dy as half-span)
      SDL_RenderDrawLine(renderer, cx - dy, cy + dx, cx + dy, cy + dx);
      // span at y - dx
      SDL_RenderDrawLine(renderer, cx - dy, cy - dx, cx + dy, cy - dx);

      ++dx;
      if (d < 0) {
        d += 2 * dx + 1;
      } else {
        --dy;
        d += 2 * (dx - dy) + 1;
      }
    }
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