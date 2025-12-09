#include "toolkit/sdl2d/engine.hpp"

namespace toolkit::sdl2d {

void engine2d::init(int width, int height) {
  // initialize sdl2, image and audio
  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_EVENTS) != 0) {
    std::cout << "SDL_Init Error: " << SDL_GetError() << std::endl;
    return;
  }
  int imgFlags = IMG_INIT_PNG | IMG_INIT_JPG;
  if (!(IMG_Init(imgFlags) & imgFlags)) {
    std::cout << "IMG_Init Error: " << IMG_GetError() << std::endl;
    return;
  }
  if (Mix_OpenAudio(44100, MIX_DEFAULT_FORMAT, 2, 2048) < 0) {
    std::cout << "Mix_OpenAudio Error: " << Mix_GetError() << std::endl;
    return;
  }

  // create sdl2 window and renderer
  SDL_SetHint(SDL_HINT_VIDEO_HIGHDPI_DISABLED, "0");
  // SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "0"); // for pixel-art style
  window =
      SDL_CreateWindow("engine2d", SDL_WINDOWPOS_CENTERED,
                       SDL_WINDOWPOS_CENTERED, width, height, SDL_WINDOW_SHOWN);
  renderer = SDL_CreateRenderer(
      window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  if (!window || !renderer) {
    std::cout << "SDL_CreateWindow/Renderer Error: " << SDL_GetError()
              << std::endl;
    return;
  }

  // Initialize high-resolution performance counter for delta timing
  perf_frequency = SDL_GetPerformanceFrequency();
  last_counter = SDL_GetPerformanceCounter();

  // init imgui
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  // // Style
  // ImGui::StyleColorsDark();
  // Backend bindings
  ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
  ImGui_ImplSDLRenderer2_Init(renderer);

  // setup ecs
  reset();
}
void engine2d::run() {
  while (engine_running) {
    while (SDL_PollEvent(&event)) {
      ImGui_ImplSDL2_ProcessEvent(&event);
      if (event.type == SDL_QUIT)
        engine_running = false;
    }
    // Update high-resolution delta time at the start of the frame
    Uint64 now_counter = SDL_GetPerformanceCounter();
    delta_time = static_cast<float>(now_counter - last_counter) /
                 static_cast<float>(perf_frequency);
    last_counter = now_counter;

    for (auto sys : systems)
      if (sys->active)
        sys->preupdate(registry, delta_time);
    for (auto sys : systems)
      if (sys->active)
        sys->update(registry, delta_time);
    for (auto sys : systems)
      if (sys->active)
        sys->lateupdate(registry, delta_time);

    // Start ImGui frame
    ImGui_ImplSDL2_NewFrame();
    ImGui_ImplSDLRenderer2_NewFrame();
    ImGui::NewFrame();

    // ---------------------------
    // ImGui UI Example
    // ---------------------------
    ImGui::Begin("Hello ImGui");
    ImGui::Text("SDL2 + ImGui initialized!");
    if (ImGui::Button("Quit")) {
      engine_running = false;
    }
    ImGui::End();

    // Rendering
    ImGui::Render();
    SDL_SetRenderDrawColor(renderer, 30, 30, 30, 255);
    SDL_RenderClear(renderer);

    // actual rendering
    draw_line(10, 10, 200, 300);

    draw_rectangle(200, 200, 30, 50, false);
    draw_rectangle(200, 200, 50, 30, true, math::vector3(1, 0, 0));

    draw_circle(300, 300, 10, false);
    draw_circle(310, 300, 10, true, math::vector3(1, 0, 0));

    ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData());
    SDL_RenderPresent(renderer);
  }
}
void engine2d::shutdown() {
  ImGui_ImplSDLRenderer2_Shutdown();
  ImGui_ImplSDL2_Shutdown();
  ImGui::DestroyContext();

  Mix_CloseAudio();
  Mix_Quit();
  IMG_Quit();
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
}

void engine2d::late_deserialize(nlohmann::json &j) {}
void engine2d::late_serialize(nlohmann::json &j) {}

void engine2d::reset() {
  registry.clear();
  systems.clear();

  add_sys<sim_sys_2d>();
}

// --- 2D drawing helper implementations ---
inline Uint8 to_u8(float v) {
  if (v <= 0.0f)
    return 0;
  if (v >= 1.0f)
    return 255;
  return static_cast<Uint8>(v * 255.0f);
}

void engine2d::draw_point(int x, int y, const math::vector3 &color) {
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

void engine2d::draw_points(const std::vector<math::vector2> &points,
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

void engine2d::draw_line(int x1, int y1, int x2, int y2,
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

void engine2d::draw_lines(const std::vector<math::vector2> &points, bool closed,
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

void engine2d::draw_rectangle(int x, int y, int w, int h, bool filled,
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

void engine2d::draw_circle(int x, int y, int radius, bool filled,
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