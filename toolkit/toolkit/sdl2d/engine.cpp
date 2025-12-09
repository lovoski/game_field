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
  // io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  // Backend bindings
  ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
  ImGui_ImplSDLRenderer2_Init(renderer);

  // setup ecs
  reset();
  add_default_objects();
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

    // start imgui if not in play mode
    if (!engine_play_mode) {
      ImGui_ImplSDL2_NewFrame();
      ImGui_ImplSDLRenderer2_NewFrame();
      ImGui::NewFrame();
      draw_editor_gui();
      ImGui::Render();
    }

    // clear framebuffer
    SDL_SetRenderDrawColor(renderer, 30, 30, 30, 255);
    SDL_RenderClear(renderer);

    draw_game_content();

    if (!engine_play_mode)
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

void engine2d::add_default_objects() {}

void engine2d::draw_game_content() {
  registry.view<body>().each([&](entt::entity entity, body &body_comp) {

  });
}

}; // namespace toolkit::sdl2d