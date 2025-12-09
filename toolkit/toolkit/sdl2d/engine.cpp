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
}
void engine2d::run() {
  while (engine_running) {
    while (SDL_PollEvent(&event)) {
      ImGui_ImplSDL2_ProcessEvent(&event);
      if (event.type == SDL_QUIT)
        engine_running = false;
    }

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

}; // namespace toolkit::sdl2d