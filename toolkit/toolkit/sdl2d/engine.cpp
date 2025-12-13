#include "toolkit/sdl2d/engine.hpp"
#include "toolkit/sdl2d/avbd/scenes.h"

namespace toolkit::sdl2d {

void engine2d::init(int width, int height) {
  screen_width = width;
  screen_height = height;
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
  // enable alpha blending for this renderer
  SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

  // Initialize high-resolution performance counter for delta timing
  perf_frequency = SDL_GetPerformanceFrequency();
  last_counter = SDL_GetPerformanceCounter();

  // init imgui
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  // io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  io.IniFilename = "engine2d_layout.ini";

  // load optional default font
  if (std::filesystem::exists("font.ttf")) {
    io.Fonts->AddFontFromFileTTF("font.ttf", 20.0f, nullptr,
                                 io.Fonts->GetGlyphRangesChineseFull());
    std::cout << "Default font loaded from ./font.ttf" << std::endl;
  } else {
    std::cout << "Default font not found at ./font.ttf, use imgui internal "
                 "font without utf-8 support"
              << std::endl;
  }

  // Backend bindings
  ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
  ImGui_ImplSDLRenderer2_Init(renderer);

  // setup ecs
  reset();
  add_default_objects();
}
void engine2d::run() {
  while (engine_running) {
    handle_event_states();

    handle_game_logic_tick();

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

    handle_game_render_tick();

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

bool engine2d::is_key_pressed(int key) const {
  auto it = key_states.find(key);
  return it != key_states.end() && it->second;
}
bool engine2d::is_key_triggered(int key) const {
  return triggered_keys.count(key) != 0;
}
bool engine2d::is_key_untriggered(int key) const {
  return untriggered_keys.count(key) != 0;
}
bool engine2d::is_mouse_button_triggered(int key) const {
  return triggered_mouse_keys.count(key) != 0;
}
bool engine2d::is_mouse_button_untriggered(int key) const {
  return untriggered_mouse_keys.count(key) != 0;
}
bool engine2d::is_mouse_button_pressed(int button) const {
  auto it = mouse_button_states.find(button);
  return it != mouse_button_states.end() && it->second;
}

void engine2d::handle_event_states() {
  // clear states
  mouse_scroll_offset = math::vector2::Zero();
  mouse_screen_delta = math::vector2::Zero();
  triggered_keys.clear();
  untriggered_keys.clear();
  triggered_mouse_keys.clear();
  untriggered_mouse_keys.clear();

  // poll events
  while (SDL_PollEvent(&event)) {
    ImGui_ImplSDL2_ProcessEvent(&event);
    if (event.type == SDL_QUIT) {
      engine_running = false;
    } else if (event.type == SDL_MOUSEMOTION) {
      mouse_screen_position.x() = static_cast<float>(event.motion.x);
      mouse_screen_position.y() = static_cast<float>(event.motion.y);
      mouse_screen_delta.x() = static_cast<float>(event.motion.xrel);
      mouse_screen_delta.y() = static_cast<float>(event.motion.yrel);
    } else if (event.type == SDL_MOUSEWHEEL) {
      mouse_scroll_offset.x() = static_cast<float>(event.wheel.x);
      mouse_scroll_offset.y() = static_cast<float>(event.wheel.y);
    } else if (event.type == SDL_MOUSEBUTTONDOWN ||
               event.type == SDL_MOUSEBUTTONUP) {
      int button = static_cast<int>(event.button.button);
      bool curState =
          (event.type == SDL_MOUSEBUTTONDOWN); // true for DOWN, false for UP
      if (mouse_button_states.count(button) == 0)
        mouse_button_states[button] = false;
      bool prevState = mouse_button_states[button];
      if (!prevState && curState)
        triggered_mouse_keys.insert(button);
      if (prevState && !curState)
        untriggered_mouse_keys.insert(button);
      mouse_button_states[button] = curState;
    } else if (event.type == SDL_KEYDOWN || event.type == SDL_KEYUP) {
      int scancode =
          event.key.keysym.scancode; // SDL_Scancode -> unique per physical key
      int keycode = event.key.keysym.sym; // SDL_Keycode
      // For KEYDOWN: only process non-repeat presses (repeat == 0).
      // For KEYUP: always process (repeat field irrelevant).
      bool curState = (event.type == SDL_KEYDOWN);
      int key = static_cast<int>(keycode);
      if (key_states.count(key) == 0)
        key_states[key] = false;
      bool prevState = key_states[key];
      if (!prevState && curState)
        triggered_keys.insert(key);
      if (prevState && !curState)
        untriggered_keys.insert(key);
      key_states[key] = curState;
      // caps lock detection
      SDL_Keymod km = SDL_GetModState();
      caps_lock_on = (km & KMOD_CAPS) != 0;
    }
  }
}

void engine2d::late_deserialize(nlohmann::json &j) {}
void engine2d::late_serialize(nlohmann::json &j) {}

void engine2d::reset() {
  registry.clear();
  systems.clear();

  // box2d_solver = register_sys<box2d::box2d_lite_world>();

  camera_zoom = 32.0f; // 1 unit in world space ---> 32 pixels
  camera_rotation = 0.0f;
  camera_position = math::vector2::Zero();
}

math::vector2 engine2d::world_to_screen(const math::vector2 &world_pos) {
  // translate relative to camera
  math::vector2 screen_pos = world_pos - camera_position;
  screen_pos.y() *= -1;
  // Apply zoom
  screen_pos *= camera_zoom;
  // Move origin to center of screen
  screen_pos.x() += screen_width * 0.5f;
  screen_pos.y() += screen_height * 0.5f;
  return screen_pos;
}

math::vector2 engine2d::screen_to_world(const math::vector2 &screen_pos) {
  math::vector2 world_pos = screen_pos;
  world_pos.x() -= screen_width * 0.5f;
  world_pos.y() -= screen_height * 0.5f;
  world_pos /= camera_zoom; // zoom back
  world_pos.y() *= -1;      // invert y back
  world_pos += camera_position;
  return world_pos;
}

void engine2d::add_default_objects() {
  // auto ground0 = registry.create();
  // auto &ground_body0 = registry.emplace<box2d::body>(ground0);
  // ground_body0.setup(math::vector2(20.0f, 1.0f));
  // ground_body0.position = math::vector2(0.0f, -1.0f);
  // ground_body0.rotation = 0.1f;

  // auto ground1 = registry.create();
  // auto &ground_body1 = registry.emplace<box2d::body>(ground1);
  // ground_body1.setup(math::vector2(20.0f, 1.0f));
  // ground_body1.position = math::vector2(-10.0f, -2.0f);
  // ground_body1.rotation = -0.1f;

  // for (int i = 0; i < 5; i++) {
  //   auto entity = registry.create();
  //   auto &body_comp = registry.emplace<box2d::body>(entity);
  //   body_comp.setup(math::vector2(1.0f, 1.0f + i), 1.0f);
  //   body_comp.position = math::vector2(0.0f, 1.0f + 5 * i);
  // }
  // for (int i = 0; i < 5; i++) {
  //   auto entity = registry.create();
  //   auto &body_comp = registry.emplace<box2d::body>(entity);
  //   body_comp.setup(math::vector2(1.0f, 1.0f), 1.0f);
  //   body_comp.position = math::vector2(-1.0f, 1.0f + 5 * i);
  // }
  // for (int i = 0; i < 5; i++) {
  //   auto entity = registry.create();
  //   auto &body_comp = registry.emplace<box2d::body>(entity);
  //   body_comp.setup(math::vector2(1.0f, 1.0f), 1.0f);
  //   body_comp.position = math::vector2(1.0f, 1.0f + 5 * i);
  // }

  // auto ground0 = registry.create();
  // auto &ground_body0 = registry.emplace<box2d::body>(ground0);
  // ground_body0.setup(math::vector2(20.0f, 1.0f));
  // ground_body0.position = math::vector2(0.0f, -1.0f);
  // ground_body0.rotation = 0.0f;

  // for (int i = 0; i < 20; i++) {
  //   auto entity = registry.create();
  //   auto &body_comp = registry.emplace<box2d::body>(entity);
  //   body_comp.setup(math::vector2(1.0f, 1.0f), 1.0f);
  //   body_comp.position = math::vector2(0.0f, 1.0f + 2 * i);
  // }

  avbd::scenes[18](&avbd_solver);
}

}; // namespace toolkit::sdl2d