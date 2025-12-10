#include "toolkit/opengl3d/engine.hpp"
#include "toolkit/opengl3d/components/actor.hpp"
#include "toolkit/opengl3d/components/mesh.hpp"
#include "toolkit/opengl3d/gui.hpp"
#include "toolkit/opengl3d/rasterize/shaders.hpp"

namespace toolkit::opengl3d {

void engine3d::init(int width, int height, std::string title, int majorVersion,
                    int minorVersion) {
  wnd_width = width;
  wnd_height = height;

  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_EVENTS |
               SDL_INIT_JOYSTICK | SDL_INIT_GAMECONTROLLER) != 0) {
    printf("Error: SDL_Init failed: %s\n", SDL_GetError());
    return;
  }

  // Set GL attributes for modern OpenGL
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, majorVersion);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, minorVersion);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  // SDL_GL_SetAttribute(SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG, 1);
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
  SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
  SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
  SDL_SetHint(SDL_HINT_VIDEO_HIGHDPI_DISABLED, "0");

  // Create window with OpenGL context
  // window = SDL_CreateWindow(
  //     title, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
  //     static_cast<int>(width), static_cast<int>(height),
  //     SDL_WINDOW_OPENGL | SDL_WINDOW_ALLOW_HIGHDPI | SDL_WINDOW_RESIZABLE);
  window = SDL_CreateWindow(title.c_str(), SDL_WINDOWPOS_CENTERED,
                            SDL_WINDOWPOS_CENTERED, static_cast<int>(width),
                            static_cast<int>(height),
                            SDL_WINDOW_OPENGL | SDL_WINDOW_ALLOW_HIGHDPI);
  if (!window) {
    printf("Failed to create SDL window: %s\n", SDL_GetError());
    return;
  }
  reset_wnd_drawable_size();

  gl_context = SDL_GL_CreateContext(window);
  if (!gl_context) {
    printf("Failed to create GL context: %s\n", SDL_GetError());
    return;
  }

  // Load GL functions with glad
  if (!gladLoadGLLoader((GLADloadproc)SDL_GL_GetProcAddress)) {
    printf("Failed to load glad\n");
    return;
  }

  // ImGui + SDL2 + OpenGL3 initialization
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();

  // Setup Platform/Renderer bindings
  ImGui_ImplSDL2_InitForOpenGL(window, gl_context);

  // Use GLSL version string for ImGui OpenGL3 backend
  // Construct version string like "#version 430"
  char glsl_version[64];
  snprintf(glsl_version, sizeof(glsl_version), "#version %d%d0", majorVersion,
           minorVersion);
  // For core profile we often pass "#version 430" etc. ImGui backend accepts
  // string like "#version 330" But some combinations require tweak; caller may
  // adjust if needed.
  ImGui_ImplOpenGL3_Init(glsl_version);

  ImPlot::CreateContext();

  // default_font = ImGui::GetIO().Fonts->AddFontFromMemoryCompressedTTF(
  //     cascadia_code_yahei_data, cascadia_code_yahei_size, 20.0f, nullptr,
  //     ImGui::GetIO().Fonts->GetGlyphRangesChineseFull());

  // create system default textures
  white_tex.create();
  black_tex.create();
  checkerboard_tex.create();
  assets::image img;
  img.resize(10, 10, 4);
  for (int i = 0; i < img.width; i++)
    for (int j = 0; j < img.height; j++) {
      for (int k = 0; k < 3; k++)
        img.pixel(i, j, k) = static_cast<unsigned char>(255);
      img.pixel(i, j, 3) = static_cast<unsigned char>(255);
    }
  white_tex.set_data_from_image(img);
  white_tex.set_parameters({{GL_TEXTURE_MIN_FILTER, GL_NEAREST},
                            {GL_TEXTURE_MAG_FILTER, GL_NEAREST},
                            {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
                            {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});

  for (int i = 0; i < img.width; i++)
    for (int j = 0; j < img.height; j++)
      for (int k = 0; k < 3; k++)
        img.pixel(i, j, k) = static_cast<unsigned char>(0);
  black_tex.set_data_from_image(img);
  black_tex.set_parameters({{GL_TEXTURE_MIN_FILTER, GL_NEAREST},
                            {GL_TEXTURE_MAG_FILTER, GL_NEAREST},
                            {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
                            {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});

  // Create checkerboard pattern
  const int squareSize = 2; // Size of each checker square
  for (int i = 0; i < img.width; i++) {
    for (int j = 0; j < img.height; j++) {
      bool isWhite = ((i / squareSize) + (j / squareSize)) % 2 == 0;
      for (int k = 0; k < 3; k++)
        img.pixel(i, j, k) = static_cast<unsigned char>(isWhite ? 255 : 120);
      img.pixel(i, j, 3) = static_cast<unsigned char>(255);
    }
  }
  checkerboard_tex.set_data_from_image(img);
  checkerboard_tex.set_parameters({{GL_TEXTURE_MIN_FILTER, GL_NEAREST},
                                   {GL_TEXTURE_MAG_FILTER, GL_NEAREST},
                                   {GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE},
                                   {GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE}});

  set_vsync_state(should_vsync);

  reset();

  // init imgui
  imgui_io = &ImGui::GetIO();
  imgui_io->ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  imgui_io->ConfigFlags |= ImGuiConfigFlags_DockingEnable;

  quad_program.compile_shader_from_source(quad_vs, R"(
#version 430 core
uniform sampler2D scene_tex;
in vec2 texcoord;
out vec4 frag_color;
void main() {
  frag_color = texture(scene_tex, texcoord);
}
)");
}

void engine3d::shutdown() {
  // ImGui + SDL cleanup
  ImPlot::DestroyContext();
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplSDL2_Shutdown();
  ImGui::DestroyContext();

  // Destroy GL context and window
  if (gl_context) {
    SDL_GL_DeleteContext(gl_context);
    gl_context = nullptr;
  }
  if (window) {
    SDL_DestroyWindow(window);
    window = nullptr;
  }

  SDL_Quit();
}

bool engine3d::is_key_pressed(int key) const {
  auto it = key_states.find(key);
  return it != key_states.end() && it->second;
}
bool engine3d::is_key_triggered(int key) const {
  return triggered_keys.count(key) != 0;
}
bool engine3d::is_key_untriggered(int key) const {
  return untriggered_keys.count(key) != 0;
}
bool engine3d::is_mouse_button_triggered(int key) const {
  return triggered_mouse_keys.count(key) != 0;
}
bool engine3d::is_mouse_button_untriggered(int key) const {
  return untriggered_mouse_keys.count(key) != 0;
}
bool engine3d::is_mouse_button_pressed(int button) const {
  auto it = mouse_button_states.find(button);
  return it != mouse_button_states.end() && it->second;
}

void engine3d::reset_wnd_drawable_size() {
  int tmp_wnd_width, tmp_wnd_height;
  SDL_GL_GetDrawableSize(window, &tmp_wnd_width, &tmp_wnd_height);
  wnd_width = tmp_wnd_width;
  wnd_height = tmp_wnd_height;
}

std::vector<SDL_GameController *> engine3d::get_game_controllers() {
  std::vector<SDL_GameController *> controllers;
  for (int i = 0; i < SDL_NumJoysticks(); ++i) {
    if (SDL_IsGameController(i)) {
      controllers.push_back(SDL_GameControllerOpen(i));
    }
  }
  return controllers;
}
std::tuple<math::vector2, math::vector2, float, float>
engine3d::get_game_controller_analog_inputs(SDL_GameController *controller) {
  float tl, tr;
  math::vector2 left = math::vector2::Zero(), right = math::vector2::Zero();
  left.x() = std::clamp(
      (float)SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTX) /
          (float)std::numeric_limits<Sint16>::max(),
      -1.0f, 1.0f);
  left.y() = std::clamp(
      (float)SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTY) /
          (float)std::numeric_limits<Sint16>::max(),
      -1.0f, 1.0f);
  right.x() = std::clamp(
      (float)SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_RIGHTX) /
          (float)std::numeric_limits<Sint16>::max(),
      -1.0f, 1.0f);
  right.y() = std::clamp(
      (float)SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_RIGHTY) /
          (float)std::numeric_limits<Sint16>::max(),
      -1.0f, 1.0f);
  tl = std::clamp((float)SDL_GameControllerGetAxis(
                      controller, SDL_CONTROLLER_AXIS_TRIGGERLEFT) /
                      (float)std::numeric_limits<Sint16>::max(),
                  0.0f, 1.0f);
  tr = std::clamp((float)SDL_GameControllerGetAxis(
                      controller, SDL_CONTROLLER_AXIS_TRIGGERRIGHT) /
                      (float)std::numeric_limits<Sint16>::max(),
                  0.0f, 1.0f);
  return {left, right, tl, tr};
}

void engine3d::handle_input_events() {
  // reset per-frame transient states
  wnd_resized = false;
  scroll_offset = math::vector2{0.0, 0.0};
  mouse_delta_x = 0.0;
  mouse_delta_y = 0.0;
  triggered_keys.clear();
  untriggered_keys.clear();
  triggered_mouse_keys.clear();
  untriggered_mouse_keys.clear();

  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    // Let ImGui process events first
    ImGui_ImplSDL2_ProcessEvent(&event);

    switch (event.type) {
    case SDL_QUIT:
      // Close window / exit loop
      if (window) {
        SDL_DestroyWindow(window);
        window = nullptr;
      }
      break;

    case SDL_WINDOWEVENT:
      if (event.window.event == SDL_WINDOWEVENT_RESIZED ||
          event.window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {
        wnd_resized = true;
        reset_wnd_drawable_size();
      }
      break;

    case SDL_KEYDOWN:
    case SDL_KEYUP: {
      int scancode =
          event.key.keysym.scancode; // SDL_Scancode -> unique per physical key
      int keycode = event.key.keysym.sym;          // SDL_Keycode
      bool curState = (event.type == SDL_KEYDOWN); // treat keydown as pressed
      // We'll use SDL_Keycode as key identifier to align with likely GLFW key
      // usage; if you used GLFW key constants, mapping may be needed.
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
    } break;

    case SDL_MOUSEBUTTONDOWN:
    case SDL_MOUSEBUTTONUP: {
      int button = static_cast<int>(event.button.button);
      bool curState = (event.type == SDL_MOUSEBUTTONDOWN);
      if (mouse_button_states.count(button) == 0)
        mouse_button_states[button] = false;
      bool prevState = mouse_button_states[button];
      if (!prevState && curState)
        triggered_mouse_keys.insert(button);
      if (prevState && !curState)
        untriggered_mouse_keys.insert(button);
      mouse_button_states[button] = curState;
    } break;

    case SDL_MOUSEMOTION: {
      // SDL provides high-precision positions
      mouse_x = static_cast<double>(event.motion.x);
      mouse_y = static_cast<double>(event.motion.y);
      mouse_delta_x = static_cast<double>(event.motion.xrel);
      mouse_delta_y = static_cast<double>(event.motion.yrel);
    } break;

    case SDL_MOUSEWHEEL: {
      // SDL mouse wheel: x,y for horizontal/vertical
      scroll_offset.x() += static_cast<double>(event.wheel.x);
      scroll_offset.y() += static_cast<double>(event.wheel.y);
    } break;

    default:
      break;
    } // switch
  }   // while SDL_PollEvent
}

void engine3d::set_vsync_state(bool enable) {
  should_vsync = enable;
  if (should_vsync)
    SDL_GL_SetSwapInterval(1);
  else
    SDL_GL_SetSwapInterval(0);
}

void engine3d::late_serialize(nlohmann::json &j) {
  nlohmann::json editor_settings;
  editor_settings["active_camera"] = active_camera;
  editor_settings["camera_manipulate_data"] = cam_manip_data;
  j["engine3d"] = editor_settings;
}

void engine3d::late_deserialize(nlohmann::json &j) {
  if (j.contains("engine3d")) {
    active_camera = j["engine3d"]["active_camera"].get<entt::entity>();
    cam_manip_data = j["engine3d"]["camera_manipulate_data"]
                         .get<active_camera_manipulate_data>();
  } else {
    // use the first camera as active camera, otherwise no active camera
    auto cam_view = registry.view<camera>();
    if (cam_view.size() == 0)
      active_camera = entt::null;
    else
      active_camera = *(cam_view.begin());
  }

  transform_sys = get_sys<transform_system>();
  render_sys = get_sys<defered_render_system>();
}

void engine3d::game_mode_main_loop() {
  auto &active_cam_trans = registry.get<transform>(active_camera);
  auto &active_cam_comp = registry.get<camera>(active_camera);
  float dt = timer.elapse_s();
  timer.reset();

  transform_sys->update_transform(registry);
  render_sys->update_scene_buffers(registry);

  for (auto sys : systems)
    if (sys->active)
      sys->preupdate(registry, dt);
  for (auto sys : systems)
    if (sys->active)
      sys->update(registry, dt);
  for (auto sys : systems)
    if (sys->active)
      sys->lateupdate(registry, dt);

  if (wnd_resized) {
    scene_wnd_size.x() = wnd_width;
    scene_wnd_size.y() = wnd_height;
    render_sys->resize(wnd_width, wnd_height);
  }
  render_sys->render(registry, active_cam_trans, active_cam_comp);

  glClearColor(0, 0, 0, 1);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glViewport(0, 0, wnd_width, wnd_height);

  quad_program.use();
  quad_program.set_texture2d("scene_tex",
                             render_sys->get_target_texture().get_handle(), 0);
  quad_draw_call();
  if (window) {
    SDL_GL_SwapWindow(window);
  }
}
void engine3d::editor_mode_main_loop() {
  auto &active_cam_trans = registry.get<transform>(active_camera);
  auto &active_cam_comp = registry.get<camera>(active_camera);
  float dt = timer.elapse_s();
  timer.reset();

  transform_sys->update_transform(registry);
  render_sys->update_scene_buffers(registry);
  if (editor_manipulate_camera)
    active_camera_manipulate(dt);

  for (auto sys : systems)
    if (sys->active)
      sys->preupdate(registry, dt);
  for (auto sys : systems)
    if (sys->active)
      sys->update(registry, dt);
  for (auto sys : systems)
    if (sys->active)
      sys->lateupdate(registry, dt);

  render_sys->render(registry, active_cam_trans, active_cam_comp);

  glClearColor(0, 0, 0, 1);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glViewport(0, 0, wnd_width, wnd_height);

  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplSDL2_NewFrame();
  ImGui::NewFrame();

  ImGui::DockSpaceOverViewport(ImGui::GetMainViewport());
  ImGui::Begin("Scene");
  ImGui::BeginChild("GameRenderer");
  auto size = ImGui::GetContentRegionAvail();
  auto pos = ImGui::GetWindowPos();
  scene_wnd_pos.x() = pos.x;
  scene_wnd_pos.y() = pos.y;
  ImGui::Image((void *)static_cast<std::uintptr_t>(
                   render_sys->get_target_texture().get_handle()),
               {size.x, size.y}, ImVec2(0, 1), ImVec2(1, 0));
  if (scene_wnd_size.x() != size.x || scene_wnd_size.y() != size.y) {
    // resize sceneFBO
    scene_wnd_size.x() = size.x;
    scene_wnd_size.y() = size.y;
    render_sys->resize(size.x, size.y);
    render_sys->render(registry, active_cam_trans, active_cam_comp);
  }
  draw_gizmos();
  ImGui::EndChild();
  ImGui::End();

  draw_main_menubar();
  draw_entity_hierarchy();
  draw_entity_components();
  editor_shortkeys();

  ImGui::EndFrame();
  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

  if (window) {
    SDL_GL_SwapWindow(window);
  }
}

void engine3d::run() {
  bool running = true;
  timer.reset();
  add_default_objects();
  while (running) {
    handle_input_events();
    if (!window)
      break;
    if (is_key_triggered(SDLK_0) && is_key_pressed(SDLK_LCTRL)) {
      scene_wnd_size.x() = wnd_width;
      scene_wnd_size.x() = wnd_height;
      render_sys->resize(wnd_width, wnd_height);
      in_game_mode = !in_game_mode;
    }
    if (!in_game_mode) {
      SDL_SetRelativeMouseMode(SDL_FALSE);
      editor_mode_main_loop();
    } else {
      SDL_SetRelativeMouseMode(SDL_TRUE);
      game_mode_main_loop();
    }
  }
}

void engine3d::reset() {
  registry.clear();
  systems.clear();

  transform_sys = add_sys<transform_system>();
  render_sys = add_sys<defered_render_system>();
}

void engine3d::add_default_objects() {
  auto ent = registry.create();
  auto &trans = registry.emplace<transform>(ent);
  trans.name = "main camera";
  trans.set_world_pos(math::vector3(0, 0, 5));
  auto &cam_comp = registry.emplace<camera>(ent);
  active_camera = ent;
}

}; // namespace toolkit::opengl3d
