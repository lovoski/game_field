#include "craft.hpp"

namespace toolkit::opengl3d {

using ::craft::BlockId;
using ::craft::CHUNK_X;
namespace Block = ::craft::Block;

// ---------------------------------------------------------------------------
// Initialization & cleanup
// ---------------------------------------------------------------------------

void craft::handle_custom_initialization() {
  // --- Create player entity with transform + camera ---
  player_entity = registry.create();
  auto &t = registry.emplace<transform>(player_entity);
  t.registry = &registry;
  t.self = player_entity;
  t.name = "player";
  t.set_local_pos(math::vector3(0.0f, 30.0f, 0.0f));

  auto &cam = registry.emplace<camera>(player_entity);
  cam.fovy_degree = 70.0f;
  cam.z_near = 0.1f;
  cam.z_far = 300.0f;

  // Point the engine's active camera at the player
  active_camera = player_entity;
  editor_manipulate_camera = false; // we drive camera ourselves

  // --- World ---
  world.view_distance = 6;
  // (You can assign a custom generator: world.generate = myFunc;)

  // --- Render pipeline ---
  voxel_pipeline.init(wnd_width, wnd_height);
  voxel_pipeline.fog_end = (float)(world.view_distance * CHUNK_X);
  voxel_pipeline.fog_start = voxel_pipeline.fog_end * 0.5f;

  // Enter game mode with captured mouse
  set_game_mode(true, true);
}

void craft::handle_custom_cleanup() {
  voxel_pipeline.shutdown();
}

void craft::reset() {
  registry.clear();
  systems.clear();
  transform_hierarchy_sys = register_sys<transform_system>();
  player_entity = entt::null;
}

// ---------------------------------------------------------------------------
// Player input
// ---------------------------------------------------------------------------

void craft::handle_player_input(float dt) {
  if (player_entity == entt::null) return;

  // Toggle game mode with Escape
  if (is_key_triggered(SDLK_ESCAPE))
    set_game_mode(!app_in_game_mode(), !app_in_game_mode());
  if (!app_in_game_mode()) return;

  auto &t = registry.get<transform>(player_entity);

  // Mouse-look (raw delta, no dt — mouse input is per-frame, not per-second)
  auto delta = get_mouse_screen_delta();
  yaw   -= delta.x() * look_sensitivity;
  pitch += delta.y() * look_sensitivity;
  pitch = std::clamp(pitch, -89.0f, 89.0f);

  math::quat rot = math::euler_to_quat(
      math::deg_to_rad(math::vector3(pitch, yaw, 0.0f)));
  t.set_local_rot(rot);

  // Keyboard movement — project forward/right onto horizontal plane
  // so WASD always moves along the ground; Space/Shift handle vertical.
  math::vector3 move = math::vector3::Zero();
  math::vector3 fwd = t.local_forward();
  fwd.y() = 0.0f;
  if (fwd.squaredNorm() > 1e-6f) fwd.normalize();
  math::vector3 right = -t.local_right();
  right.y() = 0.0f;
  if (right.squaredNorm() > 1e-6f) right.normalize();

  if (is_key_pressed(SDLK_w)) move += fwd;
  if (is_key_pressed(SDLK_s)) move -= fwd;
  if (is_key_pressed(SDLK_d)) move += right;
  if (is_key_pressed(SDLK_a)) move -= right;
  if (is_key_pressed(SDLK_SPACE))  move += math::world_up;
  if (is_key_pressed(SDLK_LSHIFT)) move -= math::world_up;

  float speed = move_speed;
  if (is_key_pressed(SDLK_LCTRL)) speed *= 3.0f; // sprint

  if (move.squaredNorm() > 1e-6f)
    t.set_local_pos(t.local_pos() + move.normalized() * speed * dt);
}

// ---------------------------------------------------------------------------
// Block interaction (place / destroy with mouse)
// ---------------------------------------------------------------------------

void craft::handle_block_interaction() {
  if (player_entity == entt::null) return;
  if (!app_in_game_mode()) return;

  auto &t = registry.get<transform>(player_entity);
  math::vector3 origin = t.world_pos();
  math::vector3 dir = t.local_forward();

  // Simple ray march
  constexpr float REACH = 6.0f;
  constexpr float STEP = 0.1f;
  math::vector3 prev_pos = origin;

  for (float d = 0.0f; d < REACH; d += STEP) {
    math::vector3 p = origin + dir * d;
    int bx = (int)std::floor(p.x());
    int by = (int)std::floor(p.y());
    int bz = (int)std::floor(p.z());

    BlockId hit = world.get_block(bx, by, bz);
    if (::craft::is_solid(hit)) {
      // Left click = destroy
      if (is_mouse_button_triggered(SDL_BUTTON_LEFT)) {
        world.set_block(bx, by, bz, Block::Air);
      }
      // Right click = place at previous position
      if (is_mouse_button_triggered(SDL_BUTTON_RIGHT)) {
        int px = (int)std::floor(prev_pos.x());
        int py = (int)std::floor(prev_pos.y());
        int pz = (int)std::floor(prev_pos.z());
        if (world.get_block(px, py, pz) == Block::Air)
          world.set_block(px, py, pz, Block::Stone);
      }
      break;
    }
    prev_pos = p;
  }
}

// ---------------------------------------------------------------------------
// Fixed update
// ---------------------------------------------------------------------------

void craft::engine_update(float dt) {
  handle_player_input(dt);
  handle_block_interaction();

  // Update transform hierarchy
  transform_hierarchy_sys->update_transform(registry);

  // Update world chunks around player
  if (player_entity != entt::null) {
    auto &t = registry.get<transform>(player_entity);
    world.update_around(t.world_pos());
  }
}

// ---------------------------------------------------------------------------
// GUI (ImGui overlay)
// ---------------------------------------------------------------------------

void craft::handle_engine_gui() {
  ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Once);
  ImGui::SetNextWindowSize(ImVec2(300, 200), ImGuiCond_Once);
  ImGui::Begin("Craft Debug");

  if (player_entity != entt::null) {
    auto &t = registry.get<transform>(player_entity);
    auto pos = t.world_pos();
    ImGui::Text("Pos: %.1f, %.1f, %.1f", pos.x(), pos.y(), pos.z());
    ImGui::Text("Yaw: %.1f  Pitch: %.1f", yaw, pitch);
  }

  ImGui::SliderFloat("Move Speed", &move_speed, 1.0f, 50.0f);
  ImGui::SliderFloat("Look Sens", &look_sensitivity, 0.01f, 1.0f);
  ImGui::SliderInt("View Dist", &world.view_distance, 1, 16);
  ImGui::Separator();

  ImGui::ColorEdit3("Sun Color", voxel_pipeline.sun_color.data());
  ImGui::ColorEdit3("Ambient", voxel_pipeline.ambient.data());
  ImGui::ColorEdit3("Sky Color", voxel_pipeline.sky_color.data());
  ImGui::SliderFloat("Fog Start", &voxel_pipeline.fog_start, 0, 300);
  ImGui::SliderFloat("Fog End", &voxel_pipeline.fog_end, 0, 500);

  ImGui::Text("ESC: toggle mouse capture");
  ImGui::Text("WASD: move | Space/Shift: up/down");
  ImGui::Text("LMB: break | RMB: place");
  ImGui::End();

  // Crosshair
  auto *dl = ImGui::GetForegroundDrawList();
  ImVec2 c(wnd_width * 0.5f, wnd_height * 0.5f);
  dl->AddLine(ImVec2(c.x - 10, c.y), ImVec2(c.x + 10, c.y),
              IM_COL32(255, 255, 255, 200), 2.0f);
  dl->AddLine(ImVec2(c.x, c.y - 10), ImVec2(c.x, c.y + 10),
              IM_COL32(255, 255, 255, 200), 2.0f);
}

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------

void craft::run() {
  timer.reset();
  while (app_running) {
    float dt = timer.elapse_s();
    timer.reset();

    handle_input_events();
    engine_update(dt);

    if (!app_running) break;

    // Resize pipeline if window changed
    voxel_pipeline.resize(wnd_width, wnd_height);

    // Render voxel world
    auto &cam_trans = registry.get<transform>(player_entity);
    auto &cam_comp = registry.get<camera>(player_entity);
    voxel_pipeline.render(world, cam_trans, cam_comp,
                          (float)wnd_width, (float)wnd_height);

    // Blit to screen
    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glViewport(0, 0, wnd_width, wnd_height);

    quad_program.use();
    quad_program.set_texture2d("scene_tex", voxel_pipeline.output_texture(), 0);
    quad_draw_call();

    // ImGui overlay
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();
    handle_engine_gui();
    ImGui::EndFrame();
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    if (window)
      SDL_GL_SwapWindow(window);
  }
}

}; // namespace toolkit::opengl3d