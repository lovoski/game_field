#include "toolkit/opengl/editor.hpp"
#include "toolkit/anim/components/actor.hpp"
#include "toolkit/anim/scripts/motion_player.hpp"
#include "toolkit/anim/scripts/vis.hpp"
#include "toolkit/opengl/components/mesh.hpp"
#include "toolkit/opengl/gui/utils.hpp"
#include "toolkit/opengl/rasterize/shaders.hpp"
#include "toolkit/opengl/scripts/mesh_panel.hpp"
#include "toolkit/opengl/scripts/smplx.hpp"

namespace toolkit::opengl {

void editor::init() {
  auto &instance = sdl_context::get_instance();
  instance.init(1920, 1080, "Editor", 4, 5);
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

void editor::shutdown() { sdl_context::get_instance().shutdown(); }

void editor::late_serialize(nlohmann::json &j) {
  nlohmann::json editor_settings;
  editor_settings["active_camera"] = active_camera;
  editor_settings["camera_manipulate_data"] = cam_manip_data;
  j["editor"] = editor_settings;
}

void editor::late_deserialize(nlohmann::json &j) {
  if (j.contains("editor")) {
    active_camera = j["editor"]["active_camera"].get<entt::entity>();
    cam_manip_data = j["editor"]["camera_manipulate_data"]
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
  ss_handler = get_sys<sub_system_handler>();
  anim_sys = get_sys<anim::anim_system>();
}

void editor::game_mode_main_loop() {
  auto &g_instance = sdl_context::get_instance();
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

  if (g_instance.wnd_resized) {
    scene_wnd_size.x() = g_instance.wnd_width;
    scene_wnd_size.y() = g_instance.wnd_height;
    render_sys->resize(g_instance.wnd_width, g_instance.wnd_height);
  }
  render_sys->render(registry, active_cam_trans, active_cam_comp);

  glClearColor(0, 0, 0, 1);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glViewport(0, 0, g_instance.wnd_width, g_instance.wnd_height);

  quad_program.use();
  quad_program.set_texture2d("scene_tex",
                             render_sys->get_target_texture().get_handle(), 0);
  quad_draw_call();
  g_instance.swap_buffer();
}
void editor::editor_mode_main_loop() {
  auto &g_instance = sdl_context::get_instance();
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
  glViewport(0, 0, g_instance.wnd_width, g_instance.wnd_height);

  g_instance.begin_imgui();

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

  g_instance.end_imgui();

  g_instance.swap_buffer();
}

void editor::run() {
  auto &g_instance = sdl_context ::get_instance();
  timer.reset();
  add_default_objects();
  g_instance.run([&]() {
    if (g_instance.is_key_triggered(SDLK_0) &&
        g_instance.is_key_pressed(SDLK_LCTRL)) {
      scene_wnd_size.x() = g_instance.wnd_width;
      scene_wnd_size.x() = g_instance.wnd_height;
      render_sys->resize(g_instance.wnd_width, g_instance.wnd_height);
      in_game_mode = !in_game_mode;
    }
    if (!in_game_mode)
      editor_mode_main_loop();
    else
      game_mode_main_loop();
  });
}

void editor::reset() {
  registry.clear();
  systems.clear();

  transform_sys = add_sys<transform_system>();
  render_sys = add_sys<defered_render_system>();
  ss_handler = add_sys<sub_system_handler>();
  anim_sys = add_sys<anim::anim_system>();
  phy_sys = add_sys<sim::phy_system>();
}

void editor::add_default_objects() {
  auto ent = registry.create();
  auto &trans = registry.emplace<transform>(ent);
  trans.name = "main camera";
  trans.set_world_pos(math::vector3(0, 0, 5));
  auto &cam_comp = registry.emplace<camera>(ent);
  active_camera = ent;
}

void editor::editor_shortkeys() {
  auto &g_instance = sdl_context::get_instance();
  auto cursor_pos = g_instance.get_mouse_position();
  if (cursor_pos.x() > scene_wnd_pos.x() &&
      cursor_pos.x() < (scene_wnd_pos.x() + scene_wnd_size.x()) &&
      cursor_pos.y() > scene_wnd_pos.y() &&
      cursor_pos.y() < (scene_wnd_pos.y() + scene_wnd_size.y())) {
    // only change the gizmo operation mode
    // if the cursor is inside scene window
    if (g_instance.is_key_pressed(SDLK_1)) {
      with_translate = true;
      with_rotate = false;
      with_scale = false;
    }
    if (g_instance.is_key_pressed(SDLK_2)) {
      with_translate = false;
      with_rotate = true;
      with_scale = false;
    }
    if (g_instance.is_key_pressed(SDLK_3)) {
      with_translate = false;
      with_rotate = false;
      with_scale = true;
    }

    // detect mouse click selection
    if (g_instance.is_key_pressed(SDLK_LCTRL) &&
        g_instance.is_mouse_button_triggered(SDL_BUTTON_LEFT)) {
      math::vector3 ray_o, ray_d;
      if (mouse_query_ray(ray_o, ray_d)) {
        std::priority_queue<ray_query_data, std::vector<ray_query_data>,
                            compare_ray_query_data>
            q;
        registry.view<entt::entity, transform>().each(
            [&](entt::entity entity, transform &trans) {
              if (entity == active_camera)
                return;
              ray_query_data data;
              data.entity = entity;
              data.dist = (trans.world_pos() - ray_o).norm();
              auto h = (trans.world_pos() - ray_o).dot(ray_d) * ray_d;
              data.pdist = ((trans.world_pos() - ray_o) - h).norm();
              q.emplace(data);
            });
        if (!q.empty()) {
          auto selection = q.top();
          SDL_Log("Click selection nearest, "
                  "name:\"{0}\",pdist:{1},dist:{2},sin:{3}",
                  registry.get<transform>(selection.entity).name,
                  selection.pdist, selection.dist,
                  selection.pdist / selection.dist);
          if (selection.pdist / selection.dist <= click_selection_max_sin) {
            q.pop();
            selection_candidates.clear();
            selection_candidates.push_back(selection);
            int counter = 0;
            while (!q.empty() && counter < 3) {
              auto cand = q.top();
              q.pop();
              if (cand.pdist / cand.dist <= click_selection_max_sin)
                selection_candidates.push_back(cand);
              counter++;
            }

            if (selection_candidates.size() <= 1) {
              selected_entity = selection.entity;
            } else {
              // open a popup to select all potential candidates
              ImGui::OpenPopup("clickselectioncandidates");
            }
          } else {
            SDL_Log("Nearest selection too far, set selection to null");
            selected_entity = entt::null;
          }
        }
      }
    }
  }
  if (selection_candidates.size() > 0 &&
      ImGui::BeginPopup("clickselectioncandidates")) {
    ImGui::MenuItem("Selection candidates", nullptr, nullptr, false);
    ImGui::Separator();
    for (int i = 0; i < selection_candidates.size(); i++) {
      if (ImGui::Selectable(
              registry.get<transform>(selection_candidates[i].entity)
                  .name.c_str())) {
        selected_entity = selection_candidates[i].entity;
        ImGui::CloseCurrentPopup();
      }
    }
    ImGui::EndPopup();
  }
}

bool editor::screen_query_ray(math::vector2 scrn_pos, math::vector3 &o,
                              math::vector3 &d) {
  if (!registry.valid(active_camera)) {
    SDL_Log("Scene active camera invalid, failed to call mouse_query_ray");
    return false;
  }
  if (auto cam_comp = registry.try_get<camera>(active_camera)) {
    math::vector4 ndc_pos = math::vector4(
        scrn_pos.x() * 2.0f - 1.0f, scrn_pos.y() * 2.0f - 1.0f, 0.0f, 1.0f);
    auto &cam_trans = registry.get<transform>(active_camera);
    math::vector4 p0 = cam_comp->vp.inverse() * ndc_pos;
    math::vector3 world_pos =
        math::vector3(p0.x() / p0.w(), p0.y() / p0.w(), p0.z() / p0.w());
    o = cam_trans.world_pos();
    d = (world_pos - o).normalized();

    return true;
  } else {
    SDL_Log("Scene active camera don't possess a camera component, "
            "failed to call mouse_query_ray");
    return false;
  }
}

bool editor::mouse_query_ray(math::vector3 &o, math::vector3 &d) {
  auto scrn_pos = sdl_context::get_instance().get_mouse_position();
  scrn_pos.x() -= scene_wnd_pos.x();
  scrn_pos.y() = scene_wnd_size.y() - scrn_pos.y() + scene_wnd_pos.y();
  scrn_pos.x() /= scene_wnd_size.x();
  scrn_pos.y() /= scene_wnd_size.y();
  return screen_query_ray(scrn_pos, o, d);
}

void editor::active_camera_manipulate(float dt) {
  auto &g_instance = sdl_context::get_instance();
  auto cursor_pos = g_instance.get_mouse_position();
  if (auto cam_trans = registry.try_get<transform>(active_camera)) {
    // focus on selected entity if `F` is triggered
    if (selected_entity != entt::null && g_instance.is_key_triggered(SDLK_f)) {
      SDL_Log("Focus camera \"{0}\" to selected entity \"{1}\"",
              registry.get<transform>(active_camera).name,
              registry.get<transform>(selected_entity).name);
      cam_manip_data.camera_pivot =
          registry.get<transform>(selected_entity).world_pos();
    }
    auto cam_comp = registry.get<camera>(active_camera);
    auto cam_pos = cam_trans->world_pos();
    if ((cam_pos - cam_manip_data.camera_pivot).norm() < 1e-9f) {
      cam_manip_data.camera_pivot = cam_pos - cam_trans->local_forward();
      SDL_Log("push pivot away from camera");
    }
    // scroll movement delta, scale with the distance to pivot
    float movement_delta =
        cam_manip_data.initial_factor * dt *
        std::min(std::pow((cam_pos - cam_manip_data.camera_pivot).norm(),
                          cam_manip_data.speed_pow),
                 cam_manip_data.max_speed);
    if (cursor_pos.x() > scene_wnd_pos.x() &&
        cursor_pos.x() < (scene_wnd_pos.x() + scene_wnd_size.x()) &&
        cursor_pos.y() > scene_wnd_pos.y() &&
        cursor_pos.y() < (scene_wnd_pos.y() + scene_wnd_size.y())) {
      // check action queue for mouse scroll event
      math::vector2 scrollOffset = g_instance.get_scroll_offsets();
      cam_trans->set_world_pos(cam_trans->world_pos() -
                               cam_trans->local_forward() * scrollOffset.y() *
                                   movement_delta);
    }
    bool press_mouse_mid_btn =
        g_instance.is_mouse_button_pressed(SDL_BUTTON_MIDDLE);
    bool press_mouse_right_btn =
        g_instance.is_mouse_button_pressed(SDL_BUTTON_RIGHT);
    math::vector2 mouse_current_pos = g_instance.get_mouse_position();
    // only handle mouse input when cursor in scene window
    if ((press_mouse_mid_btn || press_mouse_right_btn) &&
        (mouse_current_pos.x() > 0 &&
         mouse_current_pos.x() < g_instance.wnd_width &&
         mouse_current_pos.y() > 0 &&
         mouse_current_pos.y() < g_instance.wnd_width)) {
      if (cam_manip_data.mouse_first_move) {
        cam_manip_data.mouse_last_pos = mouse_current_pos;
        cam_manip_data.mouse_first_move = false;
      }
      math::vector2 mouse_offset =
          mouse_current_pos - cam_manip_data.mouse_last_pos;
      // free fps-style camera
      if (press_mouse_right_btn) {
        // modify the cameraPivot position to suite cursor movement
        if (mouse_offset.norm() > 1e-2f) {
          math::vector3 ray_o, ray_d;
          math::vector2 screen_pos =
              math::vector2(scene_wnd_size.x() / 2.0f +
                                mouse_offset.x() * cam_manip_data.fps_speed,
                            scene_wnd_size.y() / 2.0f -
                                mouse_offset.y() * cam_manip_data.fps_speed);
          screen_pos.x() /= scene_wnd_size.x();
          screen_pos.y() /= scene_wnd_size.y();
          if (screen_query_ray(screen_pos, ray_o, ray_d)) {
            cam_manip_data.camera_pivot =
                ray_o +
                ray_d * (cam_manip_data.camera_pivot - cam_trans->world_pos())
                            .norm();
          }
        }
        // move camera position with wasd key board
        math::vector3 camera_movement = math::vector3::Zero();
        math::vector3 cam_vec =
            (cam_trans->world_pos() - cam_manip_data.camera_pivot).normalized();
        if (g_instance.is_key_pressed(SDLK_w))
          camera_movement -= cam_trans->local_forward();
        if (g_instance.is_key_pressed(SDLK_s))
          camera_movement += cam_trans->local_forward();
        if (g_instance.is_key_pressed(SDLK_a))
          camera_movement -= cam_trans->local_right();
        if (g_instance.is_key_pressed(SDLK_d))
          camera_movement += cam_trans->local_right();
        camera_movement *= (cam_manip_data.fps_camera_speed * dt);
        cam_manip_data.camera_pivot += camera_movement;
        cam_trans->set_world_pos(cam_trans->world_pos() + camera_movement);
      } else if (press_mouse_mid_btn) {
        // rotate the camera around the pivot, or translate the camera
        if (g_instance.is_key_pressed(SDLK_LSHIFT)) {
          // translate the camera according to nfc offset
          if (mouse_offset.norm() > 1e-2f) {
            math::vector2 scene_size = scene_wnd_size;
            math::vector4 nfcPos = {-mouse_offset.x() / scene_size.x(),
                                    mouse_offset.y() / scene_size.y(), 1.0f,
                                    1.0f};
            math::vector4 worldRayPos =
                cam_comp.view.inverse() * cam_comp.proj.inverse() * nfcPos;
            worldRayPos /= worldRayPos.w();
            math::vector3 worldRayDir =
                (worldRayPos.head<3>() - cam_pos).normalized();
            worldRayDir =
                worldRayDir.dot(cam_manip_data.camera_pivot - cam_pos) *
                worldRayDir;
            auto deltaPos =
                worldRayDir.dot(cam_trans->local_right()) *
                    cam_trans->local_right() +
                worldRayDir.dot(cam_trans->local_up()) * cam_trans->local_up();
            cam_manip_data.camera_pivot += deltaPos;
            cam_trans->set_world_pos(cam_trans->world_pos() + deltaPos);
          }
        } else {
          // repose the camera
          auto rotateOffset = mouse_offset * 0.1f;
          math::vector3 posVector = cam_trans->world_pos();
          math::vector3 newPos =
              math::angle_axis(math::deg_to_rad(-rotateOffset.x()),
                               math::world_up) *
                  math::angle_axis(math::deg_to_rad(-rotateOffset.y()),
                                   cam_trans->local_right()) *
                  (posVector - cam_manip_data.camera_pivot) +
              cam_manip_data.camera_pivot;
          cam_trans->set_world_pos(newPos);
        }
      }
      cam_manip_data.mouse_last_pos = mouse_current_pos;
    } else
      cam_manip_data.mouse_first_move = true;

    math::vector3 lastLeft = cam_trans->local_right();
    math::vector3 forward =
        (cam_trans->world_pos() - cam_manip_data.camera_pivot).normalized();
    math::vector3 up = math::world_up;
    math::vector3 left = up.cross(forward).normalized();
    // flip left if non-consistent
    if (lastLeft.dot(left) < 0.0f)
      left *= -1;
    up = (forward.cross(left)).normalized();
    math::matrix3 rot;
    rot << left, up, forward;
    cam_trans->set_world_rot(math::quat(rot));
  }
}

void editor::draw_gizmos(bool enable) {
  if (enable && registry.valid(active_camera)) {
    ImGuizmo::AllowAxisFlip(false);
    ImGuizmo::SetDrawlist();
    ImGuizmo::SetRect(scene_wnd_pos.x(), scene_wnd_pos.y(), scene_wnd_size.x(),
                      scene_wnd_size.y());
    auto &camTrans = registry.get<transform>(active_camera);
    auto &camComp = registry.get<camera>(active_camera);
    if (registry.valid(selected_entity)) {
      auto &selTrans = registry.get<transform>(selected_entity);
      math::matrix4 transform = selTrans.matrix();
      if (ImGuizmo::Manipulate(camComp.view.data(), camComp.proj.data(),
                               current_gizmo_operation(), current_gizmo_mode,
                               transform.data(), NULL, NULL)) {
        // update object transform with modified changes
        if ((current_gizmo_operation() & ImGuizmo::TRANSLATE) != 0) {
          math::vector3 position(transform(0, 3), transform(1, 3),
                                 transform(2, 3));
          selTrans.set_world_pos(position);
        }
        math::vector3 scale(transform.col(0).norm(), transform.col(1).norm(),
                            transform.col(2).norm());
        if ((current_gizmo_operation() & ImGuizmo::ROTATE) != 0) {
          math::matrix4 rotation;
          rotation << transform.col(0) / scale.x(),
              transform.col(1) / scale.y(), transform.col(2) / scale.z(),
              math::vector4(0.0, 0.0, 0.0, 1.0);
          selTrans.set_world_rot(math::quat(rotation.block<3, 3>(0, 0)));
        }
        if ((current_gizmo_operation() & ImGuizmo::SCALE) != 0)
          selTrans.set_world_scale(scale);
      }
    }
  }
}

void editor::draw_main_menubar() {
  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("File")) {
      // ---------------------- Scene save/load menu ----------------------
      ImGui::MenuItem("Scene", nullptr, false, false);
      if (ImGui::MenuItem("Reset Scene")) {
        reset();
        add_default_objects();
        SDL_Log("Reset scene");
      }
      if (ImGui::MenuItem("Save  Scene")) {
        std::string filepath;
        if (save_file_dialog("Serialize scene file", {"*.scene"}, filepath)) {
          auto data = serialize();
          std::ofstream output(filepath);
          if (output.is_open()) {
            output << data.dump() << std::endl;
            output.close();
            SDL_Log("Save scene to %s", filepath.c_str());
          } else {
            SDL_Log("Failed to save scene to %s", filepath.c_str());
          }
        }
      }
      if (ImGui::MenuItem("Load  Scene")) {
        std::string filepath;
        if (open_file_dialog("Deserialize scene file", {"*.scene"}, filepath)) {
          std::ifstream input(filepath);
          if (input.is_open()) {
            auto data = nlohmann::json::parse(
                std::string((std::istreambuf_iterator<char>(input)),
                            std::istreambuf_iterator<char>()));
            deserialize(data);
            SDL_Log("Load scene from %s", filepath.c_str());
          } else {
            SDL_Log("Failed to load scene from %s", filepath.c_str());
          }
        }
      }
      ImGui::Separator();

      ImGui::MenuItem("Bundle", nullptr, false, false);
      if (ImGui::MenuItem("Import Bundle")) {
        std::string filepath;
        if (open_file_dialog("Import bundle to current scene", {"*.bundle"},
                             filepath)) {
          std::ifstream input(filepath);
          if (input.is_open()) {
            auto data = nlohmann::json::parse(
                std::string((std::istreambuf_iterator<char>(input)),
                            std::istreambuf_iterator<char>()));
            load_bundle(data);
            SDL_Log("Import bundle to current scene from %s", filepath.c_str());
          } else {
            SDL_Log("Failed to import bundle from %s", filepath.c_str());
          }
          input.close();
        }
      }
      ImGui::Separator();

      // ---------------------- Assets save/load menu ----------------------
      ImGui::MenuItem("Assets", nullptr, false, false);
      if (ImGui::MenuItem("Import Model")) {
        std::string filepath;
        if (open_file_dialog("Open model asset file",
                             {"*.fbx", "*.FBX", "*.obj", "*.OBJ", "*.pmx",
                              "*.PMX", "*.ply", "*.PLY", "*.stl", "*.STL"},
                             filepath)) {
          // assets::open_model_assimp(registry, filepath);
          if (endswith(filepath, ".FBX") || endswith(filepath, ".fbx")) {
            auto root_entity = assets::open_model_ufbx(registry, filepath);
            if (root_entity != entt::null)
              SDL_Log("Load model file %s with ufbx, mount at entity %s",
                      filepath.c_str(),
                      registry.get<transform>(root_entity).name.c_str());
          } else if (endswith(filepath, ".OBJ") || endswith(filepath, ".obj")) {
            std::vector<assets::mesh> loaded_meshes;
            if (assets::load_obj_mesh(filepath, loaded_meshes)) {
#ifdef _WIN32
              std::string filename = assets::wstring_to_string(
                  std::filesystem::u8path(filepath).filename().wstring());
#else
              std::string filename =
                  std::filesystem::path(filepath).filename().string();
#endif
              if (loaded_meshes.size() == 1) {
                auto root_entity = registry.create();
                auto &root_trans = registry.emplace<transform>(root_entity);
                root_trans.name = filename;
                auto &mesh_data =
                    registry.emplace<opengl::mesh_data>(root_entity);
                mesh_data.mesh_name = loaded_meshes[0].name;
                mesh_data.vertices = loaded_meshes[0].vertices;
                mesh_data.indices = loaded_meshes[0].indices;
                opengl::init_opengl_buffers(mesh_data);
              } else if (loaded_meshes.size() > 1) {
                auto root_entity = registry.create();
                auto &root_trans = registry.emplace<transform>(root_entity);
                root_trans.name = filename;
                for (int i = 0; i < loaded_meshes.size(); i++) {
                  auto mesh_entity = registry.create();
                  auto &mesh_trans = registry.emplace<transform>(mesh_entity);
                  root_trans.add_child(mesh_entity);
                  mesh_trans.name = loaded_meshes[i].name;
                  auto &mesh_data =
                      registry.emplace<opengl::mesh_data>(mesh_entity);
                  mesh_data.mesh_name = loaded_meshes[i].name;
                  mesh_data.vertices = loaded_meshes[i].vertices;
                  mesh_data.indices = loaded_meshes[i].indices;
                  opengl::init_opengl_buffers(mesh_data);
                }
              } else {
                SDL_Log("Loaded model has zero meshes, filepath %s",
                        filepath.c_str());
              }
              SDL_Log("Load mesh from %s", filepath.c_str());
            } else {
              SDL_Log("Failed to load mesh from %s", filepath.c_str());
            }
          } else {
            auto root_entity = assets::open_model_assimp(registry, filepath);
            if (root_entity != entt::null)
              SDL_Log("Load model file %s with assimp, mount at entity %s",
                      filepath.c_str(),
                      registry.get<transform>(root_entity).name.c_str());
          }
        }
      }
      if (ImGui::MenuItem("Import BVH")) {
        std::string filepath;
        if (open_file_dialog("Import .bvh motion file", {"*.bvh", "*.BVH"},
                             filepath)) {
          SDL_Log("Load motion file %s", filepath.c_str());
          auto container = registry.create();
          auto &container_trans = registry.emplace<transform>(container);
          container_trans.name =
              std::filesystem::path(filepath).filename().string();
          auto &motion_player =
              registry.emplace<anim::bvh_motion_player>(container);
          motion_player.load_motion(registry, filepath);
        }
      }
      if (ImGui::MenuItem("Import All BVH")) {
        std::string dirpath;
        if (open_folder_dialog("Import folder containing .bvh files",
                               dirpath)) {
          SDL_Log("Load all .bvh motion files under %s", dirpath.c_str());
          anim::import_all_bvh_motion(registry, dirpath);
        }
      }
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Settings")) {
      // system configuration
      ImGui::MenuItem("Configure Systems", nullptr, false, false);
      for (auto &sys : systems) {
        if (ImGui::BeginMenu(sys->get_name().c_str())) {
          ImGui::Checkbox("Active", &(sys->active));
          ImGui::Separator();
          sys->draw_menu_gui();
          ImGui::EndMenu();
        }
      }

      ImGui::Separator();
      ImGui::MenuItem("Editor Settings", nullptr, false, false);
      std::vector<std::string> valid_camera_names;
      std::vector<entt::entity> valid_cameras;
      int active_camera_index = -1, tmp_counter = 0;
      registry.view<camera, transform>().each(
          [&](entt::entity entity, camera &cam, transform &trans) {
            if (entity == active_camera)
              active_camera_index = tmp_counter;
            valid_cameras.push_back(entity);
            valid_camera_names.push_back(str_format(
                "%s: %d", trans.name.c_str(), entt::to_integral(entity)));
            tmp_counter++;
          });
      gui::combo_default("active camera", active_camera_index,
                         valid_camera_names, [&](int current) {
                           if (current == -1)
                             active_camera = entt::null;
                           else
                             active_camera = valid_cameras[current];
                         });

      gui::combo("gizmo mode", gizmo_mode_idx, {"world", "local"},
                 [&](int current) {
                   if (current == 1)
                     current_gizmo_mode = ImGuizmo::MODE::LOCAL;
                   else
                     current_gizmo_mode = ImGuizmo::MODE::WORLD;
                 });
      ImGui::Checkbox("Editor Manipulate Camera", &editor_manipulate_camera);
      if (ImGui::Checkbox("Turn On VSync", &vsync_on)) {
        sdl_context::get_instance().set_vsync_state(vsync_on);
      }

      ImGui::EndMenu();
    }

    static stopwatch _timer;
    static int _frameCount = 0, _displayFPS = 0;
    static float _frameCountTimer = 0.0f, _displayFT = 0.0;

    auto deltaTime = _timer.elapse_s();
    _frameCount += 1;
    _frameCountTimer += deltaTime;
    if (_frameCountTimer >= 1.0f) {
      _displayFPS = _frameCount;
      _displayFT = 1000.0f / _frameCount;
      _frameCount = 0;
      _frameCountTimer = 0.0f;
    }
    _timer.reset();
    ImGui::SameLine(ImGui::GetWindowWidth() -
                    ImGui::CalcTextSize("Frame Time: 0.000 ms, FPS: 000000").x -
                    ImGui::GetStyle().ItemSpacing.x);
    ImGui::PushStyleColor(ImGuiCol_Text, {1.0, 1.0, 0.0, 1.0});
    ImGui::Text("Frame Time: %.3f ms, FPS: %d", _displayFT, _displayFPS);
    ImGui::PopStyleColor();

    ImGui::EndMainMenuBar();
  }
}

void draw_entity_hierarchy_recursive(
    entt::registry &registry, entt::entity &selected, entt::entity current,
    ImGuiTreeNodeFlags flag,
    std::function<void(entt::entity)> rightClickEntity) {
  bool currentSelected = (selected == current);
  ImGuiTreeNodeFlags finalFlag = flag;
  if (currentSelected)
    finalFlag |= ImGuiTreeNodeFlags_Selected;
  auto &current_transform = registry.get<transform>(current);
  if (current_transform.m_children.size() == 0)
    finalFlag |= ImGuiTreeNodeFlags_Bullet;

  // Draw current node
  std::string entity_name = current_transform.name;
  if (registry.try_get<anim::bone_node>(current)) {
    entity_name = str_format("(骨骼) %s", entity_name.c_str());
  } else if (registry.try_get<camera>(current)) {
    entity_name = str_format("(摄像机) %s", entity_name.c_str());
  } else if (registry.try_get<point_light>(current)) {
    entity_name = str_format("(点光源) %s", entity_name.c_str());
  } else if (registry.try_get<mesh_data>(current)) {
    entity_name = str_format("(网格) %s", entity_name.c_str());
  }

  bool nodeOpen =
      ImGui::TreeNodeEx((void *)(intptr_t)entt::to_integral(current), finalFlag,
                        entity_name.c_str());
  // if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen())
  //   selected = current;

  // Drag & Drop
  if (ImGui::BeginDragDropSource()) {
    ImGui::SetDragDropPayload("ENTITY", &(current), sizeof(entt::entity));
    ImGui::Text("Drag drop to change hierarchy");
    ImGui::EndDragDropSource();
  }
  if (ImGui::BeginDragDropTarget()) {
    if (const ImGuiPayload *payload = ImGui::AcceptDragDropPayload("ENTITY")) {
      auto entity = *(entt::entity *)payload->Data;
      current_transform.add_child(entity);
    }
    ImGui::EndDragDropTarget();
  }

  // Right click context menu
  if (ImGui::BeginPopupContextItem(
          (current_transform.name + std::to_string(entt::to_integral(current)))
              .c_str(),
          ImGuiPopupFlags_MouseButtonRight)) {
    rightClickEntity(current);
    ImGui::EndPopup();
  }

  // Draw children nodes
  if (nodeOpen) {
    for (auto c : current_transform.m_children)
      draw_entity_hierarchy_recursive(registry, selected, c, flag,
                                      rightClickEntity);
    ImGui::TreePop();
  }
}

void editor::draw_entity_hierarchy() {
  static ImGuiTreeNodeFlags guiTreeNodeFlags =
      ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick |
      ImGuiTreeNodeFlags_SpanAvailWidth;
  ImGui::Begin("Entities");

  static char headerBuffer[200] = {0};
  bool selectedEntityValid = registry.valid(selected_entity);
  sprintf(headerBuffer, "Select Entity \"%s\": %d",
          selectedEntityValid
              ? registry.get<transform>(selected_entity).name.c_str()
              : "NULL",
          selectedEntityValid ? entt::to_integral(selected_entity) : -1);
  ImGui::Text(headerBuffer);
  ImGui::Separator();

  auto right_click_menu = [&]() {
    ImGui::SeparatorText("Operation");
    if (ImGui::BeginMenu("Create")) {
      if (ImGui::MenuItem("New Entity")) {
        auto ent = registry.create();
        auto &trans = registry.emplace<transform>(ent);
        trans.name = str_format("new entity: %d", entt::to_integral(ent));
      }

      ImGui::Separator();
      if (ImGui::MenuItem("New Cube"))
        create_cube(registry);
      if (ImGui::MenuItem("New Sphere"))
        create_sphere(registry);
      if (ImGui::MenuItem("New Cylinder"))
        create_cylinder(registry);
      if (ImGui::MenuItem("New Plane"))
        create_plane(registry);

      ImGui::Separator();
      if (ImGui::MenuItem("New Point Light")) {
        auto ent = registry.create();
        auto &trans = registry.emplace<transform>(ent);
        auto number = registry.view<point_light>().size();
        trans.name = str_format("Point Light (%d)", number);
        auto &light = registry.emplace<point_light>(ent);
      }

      ImGui::EndMenu();
    }
  };
  auto right_click_entity = [&](entt::entity entity) {
    ImGui::SeparatorText("Operation");
    if (ImGui::MenuItem("Select")) {
      selected_entity = entity;
    }
    if (ImGui::MenuItem("Make Bundle")) {
      std::string filepath;
      if (save_file_dialog("Save entity hierarchy as bundle", {"*.bundle"},
                           filepath)) {
        auto data = make_bundle(entity);
        std::ofstream output(filepath);
        if (output.is_open()) {
          output << data.dump() << std::endl;
          SDL_Log("Save bundle to filepath %s", filepath.c_str());
        } else {
          SDL_Log("Failed to save bundle to filepath %s", filepath.c_str());
        }
        output.close();
      }
    }
    if (ImGui::MenuItem("Clear Parent")) {
      auto &trans = registry.get<transform>(entity);
      trans.remove_parent();
    }
    if (ImGui::MenuItem("Delete Entity")) {
      destroy_hierarchy(registry, entity);
    }
  };

  if (!ImGui::IsAnyItemHovered() &&
      ImGui::IsWindowHovered(ImGuiHoveredFlags_RootAndChildWindows)) {
    // open the window context menu
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Right))
      ImGui::OpenPopup("DrawEntityHierarchy_rightclickblank");
    // // unselect entities
    // if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
    //   selected_entity = entt::null;
  }
  // ---------------------- Right click menu ----------------------
  if (ImGui::BeginPopup("DrawEntityHierarchy_rightclickblank")) {
    right_click_menu();
    ImGui::EndPopup();
  }

  // ---------------------- Entity list ----------------------
  ImGui::BeginChild("DrawEntityHierarchy_entityhierarchy",
                    ImGui::GetContentRegionAvail());
  for (auto ent : transform_sys->root_entities)
    draw_entity_hierarchy_recursive(registry, selected_entity, ent,
                                    guiTreeNodeFlags, right_click_entity);
  ImGui::EndChild();
  ImGui::End();
}

void editor::draw_entity_components() {
  ImGui::Begin("Components");
  static char headerBuffer[200] = {0};
  static entt::entity current_entity = entt::null;
  bool selectedEntityValid = registry.valid(current_entity);
  sprintf(headerBuffer, "Inspect Entity \"%s\": %d",
          selectedEntityValid
              ? registry.get<transform>(current_entity).name.c_str()
              : "NULL",
          selectedEntityValid ? entt::to_integral(current_entity) : -1);
  static bool pinCurrentEntity = false;
  ImGui::Text(headerBuffer);
  if (ImGui::Checkbox("Pin", &pinCurrentEntity) && pinCurrentEntity)
    current_entity = selected_entity;
  if (!pinCurrentEntity)
    current_entity = selected_entity;
  ImGui::Separator();

  if (registry.valid(current_entity)) {
    if (ImGui::Button("Add Component", {-1, 30})) {
      ImGui::OpenPopup("DrawEntityComponents_addcomponent");
    }
    if (ImGui::BeginPopup("DrawEntityComponents_addcomponent")) {
      ImGui::MenuItem("Component List", nullptr, nullptr, false);
      ImGui::Separator();
      for (auto &p : toolkit::iapp::__add_comp_map__) {
        if (ImGui::BeginMenu(p.first.c_str())) {
          for (auto &i : p.second) {
            if (ImGui::MenuItem(i.first.c_str())) {
              SDL_Log("create component %s for entity %d", i.first.c_str(),
                      entt::to_integral(current_entity));
              i.second(registry, current_entity);
            }
          }
          ImGui::EndMenu();
        }
      }
      ImGui::EndPopup();
    }
    ImGui::Separator();

    for (auto &sys : systems)
      sys->draw_gui(registry, current_entity);
  } else
    current_entity = entt::null;

  ImGui::End();
}

}; // namespace toolkit::opengl

namespace toolkit::assets {

#ifdef _WIN32
#include <Windows.h>
std::string wstring_to_string(const std::wstring &wstr) {
  int buffer_size = WideCharToMultiByte(
      CP_UTF8,         // UTF-8 encoding for Chinese support
      0,               // No flags
      wstr.c_str(),    // Input wide string
      -1,              // Auto-detect length
      nullptr, 0,      // Null to calculate required buffer size
      nullptr, nullptr // Optional parameters (not needed here)
  );

  if (buffer_size == 0)
    return ""; // Handle error if needed

  std::string str(buffer_size, 0);
  WideCharToMultiByte(CP_UTF8, 0, wstr.c_str(), -1, &str[0], buffer_size,
                      nullptr, nullptr);
  str.pop_back(); // Remove null terminator added by -1
  return str;
}
#endif

}; // namespace toolkit::assets
