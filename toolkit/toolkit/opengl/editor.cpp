#include "toolkit/opengl/editor.hpp"
#include "toolkit/anim/components/actor.hpp"
#include "toolkit/anim/scripts/vis.hpp"
#include "toolkit/opengl/components/materials/all.hpp"
#include "toolkit/opengl/components/mesh.hpp"
#include "toolkit/opengl/gui/utils.hpp"
#include "toolkit/opengl/scripts/test_draw.hpp"

namespace toolkit::opengl {

void editor::init() {
  auto &instance = context::get_instance();
  instance.init(1920, 1080, "Editor", 4, 6);
  reset();

  // init imgui
  imgui_io = &ImGui::GetIO();
  imgui_io->ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  imgui_io->ConfigFlags |= ImGuiConfigFlags_DockingEnable;
}

void editor::shutdown() { g_instance.shutdown(); }

void editor::late_serialize(nlohmann::json &j) {
  nlohmann::json editor_settings;
  editor_settings["active_camera"] = g_instance.active_camera;
  editor_settings["camera_manipulate_data"] = cam_manip_data;
  j["editor"] = editor_settings;
}

void editor::late_deserialize(nlohmann::json &j) {
  if (j.contains("editor")) {
    g_instance.active_camera = j["editor"]["active_camera"].get<entt::entity>();
    cam_manip_data = j["editor"]["camera_manipulate_data"]
                         .get<active_camera_manipulate_data>();
  } else {
    // use the first camera as active camera, otherwise no active camera
    auto cam_view = registry.view<camera>();
    if (cam_view.size() == 0)
      g_instance.active_camera = entt::null;
    else
      g_instance.active_camera = *(cam_view.begin());
  }

  transform_sys = get_sys<transform_system>();
  render_sys = get_sys<defered_forward_mixed>();
  script_sys = get_sys<script_system>();
  anim_sys = get_sys<anim::anim_system>();
}

void editor::run() {
  auto &instance = context::get_instance();
  timer.reset();

  add_default_objects();

  instance.run([&]() {
    float dt = timer.elapse_s();
    timer.reset();

    transform_sys->update_transform(registry);
    render_sys->update_scene_buffers(registry);
    active_camera_manipulate(dt);

    for (auto sys : systems)
      if (sys->active)
        sys->preupdate(registry, dt);
    if (script_sys->active)
      script_sys->preupdate(this, dt);
    for (auto sys : systems)
      if (sys->active)
        sys->update(registry, dt);
    if (script_sys->active)
      script_sys->update(this, dt);
    for (auto sys : systems)
      if (sys->active)
        sys->lateupdate(registry, dt);
    if (script_sys->active)
      script_sys->lateupdate(this, dt);

    render_sys->render(registry);

    glClearColor(0, 0, 0, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glViewport(0, 0, instance.wnd_width, instance.wnd_height);

    instance.begin_imgui();

    ImGui::DockSpaceOverViewport(ImGui::GetMainViewport());
    ImGui::Begin("Scene");
    ImGui::BeginChild("GameRenderer");
    auto size = ImGui::GetContentRegionAvail();
    auto pos = ImGui::GetWindowPos();
    instance.scene_pos_x = pos.x;
    instance.scene_pos_y = pos.y;
    ImGui::Image((void *)static_cast<std::uintptr_t>(
                     render_sys->get_target_texture().get_handle()),
                 {size.x, size.y}, ImVec2(0, 1), ImVec2(1, 0));
    if (instance.scene_width != size.x || instance.scene_height != size.y) {
      // resize sceneFBO
      instance.scene_width = size.x;
      instance.scene_height = size.y;
      render_sys->resize(size.x, size.y);
      render_sys->render(registry);
    }
    draw_gizmos();
    ImGui::EndChild();
    ImGui::End();

    draw_main_menubar();
    draw_entity_hierarchy();
    draw_entity_components();
    editor_shortkeys();

    instance.end_imgui();

    instance.swap_buffer();
  });
}

void editor::reset() {
  registry.clear();
  systems.clear();

  transform_sys = add_sys<transform_system>();
  render_sys = add_sys<defered_forward_mixed>();
  script_sys = add_sys<script_system>();
  anim_sys = add_sys<anim::anim_system>();
  phy_sys = add_sys<physics::physics_system>();
}

void editor::add_default_objects() {
  auto ent = registry.create();
  auto &trans = registry.emplace<transform>(ent);
  trans.name = "main camera";
  trans.set_world_pos(math::vector3(0, 0, 5));
  auto &cam_comp = registry.emplace<camera>(ent);
  g_instance.active_camera = ent;
}

void editor::editor_shortkeys() {
  if (g_instance.cursor_in_scene_window()) {
    // only change the gizmo operation mode
    // if the cursor is inside scene window
    if (g_instance.is_key_pressed(GLFW_KEY_1)) {
      with_translate = true;
      with_rotate = false;
      with_scale = false;
    }
    if (g_instance.is_key_pressed(GLFW_KEY_2)) {
      with_translate = false;
      with_rotate = true;
      with_scale = false;
    }
    if (g_instance.is_key_pressed(GLFW_KEY_3)) {
      with_translate = false;
      with_rotate = false;
      with_scale = true;
    }

    // detect mouse click selection
    if (g_instance.is_key_pressed(GLFW_KEY_LEFT_CONTROL) &&
        g_instance.is_mouse_button_triggered(GLFW_MOUSE_BUTTON_LEFT)) {
      math::vector3 ray_o, ray_d;
      if (mouse_query_ray(ray_o, ray_d)) {
        std::priority_queue<ray_query_data, std::vector<ray_query_data>,
                            compare_ray_query_data>
            q;
        registry.view<entt::entity, transform>().each(
            [&](entt::entity entity, transform &trans) {
              if (entity == g_instance.active_camera)
                return;
              ray_query_data data;
              data.entity = entity;
              data.dist = (trans.position() - ray_o).norm();
              auto h = (trans.position() - ray_o).dot(ray_d) * ray_d;
              data.pdist = ((trans.position() - ray_o) - h).norm();
              q.emplace(data);
            });
        if (!q.empty()) {
          auto selection = q.top();
          spdlog::info("Click selection nearest, "
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
            spdlog::info("Nearest selection too far, set selection to null");
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
  if (!registry.valid(g_instance.active_camera)) {
    spdlog::error(
        "Scene active camera invalid, failed to call mouse_query_ray");
    return false;
  }
  if (auto cam_comp = registry.try_get<camera>(g_instance.active_camera)) {
    math::vector4 ndc_pos = math::vector4(
        scrn_pos.x() * 2.0f - 1.0f, scrn_pos.y() * 2.0f - 1.0f, 0.0f, 1.0f);
    auto &cam_trans = registry.get<transform>(g_instance.active_camera);
    math::vector4 p0 = cam_comp->vp.inverse() * ndc_pos;
    math::vector3 world_pos =
        math::vector3(p0.x() / p0.w(), p0.y() / p0.w(), p0.z() / p0.w());
    o = cam_trans.position();
    d = (world_pos - o).normalized();

    return true;
  } else {
    spdlog::error("Scene active camera don't possess a camera component, "
                  "failed to call mouse_query_ray");
    return false;
  }
}

bool editor::mouse_query_ray(math::vector3 &o, math::vector3 &d) {
  auto scrn_pos = g_instance.get_mouse_position();
  scrn_pos.x() -= g_instance.scene_pos_x;
  scrn_pos.y() =
      g_instance.scene_height - scrn_pos.y() + g_instance.scene_pos_y;
  scrn_pos.x() /= g_instance.scene_width;
  scrn_pos.y() /= g_instance.scene_height;
  return screen_query_ray(scrn_pos, o, d);
}

void editor::active_camera_manipulate(float dt) {
  if (auto cam_trans = registry.try_get<transform>(g_instance.active_camera)) {
    // focus on selected entity if `F` is triggered
    if (selected_entity != entt::null &&
        g_instance.is_key_triggered(GLFW_KEY_F)) {
      spdlog::info("Focus camera \"{0}\" to selected entity \"{1}\"",
                   registry.get<transform>(g_instance.active_camera).name,
                   registry.get<transform>(selected_entity).name);
      cam_manip_data.camera_pivot =
          registry.get<transform>(selected_entity).position();
    }
    auto cam_comp = registry.get<camera>(g_instance.active_camera);
    auto cam_pos = cam_trans->position();
    if ((cam_pos - cam_manip_data.camera_pivot).norm() < 1e-9f) {
      cam_manip_data.camera_pivot = cam_pos - cam_trans->local_forward();
      spdlog::info("push pivot away from camera");
    }
    // scroll movement delta, scale with the distance to pivot
    float movement_delta =
        cam_manip_data.initial_factor * dt *
        std::min(std::pow((cam_pos - cam_manip_data.camera_pivot).norm(),
                          cam_manip_data.speed_pow),
                 cam_manip_data.max_speed);
    if (g_instance.cursor_in_scene_window()) {
      // check action queue for mouse scroll event
      math::vector2 scrollOffset = g_instance.get_scroll_offsets();
      cam_trans->set_world_pos(cam_trans->position() -
                               cam_trans->local_forward() * scrollOffset.y() *
                                   movement_delta);
    }
    bool press_mouse_mid_btn =
        g_instance.is_mouse_button_pressed(GLFW_MOUSE_BUTTON_MIDDLE);
    bool press_mouse_right_btn =
        g_instance.is_mouse_button_pressed(GLFW_MOUSE_BUTTON_RIGHT);
    math::vector2 mouse_current_pos = g_instance.get_mouse_position();
    // only handle mouse input when cursor in scene window
    if ((press_mouse_mid_btn || press_mouse_right_btn) &&
        g_instance.cursor_in_scene_window()) {
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
              math::vector2(g_instance.scene_width / 2.0f +
                                mouse_offset.x() * cam_manip_data.fps_speed,
                            g_instance.scene_height / 2.0f -
                                mouse_offset.y() * cam_manip_data.fps_speed);
          screen_pos.x() /= g_instance.scene_width;
          screen_pos.y() /= g_instance.scene_height;
          if (screen_query_ray(screen_pos, ray_o, ray_d)) {
            cam_manip_data.camera_pivot =
                ray_o +
                ray_d * (cam_manip_data.camera_pivot - cam_trans->position())
                            .norm();
          }
        }
        // move camera position with wasd key board
        math::vector3 camera_movement = math::vector3::Zero();
        math::vector3 cam_vec =
            (cam_trans->position() - cam_manip_data.camera_pivot).normalized();
        if (g_instance.is_key_pressed(GLFW_KEY_W))
          camera_movement -= cam_trans->local_forward();
        if (g_instance.is_key_pressed(GLFW_KEY_S))
          camera_movement += cam_trans->local_forward();
        if (g_instance.is_key_pressed(GLFW_KEY_A))
          camera_movement -= cam_trans->local_left();
        if (g_instance.is_key_pressed(GLFW_KEY_D))
          camera_movement += cam_trans->local_left();
        camera_movement *= (cam_manip_data.fps_camera_speed * dt);
        cam_manip_data.camera_pivot += camera_movement;
        cam_trans->set_world_pos(cam_trans->position() + camera_movement);
      } else if (press_mouse_mid_btn) {
        // rotate the camera around the pivot, or translate the camera
        if (g_instance.is_key_pressed(GLFW_KEY_LEFT_SHIFT)) {
          // translate the camera according to nfc offset
          if (mouse_offset.norm() > 1e-2f) {
            math::vector2 scene_size = g_instance.get_scene_size();
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
                worldRayDir.dot(cam_trans->local_left()) *
                    cam_trans->local_left() +
                worldRayDir.dot(cam_trans->local_up()) * cam_trans->local_up();
            cam_manip_data.camera_pivot += deltaPos;
            cam_trans->set_world_pos(cam_trans->position() + deltaPos);
          }
        } else {
          // repose the camera
          auto rotateOffset = mouse_offset * 0.1f;
          math::vector3 posVector = cam_trans->position();
          math::vector3 newPos =
              math::angle_axis(math::deg_to_rad(-rotateOffset.x()),
                               math::world_up) *
                  math::angle_axis(math::deg_to_rad(-rotateOffset.y()),
                                   cam_trans->local_left()) *
                  (posVector - cam_manip_data.camera_pivot) +
              cam_manip_data.camera_pivot;
          cam_trans->set_world_pos(newPos);
        }
      }
      cam_manip_data.mouse_last_pos = mouse_current_pos;
    } else
      cam_manip_data.mouse_first_move = true;

    math::vector3 lastLeft = cam_trans->local_left();
    math::vector3 forward =
        (cam_trans->position() - cam_manip_data.camera_pivot).normalized();
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
  if (enable && registry.valid(g_instance.active_camera)) {
    ImGuizmo::AllowAxisFlip(false);
    ImGuizmo::SetDrawlist();
    ImGuizmo::SetRect(g_instance.scene_pos_x, g_instance.scene_pos_y,
                      g_instance.scene_width, g_instance.scene_height);
    auto &camTrans = registry.get<transform>(g_instance.active_camera);
    auto &camComp = registry.get<camera>(g_instance.active_camera);
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
        spdlog::info("Reset scene");
      }
      if (ImGui::MenuItem("Save  Scene")) {
        std::string filepath;
        if (save_file_dialog("Serialize scene file", {"*.scene"}, "Scene File",
                             filepath)) {
          auto data = serialize();
          std::ofstream output(filepath);
          if (output.is_open()) {
            output << data.dump(2) << std::endl;
            output.close();
            spdlog::info("Save scene to {0}", filepath);
          } else {
            spdlog::error("Failed to save scene to {0}", filepath);
          }
        }
      }
      if (ImGui::MenuItem("Load  Scene")) {
        std::string filepath;
        if (open_file_dialog("Deserialize scene file", {"*.scene"},
                             "Scene File", filepath)) {
          std::ifstream input(filepath);
          if (input.is_open()) {
            auto data = nlohmann::json::parse(
                std::string((std::istreambuf_iterator<char>(input)),
                            std::istreambuf_iterator<char>()));
            deserialize(data);
            spdlog::info("Load scene from {0}", filepath);
          } else {
            spdlog::error("Failed to load scene from {0}", filepath);
          }
        }
      }
      ImGui::Separator();

      ImGui::MenuItem("Prefab", nullptr, false, false);
      if (ImGui::MenuItem("Import Prefab")) {
        std::string filepath;
        if (open_file_dialog("Import prefab to current scene", {"*.prefab"},
                             "*.prefab", filepath)) {
          std::ifstream input(filepath);
          if (input.is_open()) {
            auto data = nlohmann::json::parse(
                std::string((std::istreambuf_iterator<char>(input)),
                            std::istreambuf_iterator<char>()));
            load_prefab(data);
            spdlog::info("Import prefab to current scene from {0}", filepath);
          } else {
            spdlog::error("Failed to import prefab from {0}", filepath);
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
                             {"*.fbx", "*.FBX", "*.obj", "*.OBJ", "*.pmx", "*.PMX", "*.ply", "*.PLY"},
                             "*.fbx, *.obj, *.pmx, *.ply", filepath)) {
          spdlog::info("Load model file {0}", filepath);
          // assets::open_model_ufbx(registry, filepath);
          assets::open_model_assimp(registry, filepath);
        }
      }
      // if (ImGui::MenuItem("Import   BVH")) {
      //   std::string filepath;
      //   if (open_file_dialog("Import .bvh motion file", {"*.bvh", "*.BVH"},
      //                        "*.bvh", filepath)) {
      //     spdlog::info("Load motion file {0}", filepath);
      //     anim::create_bvh_actor(registry, filepath);
      //   }
      // }
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
            if (entity == g_instance.active_camera)
              active_camera_index = tmp_counter;
            valid_cameras.push_back(entity);
            valid_camera_names.push_back(str_format(
                "%s: %d", trans.name.c_str(), entt::to_integral(entity)));
            tmp_counter++;
          });
      gui::combo_default("active camera", active_camera_index,
                         valid_camera_names, [&](int current) {
                           if (current == -1)
                             g_instance.active_camera = entt::null;
                           else
                             g_instance.active_camera = valid_cameras[current];
                         });

      gui::combo("gizmo mode", gizmo_mode_idx, {"world", "local"},
                 [&](int current) {
                   if (current == 1)
                     current_gizmo_mode = ImGuizmo::MODE::LOCAL;
                   else
                     current_gizmo_mode = ImGuizmo::MODE::WORLD;
                 });

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
    entity_name = str_format("%s %s", ICON_LC_BONE, entity_name.c_str());
  } else if (registry.try_get<camera>(current)) {
    entity_name = str_format("%s %s", ICON_LC_CAMERA, entity_name.c_str());
  } else if (registry.try_get<point_light>(current)) {
    entity_name = str_format("%s %s", ICON_LC_LIGHTBULB, entity_name.c_str());
  } else if (registry.try_get<mesh_data>(current)) {
    entity_name = str_format("%s %s", ICON_LC_BOXES, entity_name.c_str());
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
    if (ImGui::MenuItem("Make Prefab")) {
      std::string filepath;
      if (save_file_dialog("Save entity hierarchy as prefab", {"*.prefab"},
                           "*.prefab", filepath)) {
        auto data = make_prefab(entity);
        std::ofstream output(filepath);
        if (output.is_open()) {
          output << data.dump(2) << std::endl;
          spdlog::info("Save prefab to filepath {0}", filepath);
        } else {
          spdlog::error("Failed to save prefab to filepath {0}", filepath);
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
              spdlog::info("create component {0} for entity {1}", i.first,
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
