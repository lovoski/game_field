#include "toolkit/opengl3d/components/actor.hpp"
#include "toolkit/opengl3d/components/mesh.hpp"
#include "toolkit/opengl3d/engine.hpp"
#include "toolkit/opengl3d/gui.hpp"

namespace toolkit::opengl3d {

bool engine3d::screen_query_ray(math::vector2 scrn_pos, math::vector3 &o,
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

bool engine3d::mouse_query_ray(math::vector3 &o, math::vector3 &d) {
  auto scrn_pos = mouse_screen_pos;
  scrn_pos.x() -= scene_wnd_pos.x();
  scrn_pos.y() = scene_wnd_size.y() - scrn_pos.y() + scene_wnd_pos.y();
  scrn_pos.x() /= scene_wnd_size.x();
  scrn_pos.y() /= scene_wnd_size.y();
  return screen_query_ray(scrn_pos, o, d);
}

void engine3d::active_camera_manipulate(float dt) {
  auto cursor_pos = mouse_screen_pos;
  if (auto cam_trans = registry.try_get<transform>(active_camera)) {
    // focus on selected entity if `F` is triggered
    if (selected_entity != entt::null && is_key_triggered(SDLK_f)) {
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
      cam_trans->set_world_pos(cam_trans->world_pos() -
                               cam_trans->local_forward() * scroll_offset.y() *
                                   movement_delta);
    }
    bool press_mouse_mid_btn = is_mouse_button_pressed(SDL_BUTTON_MIDDLE);
    bool press_mouse_right_btn = is_mouse_button_pressed(SDL_BUTTON_RIGHT);
    // only handle mouse input when cursor in scene window
    if ((press_mouse_mid_btn || press_mouse_right_btn) &&
        (mouse_screen_pos.x() > 0 && mouse_screen_pos.x() < wnd_width &&
         mouse_screen_pos.y() > 0 && mouse_screen_pos.y() < wnd_width)) {
      // free fps-style camera
      if (press_mouse_right_btn &&
          (cursor_pos.x() > scene_wnd_pos.x() &&
           cursor_pos.x() < (scene_wnd_pos.x() + scene_wnd_size.x()) &&
           cursor_pos.y() > scene_wnd_pos.y() &&
           cursor_pos.y() < (scene_wnd_pos.y() + scene_wnd_size.y()))) {
        // modify the cameraPivot position to suite cursor movement
        if (mouse_screen_delta.norm() > 1e-2f) {
          math::vector3 ray_o, ray_d;
          math::vector2 screen_pos = math::vector2(
              scene_wnd_size.x() / 2.0f +
                  mouse_screen_delta.x() * cam_manip_data.fps_speed,
              scene_wnd_size.y() / 2.0f -
                  mouse_screen_delta.y() * cam_manip_data.fps_speed);
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
        if (is_key_pressed(SDLK_w))
          camera_movement -= cam_trans->local_forward();
        if (is_key_pressed(SDLK_s))
          camera_movement += cam_trans->local_forward();
        if (is_key_pressed(SDLK_a))
          camera_movement -= cam_trans->local_right();
        if (is_key_pressed(SDLK_d))
          camera_movement += cam_trans->local_right();
        camera_movement *= (cam_manip_data.fps_camera_speed * dt);
        cam_manip_data.camera_pivot += camera_movement;
        cam_trans->set_world_pos(cam_trans->world_pos() + camera_movement);
      } else if (press_mouse_mid_btn) {
        // rotate the camera around the pivot, or translate the camera
        if (is_key_pressed(SDLK_LSHIFT)) {
          // translate the camera according to nfc offset
          if (mouse_screen_delta.norm() > 1e-2f) {
            math::vector2 scene_size = scene_wnd_size;
            math::vector4 nfcPos = {-mouse_screen_delta.x() / scene_size.x(),
                                    mouse_screen_delta.y() / scene_size.y(),
                                    1.0f, 1.0f};
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
          auto rotateOffset = mouse_screen_delta * 0.1f;
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
    }

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

void engine3d::editor_shortkeys() {
  auto cursor_pos = mouse_screen_pos;
  if (cursor_pos.x() > scene_wnd_pos.x() &&
      cursor_pos.x() < (scene_wnd_pos.x() + scene_wnd_size.x()) &&
      cursor_pos.y() > scene_wnd_pos.y() &&
      cursor_pos.y() < (scene_wnd_pos.y() + scene_wnd_size.y())) {
    // only change the gizmo operation mode
    // if the cursor is inside scene window
    if (is_key_pressed(SDLK_1)) {
      with_translate = true;
      with_rotate = false;
      with_scale = false;
    }
    if (is_key_pressed(SDLK_2)) {
      with_translate = false;
      with_rotate = true;
      with_scale = false;
    }
    if (is_key_pressed(SDLK_3)) {
      with_translate = false;
      with_rotate = false;
      with_scale = true;
    }

    // detect mouse click selection
    if (is_key_pressed(SDLK_LCTRL) &&
        is_mouse_button_triggered(SDL_BUTTON_LEFT)) {
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

void engine3d::draw_gizmos(bool enable) {
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

void engine3d::draw_main_menubar() {
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
                auto &mesh_data_comp = registry.emplace<mesh_data>(root_entity);
                mesh_data_comp.mesh_name = loaded_meshes[0].name;
                mesh_data_comp.vertices = loaded_meshes[0].vertices;
                mesh_data_comp.indices = loaded_meshes[0].indices;
                init_opengl_buffers(mesh_data_comp);
              } else if (loaded_meshes.size() > 1) {
                auto root_entity = registry.create();
                auto &root_trans = registry.emplace<transform>(root_entity);
                root_trans.name = filename;
                for (int i = 0; i < loaded_meshes.size(); i++) {
                  auto mesh_entity = registry.create();
                  auto &mesh_trans = registry.emplace<transform>(mesh_entity);
                  root_trans.add_child(mesh_entity);
                  mesh_trans.name = loaded_meshes[i].name;
                  auto &mesh_data_comp =
                      registry.emplace<mesh_data>(mesh_entity);
                  mesh_data_comp.mesh_name = loaded_meshes[i].name;
                  mesh_data_comp.vertices = loaded_meshes[i].vertices;
                  mesh_data_comp.indices = loaded_meshes[i].indices;
                  init_opengl_buffers(mesh_data_comp);
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
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Settings")) {
      // system configuration
      ImGui::MenuItem("Configure Systems", nullptr, false, false);
      // TODO: Extend this if there's further systems
      draw_systems_gui();

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
      combo_default("active camera", active_camera_index, valid_camera_names,
                    [&](int current) {
                      if (current == -1)
                        active_camera = entt::null;
                      else
                        active_camera = valid_cameras[current];
                    });

      combo("gizmo mode", gizmo_mode_idx, {"world", "local"}, [&](int current) {
        if (current == 1)
          current_gizmo_mode = ImGuizmo::MODE::LOCAL;
        else
          current_gizmo_mode = ImGuizmo::MODE::WORLD;
      });
      ImGui::Checkbox("Editor Manipulate Camera", &editor_manipulate_camera);
      if (ImGui::Checkbox("Turn On VSync", &should_vsync)) {
        set_vsync_state(should_vsync);
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
  if (registry.try_get<bone_node>(current)) {
    entity_name = str_format("(骨骼) %s", entity_name.c_str());
  } else if (registry.try_get<camera>(current)) {
    entity_name = str_format("(摄像机) %s", entity_name.c_str());
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

void engine3d::draw_hierarchy_window() {
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

      // ImGui::Separator();
      // if (ImGui::MenuItem("New Cube"))
      //   create_cube(registry);
      // if (ImGui::MenuItem("New Sphere"))
      //   create_sphere(registry);
      // if (ImGui::MenuItem("New Cylinder"))
      //   create_cylinder(registry);
      // if (ImGui::MenuItem("New Plane"))
      //   create_plane(registry);
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
  for (auto ent : transform_hierarchy_sys->root_entities)
    draw_entity_hierarchy_recursive(registry, selected_entity, ent,
                                    guiTreeNodeFlags, right_click_entity);
  ImGui::EndChild();
  ImGui::End();
}

void engine3d::draw_components_window() {
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

  // Entity name engine3d
  if (registry.valid(current_entity)) {
    auto &trans = registry.get<transform>(current_entity);
    static std::array<char, 256> name_buffer;
    static entt::entity last_edited_entity = entt::null;

    // Copy name to buffer when entity changes
    if (last_edited_entity != current_entity) {
      strncpy(name_buffer.data(), trans.name.c_str(), name_buffer.size() - 1);
      name_buffer[name_buffer.size() - 1] = '\0';
      last_edited_entity = current_entity;
    }

    ImGui::Text("Entity Name:");
    ImGui::SetNextItemWidth(-1);
    if (ImGui::InputText("##entity_name", name_buffer.data(),
                         name_buffer.size(),
                         ImGuiInputTextFlags_EnterReturnsTrue)) {
      trans.name = std::string(name_buffer.data());
      SDL_Log("Entity name changed to: %s", trans.name.c_str());
    }
    ImGui::Separator();
  }

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

    // TODO: manually modify or extend this field for further usage
    draw_components_gui(current_entity);

  } else
    current_entity = entt::null;

  ImGui::End();
}

void engine3d::draw_components_gui(entt::entity current_entity) {
  if (auto trans_comp = registry.try_get<transform>(current_entity)) {
    if (ImGui::CollapsingHeader("Transform"))
      trans_comp->draw_gui(registry, current_entity);
  }
  if (auto cam_comp = registry.try_get<camera>(current_entity)) {
    if (ImGui::CollapsingHeader("Camera"))
      cam_comp->draw_gui(registry, current_entity);
  }
  if (auto actor_comp = registry.try_get<actor>(current_entity)) {
    if (ImGui::CollapsingHeader("Actor"))
      actor_comp->draw_gui(registry, current_entity);
  }
  if (auto mesh_comp = registry.try_get<mesh_data>(current_entity)) {
    if (ImGui::CollapsingHeader("Mesh"))
      mesh_comp->draw_gui(registry, current_entity);
  }
  if (auto bundle_comp =
          registry.try_get<skinned_mesh_bundle>(current_entity)) {
    if (ImGui::CollapsingHeader("Skinned Mesh Bundle (Pannel)"))
      bundle_comp->draw_gui(registry, current_entity);
  }
  ss_handler_system->proxy_draw_gui(registry, current_entity);
}

void engine3d::draw_systems_gui() {
  if (ImGui::BeginMenu("Built-in Render System")) {
    default_render_sys->draw_menu_gui();
    ImGui::EndMenu();
  }
  if (ImGui::BeginMenu("Transform Hierarchy System")) {
    transform_hierarchy_sys->draw_menu_gui();
    ImGui::EndMenu();
  }
}

}; // namespace toolkit::opengl3d