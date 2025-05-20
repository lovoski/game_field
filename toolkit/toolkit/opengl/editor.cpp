#include "toolkit/opengl/editor.hpp"
#include "toolkit/anim/components/actor.hpp"
#include "toolkit/opengl/gui/utils.hpp"
#include "toolkit/opengl/scripts/camera.hpp"
#include "toolkit/opengl/scripts/test_draw.hpp"

namespace toolkit::opengl {

void editor::init() {
  auto &instance = context::get_instance();
  instance.init();
  reset();

  // init imgui
  imgui_io = &ImGui::GetIO();
  imgui_io->ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  imgui_io->ConfigFlags |= ImGuiConfigFlags_DockingEnable;
}

void editor::shutdown() { g_instance.shutdown(); }

void editor::late_serialize(nlohmann::json &j) {
  nlohmann::json editor_settings;
  editor_settings["active_camera"] =
      std::to_string(entt::to_integral(g_instance.active_camera));
  j["editor"] = editor_settings;
}

void editor::late_deserialize(nlohmann::json &j) {
  if (j.contains("editor")) {
    std::string active_camera_str =
        j["editor"]["active_camera"].get<std::string>();
    g_instance.active_camera =
        entt::entity{static_cast<std::uint32_t>(std::stoul(active_camera_str))};
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

    render_sys->update_obj_bbox(registry);
    render_sys->update_scene_buffers(registry);

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
}

void editor::add_default_objects() {
  auto ent = registry.create();
  auto &trans = registry.emplace<transform>(ent);
  trans.name = "main camera";
  trans.set_global_position(math::vector3(0, 0, 5));
  auto &cam_comp = registry.emplace<camera>(ent);
  g_instance.active_camera = ent;
  auto &editor_cam = registry.emplace<editor_camera>(ent);
}

void editor::draw_gizmos(bool enable) {
  // only change the gizmo operation mode
  // if the cursor is inside scene window
  auto &instance = context::get_instance();
  if (instance.cursor_in_scene_window()) {
    if (instance.is_key_pressed(GLFW_KEY_G)) {
      with_translate = true;
      with_rotate = false;
      with_scale = false;
    }
    if (instance.is_key_pressed(GLFW_KEY_R)) {
      with_translate = false;
      with_rotate = true;
      with_scale = false;
    }
    if (instance.is_key_pressed(GLFW_KEY_S)) {
      with_translate = false;
      with_rotate = false;
      with_scale = true;
    }
  }

  if (enable && registry.valid(instance.active_camera)) {
    ImGuizmo::AllowAxisFlip(false);
    ImGuizmo::SetDrawlist();
    ImGuizmo::SetRect(instance.scene_pos_x, instance.scene_pos_y,
                      instance.scene_width, instance.scene_height);
    auto &camTrans = registry.get<transform>(instance.active_camera);
    auto &camComp = registry.get<camera>(instance.active_camera);
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
          selTrans.set_global_position(position);
        }
        math::vector3 scale(transform.col(0).norm(), transform.col(1).norm(),
                            transform.col(2).norm());
        if ((current_gizmo_operation() & ImGuizmo::ROTATE) != 0) {
          math::matrix4 rotation;
          rotation << transform.col(0) / scale.x(),
              transform.col(1) / scale.y(), transform.col(2) / scale.z(),
              math::vector4(0.0, 0.0, 0.0, 1.0);
          selTrans.set_global_rotation(math::quat(rotation.block<3, 3>(0, 0)));
        }
        if ((current_gizmo_operation() & ImGuizmo::SCALE) != 0)
          selTrans.set_global_scale(scale);
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
            spdlog::error("Import prefab to current scene from {0}", filepath);
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
        if (open_file_dialog(
                "Open model asset file",
                {"*.fbx", "*.FBX", "*.pmx", "*.PMX", "*.obj", "*.OBJ"},
                "*.fbx, *.pmx, *.obj", filepath)) {
          spdlog::info("Load model file {0}", filepath);
          open_model(registry, filepath);
        }
      }
      // if (ImGui::MenuItem("Import  Model")) {
      //   std::string filepath;
      //   if (open_file_dialog("Import One Asset",
      //                        {"*.fbx", "*.FBX", "*.obj", "*.OBJ"},
      //                        "*.fbx, *.obj", filepath)) {
      //     spdlog::info("Load model file {0}", filepath);
      //     create_geometry_model(registry, filepath);
      //   }
      // }
      if (ImGui::MenuItem("Import   BVH")) {
        std::string filepath;
        if (open_file_dialog("Import .bvh motion file", {"*.bvh", "*.BVH"},
                             "*.bvh", filepath)) {
          spdlog::info("Load motion file {0}", filepath);
          anim::create_bvh_actor(registry, filepath);
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

      int gizmo_mode_idx = 0;
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
  bool nodeOpen =
      ImGui::TreeNodeEx((void *)(intptr_t)entt::to_integral(current), finalFlag,
                        current_transform.name.c_str());
  if (ImGui::IsItemClicked() && !ImGui::IsItemToggledOpen())
    selected = current;

  // Drag & Drop
  if (ImGui::BeginDragDropSource()) {
    ImGui::SetDragDropPayload("ENTITY", &(current), sizeof(entt::entity));
    ImGui::Text("Drag drop to change hierarchy");
    ImGui::EndDragDropSource();
  }
  if (ImGui::BeginDragDropTarget()) {
    if (const ImGuiPayload *payload = ImGui::AcceptDragDropPayload("ENTITY")) {
      auto entity = *(entt::entity *)payload->Data;
      current_transform.add_children(entity);
    }
    ImGui::EndDragDropTarget();
  }

  // Right click context menu
  if (currentSelected &&
      ImGui::BeginPopupContextItem(
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
      if (ImGui::MenuItem("New Dir Light")) {
        auto ent = registry.create();
        auto &trans = registry.emplace<transform>(ent);
        auto number = registry.view<dir_light>().size();
        trans.name = str_format("Dir Light (%d)", number);
        auto &light = registry.emplace<dir_light>(ent);
      }
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
    // unselect entities
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
      selected_entity = entt::null;
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