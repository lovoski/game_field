#include "toolkit/opengl/editor.hpp"
#include "toolkit/anim/components/actor.hpp"
#include "toolkit/anim/scripts/vis.hpp"
#include "toolkit/opengl/gui/utils.hpp"
#include "toolkit/opengl/scripts/camera.hpp"
#include "toolkit/opengl/scripts/test_draw.hpp"

#include <ufbx.h>

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
                             {"*.fbx", "*.FBX", "*.obj", "*.OBJ"},
                             "*.fbx, *.obj", filepath)) {
          spdlog::info("Load model file {0}", filepath);
          assets::open_model(registry, filepath);
        }
      }
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

math::vector2 ufbx_to_vec2(ufbx_vec2 v) {
  return math::vector2((float)v.x, (float)v.y);
}
math::vector3 ufbx_to_vec3(ufbx_vec3 v) {
  return math::vector3((float)v.x, (float)v.y, (float)v.z);
}
math::quat ufbx_to_quat(ufbx_quat v) {
  return math::quat((float)v.w, (float)v.x, (float)v.y, (float)v.z);
}
math::matrix4 ufbx_to_mat(ufbx_matrix m) {
  math::matrix4 result = math::matrix4::Zero();
  result(0, 0) = (float)m.m00;
  result(0, 1) = (float)m.m01;
  result(0, 2) = (float)m.m02;
  result(0, 3) = (float)m.m03;
  result(1, 0) = (float)m.m10;
  result(1, 1) = (float)m.m11;
  result(1, 2) = (float)m.m12;
  result(1, 3) = (float)m.m13;
  result(2, 0) = (float)m.m20;
  result(2, 1) = (float)m.m21;
  result(2, 2) = (float)m.m22;
  result(2, 3) = (float)m.m23;
  result(3, 3) = 1;
  return result;
}

ufbx_node *find_first_bone_parent(ufbx_node *node) {
  while (node) {
    if (node->bone)
      return node;
    node = node->parent;
  }
  return nullptr;
}

ufbx_node *find_last_bone_parent(ufbx_node *node) {
  ufbx_node *bone_parent = nullptr;
  while (node) {
    if (node->bone)
      bone_parent = node;
    node = node->parent;
  }
  return bone_parent;
}

std::map<ufbx_node *, entt::entity>
read_nodes(entt::registry &registry, ufbx_scene *scene, std::string filename) {
  std::map<ufbx_node *, entt::entity> ufbx_node_to_entity;
  for (int i = 0; i < scene->nodes.count; i++) {
    auto unode = scene->nodes[i];
    auto ent = registry.create();
    auto &trans = registry.emplace<transform>(ent);
    trans.name = unode->is_root ? filename : unode->name.data;
    ufbx_node_to_entity[unode] = ent;
  }
  // setup parent child hierarchy
  for (int i = 0; i < scene->nodes.count; i++) {
    auto unode = scene->nodes[i];
    if (unode->parent) {
      auto ent = ufbx_node_to_entity[unode];
      auto &trans = registry.get<transform>(ent);
      auto p_ent = ufbx_node_to_entity[unode->parent];
      auto &p_trans = registry.get<transform>(p_ent);
      p_trans.add_children(ent);
      trans.set_local_position(
          ufbx_to_vec3(unode->local_transform.translation));
      trans.set_local_rotation(ufbx_to_quat(unode->local_transform.rotation));
      trans.set_local_scale(ufbx_to_vec3(unode->local_transform.scale));
    }
  }
  if (scene->nodes.count > 0) {
    registry.get<transform>(ufbx_node_to_entity[scene->nodes[0]])
        .force_update_all();
  }
  // find root bones, add actor component to them
  std::vector<ufbx_node *> root_nodes;
  for (int i = 0; i < scene->nodes.count; i++) {
    if (scene->nodes[i]->is_root)
      root_nodes.push_back(scene->nodes[i]);
  }
  for (auto root_node : root_nodes) {
    std::queue<std::pair<ufbx_node *, ufbx_node *>> q;
    std::vector<std::pair<ufbx_node *, ufbx_node *>> bone_nodes;
    q.push(std::make_pair(root_node, nullptr));
    while (!q.empty()) {
      auto node = q.front();
      if (node.first->bone)
        bone_nodes.push_back(node);
      q.pop();
      for (auto c : node.first->children) {
        q.push(std::make_pair(c, find_first_bone_parent(node.first)));
      }
    }
    if (bone_nodes.size() > 0) {
      // use bone nodes to create a skeleton
      std::vector<ufbx_node *> roots;
      for (int i = 0; i < bone_nodes.size(); i++) {
        if (bone_nodes[i].second == nullptr)
          roots.push_back(bone_nodes[i].first);
      }
      for (auto root : roots) {
        auto root_ent = ufbx_node_to_entity[root];
        std::vector<std::pair<ufbx_node *, ufbx_node *>> bone_nodes_sub;
        for (int i = 0; i < bone_nodes.size(); i++)
          if (find_last_bone_parent(bone_nodes[i].first) == root)
            bone_nodes_sub.push_back(bone_nodes[i]);
        int joint_num = bone_nodes_sub.size();
        auto &actor_comp = registry.emplace<anim::actor>(root_ent);
        auto &vis_script = registry.emplace<anim::vis_skeleton>(root_ent);
        actor_comp.skel.as_empty(joint_num);
        actor_comp.joint_active.resize(joint_num, true);
        actor_comp.ordered_entities.resize(joint_num);
        std::map<entt::entity, int> ent_to_joint_id;
        for (int i = 0; i < joint_num; i++) {
          auto joint_ent = ufbx_node_to_entity[bone_nodes_sub[i].first];
          auto &joint_trans = registry.get<transform>(joint_ent);
          actor_comp.name_to_entity[joint_trans.name] = joint_ent;

          ent_to_joint_id[joint_ent] = i;
          actor_comp.ordered_entities[i] = joint_ent;
          actor_comp.skel.joint_names[i] = joint_trans.name;
          // actor_comp.skel.offset_matrices[i] = bone_nodes_sub[i].first->bone.

          auto p_unode = bone_nodes_sub[i].second;
          if (p_unode) {
            auto p_joint_ent = ufbx_node_to_entity[bone_nodes_sub[i].second];
            auto &p_joint_trans = registry.get<transform>(p_joint_ent);
            actor_comp.skel.joint_parent[i] = ent_to_joint_id[p_joint_ent];
            actor_comp.skel.joint_offset[i] =
                p_joint_trans.world_to_local(joint_trans.position());
            actor_comp.skel.joint_scale[i] =
                joint_trans.scale().array() / p_joint_trans.scale().array();
            actor_comp.skel.joint_rotation[i] =
                p_joint_trans.rotation().inverse() * joint_trans.rotation();
            actor_comp.skel.joint_children[ent_to_joint_id[p_joint_ent]]
                .push_back(i);
          } else {
            actor_comp.skel.joint_parent[i] = -1;
            auto &root_trans = registry.get<transform>(root_ent);
            actor_comp.skel.joint_offset[i] = root_trans.position();
            actor_comp.skel.joint_scale[i] = root_trans.scale();
            actor_comp.skel.joint_rotation[i] = root_trans.rotation();
          }
        }

        // std::ofstream output("skel.json");
        // nlohmann::json d = actor_comp.skel;
        // output << d.dump(2);
        // output.close();

        // auto container = registry.create();
        // auto &trans = registry.emplace<transform>(container);
        // trans.name = "test";
        // auto &vvv = registry.emplace<anim::vis_skeleton>(container);
        // anim::create_actor_with_skeleton(registry, container, actor_comp.skel);
      }
    }
  }
  return ufbx_node_to_entity;
}

void read_mesh(entt::registry &registry, ufbx_mesh *mesh) {}

void open_model(entt::registry &registry, std::string filepath) {
  ufbx_error error;
  ufbx_load_opts opts = {
      .load_external_files = true,
      .ignore_missing_external_files = true,
      .generate_missing_normals = true,
      .target_axes =
          {
              .right = UFBX_COORDINATE_AXIS_POSITIVE_X,
              .up = UFBX_COORDINATE_AXIS_POSITIVE_Y,
              .front = UFBX_COORDINATE_AXIS_POSITIVE_Z,
          },
      .target_unit_meters = 1.0f,
  };
  ufbx_scene *scene = ufbx_load_file(filepath.c_str(), &opts, &error);
  if (!scene) {
    spdlog::error("Failed to load: {0}", error.description.data);
    return;
  }

// create nodes
#ifdef _WIN32
  std::string filename =
      wstring_to_string(std::filesystem::u8path(filepath).filename().wstring());
#else
  std::string filename = std::filesystem::path(filepath).filename().string();
#endif
  auto ufbx_node_to_entity = read_nodes(registry, scene, filename);
  // create meshes
  for (int i = 0; i < scene->meshes.count; i++) {
    read_mesh(registry, scene->meshes[i]);
  }

  ufbx_free_scene(scene);
}

}; // namespace toolkit::assets