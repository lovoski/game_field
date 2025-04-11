#include "ToolKit/OpenGL/Editor.hpp"

namespace toolkit::opengl {

void editor::run() {
  auto &instance = context::get_instance();
  timer.reset();
  instance.run([&]() {
    float dt = timer.elapse_s();
    timer.reset();

    transform_sys->update_transform(registry);
    for (auto sys : systems) {
      if (sys->active)
        sys->preupdate(registry, dt);
      if (script_sys->active)
        script_sys->preupdate(this, dt);
    }
    for (auto sys : systems) {
      if (sys->active)
        sys->update(registry, dt);
      if (script_sys->active)
        script_sys->update(this, dt);
    }
    for (auto sys : systems) {
      if (sys->active)
        sys->lateupdate(registry, dt);
      if (script_sys->active)
        script_sys->lateupdate(this, dt);
    }

    dm_sys->render(registry);

    instance.begin_imgui();
    instance.end_imgui();
    instance.swap_buffer();
  });
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

  if (enable && registry.valid(active_camera)) {
    ImGuizmo::AllowAxisFlip(false);
    ImGuizmo::SetDrawlist();
    ImGuizmo::SetRect(instance.scene_pos_x, instance.scene_pos_y,
                      instance.scene_width, instance.scene_height);
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

// entt::entity CreateCube(entt::registry &registry) {}
// entt::entity CreateSphere(entt::registry &registry) {}
// entt::entity CreateCylinder(entt::registry &registry) {}
// entt::entity CreateCone(entt::registry &registry) {}

}; // namespace toolkit::opengl