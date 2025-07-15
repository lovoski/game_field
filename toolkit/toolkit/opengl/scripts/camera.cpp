#include "toolkit/opengl/components/camera.hpp"
#include "toolkit/opengl/draw.hpp"
#include "toolkit/opengl/editor.hpp"
#include "toolkit/opengl/scripts/camera.hpp"

namespace toolkit::opengl {

void editor_camera::draw_to_scene(iapp *app) {
  if (auto editor_ptr = dynamic_cast<editor *>(app)) {
    if (auto cam_trans =
            editor_ptr->registry.try_get<transform>(g_instance.active_camera)) {
      auto &cam_comp =
          editor_ptr->registry.get<camera>(g_instance.active_camera);
      if (vis_pivot)
        draw_quads({camera_pivot}, cam_trans->local_left(),
                   cam_trans->local_up(), cam_comp.vp, vis_pivot_size, Purple);
    }
  }
}

void editor_camera::draw_gui(iapp *app) {
  ImGui::Checkbox("Show Pivot", &vis_pivot);
  ImGui::DragFloat("Show Pivot Size", &vis_pivot_size, 0.01f, 0.0f, 100.0f);
  ImGui::DragFloat("FPS Sensitivity", &fps_speed, 0.01f, 0.0f, 10.0f);
  ImGui::DragFloat("FPS Movement Speed", &fps_camera_speed, 0.01f, 0.0f, 100.0f);
}

void editor_camera::preupdate(iapp *app, float dt) {
  if (auto editor_ptr = dynamic_cast<editor *>(app)) {
    if (auto cam_trans =
            editor_ptr->registry.try_get<transform>(g_instance.active_camera)) {
      // focus on selected entity if `F` is triggered
      if (editor_ptr->selected_entity != entt::null &&
          g_instance.is_key_triggered(GLFW_KEY_F)) {
        spdlog::info(
            "Focus camera \"{0}\" to selected entity \"{1}\"",
            registry->get<transform>(entity).name,
            registry->get<transform>(editor_ptr->selected_entity).name);
        camera_pivot =
            registry->get<transform>(editor_ptr->selected_entity).position();
      }
      auto cam_comp =
          editor_ptr->registry.get<camera>(g_instance.active_camera);
      auto cam_pos = cam_trans->position();
      if ((cam_pos - camera_pivot).norm() < 1e-9f) {
        camera_pivot = cam_pos - cam_trans->local_forward();
        spdlog::info("push pivot away from camera");
      }
      // scroll movement delta, scale with the distance to pivot
      float movement_delta =
          initial_factor * dt *
          std::min(std::pow((cam_pos - camera_pivot).norm(), speed_pow),
                   max_speed);
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
        if (mouse_first_move) {
          mouse_last_pos = mouse_current_pos;
          mouse_first_move = false;
        }
        math::vector2 mouse_offset = mouse_current_pos - mouse_last_pos;
        // free fps-style camera
        if (press_mouse_right_btn) {
          // modify the cameraPivot position to suite cursor movement
          if (mouse_offset.norm() > 1e-2f) {
            math::vector3 ray_o, ray_d;
            math::vector2 screen_pos = math::vector2(
                g_instance.scene_width / 2.0f + mouse_offset.x() * fps_speed,
                g_instance.scene_height / 2.0f - mouse_offset.y() * fps_speed);
            screen_pos.x() /= g_instance.scene_width;
            screen_pos.y() /= g_instance.scene_height;
            if (editor_ptr->screen_query_ray(screen_pos, ray_o, ray_d)) {
              camera_pivot =
                  ray_o + ray_d * (camera_pivot - cam_trans->position()).norm();
            }
          }
          // move camera position with wasd key board
          math::vector3 camera_movement = math::vector3::Zero();
          math::vector3 cam_vec =
              (cam_trans->position() - camera_pivot).normalized();
          if (g_instance.is_key_pressed(GLFW_KEY_W))
            camera_movement -= cam_trans->local_forward();
          if (g_instance.is_key_pressed(GLFW_KEY_S))
            camera_movement += cam_trans->local_forward();
          if (g_instance.is_key_pressed(GLFW_KEY_A))
            camera_movement -= cam_trans->local_left();
          if (g_instance.is_key_pressed(GLFW_KEY_D))
            camera_movement += cam_trans->local_left();
          camera_movement *= (fps_camera_speed * dt);
          camera_pivot += camera_movement;
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
                  worldRayDir.dot(camera_pivot - cam_pos) * worldRayDir;
              auto deltaPos = worldRayDir.dot(cam_trans->local_left()) *
                                  cam_trans->local_left() +
                              worldRayDir.dot(cam_trans->local_up()) *
                                  cam_trans->local_up();
              camera_pivot += deltaPos;
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
                    (posVector - camera_pivot) +
                camera_pivot;
            cam_trans->set_world_pos(newPos);
          }
        }
        mouse_last_pos = mouse_current_pos;
      } else
        mouse_first_move = true;

      math::vector3 lastLeft = cam_trans->local_left();
      math::vector3 forward =
          (cam_trans->position() - camera_pivot).normalized();
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
}

}; // namespace toolkit::opengl