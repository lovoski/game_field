#include "toolkit/opengl/components/camera.hpp"
#include "toolkit/opengl/editor.hpp"
#include "toolkit/opengl/scripts/camera.hpp"
#include "toolkit/opengl/draw.hpp"

namespace toolkit::opengl {

void editor_camera::draw_to_scene(iapp *app) {
  spdlog::info("call draw to scene");
  if (auto editor_ptr = dynamic_cast<editor *>(app)) {
    if (auto cam_trans = editor_ptr->registry.try_get<transform>(g_instance.active_camera)) {
      auto cam_comp = editor_ptr->registry.get<camera>(g_instance.active_camera);

      draw_wire_sphere(math::vector3(0,0,0), cam_comp.vp, 0.5, Purple);
    }
  }
}

void editor_camera::preupdate(iapp *app, float dt) {
  spdlog::info("call preupdate");
  return;
  if (auto editor_ptr = dynamic_cast<editor *>(app)) {
    auto &instance = context::get_instance();
    if (auto cam_trans =
            editor_ptr->registry.try_get<transform>(instance.active_camera)) {
      auto cam_comp = editor_ptr->registry.get<camera>(instance.active_camera);
      auto cam_pos = cam_trans->position();
      if ((cam_pos - cameraPivot).norm() < 1e-9f) {
        cameraPivot = cam_pos - cam_trans->local_forward();
        spdlog::info("push pivot away from camera");
      }
      float movementDelta =
          initialFactor * dt *
          std::min(std::pow((cam_pos - cameraPivot).norm(), speedPow),
                   maxSpeed);
      if (instance.cursor_in_scene_window()) {
        // check action queue for mouse scroll event
        math::vector2 scrollOffset = instance.get_scroll_offsets();
        cam_trans->set_global_position(cam_trans->position() -
                                       cam_trans->local_forward() *
                                           scrollOffset.y() * movementDelta);
      }
      bool pressMouseMidBtn =
          instance.is_mouse_button_pressed(GLFW_MOUSE_BUTTON_MIDDLE);
      bool pressMouseRgtBtn =
          instance.is_mouse_button_pressed(GLFW_MOUSE_BUTTON_RIGHT);
      if (pressMouseMidBtn || pressMouseRgtBtn) {
        if (instance.loop_cursor_in_window())
          mouseFirstMove = true;
        math::vector2 mouseCurrentPos = instance.get_mouse_position();
        // loop the camera if the camera is locked
        if (mouseFirstMove) {
          mouseLastPos = mouseCurrentPos;
          mouseFirstMove = false;
        }
        math::vector2 mouseOffset = mouseCurrentPos - mouseLastPos;
        if (pressMouseRgtBtn) {
          // modify the cameraPivot position to suite cursor movement
          if (mouseOffset.norm() > 1e-2f) {
            math::vector2 screenSize = instance.get_scene_size();
            math::vector4 nfcPos = {-mouseOffset.x() / screenSize.x(),
                                    mouseOffset.y() / screenSize.y(), 1.0f,
                                    1.0f};
            math::vector4 worldRayPos =
                cam_comp.view.inverse() * cam_comp.proj.inverse() * nfcPos;
            worldRayPos /= worldRayPos.w();
            math::vector3 worldRayDir =
                (worldRayPos.head<3>() - cam_pos).normalized();
            worldRayDir = worldRayDir.dot(cameraPivot - cam_pos) * worldRayDir;
            auto deltaPos =
                worldRayDir.dot(cam_trans->local_left()) *
                    cam_trans->local_left() +
                worldRayDir.dot(cam_trans->local_up()) * cam_trans->local_up();
            cameraPivot -= fpsSpeed * deltaPos;
          }
        } else if (pressMouseMidBtn) {
          if (instance.is_key_pressed(GLFW_KEY_LEFT_SHIFT)) {
            // move the view position if there's mouseOffset
            if (mouseOffset.norm() > 1e-2f) {
              math::vector2 screenSize = instance.get_scene_size();
              math::vector4 nfcPos = {-mouseOffset.x() / screenSize.x(),
                                      mouseOffset.y() / screenSize.y(), 1.0f,
                                      1.0f};
              math::vector4 worldRayPos =
                  cam_comp.view.inverse() * cam_comp.proj.inverse() * nfcPos;
              worldRayPos /= worldRayPos.w();
              math::vector3 worldRayDir =
                  (worldRayPos.head<3>() - cam_pos).normalized();
              worldRayDir =
                  worldRayDir.dot(cameraPivot - cam_pos) * worldRayDir;
              auto deltaPos = worldRayDir.dot(cam_trans->local_left()) *
                                  cam_trans->local_left() +
                              worldRayDir.dot(cam_trans->local_up()) *
                                  cam_trans->local_up();
              cameraPivot += deltaPos;
              cam_trans->set_global_position(cam_trans->position() + deltaPos);
            }
          } else {
            // repose the camera
            auto rotateOffset = mouseOffset * 0.1f;
            math::vector3 posVector = cam_trans->position();
            math::vector3 newPos =
                math::angle_axis(math::deg_to_rad(-rotateOffset.x()),
                                 math::world_up) *
                    math::angle_axis(math::deg_to_rad(-rotateOffset.y()),
                                     cam_trans->local_left()) *
                    (posVector - cameraPivot) +
                cameraPivot;
            cam_trans->set_global_position(newPos);
          }
        }
        mouseLastPos = mouseCurrentPos;
      } else
        mouseFirstMove = true;
      math::vector3 lastLeft = cam_trans->local_left();
      math::vector3 forward =
          (cam_trans->position() - cameraPivot).normalized();
      math::vector3 up = math::world_up;
      math::vector3 left = up.cross(forward).normalized();
      // flip left if non-consistent
      if (lastLeft.dot(left) < 0.0f)
        left *= -1;
      up = (forward.cross(left)).normalized();
      math::matrix3 rot;
      rot << left, up, forward;
      cam_trans->set_global_rotation(math::quat(rot));
    }
  }
}

}; // namespace toolkit::opengl