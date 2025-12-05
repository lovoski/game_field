#include "toolkit/anim/scripts/controller.hpp"
#include "toolkit/opengl/editor.hpp"
#include "toolkit/opengl/gui/utils.hpp"

namespace toolkit::anim {

void character_controller::start(entt::registry &registry) {}
void character_controller::destroy(entt::registry &registry) {}

void character_controller::draw_to_scene(entt::registry &registry,
                                         transform &cam_trans,
                                         camera &cam_comp) {}
void character_controller::draw_gui(entt::registry &registry,
                                    entt::entity entity) {
  gui::combo("Input Ticks", input_ticks_index, {"60", "90", "120"},
             [&](int index) {
               if (index == 0)
                 fixed_interval = 1.0f / 60;
               else if (index == 1)
                 fixed_interval = 1.0f / 90;
               else if (index == 2)
                 fixed_interval = 1.0f / 120;
             });
  ImGui::DragFloat("Cam Rotate Speed", &cam_move_speed, 0.001f, 0.0f, 1e9);
}

void character_controller::update(entt::registry &registry, float dt) {
  auto app = dynamic_cast<opengl::editor *>(registry.ctx().get<iapp *>());
  auto &sdl_ctx = opengl::sdl_context::get_instance();
  if (app != nullptr && app->app_in_game_mode()) {
    auto mouse_movement = sdl_ctx.get_mouse_delta_gamemode();
    cam_angle_horizontal -= dt * cam_move_speed * mouse_movement.x();
    cam_angle_vertical += dt * cam_move_speed * mouse_movement.y();
    cam_angle_vertical = std::clamp(cam_angle_vertical, -20.0f, 80.0f);
    // std::cout << cam_angle_horizontal << "," << cam_angle_vertical << std::endl;
    float cos_z = cos(math::deg_to_rad(cam_angle_vertical));
    math::vector3 cam_z =
        math::vector3(cos_z * sin(math::deg_to_rad(cam_angle_horizontal)),
                      sin(math::deg_to_rad(cam_angle_vertical)),
                      cos_z * cos(math::deg_to_rad(cam_angle_horizontal)))
            .normalized();
    math::vector3 cam_y(0.0, 1.0, 0.0);
    math::vector3 cam_x = (cam_y.cross(cam_z)).normalized();
    cam_y = (cam_z.cross(cam_x)).normalized();
    math::matrix3 cam_rot = math::matrix3::Identity();
    cam_rot << cam_x, cam_y, cam_z;
    auto &cam_trans = registry.get<transform>(app->active_camera);
    cam_trans.set_world_rot(math::quat(cam_rot));
    cam_trans.set_world_pos(cam_z * 10);
  }
}

void character_controller::fixedupdate(entt::registry &registry, float dt) {}

}; // namespace toolkit::anim