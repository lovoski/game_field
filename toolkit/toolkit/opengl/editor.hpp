#pragma once

#include "toolkit/opengl/base.hpp"
#include "toolkit/scriptable.hpp"
#include "toolkit/system.hpp"
#include "toolkit/utils.hpp"

#include "toolkit/transform.hpp"

#include "toolkit/anim/anim_system.hpp"
#include "toolkit/opengl/components/camera.hpp"
#include "toolkit/opengl/rasterize/mixed.hpp"
#include "toolkit/physics/system.hpp"

namespace toolkit::assets {

void open_model(entt::registry &registry, std::string filepath);

};

namespace toolkit::opengl {

class editor;

struct ray_query_data {
  float pdist, dist;
  entt::entity entity;
  const float weighted_dist() const { return pdist / dist; }
};
struct compare_ray_query_data {
  bool operator()(const ray_query_data &a, const ray_query_data &b) const {
    return a.weighted_dist() > b.weighted_dist();
  }
};
struct active_camera_manipulate_data {
  bool mouse_first_move = true;
  math::vector2 mouse_last_pos;
  math::vector3 camera_pivot{0.0, 0.0, 0.0};
  // Some parameter related to camera control
  float initial_factor = 0.6f;
  float speed_pow = 1.5f;
  float max_speed = 8e2f;
  // fps-style camera parameters
  float fps_speed = 4;
  float fps_camera_speed = 3.0f;
};
REFLECT(active_camera_manipulate_data, camera_pivot, initial_factor, speed_pow,
        max_speed, fps_speed, fps_camera_speed)

class editor : public iapp {
public:
  void init();
  void run();
  void shutdown();

  void reset();
  void add_default_objects();

  ImGuiIO *imgui_io = nullptr;
  stopwatch timer;

  bool with_translate = true, with_rotate = false, with_scale = false;
  ImGuizmo::OPERATION current_gizmo_operation() {
    return (ImGuizmo::OPERATION)(
        (with_translate ? (int)ImGuizmo::OPERATION::TRANSLATE : 0) |
        (with_rotate ? (int)ImGuizmo::OPERATION::ROTATE : 0) |
        (with_scale ? (int)ImGuizmo::OPERATION::SCALE : 0));
  };
  ImGuizmo::MODE current_gizmo_mode = ImGuizmo::MODE::WORLD;

  entt::entity selected_entity = entt::null;

  defered_forward_mixed *render_sys = nullptr;
  transform_system *transform_sys = nullptr;
  script_system *script_sys = nullptr;
  anim::anim_system *anim_sys = nullptr;
  physics::physics_system *phy_sys = nullptr;

  float click_selection_max_sin = 2e-2f;
  std::vector<ray_query_data> selection_candidates;

  void late_deserialize(nlohmann::json &j) override;
  void late_serialize(nlohmann::json &j) override;

  void draw_main_menubar();
  void draw_entity_hierarchy();
  void draw_entity_components();
  void draw_gizmos(bool enable = true);

  /**
   * Handle keyboard short cut inputs, modify editor states
   */
  void editor_shortkeys();

  /**
   * Unproject the 2d position of cursor to 3d space as a ray. `o` for origin,
   * `d` for direction. Returns whether the ray generation succeeded or not.
   */
  bool mouse_query_ray(math::vector3 &o, math::vector3 &d);
  /**
   * Screen pos ranges [0.0, 1.0]
   */
  bool screen_query_ray(math::vector2 screen_pos, math::vector3 &o,
                        math::vector3 &d);

  void active_camera_manipulate(float dt);

private:
  int gizmo_mode_idx = 0;

  active_camera_manipulate_data cam_manip_data;
};

inline void script_draw_to_scene_proxy(
    iapp *app, std::function<void(editor *, transform &, camera &)> &&f) {
  if (auto eptr = dynamic_cast<editor *>(app)) {
    auto cam_trans =
        eptr->registry.try_get<transform>(g_instance.active_camera);
    auto cam_comp =
        eptr->registry.try_get<opengl::camera>(g_instance.active_camera);
    if (cam_trans != nullptr && cam_comp != nullptr) {
      f(eptr, *cam_trans, *cam_comp);
    }
  }
}

}; // namespace toolkit::opengl