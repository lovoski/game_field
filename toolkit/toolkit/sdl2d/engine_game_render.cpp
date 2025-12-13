#include "toolkit/sdl2d/engine.hpp"

namespace toolkit::sdl2d {

void engine2d::handle_game_render_tick() {
  // registry.view<box2d::body>().each(
  //     [&](entt::entity entity, box2d::body &body_comp) {
  //       auto rotation = math::from_angle(body_comp.rotation);
  //       std::vector<math::vector2> points{
  //           rotation * math::vector2(0.5 * body_comp.size.x(),
  //                                    0.5 * body_comp.size.y()) +
  //               body_comp.position,
  //           rotation * math::vector2(-0.5 * body_comp.size.x(),
  //                                    0.5 * body_comp.size.y()) +
  //               body_comp.position,
  //           rotation * math::vector2(-0.5 * body_comp.size.x(),
  //                                    -0.5 * body_comp.size.y()) +
  //               body_comp.position,
  //           rotation * math::vector2(0.5 * body_comp.size.x(),
  //                                    -0.5 * body_comp.size.y()) +
  //               body_comp.position,
  //       };
  //       for (int i = 0; i < 4; i++) {
  //         points[i] = world_to_screen(points[i]);
  //       }
  //       ss_draw_lines(points, true);
  //     });

  avbd_solver.debug_draw(this);

  ss_draw_circle(100, 100, 20, true, math::vector4(1, 0, 0, 0.5));
  ss_draw_circle(140, 100, 50, true, math::vector4(1, 1, 0, 0.5));
  ss_draw_rectangle(100, 150, 20, 30, true, math::vector4(1, 0, 0, 0.5));
  ss_draw_rectangle(110, 170, 20, 30, true, math::vector4(1, 1, 0, 0.5));
}

}; // namespace toolkit::sdl2d