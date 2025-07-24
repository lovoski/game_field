#include "vis_frustom_bbs.hpp"

void draw_view_frustom(toolkit::transform &trans, float fovy_deg, float znear,
                       float zfar, float w_div_h, toolkit::math::matrix4 &vp,
                       toolkit::math::vector3 color) {
  std::vector<std::pair<toolkit::math::vector3, toolkit::math::vector3>>
      frustom_lines(12);
  float half_fovy_rad = fovy_deg / 360.0f * 3.1415927f;
  float z0 = znear, z1 = zfar;
  float h0 = z0 * tan(half_fovy_rad), h1 = z1 * tan(half_fovy_rad),
        asp = w_div_h;
  toolkit::math::vector3 base0 = trans.position() - z0 * trans.local_forward();
  toolkit::math::vector3 base1 = trans.position() - z1 * trans.local_forward();
  frustom_lines[0] = std::make_pair(
      base0 + h0 * trans.local_up() + asp * h0 * trans.local_left(),
      base0 + h0 * trans.local_up() - asp * h0 * trans.local_left());
  frustom_lines[1] = std::make_pair(
      base0 - h0 * trans.local_up() + asp * h0 * trans.local_left(),
      base0 - h0 * trans.local_up() - asp * h0 * trans.local_left());
  frustom_lines[2] = std::make_pair(
      base0 + h0 * trans.local_up() + asp * h0 * trans.local_left(),
      base0 - h0 * trans.local_up() + asp * h0 * trans.local_left());
  frustom_lines[3] = std::make_pair(
      base0 + h0 * trans.local_up() - asp * h0 * trans.local_left(),
      base0 - h0 * trans.local_up() - asp * h0 * trans.local_left());

  frustom_lines[4] = std::make_pair(
      base1 + h1 * trans.local_up() + asp * h1 * trans.local_left(),
      base1 + h1 * trans.local_up() - asp * h1 * trans.local_left());
  frustom_lines[5] = std::make_pair(
      base1 - h1 * trans.local_up() + asp * h1 * trans.local_left(),
      base1 - h1 * trans.local_up() - asp * h1 * trans.local_left());
  frustom_lines[6] = std::make_pair(
      base1 + h1 * trans.local_up() + asp * h1 * trans.local_left(),
      base1 - h1 * trans.local_up() + asp * h1 * trans.local_left());
  frustom_lines[7] = std::make_pair(
      base1 + h1 * trans.local_up() - asp * h1 * trans.local_left(),
      base1 - h1 * trans.local_up() - asp * h1 * trans.local_left());

  frustom_lines[8] = std::make_pair(
      base0 + h0 * trans.local_up() + asp * h0 * trans.local_left(),
      base1 + h1 * trans.local_up() + asp * h1 * trans.local_left());
  frustom_lines[9] = std::make_pair(
      base0 + h0 * trans.local_up() - asp * h0 * trans.local_left(),
      base1 + h1 * trans.local_up() - asp * h1 * trans.local_left());
  frustom_lines[10] = std::make_pair(
      base0 - h0 * trans.local_up() + asp * h0 * trans.local_left(),
      base1 - h1 * trans.local_up() + asp * h1 * trans.local_left());
  frustom_lines[11] = std::make_pair(
      base0 - h0 * trans.local_up() - asp * h0 * trans.local_left(),
      base1 - h1 * trans.local_up() - asp * h1 * trans.local_left());

  toolkit::opengl::draw_lines(frustom_lines, vp, color);
}