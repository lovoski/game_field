#include "toolkit/sdl2d/header.hpp"

namespace toolkit::math {

math::matrix2 from_angle(float angle) {
  math::matrix2 m;
  float c = std::cosf(angle), s = std::sinf(angle);
  m.col(0).x() = c;
  m.col(0).y() = s;
  m.col(1).x() = -s;
  m.col(1).y() = c;
  return m;
}

}; // namespace toolkit::math