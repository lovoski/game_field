#include "utils.hpp"

namespace toolkit::math {

vector3 solve(matrix3 a, vector3 b) {
  // Compute LDL^T decomposition
  float D1 = a(0, 0);
  float L21 = a(1, 0) / a(0, 0);
  float L31 = a(2, 0) / a(0, 0);
  float D2 = a(1, 1) - L21 * L21 * D1;
  float L32 = (a(2, 1) - L21 * L31 * D1) / D2;
  float D3 = a(2, 2) - (L31 * L31 * D1 + L32 * L32 * D2);

  // Forward substitution: Solve Ly = b
  float y1 = b.x();
  float y2 = b.y() - L21 * y1;
  float y3 = b.z() - L31 * y1 - L32 * y2;

  // Diagonal solve: Solve Dz = y
  float z1 = y1 / D1;
  float z2 = y2 / D2;
  float z3 = y3 / D3;

  // Backward substitution: Solve L^T x = z
  vector3 x;
  x[2] = z3;
  x[1] = z2 - L32 * x[2];
  x[0] = z1 - L21 * x[1] - L31 * x[2];

  return x;
}

float cross(vector2 a, vector2 b) { return a.x() * b.y() - a.y() * b.x(); }

matrix2 outer(vector2 a, vector2 b) {
  matrix2 m;
  m.row(0) = b * a.x();
  m.row(1) = b * a.y();
  return m;
}

matrix3 outer(vector3 a, vector3 b) {
  matrix3 m;
  m.row(0) = b * a.x();
  m.row(1) = b * a.y();
  m.row(2) = b * a.z();
  return m;
}

vector2 abs_vec2(vector2 v) { return {fabsf(v.x()), fabsf(v.y())}; }

matrix2 abs_mat2(matrix2 a) {
  matrix2 m;
  m.row(0) = abs_vec2(a.row(0));
  m.row(1) = abs_vec2(a.row(1));
  return m;
}

matrix2 rotation(float angle) {
  matrix2 m;
  float c = cos(angle);
  float s = sin(angle);
  m(0, 0) = c;
  m(0, 1) = -s;
  m(1, 0) = s;
  m(1, 1) = c;
  return m;
}

vector2 rotate(float angle, vector2 v) { return rotation(angle) * v; }

vector2 transform(vector3 q, vector2 v) { return rotation(q.z()) * v + q.head<2>(); }

}; // namespace toolkit::math