#pragma once

#include "toolkit/math.hpp"

namespace toolkit::math {

vector3 solve(matrix3 a, vector3 b);

float cross(vector2 a, vector2 b);

matrix2 outer(vector2 a, vector2 b);

matrix3 outer(vector3 a, vector3 b);

vector2 abs_vec2(vector2 v);

matrix2 abs_mat2(matrix2 a);

matrix2 rotation(float angle);
inline vector2 rotate(float angle, vector2 v);

vector2 transform_position(vector3 q, vector2 v);

}; // namespace toolkit::math