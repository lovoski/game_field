#include "toolkit/math.hpp"
#include <random>

namespace toolkit::math {

vector3 world_up = vector3(0.0, 1.0, 0.0);
vector3 world_right = vector3(1.0, 0.0, 0.0);
vector3 world_forward = vector3(0.0, 0.0, 1.0);

vector3 quat_to_euler(const quat &q) {
  float sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
  float cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  float roll = std::atan2(sinr_cosp, cosr_cosp);

  float sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
  float pitch;
  if (std::abs(sinp) >= 1)
    pitch =
        std::copysign(3.1415926535 / 2, sinp); // use 90 degrees if out of range
  else
    pitch = std::asin(sinp);

  float siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
  float cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  float yaw = std::atan2(siny_cosp, cosy_cosp);

  return vector3(roll, pitch, yaw);
}

quat euler_to_quat(const vector3 &a) {
  quat ret;
  vector3 c = (a * 0.5).array().cos();
  vector3 s = (a * 0.5).array().sin();
  ret.w() = c.x() * c.y() * c.z() + s.x() * s.y() * s.z();
  ret.x() = s.x() * c.y() * c.z() - c.x() * s.y() * s.z();
  ret.y() = c.x() * s.y() * c.z() + s.x() * c.y() * s.z();
  ret.z() = c.x() * c.y() * s.z() - s.x() * s.y() * c.z();
  return ret;
}
vector3 rad_to_deg(const vector3 &radVector) {
  return radVector * (180.0 / 3.1415926535);
}
vector3 deg_to_rad(const vector3 &degVector) {
  return degVector * (3.1415926535 / 180.0);
}
float rad_to_deg(const float rad) { return rad * (180.0 / 3.1415926535); }
float deg_to_rad(const float deg) { return deg * (3.1415926535 / 180.0); }
double rand(double low, double high) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(low, high);
  return dis(gen);
}

matrix4 lookat(vector3 eye, vector3 center, vector3 up) {
  // z
  vector3 f = (eye - center).normalized();
  // x
  vector3 s = (up.cross(f)).normalized();
  // y
  vector3 u = f.cross(s);

  matrix4 result = matrix4::Identity();
  result(0, 0) = s.x();
  result(0, 1) = s.y();
  result(0, 2) = s.z();
  result(1, 0) = u.x();
  result(1, 1) = u.y();
  result(1, 2) = u.z();
  result(2, 0) = f.x();
  result(2, 1) = f.y();
  result(2, 2) = f.z();

  result(0, 3) = -s.dot(eye);
  result(1, 3) = -u.dot(eye);
  result(2, 3) = -f.dot(eye);

  return result;
}

matrix4 perspective(float fovy, float aspect, float znear, float zfar) {
  float tanHalfFovy = std::tan(fovy / 2.0);
  matrix4 result = matrix4::Zero();
  result(0, 0) = 1.0 / (aspect * tanHalfFovy);
  result(1, 1) = 1.0 / (tanHalfFovy);
  result(2, 2) = -(zfar + znear) / (zfar - znear);
  result(3, 2) = -1.0;
  result(2, 3) = -(2.0 * zfar * znear) / (zfar - znear);
  return result;
}

matrix4 ortho(float left, float right, float top, float bottom, float zNear,
              float zFar) {
  matrix4 Result = matrix4::Identity();
  Result(0, 0) = 2.0f / (right - left);
  Result(1, 1) = 2.0f / (top - bottom);
  Result(2, 2) = -2.0f / (zFar - zNear);
  Result(0, 3) = -(right + left) / (right - left);
  Result(1, 3) = -(top + bottom) / (top - bottom);
  Result(2, 3) = -(zFar + zNear) / (zFar - zNear);
  return Result;
}

vector3 mat_log(matrix3 R) {
  // Get the angle from the trace
  float trace = R.trace();
  float cos_theta = (trace - 1.0f) * 0.5f;
  cos_theta = std::clamp(cos_theta, -1.0f, 1.0f); // Numerical stability
  float theta = std::acos(cos_theta);

  // If angle is close to 0 or π, handle special cases
  if (theta < 1e-6f) {
    return vector3::Zero(); // No rotation
  } else if (std::abs(theta - 3.1415926) < 1e-6f) {
    // When theta = π, find the axis using the diagonal elements
    vector3 axis(std::sqrt((R(0, 0) + 1.0f) * 0.5f),
                 std::sqrt((R(1, 1) + 1.0f) * 0.5f),
                 std::sqrt((R(2, 2) + 1.0f) * 0.5f));
    // Fix signs using off-diagonal elements
    if (R(2, 1) < R(1, 2))
      axis.x() = -axis.x();
    if (R(0, 2) < R(2, 0))
      axis.y() = -axis.y();
    if (R(1, 0) < R(0, 1))
      axis.z() = -axis.z();
    return theta * axis.normalized();
  } else {
    // General case: extract axis using skew-symmetric part
    vector3 axis(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
    axis = axis.normalized() * theta;
    return axis;
  }
}

matrix3 mat_exp(vector3 v) {
  float theta = v.norm();

  // Handle small angle case
  if (theta < 1e-6f) {
    return matrix3::Identity();
  }

  vector3 axis = v.normalized();
  float cos_theta = std::cos(theta);
  float sin_theta = std::sin(theta);
  float one_minus_cos = 1.0f - cos_theta;

  // Build rotation matrix using Rodrigues' formula
  matrix3 R;
  R(0, 0) = cos_theta + axis.x() * axis.x() * one_minus_cos;
  R(1, 1) = cos_theta + axis.y() * axis.y() * one_minus_cos;
  R(2, 2) = cos_theta + axis.z() * axis.z() * one_minus_cos;

  float tmp1 = axis.x() * axis.y() * one_minus_cos;
  float tmp2 = axis.z() * sin_theta;
  R(0, 1) = tmp1 - tmp2;
  R(1, 0) = tmp1 + tmp2;

  tmp1 = axis.x() * axis.z() * one_minus_cos;
  tmp2 = axis.y() * sin_theta;
  R(0, 2) = tmp1 + tmp2;
  R(2, 0) = tmp1 - tmp2;

  tmp1 = axis.y() * axis.z() * one_minus_cos;
  tmp2 = axis.x() * sin_theta;
  R(1, 2) = tmp1 - tmp2;
  R(2, 1) = tmp1 + tmp2;

  return R;
}

std::tuple<quat, quat> decompose_axis(quat q, vector3 axis) {
  axis = axis.normalized();
  vector3 axisRot = q * axis;
  quat q2 = from_to_rot(axis, axisRot);
  quat q1 = q2.inverse() * q;
  return {q1, q2};
}

quat from_to_rot(vector3 from, vector3 to) {
  from = from.normalized();
  to = to.normalized();
  vector3 axis = from.cross(to);
  float dotProduct = std::clamp(from.dot(to), -1.0f, 1.0f);
  if (dotProduct > 1.0f - 1e-6f)
    return quat::Identity(); // no rotation needed
  if (dotProduct < -1.0f + 1e-6f)
    return quat(angle_axis(3.1415926535f, from.unitOrthogonal()));
  float theta = std::acos(dotProduct);
  return quat(angle_axis(theta, axis.normalized()));
}

void decompose_transform(matrix4 transform, vector3 &translation,
                         quat &rotation, vector3 &scale) {
  translation = vector3(transform(0, 3), transform(1, 3), transform(2, 3));
  scale = vector3(transform.col(0).norm(), transform.col(1).norm(),
                  transform.col(2).norm());
  matrix4 rot;
  rot << transform.col(0) / scale.x(), transform.col(1) / scale.y(),
      transform.col(2) / scale.z(), math::vector4(0.0, 0.0, 0.0, 1.0);
  rotation = math::quat(rot.block<3, 3>(0, 0));
}
math::matrix4 compose_transform(vector3 &translation, quat &rotation,
                                vector3 &scale) {
  Eigen::Transform<float, 3, 2> transform =
      Eigen::Transform<float, 3, 2>::Identity();
  transform.translate(translation).rotate(rotation).scale(scale);
  return transform.matrix();
}

math::vector3 quat_to_rot_vec(math::quat q) {
  float half_theta = std::atan2(q.vec().norm(), q.w());
  float sin_half_theta = sin(half_theta);
  if (abs(sin_half_theta) < 1e-5f)
    return math::vector3::Zero();
  return half_theta * 2 / sin_half_theta * math::vector3(q.x(), q.y(), q.z());
}
math::quat rot_vec_to_quat(math::vector3 a) {
  float theta = a.norm();
  if (abs(theta) < 1e-5f)
    return math::quat::Identity();
  float sin_half_theta = sin(theta / 2);
  float cos_half_theta = cos(theta / 2);
  if (cos_half_theta < 0) {
    cos_half_theta = -cos_half_theta;
    sin_half_theta = -sin_half_theta;
  }
  return math::quat(cos(theta / 2), sin_half_theta / theta * a.x(),
                    sin_half_theta / theta * a.y(),
                    sin_half_theta / theta * a.z());
}

}; // namespace toolkit::math