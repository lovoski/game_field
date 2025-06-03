#include "toolkit/math.hpp"
#include <random>

namespace toolkit::math {

vector3 world_up = vector3(0.0, 1.0, 0.0);
vector3 world_left = vector3(1.0, 0.0, 0.0);
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
  vector3 f = (center - eye).normalized();
  vector3 s = (f.cross(up)).normalized();
  vector3 u = s.cross(f);

  matrix4 result = matrix4::Identity();
  result(0, 0) = s.x();
  result(0, 1) = s.y();
  result(0, 2) = s.z();
  result(1, 0) = u.x();
  result(1, 1) = u.y();
  result(1, 2) = u.z();
  result(2, 0) = -f.x();
  result(2, 1) = -f.y();
  result(2, 2) = -f.z();
  result(0, 3) = -s.dot(eye);
  result(1, 3) = -u.dot(eye);
  result(2, 3) = f.dot(eye);
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

}; // namespace toolkit::math