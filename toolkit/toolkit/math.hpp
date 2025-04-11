#pragma once

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Eigen"
#include "Eigen/Geometry"

#include <json.hpp>

#ifdef _WIN32
  #define NOMINMAX
  #define WIN32_LEAN_AND_MEAN
#endif

namespace toolkit::math {

using vector4 = Eigen::Vector4<float>;
using vector3 = Eigen::Vector3<float>;
using vector2 = Eigen::Vector2<float>;
using matrix4 = Eigen::Matrix<float, 4, 4>;
using matrix3 = Eigen::Matrix<float, 3, 3>;
using matrix2 = Eigen::Matrix<float, 2, 2>;
using quat = Eigen::Quaternion<float>;
using angle_axis = Eigen::AngleAxis<float>;
using float2 = vector2;
using float3 = vector3;
using float4 = vector4;
constexpr float MAX_FLOAT = std::numeric_limits<float>::max();

template <typename T1, typename T2> vector3 min3(T1 &&a, T2 &&b) {
  return vector3(std::min(std::forward<T1>(a)(0), std::forward<T2>(b)(0)),
                 std::min(std::forward<T1>(a)(1), std::forward<T2>(b)(1)),
                 std::min(std::forward<T1>(a)(2), std::forward<T2>(b)(2)));
}
template <typename T1, typename T2> vector3 max3(T1 &&a, T2 &&b) {
  return vector3(std::max(std::forward<T1>(a)(0), std::forward<T2>(b)(0)),
                 std::max(std::forward<T1>(a)(1), std::forward<T2>(b)(1)),
                 std::max(std::forward<T1>(a)(2), std::forward<T2>(b)(2)));
}

extern vector3 world_up;
extern vector3 world_left;
extern vector3 world_forward;

// Convert a quaternion to (pitch, yaw, roll) in radians
vector3 quat_to_euler(const quat &q);
// Build a quaternion from euler angles (pitch, yaw, roll), in radians.
quat euler_to_quat(const vector3 &a);

// Decompose q into q2 * q1, q1 is the rotation along given axis
std::tuple<quat, quat> decompose_axis(quat q, vector3 axis);
// Create rotation between from and to
quat from_to_rot(vector3 from, vector3 to);

float rad_to_deg(const float rad);
float deg_to_rad(const float deg);
vector3 rad_to_deg(const vector3 &radVector);
vector3 deg_to_rad(const vector3 &degVector);

double rand(double low, double high);

matrix4 lookat(vector3 eye, vector3 center, vector3 up);
/**
 * `fovy`: vertical fov in radians
 * `aspect`: screen width / screen height
 */
matrix4 perspective(float fovy, float aspect, float znear, float zfar);
matrix4 ortho(float left, float right, float top, float bottom, float zNear,
              float zFar);

}; // namespace toolkit::Math

namespace nlohmann {
template <typename Scalar, int Rows, int Cols, int Options, int MaxRows,
          int MaxCols>
struct adl_serializer<
    Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>> {
  static void to_json(
      nlohmann::json &j,
      const Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols> &mat) {
    if (Rows == Eigen::Dynamic || Cols == Eigen::Dynamic) {
      j["rows"] = mat.rows();
      j["cols"] = mat.cols();
    }

    nlohmann::json data = nlohmann::json::array();
    if (Options & Eigen::RowMajor) {
      for (int i = 0; i < mat.rows(); ++i) {
        for (int k = 0; k < mat.cols(); ++k) {
          data.push_back(mat(i, k));
        }
      }
    } else {
      for (int k = 0; k < mat.cols(); ++k) {
        for (int i = 0; i < mat.rows(); ++i) {
          data.push_back(mat(i, k));
        }
      }
    }
    j["data"] = data;
  }

  static void
  from_json(const nlohmann::json &j,
            Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols> &mat) {
    if (Rows == Eigen::Dynamic || Cols == Eigen::Dynamic) {
      const int rows = j["rows"].get<int>();
      const int cols = j["cols"].get<int>();
      mat.resize(rows, cols);
    }

    const auto &data = j["data"];
    int index = 0;
    if (Options & Eigen::RowMajor) {
      for (int i = 0; i < mat.rows(); ++i) {
        for (int k = 0; k < mat.cols(); ++k) {
          mat(i, k) = data[index++].get<Scalar>();
        }
      }
    } else {
      for (int k = 0; k < mat.cols(); ++k) {
        for (int i = 0; i < mat.rows(); ++i) {
          mat(i, k) = data[index++].get<Scalar>();
        }
      }
    }
  }
};
template <typename Scalar> struct adl_serializer<Eigen::Quaternion<Scalar>> {
  static void to_json(nlohmann::json &j, const Eigen::Quaternion<Scalar> &q) {
    j = nlohmann::json{{"w", q.w()}, {"x", q.x()}, {"y", q.y()}, {"z", q.z()}};
  }

  static void from_json(const nlohmann::json &j, Eigen::Quaternion<Scalar> &q) {
    q.w() = j["w"].get<Scalar>();
    q.x() = j["x"].get<Scalar>();
    q.y() = j["y"].get<Scalar>();
    q.z() = j["z"].get<Scalar>();
    q.normalize();
  }
};
} // namespace nlohmann