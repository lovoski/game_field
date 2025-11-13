#include "toolkit/sim/components/algo.hpp"
#include <algorithm>
#include <random>
#include <limits>

namespace toolkit::sim {

// numerical epsilons
static constexpr float EPS_COLLINEAR = 1e-10f;
static constexpr float EPS_COPLANAR = 1e-10f;
static constexpr float EPS_INSIDE = 1e-6f;

// Helper: check if point is inside sphere (allow small tolerance)
static bool point_in_sphere(const math::vector3 &p, const math::vector3 &center,
                            float radius) {
  float rtol = radius + EPS_INSIDE;
  return (p - center).squaredNorm() <= rtol * rtol;
}

// Helper: compute sphere from 2 points (diameter)
static std::pair<math::vector3, float>
sphere_from_2pts(const math::vector3 &a, const math::vector3 &b) {
  math::vector3 center = (a + b) * 0.5f;
  float radius = (a - b).norm() * 0.5f;
  return std::make_pair(center, radius);
}

// Helper: compute sphere from 3 points (circumcircle in the plane of the three points)
// Uses a stable 2x2 linear solve inside the plane spanned by ba and ca.
static std::pair<math::vector3, float>
sphere_from_3pts(const math::vector3 &a, const math::vector3 &b,
                 const math::vector3 &c) {
  math::vector3 ba = b - a;
  math::vector3 ca = c - a;

  // If points are nearly collinear, return the diameter sphere between the farthest pair.
  math::vector3 normal = ba.cross(ca);
  if (normal.squaredNorm() < EPS_COLLINEAR) {
    // pick the pair with largest separation
    auto s1 = sphere_from_2pts(a, b);
    auto s2 = sphere_from_2pts(a, c);
    auto s3 = sphere_from_2pts(b, c);
    // choose smallest sphere that contains all three (diameter between farthest pair)
    std::pair<math::vector3, float> best = s1;
    if (s2.second > best.second) best = s2;
    if (s3.second > best.second) best = s3;
    return best;
  }

  // Solve for center = a + u*ba + v*ca
  // We want center·ba = (b·b - a·a)/2 and center·ca = (c·c - a·a)/2
  // Substitute center = a + u*ba + v*ca and solve 2x2 system for u and v.
  float S00 = ba.dot(ba);        // ba·ba
  float S01 = ba.dot(ca);        // ba·ca
  float S11 = ca.dot(ca);        // ca·ca

  // RHS
  float rhs0 = 0.5f * (b.dot(b) - a.dot(a)) - a.dot(ba);
  float rhs1 = 0.5f * (c.dot(c) - a.dot(a)) - a.dot(ca);

  float det = S00 * S11 - S01 * S01;
  if (std::abs(det) < EPS_COLLINEAR) {
    // numerically degenerate — fallback to longest-pair diameter
    auto s1 = sphere_from_2pts(a, b);
    auto s2 = sphere_from_2pts(a, c);
    auto s3 = sphere_from_2pts(b, c);
    std::pair<math::vector3, float> best = s1;
    if (s2.second > best.second) best = s2;
    if (s3.second > best.second) best = s3;
    return best;
  }

  float invDet = 1.0f / det;
  float u = ( rhs0 * S11 - rhs1 * S01) * invDet;
  float v = (-rhs0 * S01 + rhs1 * S00) * invDet;

  math::vector3 center = a + ba * u + ca * v;
  float radius = (center - a).norm();
  return std::make_pair(center, radius);
}

// Helper: compute sphere from 4 points (circumsphere of tetrahedron)
// If matrix nearly singular (coplanar/near-coplanar), fallback to testing triple spheres.
static std::pair<math::vector3, float>
sphere_from_4pts(const math::vector3 &a, const math::vector3 &b,
                 const math::vector3 &c, const math::vector3 &d) {
  math::vector3 ba = b - a;
  math::vector3 ca = c - a;
  math::vector3 da = d - a;

  math::matrix3 M;
  // [REVISED] The matrix rows should be ba, ca, da to solve the system
  // (C-a)·ba = 0.5*|ba|², (C-a)·ca = 0.5*|ca|², (C-a)·da = 0.5*|da|²
  M.row(0) = ba;
  M.row(1) = ca;
  M.row(2) = da;

  float det = M.determinant();
  if (std::abs(det) < EPS_COPLANAR) {
    // Points are (nearly) coplanar. The minimal sphere is determined by at most 3 points.
    // We'll compute candidate spheres for each triple and choose a candidate that
    // contains all 4 points (if any), preferring the one with smallest radius.
    std::array<std::pair<math::vector3, float>, 4> candidates = {
        sphere_from_3pts(a, b, c), sphere_from_3pts(a, b, d),
        sphere_from_3pts(a, c, d), sphere_from_3pts(b, c, d)};

    bool found = false;
    std::pair<math::vector3, float> best;
    for (const auto &cand : candidates) {
      // check that candidate contains all four points (with tolerance)
      if (point_in_sphere(a, cand.first, cand.second) &&
          point_in_sphere(b, cand.first, cand.second) &&
          point_in_sphere(c, cand.first, cand.second) &&
          point_in_sphere(d, cand.first, cand.second)) {
        if (!found || cand.second < best.second) {
          best = cand;
          found = true;
        }
      }
    }
    if (found) return best;

    // If none of the triple-spheres contains all points (rare), fall back to picking
    // the largest pairwise-diameter (safe fallback, not minimal but robust).
    auto s1 = sphere_from_2pts(a, b);
    auto s2 = sphere_from_2pts(a, c);
    auto s3 = sphere_from_2pts(a, d);
    auto s4 = sphere_from_2pts(b, c);
    auto s5 = sphere_from_2pts(b, d);
    auto s6 = sphere_from_2pts(c, d);
    std::pair<math::vector3, float> best_pair = s1;
    for (const auto &s : {s2, s3, s4, s5, s6})
      if (s.second > best_pair.second) best_pair = s;
    return best_pair;
  }

  // Solve M * x = rhs for x, where center = a + x
  math::vector3 rhs(ba.squaredNorm() * 0.5f, ca.squaredNorm() * 0.5f,
                    da.squaredNorm() * 0.5f);

  // Use inverse (if M is well conditioned via determinant check above).
  // If you link Eigen, you could replace this with a direct solver like:
  // math::vector3 center_local = M.colPivHouseholderQr().solve(rhs);
  math::vector3 center_local = M.inverse() * rhs;
  math::vector3 center = a + center_local;
  float radius = (a - center).norm();

  return std::make_pair(center, radius);
}

// Welzl's algorithm helper (keeps the same signature as requested)
static std::pair<math::vector3, float>
welzl_helper(const std::vector<math::vector3>& points, // [REVISED] Pass by const&
             std::vector<math::vector3> boundary,     // [REVISED] Pass boundary by value
             size_t n) {
  if (n == 0 || boundary.size() == 4) {
    if (boundary.empty())
      return std::make_pair(math::vector3::Zero(), 0.0f);
    if (boundary.size() == 1)
      return std::make_pair(boundary[0], 0.0f);
    if (boundary.size() == 2)
      return sphere_from_2pts(boundary[0], boundary[1]);
    if (boundary.size() == 3)
      return sphere_from_3pts(boundary[0], boundary[1], boundary[2]);
    return sphere_from_4pts(boundary[0], boundary[1], boundary[2], boundary[3]);
  }

  size_t idx = n - 1;
  math::vector3 p = points[idx];

  // Recurse without p
  // Note: points is const&, so no copy is made here
  auto s = welzl_helper(points, boundary, n - 1);

  // If p is outside the current sphere, then p must be on the boundary.
  if (!point_in_sphere(p, s.first, s.second)) {
    // add p to boundary and recurse
    // note: 'boundary' is a copy, so this push_back doesn't affect the
    //       sibling recursive call branch.
    boundary.push_back(p);
    s = welzl_helper(points, boundary, n - 1);
  }

  return s;
}

std::pair<math::vector3, float>
welzl_bounding_sphere(const std::vector<math::vector3> &points, bool shuffle) { // [REVISED]
  if (points.size() == 0) {
    return std::make_pair(math::vector3::Zero(), 0.0f);
  } else if (points.size() == 1) {
    return std::make_pair(points[0], 0.0f);
  }
  if (shuffle) {
    // Shuffle points for better average-case performance
    auto shuffled_points = points; // Copy is intentional for shuffling
    std::mt19937 rng(std::random_device{}());
    std::shuffle(shuffled_points.begin(), shuffled_points.end(), rng);
    std::vector<math::vector3> boundary;
    // Pass shuffled_points by const&
    return welzl_helper(shuffled_points, boundary, shuffled_points.size());
  } else {
    std::vector<math::vector3> boundary;
    // Pass original points by const&
    return welzl_helper(points, boundary, points.size());
  }
}

}; // namespace toolkit::sim