#include "toolkit/sim/algorithms/algo.hpp"

#define FLOAT_MAX std::numeric_limits<float>::max()
#define EPA_MAX_ITERATIONS 100
using vector3i = Eigen::Vector3i;
using namespace toolkit::math;

namespace toolkit::sim {

void get_face_normal_dist_to_origin(const std::vector<vector3> &polytope,
                                    const std::vector<vector3i> &faces,
                                    std::vector<vector3> &normals,
                                    std::vector<float> &dist_to_origin,
                                    int &min_dist_face_idx);

void add_edge_if_no_reversed(
    std::vector<std::pair<std::size_t, std::size_t>> &edges, std::size_t a,
    std::size_t b) {
  // Search for the reverse edge (b, a)
  auto reverse = std::find(edges.begin(), edges.end(), std::make_pair(b, a));
  if (reverse != edges.end()) {
    // Found (b, a). This is an interior edge.
    // Remove (b, a) and do not add (a, b).
    edges.erase(reverse);
  } else {
    // Did not find (b, a). This is a unique edge (so far).
    edges.emplace_back(std::make_pair(a, b));
  }
}

/**
 * Reference:
 * https://winter.dev/articles/epa-algorithm
 *
 * This algorithm takes the final simplex that contained the origin and finds
 * the normal of collision, aka the shortest vector to nudge the shapes out of
 * each other.
 *
 * The naive solution is to use the normal of the closest face to the origin,
 * but remember, a simplex does not need to contain any of the original
 * polygon's faces, so we could end up with an incorrect normal.
 */
bool epa_contact(base_collider *c1, base_collider *c2, gjk_simplex &simplex,
                 vector3 &_normal, float &_penetration) {
  std::vector<vector3> polytope;
  std::vector<vector3i> faces;
  // build initial polytope from GJK simplex
  assert(simplex.dim == 4);
  polytope.resize(4);
  polytope[0] = simplex.a;
  polytope[1] = simplex.b;
  polytope[2] = simplex.c;
  polytope[3] = simplex.d;
  faces.resize(4);
  faces[0] = vector3i(0, 1, 2);
  faces[1] = vector3i(0, 3, 1);
  faces[2] = vector3i(0, 2, 3);
  faces[3] = vector3i(1, 3, 2);

  std::vector<vector3> normals;
  std::vector<float> dist_to_origin;
  int min_dist_face_idx = -1;
  // normals computed by thiss function always points outwards the origin, so we
  // can search the minkowski difference for another support point directly with
  // this normal as direction
  get_face_normal_dist_to_origin(polytope, faces, normals, dist_to_origin,
                                 min_dist_face_idx);

  // find the normal of the face closest to the origin
  vector3 min_normal = normals[min_dist_face_idx];
  float min_distance = dist_to_origin[min_dist_face_idx];
  bool converged = false;
  for (int it = 0; it < EPA_MAX_ITERATIONS; it++) {
    // min_normal always points to the origin
    vector3 support = support_point_of_minkowski_difference(c1, c2, min_normal);
    float support_dist = min_normal.dot(support);
    // compute the centroid for face that's closest to the origin, when the
    // support point is too close to this centroid, a degenerated case might
    // happen, we simply call this as converged
    vector3 min_face_centroid = polytope[faces[min_dist_face_idx].x()] +
                                polytope[faces[min_dist_face_idx].y()] +
                                polytope[faces[min_dist_face_idx].z()] / 3.0f;
    // another convergence case is when we can't find a support point on the
    // minkowski difference that is closer to the origin, considering a small
    // enough tolerance to improve numerical stability
    // TODO: early termination of convergence could result in inconsistent
    // contact normal between primitive and convex hull polytope collision
    if ((std::abs(support_dist - min_distance) < 1e-2f) ||
        ((support - min_face_centroid).norm() < 1e-2f)) {
      _normal = min_normal;
      _penetration = min_distance + 1e-5f;
      converged = true;
      break;
    }
    // add 'support' to the polytope
    int support_point_idx = polytope.size();
    polytope.push_back(support);
    std::vector<std::pair<std::size_t, std::size_t>> edges;
    for (int i = normals.size() - 1; i >= 0; i--) {
      vector3 centroid = (polytope[faces[i].x()] + polytope[faces[i].y()] +
                          polytope[faces[i].z()]) /
                         3.0f;
      // the normal direction points outwards the origin, this determines
      // whether the face are visible to the new support point, if so, this face
      // need to be removed
      if (normals[i].dot(support - centroid) > 0.0f) {
        // the rule of adding edge is, we can't have an edge both in forward and
        // reverse order, this would create an intermediate face that's not
        // desired. when we attempt to add one edge to the collection, we check
        // for reversed edge existence, if so, we remove the existing reverse
        // edge and do nothing, since adding this edge would lead to undesired
        // intermediate face; if reversed edge not found, we can safely add it.
        add_edge_if_no_reversed(edges, faces[i].x(), faces[i].y());
        add_edge_if_no_reversed(edges, faces[i].y(), faces[i].z());
        add_edge_if_no_reversed(edges, faces[i].z(), faces[i].x());
        // remove this face
        faces.erase(faces.begin() + i);
        normals.erase(normals.begin() + i);
        dist_to_origin.erase(dist_to_origin.begin() + i);
      }
    }
    // reconstruct the removed faces from edges
    for (int i = 0; i < edges.size(); i++) {
      faces.push_back(
          vector3i(edges[i].first, edges[i].second, support_point_idx));
    }
    // recompute normals and distance to origin for the expanded polytope
    get_face_normal_dist_to_origin(polytope, faces, normals, dist_to_origin,
                                   min_dist_face_idx);
    min_normal = normals[min_dist_face_idx];
    min_distance = dist_to_origin[min_dist_face_idx];
  }

  if (!converged) {
    spdlog::error("EPA did not converge");
    return false;
  }
  return true;
}

void get_face_normal_dist_to_origin(const std::vector<vector3> &polytope,
                                    const std::vector<vector3i> &faces,
                                    std::vector<vector3> &normals,
                                    std::vector<float> &dist_to_origin,
                                    int &min_dist_face_idx) {
  normals.resize(faces.size());
  dist_to_origin.resize(faces.size());
  float min_distance = FLOAT_MAX;
  for (int i = 0; i < faces.size(); i++) {
    auto a = polytope[faces[i].x()];
    auto b = polytope[faces[i].y()];
    auto c = polytope[faces[i].z()];
    vector3 ab = b - a;
    vector3 ac = c - a;
    vector3 normal = ab.cross(ac);
    float len = normal.norm();
    // reject degenerate face
    if (len < 1e-6f) {
      normals[i] = vector3(0, 0, 0);
      dist_to_origin[i] = FLOAT_MAX;
      continue;
    }
    normal /= len;
    float distance = normal.dot(a); // signed distance
    // ensure outward direction:
    if (distance < 0) {
      normal = -normal;
      distance = -distance;
    }
    normals[i] = normal;
    dist_to_origin[i] = distance;
    if (distance < min_distance) {
      min_dist_face_idx = i;
      min_distance = distance;
    }
  }
}

}; // namespace toolkit::sim