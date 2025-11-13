#include "toolkit/sim/components/algo.hpp"

namespace toolkit::sim {

void polytope_from_gjk_simplex(const gjk_simplex &simplex,
                               std::vector<math::vector3> &polytope,
                               std::vector<Eigen::Vector3i> &faces) {
  assert(simplex.num == 4);
  polytope.resize(4);
  polytope[0] = simplex.a;
  polytope[1] = simplex.b;
  polytope[2] = simplex.c;
  polytope[3] = simplex.d;
  faces.resize(4);
  faces[0] = Eigen::Vector3i(0, 1, 2); // ABC
  faces[1] = Eigen::Vector3i(0, 2, 3); // ACD
  faces[2] = Eigen::Vector3i(0, 3, 1); // ADB
  faces[3] = Eigen::Vector3i(1, 2, 3); // BCD
}

void get_face_normal_and_distance_to_origin(
    Eigen::Vector3i &face, std::vector<math::vector3> &polytope,
    math::vector3 &_normal, float &_distance) {
  math::vector3 a = polytope[face.x()];
  math::vector3 b = polytope[face.y()];
  math::vector3 c = polytope[face.z()];
  math::vector3 ab = b - a;
  math::vector3 ac = c - a;
  math::vector3 normal = (ab.cross(ac)).normalized();
  assert(normal.x() != 0.0 || normal.y() != 0.0 || normal.z() != 0.0);
  // When this value is not 0, it is possible that the normals are not found
  // even if the polytope is not degenerate
  const float DISTANCE_TO_ORIGIN_TOLERANCE = 0.0f;
  float distance = normal.dot(a);
  // the distance from the face's *plane* to the origin (considering an infinite
  // plane).
  if (distance < -DISTANCE_TO_ORIGIN_TOLERANCE) {
    // if the distance is less than 0, it means that our normal is point inwards
    // instead of outwards in this case, we just invert both normal and distance
    // this way, we don't need to worry about face's winding
    normal = -normal;
    distance = -distance;
  } else if (distance >= -DISTANCE_TO_ORIGIN_TOLERANCE &&
             distance <= DISTANCE_TO_ORIGIN_TOLERANCE) {
    // if the distance is exactly 0.0, then it means that the origin is lying
    // exactly on the face. in this case, we can't directly infer the
    // orientation of the normal. since our shape is convex, we analyze the
    // other vertices of the hull to deduce the orientation
    bool was_able_to_calculate_normal = false;
    for (int i = 0; i < polytope.size(); ++i) {
      math::vector3 current = polytope[i];
      float auxiliar_distance = normal.dot(current);
      if (auxiliar_distance < -DISTANCE_TO_ORIGIN_TOLERANCE ||
          auxiliar_distance > DISTANCE_TO_ORIGIN_TOLERANCE) {
        // since the shape is convex, the other vertices should always be
        // "behind" the normal plane
        normal = auxiliar_distance < -DISTANCE_TO_ORIGIN_TOLERANCE ? normal
                                                                   : -normal;
        was_able_to_calculate_normal = true;
        break;
      }
    }
    // If we were not able to calculate the normal, it means that ALL points of
    // the polytope are in the same plane Therefore, we either have a degenerate
    // polytope or our tolerance is not big enough
    assert(was_able_to_calculate_normal);
  }
  _normal = normal;
  _distance = distance;
}

void add_edge(std::vector<Eigen::Vector2i> &edges, Eigen::Vector2i edge,
              const std::vector<math::vector3> &polytope) {
  for (int i = 0; i < edges.size(); i++) {
    auto current = edges[i];
    if ((edge.x() == current.x() && edge.y() == current.y()) ||
        (edge.x() == current.y() && edge.y() == current.x())) {
      edges.erase(edges.begin() + i);
      return;
    }
    // @TEMPORARY: Once indexes point to unique vertices, this won't be needed.
    math::vector3 current_v1 = polytope[current.x()];
    math::vector3 current_v2 = polytope[current.y()];
    math::vector3 edge_v1 = polytope[edge.x()];
    math::vector3 edge_v2 = polytope[edge.y()];
    if (((current_v1 == edge_v1) && (current_v2 == edge_v2)) ||
        ((current_v1 == edge_v2) && (current_v2 == edge_v1))) {
      edges.erase(edges.begin() + i);
      return;
    }
  }
  edges.push_back(edge);
}

bool epa(base_collider *c1, base_collider *c2, gjk_simplex &simplex,
         math::vector3 &_normal, float &_penetration) {
  std::vector<math::vector3> polytope;
  std::vector<Eigen::Vector3i> faces;
  // build initial polytope from GJK simplex
  polytope_from_gjk_simplex(simplex, polytope, faces);

  std::vector<math::vector3> normals(128);
  std::vector<float> faces_distance_to_origin(128);

  math::vector3 min_normal;
  float min_distance = std::numeric_limits<float>::max();
  for (int i = 0; i < faces.size(); i++) {
    math::vector3 normal;
    float distance;
    auto face = faces[i];

    get_face_normal_and_distance_to_origin(face, polytope, normal, distance);

    normals.push_back(normal);
    faces_distance_to_origin.push_back(distance);

    if (distance < min_distance) {
      min_distance = distance;
      min_normal = normal;
    }
  }

  std::vector<Eigen::Vector2i> edges(1024);
  bool converged = false;
  for (int it = 0; it < 100; it++) {
    math::vector3 support_point =
        support_point_of_minkowski_difference(c1, c2, min_normal);
    // If the support time lies on the face currently set as the closest to the
    // origin, we are done.
    float d = min_normal.dot(support_point);
    if (fabs(d - min_distance) < 1e-4f) {
      _normal = min_normal;
      _penetration = min_distance;
      converged = true;
      break;
    }
    // add new point to polytope
    int new_point_index = polytope.size();
    polytope.push_back(support_point);
    // expand polytope
    for (int i = 0; i < normals.size(); i++) {
      math::vector3 normal = normals[i];
      auto face = faces[i];
      // If the face normal points towards the support point, we need to
      // reconstruct it.
      math::vector3 centroid =
          (polytope[face.x()] + polytope[face.y()] + polytope[face.z()]) / 3.0f;
      // If the face normal points towards the support point, we need to
      // reconstruct it.
      if (normal.dot(support_point - centroid) > 0.0f) {
        Eigen::Vector2i edge1 = Eigen::Vector2i(face.x(), face.y()),
                        edge2 = Eigen::Vector2i(face.y(), face.z()),
                        edge3 = Eigen::Vector2i(face.z(), face.x());
        add_edge(edges, edge1, polytope);
        add_edge(edges, edge2, polytope);
        add_edge(edges, edge3, polytope);

        faces.erase(faces.begin() + i);
        faces_distance_to_origin.erase(faces_distance_to_origin.begin() + i);
        normals.erase(normals.begin() + i);
        --i;
      }
    }
    for (int i = 0; i < edges.size(); i++) {
      auto edge = edges[i];
      Eigen::Vector3i new_face;
      new_face.x() = edge.x();
      new_face.y() = edge.y();
      new_face.z() = new_point_index;
      faces.push_back(new_face);
      math::vector3 new_face_normal;
      float new_face_distance;
      get_face_normal_and_distance_to_origin(
          new_face, polytope, new_face_normal, new_face_distance);
      normals.push_back(new_face_normal);
      faces_distance_to_origin.push_back(new_face_distance);
    }
    min_distance = std::numeric_limits<float>::max();
    for (int i = 0; i < faces_distance_to_origin.size(); i++) {
      float distance = faces_distance_to_origin[i];
      if (distance < min_distance) {
        min_distance = distance;
        min_normal = normals[i];
      }
    }
    edges.clear();
  }
  if (!converged)
    spdlog::error("EPA did not converge");
  return converged;
}

}; // namespace toolkit::sim