#include "toolkit/sim/algorithms/algo.hpp"

using namespace toolkit::math;
const float EPSILON = 0.0001f;

namespace toolkit::sim {

struct _cm_plane {
  vector3 normal;
  vector3 point;
};

vector3 get_closest_point_polygon(const vector3 &position,
                                  _cm_plane &reference_plane);

// Clips the input polygon to the input clip planes
// If remove_instead_of_clipping is true, vertices that are lying outside the
// clipping planes will be removed instead of clipped Based on
// https://research.ncl.ac.uk/game/mastersdegree/gametechnologies/previousinformation/physics5collisionmanifolds/
void sutherland_hodgman(std::vector<vector3> &input_polygon,
                        int num_clip_planes,
                        const std::vector<_cm_plane> &clip_planes,
                        std::vector<vector3> &out_polygon,
                        bool remove_instead_of_clipping);

int face_with_most_fitting_normal(convex_hull_collider *collider,
                                  int support_point_idx, vector3 normal);

int convex_hull_support_idx(convex_hull_collider *collider,
                            const vector3 &direction);

std::vector<vector3>
get_vertices_of_faces(const convex_hull_collider *hull,
                      const convex_hull_collider_face &face);

std::pair<std::pair<int, int>, std::pair<int, int>>
edge_with_most_fitting_normal(int sp1_idx, int sp2_idx,
                              convex_hull_collider *c1,
                              convex_hull_collider *c2, vector3 normal,
                              vector3 &edge_normal);

std::vector<_cm_plane> build_boundary_planes(convex_hull_collider *convex_hull,
                                             int target_face_idx);

/**
 * This function calculates the distance between two indepedent skew lines in
 * the 3D world The first line is given by a known point P1 and a direction
 * vector D1 The second line is given by a known point P2 and a direction vector
 * D2 Outputs: L1 is the closest POINT to the second line that belongs to the
 * first line L2 is the closest POINT to the first line that belongs to the
 * second line _N is the number that satisfies L1 = P1 + _N * D1 _M is the
 * number that satisfies L2 = P2 + _M * D2
 */
bool collision_distance_between_skew_lines(const vector3 &p1, const vector3 &d1,
                                           const vector3 &p2, const vector3 &d2,
                                           vector3 &l1, vector3 &l2, float &n,
                                           float &m);

void convex_convex_contact_manifold(convex_hull_collider *c1,
                                    convex_hull_collider *c2, vector3 normal,
                                    std::vector<collider_contact> &contacts) {
  // the contact normal is a rough estimate of how c1 penetrates into c2

  // the greatest penetration point on c1 that penetrates to c2
  auto sp1_idx = convex_hull_support_idx(c1, normal);
  // the greatest penetration point on c2 that penetrates to c1
  auto sp2_idx = convex_hull_support_idx(c2, -normal);
  // the face on c1 that matchess best with the penetration normal (close to the
  // support point maybe?)
  auto f1_idx = face_with_most_fitting_normal(c1, sp1_idx, normal);
  auto f2_idx = face_with_most_fitting_normal(c2, sp2_idx, -normal);
  auto face1 = c1->transformed_faces[f1_idx];
  auto face2 = c2->transformed_faces[f2_idx];
  vector3 edge_normal;
  auto edges = edge_with_most_fitting_normal(sp1_idx, sp2_idx, c1, c2, normal,
                                             edge_normal);
  float chosen_normal1_dot = face1.normal.dot(normal);
  float chosen_normal2_dot = face2.normal.dot(-normal);
  float edge_normal_dot = edge_normal.dot(normal);
  if ((edge_normal_dot > chosen_normal1_dot + EPSILON) &&
      (edge_normal_dot > chosen_normal2_dot + EPSILON)) {
    // edge
    // std::cout << "Edge collision between convex hulls" << std::endl;
    float n, m;
    vector3 l1, l2;
    vector3 p1 = c1->transformed_vertices[edges.first.first];
    vector3 d1 = c1->transformed_vertices[edges.first.second] - p1;
    vector3 p2 = c2->transformed_vertices[edges.second.first];
    vector3 d2 = c2->transformed_vertices[edges.second.second] - p2;
    if (collision_distance_between_skew_lines(p1, d1, p2, d2, l1, l2, n, m)) {
      collider_contact contact;
      contact.contact_point1 = l1;
      contact.contact_point2 = l2;
      contact.normal = normal;
      contacts.emplace_back(contact);
    }
  } else {
    // face
    // std::cout << "Face collision between convex hulls" << std::endl;
    bool is_face1_the_reference_face = chosen_normal1_dot > chosen_normal2_dot;
    std::vector<vector3> reference_face_support_points =
        is_face1_the_reference_face ? get_vertices_of_faces(c1, face1)
                                    : get_vertices_of_faces(c2, face2);
    std::vector<vector3> incident_face_support_points =
        is_face1_the_reference_face ? get_vertices_of_faces(c2, face2)
                                    : get_vertices_of_faces(c1, face1);
    auto boundary_planes = is_face1_the_reference_face
                               ? build_boundary_planes(c1, f1_idx)
                               : build_boundary_planes(c2, f2_idx);
    std::vector<vector3> clipped_points;
    sutherland_hodgman(incident_face_support_points, boundary_planes.size(),
                       boundary_planes, clipped_points, false);
    _cm_plane reference_plane;
    reference_plane.normal =
        is_face1_the_reference_face ? -face1.normal : -face2.normal;
    reference_plane.point = reference_face_support_points[0];
    std::vector<_cm_plane> reference_planes;
    reference_planes.push_back(reference_plane);
    std::vector<vector3> final_clipped_points;
    sutherland_hodgman(clipped_points, 1, reference_planes,
                       final_clipped_points, true);
    for (int i = 0; i < final_clipped_points.size(); i++) {
      auto point = final_clipped_points[i];
      auto closest_point = get_closest_point_polygon(point, reference_plane);
      vector3 point_diff = point - closest_point;
      float contact_penetration;
      // we are projecting the points that are in the incident face on the
      // reference planes so the points that we have are part of the incident
      // object.
      collider_contact contact;
      if (is_face1_the_reference_face) {
        contact_penetration = point_diff.dot(normal);
        contact.contact_point1 = point - contact_penetration * normal;
        contact.contact_point2 = point;
      } else {
        contact_penetration = -point_diff.dot(normal);
        contact.contact_point1 = point;
        contact.contact_point2 = point + contact_penetration * normal;
      }
      contact.normal = normal;
      contact.penetration = contact_penetration;
      if (contact_penetration < 0.0f)
        contacts.push_back(contact);
    }
  }
}

bool is_point_in_plane(const _cm_plane &plane, vector3 position) {
  float distance = -(plane.normal.dot(plane.point));
  if (position.dot(plane.normal) + distance < 0.0) {
    return false;
  }
  return true;
}

vector3 get_closest_point_polygon(const vector3 &position,
                                  _cm_plane &reference_plane) {
  float d = (-reference_plane.normal).dot(reference_plane.point);
  return position -
         (reference_plane.normal.dot(position) + d) * (reference_plane.normal);
}

bool plane_edge_intersection(const _cm_plane &plane, const vector3 &start,
                             const vector3 &end, vector3 &out_point) {
  const float EPSILON = 0.000001f;
  vector3 ab = end - start;
  // Check that the edge and plane are not parallel and thus never intersect
  // We do this by projecting the line (start - A, End - B) ab along the plane
  float ab_p = plane.normal.dot(ab);
  if (std::fabs(ab_p) > EPSILON) {
    // Generate a random point on the plane (any point on the plane will
    // suffice)
    float distance = -plane.normal.dot(plane.point);
    vector3 p_co = -distance * plane.normal;
    // Work out the edge factor to scale edge by
    // e.g. how far along the edge to traverse before it meets the plane.
    // This is computed by: -proj<plane_nrml>(edge_start - any_planar_point) /
    // proj<plane_nrml>(edge_start - edge_end)
    float fac = -(plane.normal.dot(start - p_co)) / ab_p;
    // Stop any large floating point divide issues with almost parallel planes
    fac = std::min(std::max(fac, 0.0f), 1.0f);
    // Return point on edge
    out_point = start + fac * ab;
    return true;
  }
  return false;
}

void sutherland_hodgman(std::vector<vector3> &input_polygon,
                        int num_clip_planes,
                        const std::vector<_cm_plane> &clip_planes,
                        std::vector<vector3> &out_polygon,
                        bool remove_instead_of_clipping) {
  assert(num_clip_planes > 0);
  // Create temporary list of vertices
  // We will keep ping-pong'ing between the two lists updating them as we go.
  std::vector<vector3> input = input_polygon;
  std::vector<vector3> output;
  for (int i = 0; i < num_clip_planes; ++i) {
    // If every single point has already been removed previously, just exit
    if (input.size() == 0) {
      break;
    }
    const _cm_plane &plane = clip_planes[i];
    // Loop through each edge of the polygon and clip that edge against the
    // current plane.
    vector3 temp_point, start_point = input[input.size() - 1];
    for (int j = 0; j < input.size(); ++j) {
      vector3 end_point = input[j];
      bool start_in_plane = is_point_in_plane(plane, start_point);
      bool end_in_plane = is_point_in_plane(plane, end_point);

      if (remove_instead_of_clipping) {
        if (end_in_plane) {
          output.push_back(end_point);
        }
      } else {
        // If the edge is entirely within the clipping plane, keep it as it is
        if (start_in_plane && end_in_plane) {
          output.push_back(end_point);
        }
        // If the edge interesects the clipping plane, cut the edge along clip
        // plane
        else if (start_in_plane && !end_in_plane) {
          if (plane_edge_intersection(plane, start_point, end_point,
                                      temp_point)) {
            output.push_back(temp_point);
          }
        } else if (!start_in_plane && end_in_plane) {
          if (plane_edge_intersection(plane, start_point, end_point,
                                      temp_point)) {
            output.push_back(temp_point);
          }
          output.push_back(end_point);
        }
      }
      // ..otherwise the edge is entirely outside the clipping plane and should
      // be removed/ignored
      start_point = end_point;
    }
    // Swap input/output polygons, and clear output list for us to generate
    // afresh
    input = output;
  }
  out_polygon = input;
}

std::vector<_cm_plane> build_boundary_planes(convex_hull_collider *convex_hull,
                                             int target_face_idx) {
  std::vector<_cm_plane> result;
  auto face_neighbors = convex_hull->face_to_neighbors[target_face_idx];
  for (auto neighbor_face_idx : face_neighbors) {
    auto neighbor_face = convex_hull->transformed_faces[neighbor_face_idx];
    _cm_plane p;
    p.point = convex_hull->transformed_vertices[neighbor_face.elements[0]];
    p.normal = -neighbor_face.normal;
    result.emplace_back(p);
  }
  return result;
}

std::vector<vector3>
get_vertices_of_faces(const convex_hull_collider *hull,
                      const convex_hull_collider_face &face) {
  std::vector<vector3> vertices;
  for (int i = 0; i < face.elements.size(); ++i) {
    vertices.push_back(hull->transformed_vertices[face.elements[i]]);
  }
  return vertices;
}

bool collision_distance_between_skew_lines(const vector3 &p1, const vector3 &d1,
                                           const vector3 &p2, const vector3 &d2,
                                           vector3 &l1, vector3 &l2, float &n,
                                           float &m) {
  float n1 = d1.x() * d2.x() + d1.y() * d2.y() + d1.z() * d2.z();
  float n2 = d2.x() * d2.x() + d2.y() * d2.y() + d2.z() * d2.z();
  float m1 = -d1.x() * d1.x() - d1.y() * d1.y() - d1.z() * d1.z();
  float m2 = -d2.x() * d1.x() - d2.y() * d1.y() - d2.z() * d1.z();
  float r1 = -d1.x() * p2.x() + d1.x() * p1.x() - d1.y() * p2.y() +
             d1.y() * p1.y() - d1.z() * p2.z() + d1.z() * p1.z();
  float r2 = -d2.x() * p2.x() + d2.x() * p1.x() - d2.y() * p2.y() +
             d2.y() * p1.y() - d2.z() * p2.z() + d2.z() * p1.z();
  // Solve 2x2 linear system
  if ((n1 * m2) - (n2 * m1) == 0) {
    return false;
  }
  n = ((r1 * m2) - (r2 * m1)) / ((n1 * m2) - (n2 * m1));
  m = ((n1 * r2) - (n2 * r1)) / ((n1 * m2) - (n2 * m1));
  l1 = p1 + m * d1;
  l2 = p2 + n * d2;
  return true;
}

std::pair<std::pair<int, int>, std::pair<int, int>>
edge_with_most_fitting_normal(int sp1_idx, int sp2_idx,
                              convex_hull_collider *c1,
                              convex_hull_collider *c2, vector3 normal,
                              vector3 &edge_normal) {
  std::pair<std::pair<int, int>, std::pair<int, int>> selected_edges;
  auto sp1 = c1->transformed_vertices[sp1_idx];
  auto sp2 = c2->transformed_vertices[sp2_idx];
  auto &sp1_neighbors = c1->vertex_to_neighbors[sp1_idx];
  auto &sp2_neighbors = c2->vertex_to_neighbors[sp2_idx];
  float max_dot = -std::numeric_limits<float>::max();
  for (auto sp1_neighbor_idx : sp1_neighbors) {
    auto neighbor1 = c1->transformed_vertices[sp1_neighbor_idx];
    vector3 edge1 = sp1 - neighbor1;
    for (auto sp2_neighbor_idx : sp2_neighbors) {
      auto neighbor2 = c2->transformed_vertices[sp2_neighbor_idx];
      vector3 edge2 = sp2 - neighbor2;
      vector3 current_normal = (edge1.cross(edge2)).normalized();
      float dot_value = current_normal.dot(normal);
      if (std::abs(dot_value) > max_dot) {
        max_dot = std::abs(dot_value);
        edge_normal = dot_value > 0 ? current_normal : -current_normal;
        selected_edges =
            std::make_pair(std::make_pair(sp1_idx, sp1_neighbor_idx),
                           std::make_pair(sp2_idx, sp2_neighbor_idx));
      }
    }
  }
  return selected_edges;
}

int convex_hull_support_idx(convex_hull_collider *collider,
                            const vector3 &direction) {
  vector3 norm_dir = direction.normalized();
  float max_dot = -std::numeric_limits<float>::max();
  int point_idx = -1;
  for (int i = 0; i < collider->transformed_vertices.size(); i++) {
    float dot = collider->transformed_vertices[i].dot(norm_dir);
    if (dot > max_dot) {
      max_dot = dot;
      point_idx = i;
    }
  }
  return point_idx;
}

int face_with_most_fitting_normal(convex_hull_collider *collider,
                                  int support_point_idx, vector3 normal) {
  auto support_faces = collider->vertex_to_faces[support_point_idx];
  float max_proj = -std::numeric_limits<float>::max();
  int selected_face_idx;
  for (auto support_face_idx : support_faces) {
    auto face = collider->transformed_faces[support_face_idx];
    float proj = face.normal.dot(normal);
    if (proj > max_proj) {
      max_proj = proj;
      selected_face_idx = support_face_idx;
    }
  }
  return selected_face_idx;
}

}; // namespace toolkit::sim