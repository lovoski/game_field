#include "toolkit/sim/components/colliders.hpp"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Min_sphere_of_spheres_d.h>
#include <CGAL/Min_sphere_of_spheres_d_traits_3.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/convex_hull_3.h>

namespace toolkit::sim {

using Kernal = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point3 = Kernal::Point_3;
using Surface_mesh = CGAL::Surface_mesh<Point3>;
using Traits = CGAL::Min_sphere_of_spheres_d_traits_3<Kernal, Kernal::FT>;
using Min_Sphere = CGAL::Min_sphere_of_spheres_d<Traits>;
using Vertex_index = CGAL::SM_Vertex_index;
using Face_index = CGAL::SM_Face_index;
using Halfedge_index = CGAL::SM_Halfedge_index;

void extract_connectivity(const Surface_mesh &mesh,
                          convex_hull_collider &target) {
  // Create vertex index mapping
  std::vector<Vertex_index> vertex_indices;
  std::unordered_map<Vertex_index, int> vertex_to_int;
  int vertex_count = 0;
  for (Vertex_index vi : mesh.vertices()) {
    vertex_indices.push_back(vi);
    vertex_to_int[vi] = vertex_count++;
  }

  // Create face index mapping
  std::vector<Face_index> face_indices;
  std::unordered_map<Face_index, int> face_to_int;
  int face_count = 0;
  for (Face_index fi : mesh.faces()) {
    face_indices.push_back(fi);
    face_to_int[fi] = face_count++;
  }

  // Initialize containers
  target.vertices.resize(vertex_count);
  target.faces.resize(face_count);
  target.indices.resize(face_count * 3);
  target.vertex_to_faces.resize(vertex_count);
  target.vertex_to_neighbors.resize(vertex_count);
  target.face_to_neighbors.resize(face_count);

  // Extract vertex_to_faces and vertex_to_neighbors
  for (Vertex_index vi : mesh.vertices()) {
    int v_idx = vertex_to_int[vi];
    target.vertices[v_idx] = math::vector3(
        mesh.point(vi).x(), mesh.point(vi).y(), mesh.point(vi).z());

    // Get a halfedge starting at this vertex
    Halfedge_index hi = mesh.halfedge(vi);
    if (hi == Surface_mesh::null_halfedge())
      continue;

    // Iterate around the vertex using halfedges
    Halfedge_index start = hi;
    Halfedge_index current = hi;
    do {
      // Get face incident to this halfedge
      Face_index fi = mesh.face(current);
      if (fi != Surface_mesh::null_face()) {
        target.vertex_to_faces[v_idx].push_back(face_to_int[fi]);
      }

      // Get neighbor vertex (target of the next halfedge around the face)
      Halfedge_index next = mesh.next(current);
      Vertex_index neighbor_vi = mesh.target(next);
      target.vertex_to_neighbors[v_idx].push_back(vertex_to_int[neighbor_vi]);

      // Move to next halfedge around vertex
      current = mesh.opposite(mesh.next(current));
    } while (current != start && current != Surface_mesh::null_halfedge());

    // Remove duplicates from faces and neighbors
    std::sort(target.vertex_to_faces[v_idx].begin(),
              target.vertex_to_faces[v_idx].end());
    auto last_face = std::unique(target.vertex_to_faces[v_idx].begin(),
                                 target.vertex_to_faces[v_idx].end());
    target.vertex_to_faces[v_idx].erase(last_face,
                                        target.vertex_to_faces[v_idx].end());

    std::sort(target.vertex_to_neighbors[v_idx].begin(),
              target.vertex_to_neighbors[v_idx].end());
    auto last_neighbor = std::unique(target.vertex_to_neighbors[v_idx].begin(),
                                     target.vertex_to_neighbors[v_idx].end());
    target.vertex_to_neighbors[v_idx].erase(
        last_neighbor, target.vertex_to_neighbors[v_idx].end());
  }

  // Extract face_to_neighbors
  for (Face_index fi : mesh.faces()) {
    int f_idx = face_to_int[fi];

    // For each halfedge of the face, find adjacent face
    Halfedge_index he_start = mesh.halfedge(fi);
    Halfedge_index he_current = he_start;
    do {
      Halfedge_index opp_he = mesh.opposite(he_current);
      Face_index neighbor_fi = mesh.face(opp_he);

      if (neighbor_fi != Surface_mesh::null_face() && neighbor_fi != fi) {
        target.face_to_neighbors[f_idx].push_back(face_to_int[neighbor_fi]);
      }

      target.faces[f_idx].elements.push_back(
          vertex_to_int[mesh.target(he_current)]);
      he_current = mesh.next(he_current);
    } while (he_current != he_start);

    // Remove duplicate neighbors
    std::sort(target.face_to_neighbors[f_idx].begin(),
              target.face_to_neighbors[f_idx].end());
    auto last = std::unique(target.face_to_neighbors[f_idx].begin(),
                            target.face_to_neighbors[f_idx].end());
    target.face_to_neighbors[f_idx].erase(
        last, target.face_to_neighbors[f_idx].end());
  }

  for (int i = 0; i < target.faces.size(); i++) {
    int vi0 = target.faces[i].elements[0], vi1 = target.faces[i].elements[1],
        vi2 = target.faces[i].elements[2];
    target.indices[3 * i + 0] = vi0;
    target.indices[3 * i + 1] = vi1;
    target.indices[3 * i + 2] = vi2;
    target.faces[i].normal =
        ((target.vertices[vi1] - target.vertices[vi0])
             .cross(target.vertices[vi2] - target.vertices[vi0]))
            .normalized();
  }
}

void convex_hull_collider::create_from_data(
    std::vector<assets::mesh_vertex> &vertices_data,
    math::vector3 world_scale) {
  std::vector<Point3> points(vertices_data.size());
  for (int i = 0; i < vertices_data.size(); i++) {
    points[i] = Point3(vertices_data[i].position.x() * world_scale.x(),
                       vertices_data[i].position.y() * world_scale.y(),
                       vertices_data[i].position.z() * world_scale.z());
  }
  Surface_mesh convex_hull_mesh;
  CGAL::convex_hull_3(points.begin(), points.end(), convex_hull_mesh);
  CGAL::Polygon_mesh_processing::triangulate_faces(convex_hull_mesh);

  extract_connectivity(convex_hull_mesh, *this);

  std::vector<Traits::Sphere> spheres;
  for (const auto &p : points)
    spheres.emplace_back(Traits::Sphere(p, 0.0));
  Min_Sphere ms(spheres.begin(), spheres.end());
  bounding_sphere_center = math::vector3(ms.center_cartesian_begin()[0],
                                         ms.center_cartesian_begin()[1],
                                         ms.center_cartesian_begin()[2]);
  bounding_sphere_radius = ms.radius();

  transformed_vertices = vertices;
  transformed_faces = faces;
}

}; // namespace toolkit::sim