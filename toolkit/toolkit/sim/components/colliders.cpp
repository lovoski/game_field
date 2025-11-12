#include "toolkit/sim/components/colliders.hpp"
#include "QuickHull.hpp"

namespace toolkit::sim {

void convex_hull_collider::create_from_data(
    std::vector<assets::mesh_vertex> &vertices_data) {
  quickhull::QuickHull<float> qh;
  std::vector<quickhull::Vector3<float>> points(vertices_data.size());
  for (int i = 0; i < vertices_data.size(); i++) {
    points[i].x = vertices_data[i].position.x();
    points[i].y = vertices_data[i].position.y();
    points[i].z = vertices_data[i].position.z();
  }
  auto hull = qh.getConvexHull(points, true, false);
  auto &index_buffer = hull.getIndexBuffer();
  auto &vertex_buffer = hull.getVertexBuffer();

  vertices.resize(vertex_buffer.size());
  faces.resize(index_buffer.size() / 3);

  for (int i = 0; i < vertices.size(); i++) {
    vertices[i].x() = vertex_buffer[i].x;
    vertices[i].y() = vertex_buffer[i].y;
    vertices[i].z() = vertex_buffer[i].z;
  }
  for (int i = 0; i < faces.size(); i++) {
    faces[i].elements.push_back(index_buffer[3 * i + 0]);
    faces[i].elements.push_back(index_buffer[3 * i + 1]);
    faces[i].elements.push_back(index_buffer[3 * i + 2]);
    auto v0 = vertices[index_buffer[3 * i + 0]],
         v1 = vertices[index_buffer[3 * i + 1]],
         v2 = vertices[index_buffer[3 * i + 2]];
    faces[i].normal = ((v1 - v0).cross(v2 - v1)).normalized();
  }
}

void rigid_sim_object::setup_mass(float mass_value, bool is_fixed) {
  if (is_fixed) {
    inverse_mass = 0;
    fixed = is_fixed;
    inertia_tensor = math::matrix3::Zero();
    inverse_inertia_tensor = math::matrix3::Zero();
  } else {
    inverse_mass = 1.0 / mass_value;
  }
}

nlohmann::json rigid_sim_object::late_serialize() {
  nlohmann::json extra_data;
  return extra_data;
}

void rigid_sim_object::late_deserialize(nlohmann::json &data) {}

void rigid_sim_object::update_bounding_volumn_given_colliders() {
  if (colliders.size() == 0) {
    bounding_sphere_radius = 0.0f;
    bounding_sphere_center = math::vector3::Zero();
  } else {
    std::vector<std::pair<math::vector3, float>> spheres;
    for (int i = 0; i < colliders.size(); i++) {
      if (colliders[i]->type == collider_type::SPHERE) {
        auto collider = dynamic_cast<sphere_collider *>(colliders[i].get());
        spheres.emplace_back(
            std::make_pair(collider->world_pos, collider->radius));
      } else if (colliders[i]->type == collider_type::CAPSULE) {
        // TODO:
      } else if (colliders[i]->type == collider_type::CONVEX_HULL) {
        // TODO:
      }
    }
    bounding_sphere_center = spheres[0].first;
    bounding_sphere_radius = spheres[0].second;
    for (int i = 1; i < spheres.size(); i++) {
      float center_dist = (bounding_sphere_center - spheres[i].first).norm();
      if (center_dist + spheres[i].second <= bounding_sphere_radius)
        continue;
      else if (center_dist + bounding_sphere_radius <= spheres[i].second) {
        bounding_sphere_center = spheres[i].first;
        bounding_sphere_radius = spheres[i].second;
      } else {
        auto center_dir =
            (spheres[i].first - bounding_sphere_center).normalized();
        bounding_sphere_center =
            bounding_sphere_center +
            (-0.5f * bounding_sphere_radius + 0.5f * center_dist +
             0.5f * spheres[i].second) *
                center_dir;
        bounding_sphere_radius =
            0.5f * (center_dist + spheres[i].second + bounding_sphere_radius);
      }
    }
  }
}

void rigid_sim_object::draw_gui(entt::registry &registry, entt::entity entity) {
  if (ImGui::TreeNode("Colliders")) {
    if (ImGui::BeginMenu("Add Collider")) {
      if (ImGui::MenuItem("Sphere Collider")) {
        sphere_collider collider;
        colliders.emplace_back(std::make_shared<sphere_collider>(collider));
      }
      if (ImGui::MenuItem("Capsule Collider")) {
        capsule_collider collider;
        colliders.emplace_back(std::make_shared<capsule_collider>(collider));
      }
      if (ImGui::MenuItem("Convex Hull Collider")) {
        convex_hull_collider collider;
        colliders.emplace_back(
            std::make_shared<convex_hull_collider>(collider));
      }
      ImGui::EndMenu();
    }
    ImGui::Separator();

    int index_to_remove = -1;
    for (int i = 0; i < colliders.size(); i++) {
      if (colliders[i]->type == collider_type::SPHERE) {
        ImGui::SeparatorText("Sphere Collider");
        sphere_collider *collider =
            dynamic_cast<sphere_collider *>(colliders[i].get());
        ImGui::DragFloat(("Radius##" + std::to_string(i)).c_str(),
                         &(collider->radius), 0.001f, 0.0f, 1e10f);
        ImGui::DragFloat3(("Local Pos##" + std::to_string(i)).c_str(),
                          collider->local_pos.data(), 0.001f, -1e38f, 1e38f);
        if (ImGui::Button(("Remove##sc" + std::to_string(i)).c_str(), {-1, 30}))
          index_to_remove = i;
      } else if (colliders[i]->type == collider_type::CAPSULE) {
        ImGui::SeparatorText("Capsule Collider");
        if (ImGui::Button(("Remove##cc" + std::to_string(i)).c_str(), {-1, 30}))
          index_to_remove = i;
      } else if (colliders[i]->type == collider_type::CONVEX_HULL) {
        ImGui::SeparatorText("Convex Hull Collider");
        if (ImGui::Button(("Remove##chc" + std::to_string(i)).c_str(),
                          {-1, 30}))
          index_to_remove = i;
      }
    }
    if (index_to_remove >= 0 && index_to_remove < colliders.size())
      colliders.erase(colliders.begin() + index_to_remove);
    ImGui::TreePop();
  }
}

}; // namespace toolkit::sim