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
  indices.resize(index_buffer.size());
  faces.resize(index_buffer.size() / 3);

  for (int i = 0; i < vertices.size(); i++) {
    vertices[i].x() = vertex_buffer[i].x;
    vertices[i].y() = vertex_buffer[i].y;
    vertices[i].z() = vertex_buffer[i].z;
  }
  for (int i = 0; i < faces.size(); i++) {
    indices[3 * i + 0] = index_buffer[3 * i + 0];
    indices[3 * i + 1] = index_buffer[3 * i + 1];
    indices[3 * i + 2] = index_buffer[3 * i + 2];
    faces[i].elements.push_back(index_buffer[3 * i + 0]);
    faces[i].elements.push_back(index_buffer[3 * i + 1]);
    faces[i].elements.push_back(index_buffer[3 * i + 2]);
    auto v0 = vertices[index_buffer[3 * i + 0]],
         v1 = vertices[index_buffer[3 * i + 1]],
         v2 = vertices[index_buffer[3 * i + 2]];
    faces[i].normal = ((v1 - v0).cross(v2 - v1)).normalized();
  }

  transformed_vertices = vertices;
  transformed_faces = faces;
}

void rigid_sim_object::setup_mass_inertia(float mass_value, bool is_fixed) {
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
  if (collider == nullptr) {
    bounding_sphere_radius = 0.0f;
    bounding_sphere_center = math::vector3::Zero();
  } else {
    if (collider->type == collider_type::SPHERE) {
      auto c = dynamic_cast<sphere_collider *>(collider.get());
      bounding_sphere_radius = c->radius;
      bounding_sphere_center = c->world_pos;
    } else if (collider->type == collider_type::CAPSULE) {
      auto c = dynamic_cast<capsule_collider *>(collider.get());
      auto capsule_dir = c->world_rot * math::world_up;
      bounding_sphere_radius = c->cap_distance * 0.5f + c->cap_radius;
      bounding_sphere_center = c->world_pos;
    } else if (collider->type == collider_type::CONVEX_HULL) {
      auto c = dynamic_cast<convex_hull_collider *>(collider.get());
      bounding_sphere_center = c->transformed_bounding_sphere_center;
      bounding_sphere_radius = c->bounding_sphere_radius;
    }
  }
}

void rigid_sim_object::draw_gui(entt::registry &registry, entt::entity entity) {
  if (ImGui::BeginMenu("Collider Type")) {
    if (ImGui::MenuItem("Sphere Collider")) {
      sphere_collider c;
      collider = std::make_shared<sphere_collider>(c);
    }
    if (ImGui::MenuItem("Capsule Collider")) {
      capsule_collider c;
      collider = std::make_shared<capsule_collider>(c);
    }
    if (ImGui::MenuItem("Convex Hull Collider")) {
      auto mesh_ptr = registry.try_get<opengl::mesh_data>(entity);
      if (mesh_ptr == nullptr) {
        spdlog::error("Entity doesn't have mesh component, can't set to "
                      "convex collider");
      } else {
        spdlog::info("Create convex hull collider for mesh");
        convex_hull_collider c;
        c.create_from_data(mesh_ptr->vertices);
        auto bs = welzl_bounding_sphere(c.vertices, true);
        c.bounding_sphere_center = bs.first;
        c.bounding_sphere_radius = bs.second;
        collider = std::make_shared<convex_hull_collider>(c);
      }
    }
    ImGui::EndMenu();
  }

  if (collider != nullptr) {
    if (collider->type == collider_type::SPHERE) {
      ImGui::SeparatorText("Sphere Collider");
      sphere_collider *c = dynamic_cast<sphere_collider *>(collider.get());
      ImGui::DragFloat(("Radius##" + std::to_string(0)).c_str(), &(c->radius),
                       0.001f, 0.0f, 1e10f);
      ImGui::DragFloat3(("Local Pos##" + std::to_string(0)).c_str(),
                        c->local_pos.data(), 0.001f, -1e38f, 1e38f);
    } else if (collider->type == collider_type::CAPSULE) {
      ImGui::SeparatorText("Capsule Collider");
      capsule_collider *c = dynamic_cast<capsule_collider *>(collider.get());
      ImGui::DragFloat("Cap Radius", &c->cap_radius, 0.001f, 0.0f, 10.0f);
      ImGui::DragFloat("Cap Length", &c->cap_distance, 0.001f, 0.0f, 20.0f);
      ImGui::DragFloat3(("Local Pos##" + std::to_string(0)).c_str(),
                        c->local_pos.data(), 0.001f, -1e38f, 1e38f);
      ImGui::DragFloat3(("Local Rot##" + std::to_string(0)).c_str(),
                        c->local_angle.data(), 0.001f, -180.0f, 180.0f);
    } else if (collider->type == collider_type::CONVEX_HULL) {
      ImGui::SeparatorText("Convex Hull Collider");
      if (ImGui::BeginTable("##convex_hull_collider", 2,
                            ImGuiTableFlags_Resizable |
                                ImGuiTableFlags_Borders)) {
        ImGui::TableSetupColumn("Property");
        ImGui::TableSetupColumn("Value");
        ImGui::TableHeadersRow();

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("Num Vertex");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%d", dynamic_cast<convex_hull_collider *>(collider.get())
                              ->vertices.size());

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("Num Faces");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text(
            "%d",
            dynamic_cast<convex_hull_collider *>(collider.get())->faces.size());

        ImGui::EndTable();
      }
    }
  }
}
}; // namespace toolkit::sim